//! DMA controller driver: three channels moving bytes/words without the CPU.
//!
//! The MSP430FR5969's DMA controller has **three identical channels**. Each
//! holds a source address, a destination address, a transfer count, and a
//! *trigger*: one of 32 hardware signals (device table in SLAS704G; values
//! double-checked against TI's `msp430fr5969.h`) whose rising edge moves the
//! next item. Trigger 0 is the software `DMAREQ` bit, which is what turns a
//! channel into a plain memory-to-memory copier; the interesting triggers are
//! peripheral flags — `UCA0TXIFG` says "TXBUF is free", so a channel triggered
//! by it feeds a UART exactly as fast as the wire drains it, with the CPU idle
//! (or asleep — DMA runs in LPM0/LPM1, and even wakes the clocks it needs per
//! transfer in LPM3 for the duration of the transfer).
//!
//! # What "DMA" means on this bus
//!
//! There is one address/data bus and the DMA controller *takes* it: every DMA
//! transfer halts the CPU for a few MCLK cycles (~2 per item plus setup —
//! SLAU367P Table 11-2) and performs an ordinary bus read + bus write. Two
//! consequences worth internalizing:
//!
//! - A **block transfer** (`DMADT = 1`) with the software trigger halts the
//!   CPU until the whole block has moved — so by the time the instruction
//!   after "set `DMAREQ`" executes, the copy is *done*. The safe
//!   [`copy_bytes`](Channel::copy_bytes)/[`copy_words`](Channel::copy_words)
//!   APIs lean on this: they are blocking by hardware construction, which is
//!   what lets them borrow plain slices with no `'static` gymnastics.
//! - A **single-transfer** channel (`DMADT = 0`) armed on a peripheral flag
//!   steals the bus for one item per trigger, interleaved with normal CPU
//!   execution. Anything the CPU believes about the armed buffer is stale the
//!   moment the peripheral fires — which is why the arming primitive
//!   ([`arm_single_bytes`](Channel::arm_single_bytes)) is `unsafe` and the
//!   safe wrappers in [`serial`](crate::serial) and [`spi`](crate::spi) block
//!   until the channel reports done before releasing the borrow.
//!
//! Because a DMA transfer can slot *between any two CPU instructions*, it can
//! also split a read-modify-write. [`DmaExt::split`] therefore sets
//! `DMARMWDIS` (inhibit DMA during CPU RMW operations) once at module init,
//! keeping the crate's critical-section discipline honest: `DINT` does not
//! stop DMA, but with `DMARMWDIS` set a `bis.b`/`bic.b` on a shared register
//! at least cannot be torn by one.
//!
//! # Triggers are edge-sensitive here
//!
//! SLAU367P allows level-sensitive triggering (`DMALEVEL = 1`) **only for the
//! external `DMAE0` trigger**, so this driver uses edge-sensitive triggering
//! throughout. That has one classic consequence for peripheral pacing: a flag
//! that is *already high* when the channel is armed never presents an edge,
//! and the channel stalls. The pattern for `UCxTXIFG` (idles high) is to arm
//! the channel on `buf[1..]` and write `buf[0]` manually — the flag's
//! 0→1 hop when that byte moves to the shift register is the first edge the
//! channel sees. For `UCxRXIFG` (idles low) the buffer must be drained
//! *before* arming. The serial/SPI integration methods implement both
//! patterns; they are documented here because anyone using
//! [`arm_single_bytes`](Channel::arm_single_bytes) directly must replicate
//! them.
//!
//! # Register access
//!
//! The three channels are one register block with a regular 0x10 stride, so —
//! as in [`serial`](crate::serial) and [`spi`](crate::spi) — the driver uses
//! raw volatile accesses at documented offsets: a single `Channel<const N>`
//! implementation instead of three copies of PAC-typed code. (The PAC models
//! the block fully; ownership is still tracked by consuming the `pac::Dma`
//! singleton in [`DmaExt::split`], which is also where the module-level
//! `DMACTL4` write happens through the typed API.)
//!
//! # Interrupt
//!
//! Each channel's `DMAIFG` (with `DMAIE` set, see
//! [`enable_done_interrupt`](Channel::enable_done_interrupt)) fires the single
//! shared `DMA` vector. As with GPIO/RTC, the ISR demuxes via an IV register:
//! [`read_iv`] returns 0x02/0x04/0x06 for channel 0/1/2, and the read
//! atomically clears the reported flag in silicon — consume it once, don't
//! double-dip with [`Channel::is_done`].
//!
//! Since the IV read clears `DMAxCTL.DMAIFG` behind the CPU's back, every
//! thread-mode RMW of a `DMAxCTL` register in this driver runs inside
//! `critical_section::with` — the same lost-flag hazard the GPIO arming rules
//! guard against.

use core::marker::PhantomData;

use crate::pac;

// ---------------------------------------------------------------------------
// Register layout (absolute addresses; DMA block base is 0x0500)
// ---------------------------------------------------------------------------

const DMA_BASE: usize = 0x0500;
const DMACTL0: usize = DMA_BASE + 0x00; // Trigger select: ch0 bits 4:0, ch1 bits 12:8
const DMACTL1: usize = DMA_BASE + 0x02; // Trigger select: ch2 bits 4:0
const DMAIV: usize = DMA_BASE + 0x0E; // Interrupt vector (read clears reported flag)

/// Per-channel register offsets from the channel's own base
/// (`0x0510 + 0x10 * N`): CTL, then the 20-bit SA/DA as two words each, then SZ.
const CH_STRIDE: usize = 0x10;
const CH0_BASE: usize = DMA_BASE + 0x10;
const CTL: usize = 0x00; // DMAxCTL
const SA: usize = 0x02; // DMAxSA (20-bit: low word + high word)
const DA: usize = 0x06; // DMAxDA (20-bit: low word + high word)
const SZ: usize = 0x0A; // DMAxSZ (transfer count, decrements; reloads on completion)

// DMAxCTL bit fields (SLAU367P; values verified against msp430fr5969.h)
const DMAREQ: u16 = 1 << 0; // Software trigger (trigger source 0)
const DMAIE: u16 = 1 << 2; // Interrupt enable (DMAIFG -> DMA vector)
const DMAIFG: u16 = 1 << 3; // Transfer-complete flag (DMAxSZ reached 0)
const DMAEN: u16 = 1 << 4; // Channel enable
const DMALEVEL: u16 = 1 << 5; // Level-sensitive trigger (edge-sensitive when 0)
const DMASRCBYTE: u16 = 1 << 6; // Source is byte-wide (0 = word)
const DMADSTBYTE: u16 = 1 << 7; // Destination is byte-wide (0 = word)
const DMASRCINCR_SHIFT: u16 = 8; // Source address mode, bits 9:8
const DMADSTINCR_SHIFT: u16 = 10; // Destination address mode, bits 11:10
const DMADT_SINGLE: u16 = 0 << 12; // One item per trigger; DMAEN clears at SZ = 0
const DMADT_BLOCK: u16 = 1 << 12; // Whole block per trigger; CPU halted throughout

/// Trigger-select field masks within DMACTL0/DMACTL1 (5 bits per channel).
const TSEL_MASK: u16 = 0x001F;

#[inline(always)]
unsafe fn read_reg(addr: usize) -> u16 {
    (addr as *const u16).read_volatile()
}

#[inline(always)]
unsafe fn write_reg(addr: usize, val: u16) {
    (addr as *mut u16).write_volatile(val);
}

/// Compiler memory barrier (same construct as the `msp430` crate's
/// `asm::barrier`). Volatile register accesses are not ordered against
/// ordinary buffer reads/writes, so without this the compiler may hoist a
/// read of the destination buffer above the register write that started the
/// transfer filling it — or sink a source-buffer write below it.
#[inline(always)]
fn barrier() {
    // SAFETY: an empty asm block; without `nomem` it is a full memory clobber.
    unsafe { core::arch::asm!("", options(nostack, preserves_flags)) }
}

// ---------------------------------------------------------------------------
// Configuration enums
// ---------------------------------------------------------------------------

/// Hardware trigger sources for a DMA channel — device-specific wiring, from
/// SLAS704G's trigger-assignment table (values double-checked against TI's
/// `msp430fr5969.h`). All three channels share the same assignment on this
/// part. Gaps in the numbering are reserved values.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum TriggerSource {
    /// Software: the channel's own `DMAREQ` bit (what the block-copy APIs use).
    DmaReq = 0,
    /// Timer_A0 CCR0 CCIFG.
    Ta0Ccr0 = 1,
    /// Timer_A0 CCR2 CCIFG.
    Ta0Ccr2 = 2,
    /// Timer_A1 CCR0 CCIFG.
    Ta1Ccr0 = 3,
    /// Timer_A1 CCR2 CCIFG.
    Ta1Ccr2 = 4,
    /// Timer_A2 CCR0 CCIFG.
    Ta2Ccr0 = 5,
    /// Timer_A3 CCR0 CCIFG.
    Ta3Ccr0 = 6,
    /// Timer_B0 CCR0 CCIFG.
    Tb0Ccr0 = 7,
    /// Timer_B0 CCR2 CCIFG.
    Tb0Ccr2 = 8,
    /// eUSCI_A0 receive (`UCA0RXIFG`).
    UcA0Rx = 14,
    /// eUSCI_A0 transmit ready (`UCA0TXIFG`).
    UcA0Tx = 15,
    /// eUSCI_A1 receive (`UCA1RXIFG`).
    UcA1Rx = 16,
    /// eUSCI_A1 transmit ready (`UCA1TXIFG`).
    UcA1Tx = 17,
    /// eUSCI_B0 receive, slave address 0 (`UCB0RXIFG0` — also the SPI-mode RX flag).
    UcB0Rx0 = 18,
    /// eUSCI_B0 transmit, slave address 0 (`UCB0TXIFG0` — also the SPI-mode TX flag).
    UcB0Tx0 = 19,
    /// eUSCI_B0 receive, slave address 1 (`UCB0RXIFG1`).
    UcB0Rx1 = 20,
    /// eUSCI_B0 transmit, slave address 1 (`UCB0TXIFG1`).
    UcB0Tx1 = 21,
    /// eUSCI_B0 receive, slave address 2 (`UCB0RXIFG2`).
    UcB0Rx2 = 22,
    /// eUSCI_B0 transmit, slave address 2 (`UCB0TXIFG2`).
    UcB0Tx2 = 23,
    /// eUSCI_B0 receive, slave address 3 (`UCB0RXIFG3`).
    UcB0Rx3 = 24,
    /// eUSCI_B0 transmit, slave address 3 (`UCB0TXIFG3`).
    UcB0Tx3 = 25,
    /// ADC12_B end of conversion (`ADC12IFGx`).
    Adc12 = 26,
    /// Hardware multiplier result ready.
    MpyReady = 29,
    /// The *previous* channel's `DMAxIFG` (ch0 <- DMA2IFG, ch1 <- DMA0IFG,
    /// ch2 <- DMA1IFG) — for chaining channels.
    PrevChannelDone = 30,
    /// External `DMAE0` trigger (the one source SLAU367P allows level-sensitive
    /// triggering for; this driver still arms it edge-sensitive).
    DmaE0 = 31,
}

/// What happens to an address after each transferred item.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AddrMode {
    /// Address unchanged (a peripheral data register, or a fill constant).
    Fixed,
    /// Address decremented by the item width.
    Decrement,
    /// Address incremented by the item width (a normal buffer walk).
    Increment,
}

impl AddrMode {
    /// The 2-bit `DMAxINCR` field value (0b00 unchanged, 0b10 dec, 0b11 inc).
    const fn bits(self) -> u16 {
        match self {
            AddrMode::Fixed => 0b00,
            AddrMode::Decrement => 0b10,
            AddrMode::Increment => 0b11,
        }
    }
}

// ---------------------------------------------------------------------------
// Channels
// ---------------------------------------------------------------------------

/// One DMA channel (`N` = 0, 1, or 2). Obtained from [`DmaExt::split`].
///
/// `PhantomData<*const ()>` keeps it `!Send`/`!Sync` (it fronts shared
/// memory-mapped state) and zero-sized.
pub struct Channel<const N: u8> {
    _not_send: PhantomData<*const ()>,
}

/// The DMA controller split into its three independent channels.
pub struct Channels {
    pub ch0: Channel<0>,
    pub ch1: Channel<1>,
    pub ch2: Channel<2>,
}

/// Extension trait to turn the PAC `Dma` peripheral into three [`Channel`]s.
pub trait DmaExt {
    /// Consume the PAC peripheral and split it into channels.
    fn split(self) -> Channels;
}

impl DmaExt for pac::Dma {
    /// Split into channels, setting `DMARMWDIS` on the way: DMA transfers
    /// wait out any CPU read-modify-write in flight instead of splitting it.
    /// A torn RMW is exactly the failure mode the crate's critical-section
    /// rules exist to prevent, and `DINT` alone does not pause DMA — this bit
    /// does. (Cost: a trigger is serviced at most a few cycles later.)
    fn split(self) -> Channels {
        // We own the whole peripheral by value here, so the typed PAC write
        // needs no critical section.
        self.dmactl4().write(|w| w.dmarmwdis().set_bit());
        Channels {
            ch0: Channel { _not_send: PhantomData },
            ch1: Channel { _not_send: PhantomData },
            ch2: Channel { _not_send: PhantomData },
        }
    }
}

impl<const N: u8> Channel<N> {
    /// This channel's register base (`DMAxCTL` address).
    const BASE: usize = CH0_BASE + CH_STRIDE * N as usize;

    /// Read this channel's `DMAxCTL`.
    #[inline]
    fn ctl(&self) -> u16 {
        unsafe { read_reg(Self::BASE + CTL) }
    }

    /// Transfers remaining in the armed block (`DMAxSZ`). Note the hardware
    /// *reloads* the initial count when a transfer completes, so this reads
    /// the original size — not 0 — once [`is_done`](Self::is_done) is true.
    pub fn remaining(&self) -> u16 {
        unsafe { read_reg(Self::BASE + SZ) }
    }

    /// Has the armed transfer run to completion? (`DMAIFG`, set when the
    /// transfer count reaches zero.) Cleared by
    /// [`clear_done`](Self::clear_done)/[`wait_done`](Self::wait_done), or by
    /// the `DMA`-vector ISR's [`read_iv`] when the interrupt is enabled.
    pub fn is_done(&self) -> bool {
        self.ctl() & DMAIFG != 0
    }

    /// Is the channel currently armed (`DMAEN`)? Hardware clears this on
    /// completion of a single/block transfer.
    pub fn is_armed(&self) -> bool {
        self.ctl() & DMAEN != 0
    }
}

#[cfg(feature = "critical-section")]
impl<const N: u8> Channel<N> {
    /// Program this channel's 5-bit trigger-select field. Channels 0 and 1
    /// share `DMACTL0` (and other channel owners may arm concurrently from an
    /// ISR), so the RMW runs under `critical_section::with` per the crate's
    /// shared-register rule.
    fn set_trigger(&mut self, trigger: TriggerSource) {
        let (reg, shift) = match N {
            0 => (DMACTL0, 0),
            1 => (DMACTL0, 8),
            _ => (DMACTL1, 0),
        };
        critical_section::with(|_| unsafe {
            let cur = read_reg(reg);
            write_reg(
                reg,
                (cur & !(TSEL_MASK << shift)) | ((trigger as u16) << shift),
            );
        });
    }

    /// Program source/destination addresses and the transfer count. The
    /// 20-bit SA/DA registers are written as two words, low then high; all
    /// addresses on this 64 KB part fit in the low word, so the high word
    /// just zeroes bits 19:16.
    unsafe fn set_addresses(&mut self, src: usize, dst: usize, count: u16) {
        write_reg(Self::BASE + SA, src as u16);
        write_reg(Self::BASE + SA + 2, 0);
        write_reg(Self::BASE + DA, dst as u16);
        write_reg(Self::BASE + DA + 2, 0);
        write_reg(Self::BASE + SZ, count);
    }

    /// The shared core of the safe blocking copies: a **block-mode,
    /// software-triggered** transfer. Setting `DMAREQ` halts the CPU while
    /// the whole block moves, so this returns with the copy complete — the
    /// borrow-checker-friendly property all the safe wrappers rely on.
    ///
    /// `width_bits` is 0 for word transfers or `DMASRCBYTE | DMADSTBYTE` for
    /// byte transfers; `count` is in items of that width and must be nonzero.
    ///
    /// # Safety
    ///
    /// `src`/`dst` must be valid for `count` reads/writes of the given width
    /// (starting points of the walks implied by the address modes), and `dst`
    /// must not overlap `src` in a way the walk order makes self-corrupting.
    unsafe fn block_transfer(
        &mut self,
        src: usize,
        src_mode: AddrMode,
        dst: usize,
        dst_mode: AddrMode,
        width_bits: u16,
        count: u16,
    ) {
        self.set_trigger(TriggerSource::DmaReq);
        self.set_addresses(src, dst, count);
        let ctl = DMADT_BLOCK
            | (src_mode.bits() << DMASRCINCR_SHIFT)
            | (dst_mode.bits() << DMADSTINCR_SHIFT)
            | width_bits
            | DMAEN;
        // Whole-word write: also clears any stale DMAIFG/DMAIE state — this
        // channel is (re)dedicated to a fresh software-triggered block.
        write_reg(Self::BASE + CTL, ctl);
        // Source buffer writes must be visible before the transfer starts...
        barrier();
        write_reg(Self::BASE + CTL, ctl | DMAREQ);
        // The CPU was halted for the duration; DMAIFG is already up when this
        // executes. Poll anyway (cheap) rather than lean on that timing.
        while !self.is_done() {}
        self.clear_done();
        // ...and destination reads must not be hoisted above the completion.
        barrier();
    }

    /// Copy `src` into `dst` as a CPU-halting block transfer, returning the
    /// number of bytes moved (`min` of the lengths; 0-length is a no-op —
    /// `DMAxSZ = 0` means "no transfer" in silicon, so it never reaches the
    /// hardware).
    pub fn copy_bytes(&mut self, src: &[u8], dst: &mut [u8]) -> usize {
        let n = src.len().min(dst.len());
        if n > 0 {
            unsafe {
                self.block_transfer(
                    src.as_ptr() as usize,
                    AddrMode::Increment,
                    dst.as_mut_ptr() as usize,
                    AddrMode::Increment,
                    DMASRCBYTE | DMADSTBYTE,
                    n as u16,
                );
            }
        }
        n
    }

    /// [`copy_bytes`](Self::copy_bytes), word-wide: half the bus transactions
    /// for word-aligned data (the natural width — `&[u16]` guarantees the
    /// alignment the DMA's word accesses require). Returns words moved.
    pub fn copy_words(&mut self, src: &[u16], dst: &mut [u16]) -> usize {
        let n = src.len().min(dst.len());
        if n > 0 {
            unsafe {
                self.block_transfer(
                    src.as_ptr() as usize,
                    AddrMode::Increment,
                    dst.as_mut_ptr() as usize,
                    AddrMode::Increment,
                    0, // word-to-word
                    n as u16,
                );
            }
        }
        n
    }

    /// Copy `src` into `dst` **reversed** (`src[0]` -> `dst[n-1]`, …): the
    /// source walks forward while the destination walks *backward* — the
    /// decrement address mode doing something the increment modes can't.
    pub fn copy_bytes_reversed(&mut self, src: &[u8], dst: &mut [u8]) -> usize {
        let n = src.len().min(dst.len());
        if n > 0 {
            unsafe {
                self.block_transfer(
                    src.as_ptr() as usize,
                    AddrMode::Increment,
                    // Decrement mode starts at the *end* of the walk.
                    dst.as_mut_ptr().add(n - 1) as usize,
                    AddrMode::Decrement,
                    DMASRCBYTE | DMADSTBYTE,
                    n as u16,
                );
            }
        }
        n
    }

    /// Fill `dst` with `value` — the source address mode is `Fixed`, pointed
    /// at a stack byte that the halted-CPU block transfer reads `dst.len()`
    /// times. Returns bytes written.
    pub fn fill_bytes(&mut self, value: u8, dst: &mut [u8]) -> usize {
        let n = dst.len();
        if n > 0 {
            unsafe {
                self.block_transfer(
                    core::ptr::addr_of!(value) as usize,
                    AddrMode::Fixed,
                    dst.as_mut_ptr() as usize,
                    AddrMode::Increment,
                    DMASRCBYTE | DMADSTBYTE,
                    n as u16,
                );
            }
        }
        n
    }

    /// Arm this channel for **edge-triggered, single-transfer-mode byte
    /// moves**: one byte per rising edge of `trigger`, `count` bytes total,
    /// after which hardware clears `DMAEN` and raises `DMAIFG`
    /// ([`is_done`](Self::is_done)). This is the peripheral-pacing primitive
    /// under the safe DMA methods in [`serial`](crate::serial) and
    /// [`spi`](crate::spi).
    ///
    /// Returns immediately; the transfers happen as the trigger fires, the
    /// CPU stolen from for ~2 MCLK cycles per byte.
    ///
    /// # Safety
    ///
    /// - `src`/`dst` must stay valid — and, for the buffer side, unread and
    ///   unwritten by the CPU — until [`is_done`](Self::is_done) reports
    ///   completion or the channel is [`disarm`](Self::disarm)ed. The
    ///   compiler cannot see the DMA's accesses.
    /// - `count` must be nonzero (`DMAxSZ = 0` arms a channel that will never
    ///   fire *or* complete).
    /// - Edge sensitivity is the caller's problem: if the trigger flag is
    ///   already high when this returns, that occurrence presents no edge and
    ///   the channel waits for the *next* one (see the module docs for the
    ///   TXIFG priming / RXIFG draining patterns).
    pub unsafe fn arm_single_bytes(
        &mut self,
        trigger: TriggerSource,
        src: *const u8,
        src_mode: AddrMode,
        dst: *mut u8,
        dst_mode: AddrMode,
        count: u16,
    ) {
        self.arm_single(
            trigger,
            src as usize,
            src_mode,
            dst as usize,
            dst_mode,
            DMASRCBYTE | DMADSTBYTE,
            count,
        );
    }

    /// [`arm_single_bytes`](Self::arm_single_bytes), word-wide: one 16-bit
    /// item per trigger edge, `count` **words** total. The natural width for
    /// 16-bit peripheral data registers — e.g. draining `ADC12MEM0` results,
    /// where a byte-wide read would truncate the conversion. `&[u16]`-derived
    /// pointers guarantee the alignment the word accesses require.
    ///
    /// # Safety
    ///
    /// Same contract as [`arm_single_bytes`](Self::arm_single_bytes), with
    /// `count` in words and both pointers word-aligned.
    pub unsafe fn arm_single_words(
        &mut self,
        trigger: TriggerSource,
        src: *const u16,
        src_mode: AddrMode,
        dst: *mut u16,
        dst_mode: AddrMode,
        count: u16,
    ) {
        self.arm_single(
            trigger,
            src as usize,
            src_mode,
            dst as usize,
            dst_mode,
            0, // word-to-word
            count,
        );
    }

    /// Shared core of the single-transfer-mode arming wrappers above.
    unsafe fn arm_single(
        &mut self,
        trigger: TriggerSource,
        src: usize,
        src_mode: AddrMode,
        dst: usize,
        dst_mode: AddrMode,
        width_bits: u16,
        count: u16,
    ) {
        self.set_trigger(trigger);
        self.set_addresses(src, dst, count);
        let ctl = DMADT_SINGLE
            | (src_mode.bits() << DMASRCINCR_SHIFT)
            | (dst_mode.bits() << DMADSTINCR_SHIFT)
            | width_bits
            | DMAEN;
        // Buffer contents must be committed before the peripheral can pace
        // reads out of it.
        barrier();
        write_reg(Self::BASE + CTL, ctl);
    }

    /// Consume a **stale-high trigger latch**: arm one *level-sensitive*
    /// word transfer (`src` → `dst`, both fixed) on `trigger`, give it a few
    /// cycles to fire, and disarm. Returns whether it fired.
    ///
    /// Exists because at least one trigger source is not a flag but a
    /// **latch**: the ADC12 trigger (hardware-observed on this device, and a
    /// known undocumented erratum — TI E2E #401588) is set by a conversion
    /// completing while `ADC12IE0` is clear and reset **only by a DMA
    /// transfer the ADC trigger itself fires**. CPU reads of `MEM0` do not
    /// touch it, so one unserviced completion parks the line high and an
    /// edge-sensitive channel never sees another rising edge — on *any*
    /// channel, surviving even an `ADC12ON` power-cycle. A level-sensitive
    /// transfer fires on the stuck-high level immediately, and being a real
    /// ADC-triggered DMA service, resets the latch.
    ///
    /// (SLAU367P sanctions `DMALEVEL` only for the external `DMAE0` trigger;
    /// this one-word scrub is deliberately minimal, hardware-verified
    /// 2026-07-05, and the channel is back to edge-sensitive before it is
    /// used for anything else.)
    ///
    /// # Safety
    ///
    /// `src` must be valid to read and `dst` valid to write, one word each,
    /// for the duration of the call (it is blocking-bounded, so locals are
    /// fine). Reading `src` must be acceptable — for the ADC scrub the read
    /// of `MEM0` also clears `ADC12IFG0`, which the caller must expect.
    pub unsafe fn consume_stale_trigger_word(
        &mut self,
        trigger: TriggerSource,
        src: *const u16,
        dst: *mut u16,
    ) -> bool {
        self.set_trigger(trigger);
        self.set_addresses(src as usize, dst as usize, 1);
        write_reg(Self::BASE + CTL, DMADT_SINGLE | DMALEVEL | DMAEN);
        // A high latch is serviced within a couple of MCLK cycles; a short
        // bounded spin distinguishes "consumed" from "was never pending".
        let mut spins = 0u16;
        while self.ctl() & DMAIFG == 0 && spins < 100 {
            spins += 1;
        }
        let fired = self.ctl() & DMAIFG != 0;
        // Disarm and clear completion; the whole-word write also drops
        // DMALEVEL, returning the channel to its edge-sensitive default.
        critical_section::with(|_| unsafe {
            write_reg(Self::BASE + CTL, 0);
        });
        barrier();
        fired
    }

    /// Software trigger: with the channel armed on
    /// [`TriggerSource::DmaReq`], each call moves one item in single-transfer
    /// mode (or the whole block in block mode). The bit self-clears when the
    /// transfer starts. This is how a software-paced channel — e.g. one whose
    /// completion interrupt is under test, where the self-clearing blocking
    /// APIs would race the ISR for the flag — is stepped manually.
    pub fn request(&mut self) {
        critical_section::with(|_| unsafe {
            write_reg(Self::BASE + CTL, self.ctl() | DMAREQ);
        });
    }

    /// Disarm the channel (clear `DMAEN`); in-flight state (`DMAxSZ` etc.) is
    /// abandoned. RMW under critical section — see [`clear_done`](Self::clear_done).
    pub fn disarm(&mut self) {
        critical_section::with(|_| unsafe {
            write_reg(Self::BASE + CTL, self.ctl() & !DMAEN);
        });
    }

    /// Clear the completion flag (`DMAIFG`).
    ///
    /// Under critical section: a `DMA`-vector ISR's [`read_iv`] clears
    /// `DMAIFG` bits *in silicon, mid-RMW* — an unguarded read-modify-write
    /// here could write a stale 1 back and resurrect a flag the ISR already
    /// consumed (on any channel with `DMAIE` up, not just this one, since the
    /// RMW is of this register but the hazard window is global).
    pub fn clear_done(&mut self) {
        critical_section::with(|_| unsafe {
            write_reg(Self::BASE + CTL, self.ctl() & !DMAIFG);
        });
    }

    /// Busy-wait until the armed transfer completes, then clear the flag.
    /// The blocking companion to [`arm_single_bytes`](Self::arm_single_bytes):
    /// once this returns, the DMA is out of the buffers and the borrows can
    /// safely end.
    pub fn wait_done(&mut self) {
        while !self.is_done() {}
        self.clear_done();
        barrier();
    }

    /// Set `DMAIE`: completion (`DMAIFG`) fires the shared `DMA` vector once
    /// GIE is up. Demux and consume with [`read_iv`] in the ISR. The blocking
    /// APIs on this channel poll-and-clear the same flag, so don't mix them
    /// with the interrupt on one channel.
    pub fn enable_done_interrupt(&mut self) {
        critical_section::with(|_| unsafe {
            write_reg(Self::BASE + CTL, self.ctl() | DMAIE);
        });
    }

    /// Clear `DMAIE`, returning the channel to polled completion.
    pub fn disable_done_interrupt(&mut self) {
        critical_section::with(|_| unsafe {
            write_reg(Self::BASE + CTL, self.ctl() & !DMAIE);
        });
    }
}

// ---------------------------------------------------------------------------
// ISR-side helpers
// ---------------------------------------------------------------------------

/// ISR-side: read `DMAIV` — 0x00 = nothing pending, 0x02/0x04/0x06 = channel
/// 0/1/2 transfer complete. The read atomically clears the reported channel's
/// `DMAIFG` in silicon (highest-priority channel first when several are
/// pending), immune to the lost-flag RMW hazard — consume each value once.
///
/// Free function backed by the fixed register address (the driver-struct
/// convention for ISR-facing bits: the `Channel` owners live in thread mode).
pub fn read_iv() -> u16 {
    unsafe { read_reg(DMAIV) }
}
