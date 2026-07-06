//! UART (asynchronous serial) driver for the MSP430FR5969 eUSCI_A modules.
//!
//! This implements the embedded-hal serial traits on top of the device's two
//! eUSCI_A peripherals (`eUSCI_A0`, `eUSCI_A1`) running in UART mode:
//!
//! - [`embedded_hal_nb::serial::Read`] / [`embedded_hal_nb::serial::Write`] —
//!   the non-blocking, word-at-a-time UART abstraction.
//! - [`embedded_io::Read`] / [`embedded_io::Write`] plus
//!   [`embedded_io::ReadReady`] / [`embedded_io::WriteReady`] — blocking,
//!   byte-stream I/O.
//! - [`core::fmt::Write`] on the transmit side, so `write!`/`writeln!` work.
//!
//! # Register access
//!
//! The PAC's UART-mode register block does not expose the interrupt-flag
//! (`UCAxIFG`) or interrupt-enable (`UCAxIE`) registers, which are required to
//! poll TX/RX readiness. We therefore drive the peripheral through raw volatile
//! accesses at its documented base address — the same approach taken by
//! [`crate::gpio`]. The PAC peripheral is still consumed by value so that
//! ownership of the eUSCI_A module is tracked by the type system.
//!
//! # Pin muxing
//!
//! Each eUSCI_A has a fixed pair of port pins for its TXD/RXD signals. The
//! constructor configures the relevant `PxSEL1`/`PxSEL0` bits automatically:
//!
//! | Module    | TXD          | RXD          |
//! |-----------|--------------|--------------|
//! | eUSCI_A0  | P2.0 (SEL=10)| P2.1 (SEL=10)|
//! | eUSCI_A1  | P2.5 (SEL=10)| P2.6 (SEL=10)|
//!
//! # Baud rate
//!
//! [`Config`] takes the BRCLK source frequency and the desired baud rate; the
//! prescaler/modulator registers (`UCBRx`, `UCBRFx`, `UCBRSx`, `UCOS16`) are
//! computed per the procedure in the eUSCI_A UART chapter (SLAU367P §30.3.10),
//! including the `UCBRSx` fractional-modulation lookup table (Table 30-4).

use core::marker::PhantomData;

use embedded_hal_nb::nb;

use crate::baud::compute_baud;

// ---------------------------------------------------------------------------
// Register layout (offsets from the eUSCI_A base address)
// ---------------------------------------------------------------------------

const CTLW0: usize = 0x00; // Control word 0 (UCSWRST, clock select, frame format)
const BRW: usize = 0x06; // Baud-rate prescaler (UCBRx)
const MCTLW: usize = 0x08; // Modulation control (UCBRSx, UCBRFx, UCOS16)
const STATW: usize = 0x0A; // Status (error flags, UCBUSY)
const RXBUF: usize = 0x0C; // Receive buffer
const TXBUF: usize = 0x0E; // Transmit buffer
// Referenced only from the `critical-section`-gated RX-interrupt methods —
// without that feature it is (correctly) unreferenced, not dead.
#[cfg_attr(not(feature = "critical-section"), allow(dead_code))]
const IE: usize = 0x1A; // Interrupt enables (UCTXIE, UCRXIE)
const IFG: usize = 0x1C; // Interrupt flags (UCTXIFG, UCRXIFG)
const IV: usize = 0x1E; // Interrupt vector (read clears the reported source)

// CTLW0 bit fields
const UCSWRST: u16 = 1 << 0; // Software reset (hold module in reset while = 1)
const UCSSEL_SHIFT: u16 = 6; // Clock source select, bits 7-6
const UCSPB: u16 = 1 << 11; // 0 = one stop bit, 1 = two stop bits
const UC7BIT: u16 = 1 << 12; // 0 = 8-bit data, 1 = 7-bit data
const UCPAR: u16 = 1 << 14; // 0 = odd, 1 = even (only when UCPEN = 1)
const UCPEN: u16 = 1 << 15; // Parity enable

// MCTLW bit fields
const UCOS16: u16 = 1 << 0; // Oversampling mode enable

// STATW bit fields
const UCBUSY: u16 = 1 << 0; // Transmit/receive in progress
const UCRXERR: u16 = 1 << 2; // One or more receive errors occurred
const UCBRK: u16 = 1 << 3; // Break detected
const UCPE: u16 = 1 << 4; // Parity error
const UCOE: u16 = 1 << 5; // Overrun error
const UCFE: u16 = 1 << 6; // Framing error

// IE bit fields
#[cfg_attr(not(feature = "critical-section"), allow(dead_code))]
const UCRXIE: u16 = 1 << 0; // Receive interrupt enable

// IFG bit fields
const UCRXIFG: u16 = 1 << 0; // Receive buffer full
const UCTXIFG: u16 = 1 << 1; // Transmit buffer empty

// Port 2 function-select registers (see crate::gpio for the full map).
const P2SEL0: usize = 0x020B;
const P2SEL1: usize = 0x020D;

#[inline(always)]
unsafe fn read_reg(addr: usize) -> u16 {
    (addr as *const u16).read_volatile()
}

#[inline(always)]
unsafe fn write_reg(addr: usize, val: u16) {
    (addr as *mut u16).write_volatile(val);
}

#[inline(always)]
unsafe fn set_bits_u8(addr: usize, mask: u8) {
    let p = addr as *mut u8;
    p.write_volatile(p.read_volatile() | mask);
}

#[inline(always)]
unsafe fn clear_bits_u8(addr: usize, mask: u8) {
    let p = addr as *mut u8;
    p.write_volatile(p.read_volatile() & !mask);
}

// ---------------------------------------------------------------------------
// eUSCI_A instance markers
// ---------------------------------------------------------------------------

mod sealed {
    pub trait Sealed {}
}

/// Describes a concrete eUSCI_A instance: its register base address and the
/// `PxSEL` bits that mux its TXD/RXD pins to the peripheral.
pub trait Instance: sealed::Sealed {
    /// Absolute base address of the eUSCI_A register block.
    const BASE: usize;
    /// Mask of the TXD+RXD bits within the P2 SEL registers.
    const PIN_MASK: u8;
    /// This instance's `UCAxRXIFG` in the device DMA trigger table.
    const DMA_RX_TRIGGER: crate::dma::TriggerSource;
    /// This instance's `UCAxTXIFG` in the device DMA trigger table.
    const DMA_TX_TRIGGER: crate::dma::TriggerSource;
}

/// Marker for the eUSCI_A0 module (UCA0TXD = P2.0, UCA0RXD = P2.1).
pub struct UsciA0;
/// Marker for the eUSCI_A1 module (UCA1TXD = P2.5, UCA1RXD = P2.6).
pub struct UsciA1;

impl sealed::Sealed for UsciA0 {}
impl Instance for UsciA0 {
    const BASE: usize = 0x05C0;
    const PIN_MASK: u8 = (1 << 0) | (1 << 1); // P2.0, P2.1
    const DMA_RX_TRIGGER: crate::dma::TriggerSource = crate::dma::TriggerSource::UcA0Rx;
    const DMA_TX_TRIGGER: crate::dma::TriggerSource = crate::dma::TriggerSource::UcA0Tx;
}

impl sealed::Sealed for UsciA1 {}
impl Instance for UsciA1 {
    const BASE: usize = 0x05E0;
    const PIN_MASK: u8 = (1 << 5) | (1 << 6); // P2.5, P2.6
    const DMA_RX_TRIGGER: crate::dma::TriggerSource = crate::dma::TriggerSource::UcA1Rx;
    const DMA_TX_TRIGGER: crate::dma::TriggerSource = crate::dma::TriggerSource::UcA1Tx;
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// BRCLK source clock for the baud-rate generator (UCSSELx field).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// ACLK (UCSSELx = 01).
    Aclk = 1,
    /// SMCLK (UCSSELx = 10).
    Smclk = 2,
}

/// Parity configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Parity {
    /// No parity bit.
    None,
    /// Even parity.
    Even,
    /// Odd parity.
    Odd,
}

/// Number of stop bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StopBits {
    /// One stop bit.
    One,
    /// Two stop bits.
    Two,
}

/// Character length.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DataBits {
    /// 7-bit characters.
    Seven,
    /// 8-bit characters.
    Eight,
}

/// UART configuration: clock, baud rate, and frame format.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Which clock feeds the baud-rate generator.
    pub clock_source: ClockSource,
    /// Frequency of that clock (BRCLK), in Hz.
    pub clock_freq: u32,
    /// Desired baud rate.
    pub baud: u32,
    /// Parity setting.
    pub parity: Parity,
    /// Number of stop bits.
    pub stop_bits: StopBits,
    /// Character length.
    pub data_bits: DataBits,
}

impl Config {
    /// Start from a BRCLK frequency, defaulting to 9600 baud, 8N1, SMCLK.
    pub fn new(clock_freq: u32) -> Self {
        Config {
            clock_source: ClockSource::Smclk,
            clock_freq,
            baud: 9600,
            parity: Parity::None,
            stop_bits: StopBits::One,
            data_bits: DataBits::Eight,
        }
    }

    /// Set the baud rate (builder style).
    pub fn baud(mut self, baud: u32) -> Self {
        self.baud = baud;
        self
    }

    /// Select the clock source (builder style).
    pub fn clock_source(mut self, src: ClockSource) -> Self {
        self.clock_source = src;
        self
    }

    /// Set the parity (builder style).
    pub fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    /// Set the number of stop bits (builder style).
    pub fn stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.stop_bits = stop_bits;
        self
    }

    /// Set the character length (builder style).
    pub fn data_bits(mut self, data_bits: DataBits) -> Self {
        self.data_bits = data_bits;
        self
    }
}

impl Default for Config {
    /// 1 MHz BRCLK (the reset SMCLK on this device: 8 MHz DCO / 8), 9600 8N1.
    fn default() -> Self {
        Config::new(1_000_000)
    }
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors that can occur during a UART receive.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// A character was received before the previous one was read (UCOE).
    Overrun,
    /// Parity check failed (UCPE).
    Parity,
    /// Stop bit was not detected (UCFE).
    Framing,
    /// A break condition was detected (UCBRK).
    Break,
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        use embedded_hal_nb::serial::ErrorKind;
        match self {
            Error::Overrun => ErrorKind::Overrun,
            Error::Parity => ErrorKind::Parity,
            Error::Framing => ErrorKind::FrameFormat,
            Error::Break => ErrorKind::Other,
        }
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// A configured UART. Implements the transmit and receive halves of the
/// embedded-hal serial traits; call [`Serial::split`] to separate them.
pub struct Serial<USCI> {
    _usci: PhantomData<USCI>,
}

/// Transmit half of a [`Serial`].
pub struct Tx<USCI> {
    _usci: PhantomData<USCI>,
}

/// Receive half of a [`Serial`].
pub struct Rx<USCI> {
    _usci: PhantomData<USCI>,
}

impl<USCI: Instance> Serial<USCI> {
    /// Configure the eUSCI_A module for UART operation and return the driver.
    ///
    /// Follows the initialization sequence from SLAU367P §30.3.1: hold the
    /// module in reset (`UCSWRST = 1`), program all registers, mux the pins,
    /// then release reset.
    fn init(config: Config) -> Self {
        let base = USCI::BASE;
        let regs = compute_baud(config.clock_freq, config.baud);

        // Build the CTLW0 value (frame format + clock select), keeping the
        // module in reset for now.
        //
        // CTLW0 is the primary 16-bit control word register for the eUSCI serial communication peripherals.
        // The W0 suffix means "Word 0" — a 16-bit (word) access to control register 0. On older MSP430
        // families this was split into two separate 8-bit registers (CTL0 and CTL1); the FR5969's eUSCI combines them into one word.
        let mut ctlw0 = UCSWRST | ((config.clock_source as u16) << UCSSEL_SHIFT);
        match config.parity {
            Parity::None => {}
            Parity::Even => ctlw0 |= UCPEN | UCPAR,
            Parity::Odd => ctlw0 |= UCPEN,
        }
        if let StopBits::Two = config.stop_bits {
            ctlw0 |= UCSPB;
        }
        if let DataBits::Seven = config.data_bits {
            ctlw0 |= UC7BIT;
        }
        // UCSYNC = 0 and UCMODEx = 00 (plain UART) fall out of the zeroed bits.

        let mut mctlw = ((regs.ucbrs as u16) << 8) | ((regs.ucbrf as u16) << 4);
        if regs.oversampling {
            mctlw |= UCOS16;
        }

        unsafe {
            // 1. Hold in reset (also begins programming the frame format).
            write_reg(base + CTLW0, ctlw0);
            // 2. Program baud-rate generator while UCSWRST = 1.
            write_reg(base + BRW, regs.ucbr);
            write_reg(base + MCTLW, mctlw);
            // 3. Mux the TXD/RXD pins to the eUSCI_A function (SEL1:SEL0 = 10).
            set_bits_u8(P2SEL1, USCI::PIN_MASK);
            clear_bits_u8(P2SEL0, USCI::PIN_MASK);
            // 4. Release the module for operation.
            write_reg(base + CTLW0, ctlw0 & !UCSWRST);
        }

        Serial {
            _usci: PhantomData,
        }
    }

    /// Split the driver into independent transmit and receive halves.
    pub fn split(self) -> (Tx<USCI>, Rx<USCI>) {
        (
            Tx {
                _usci: PhantomData,
            },
            Rx {
                _usci: PhantomData,
            },
        )
    }
}

// ---------------------------------------------------------------------------
// RX interrupt
// ---------------------------------------------------------------------------
//
// With `UCRXIE` set, a received byte fires the `USCI_A0`/`USCI_A1` vector.
// The division of labor follows the project's `OVERFLOWS` precedent: the HAL
// owns no hidden buffer — the application's ISR calls [`isr_read_byte`] and
// pushes into its own queue (see `hal::rx_queue::RxQueue`), and thread-mode
// code drains that queue. While the interrupt is enabled the ISR is the
// consumer of RXBUF, so `Rx::read`/`try_read_byte` in thread mode will simply
// see `WouldBlock` — the polling path still works whenever the interrupt is
// left off.

#[cfg(feature = "critical-section")]
impl<USCI: Instance> Rx<USCI> {
    /// Set `UCRXIE`: from now on (once GIE is up) each received byte fires
    /// the eUSCI's shared vector, and the ISR — not thread-mode `read` — must
    /// consume `RXBUF` via [`isr_read_byte`].
    ///
    /// `UCAxIE` is shared with the (future) `UCTXIE` bit, so the RMW runs
    /// under `critical_section::with` per the shared-IE-register rule.
    pub fn enable_rx_interrupt(&mut self) {
        critical_section::with(|_| unsafe {
            let addr = USCI::BASE + IE;
            write_reg(addr, read_reg(addr) | UCRXIE);
        });
    }

    /// Clear `UCRXIE`, returning RX to pure polling. A byte already latched
    /// in `RXBUF` stays readable via the normal `read` path.
    pub fn disable_rx_interrupt(&mut self) {
        critical_section::with(|_| unsafe {
            let addr = USCI::BASE + IE;
            write_reg(addr, read_reg(addr) & !UCRXIE);
        });
    }
}

// ---------------------------------------------------------------------------
// DMA pacing
// ---------------------------------------------------------------------------
//
// `UCAxTXIFG`/`UCAxRXIFG` are DMA trigger sources, so a channel in
// single-transfer mode can feed TXBUF exactly as fast as the wire drains it
// (or drain RXBUF as bytes land) with no per-byte CPU work. Triggers are
// edge-sensitive on this part (see `crate::dma`), which shapes both methods:
//
// - TX: `UCTXIFG` idles *high*, so arming a channel on it presents no edge.
//   `write_all_dma` arms the channel on `buf[1..]` and writes `buf[0]`
//   manually — TXIFG's 0->1 hop when that byte moves to the shift register
//   is the first edge the channel sees, and each subsequent hop pulls the
//   next byte.
// - RX: `UCRXIFG` idles low, but a byte *already latched* when the channel
//   is armed has spent its edge. `read_exact_dma` drains such bytes by CPU
//   before arming, and re-checks after arming for the one that can slip in
//   between (its edge fires before DMAEN is up, so it would stall the
//   channel forever).

#[cfg(feature = "critical-section")]
impl<USCI: Instance> Tx<USCI> {
    /// Transmit `buf` paced by DMA: the CPU sets up the channel, sends the
    /// first byte, and then blocks while hardware moves the rest. Returns
    /// once the final byte has been *accepted* into TXBUF (same contract as
    /// the polled `write_all`) — `flush` still tells when the wire is idle.
    ///
    /// Blocking keeps this safe: the channel is out of `buf` before the
    /// borrow ends. One- and zero-byte writes skip the DMA entirely (a
    /// one-byte "rest" would be a zero-length arm, which silicon treats as
    /// "never fires").
    pub fn write_all_dma<const N: u8>(
        &mut self,
        ch: &mut crate::dma::Channel<N>,
        buf: &[u8],
    ) -> Result<(), Error> {
        let Some((&first, rest)) = buf.split_first() else {
            return Ok(());
        };
        if rest.is_empty() {
            return nb::block!(try_write_byte::<USCI>(first));
        }
        // TXBUF must be free BEFORE the channel is armed. If a previous
        // byte were still queued, its hand-off edge would fire the fresh
        // channel — and from then on the CPU primer below races the DMA for
        // every TXIFG edge, transposing and dropping bytes (observed on
        // hardware as exactly that, on every burst after the first). With
        // TXBUF verifiably empty, no edge can occur between arming and the
        // priming write: an empty TXBUF has nothing left to hand off.
        while !tx_ready::<USCI>() {}
        unsafe {
            ch.arm_single_bytes(
                USCI::DMA_TX_TRIGGER,
                rest.as_ptr(),
                crate::dma::AddrMode::Increment,
                (USCI::BASE + TXBUF) as *mut u8,
                crate::dma::AddrMode::Fixed,
                rest.len() as u16,
            );
            // Prime the pump — TXIFG is still up (checked above, and nothing
            // drains an empty TXBUF), so write directly. The byte's move into
            // the shift register re-asserts TXIFG: the first edge the channel
            // sees, and the only writer from here on is the DMA.
            write_reg(USCI::BASE + TXBUF, first as u16);
        }
        ch.wait_done();
        Ok(())
    }
}

#[cfg(feature = "critical-section")]
impl<USCI: Instance> Rx<USCI> {
    /// Arm `ch` to DMA-receive exactly `buf.len()` bytes into `buf`, then
    /// return immediately — the non-blocking sibling of
    /// [`read_exact_dma`](Self::read_exact_dma), for the *arm-before-the-peer-
    /// transmits* pattern: armed first, even the payload's opening byte meets
    /// the channel's trigger edge and lands by DMA, no matter how fast the
    /// bytes stream in. (A CPU poll loop cannot make that guarantee: at 9600
    /// baud, bytes 1 ms apart overrun RXBUF between polls.)
    ///
    /// Anything stale in RXBUF is drained and **discarded** first (its
    /// trigger edge is spent, and under this pattern nothing real has been
    /// sent yet). Poll [`Channel::is_done`](crate::dma::Channel::is_done) /
    /// [`wait_done`](crate::dma::Channel::wait_done) for completion, or
    /// [`disarm`](crate::dma::Channel::disarm) to abandon (e.g. on timeout).
    ///
    /// # Safety
    ///
    /// `buf` must stay valid — and untouched by the CPU — until the channel
    /// reports done or is disarmed; the compiler cannot see the DMA's writes.
    /// `buf` must be non-empty (a zero-length arm never fires or completes).
    pub unsafe fn start_read_dma<const N: u8>(
        &mut self,
        ch: &mut crate::dma::Channel<N>,
        buf: &mut [u8],
    ) {
        while rx_ready::<USCI>() {
            let _ = try_read_byte::<USCI>();
        }
        ch.arm_single_bytes(
            USCI::DMA_RX_TRIGGER,
            (USCI::BASE + RXBUF) as *const u8,
            crate::dma::AddrMode::Fixed,
            buf.as_mut_ptr(),
            crate::dma::AddrMode::Increment,
            buf.len() as u16,
        );
    }

    /// Receive exactly `buf.len()` bytes, DMA-paced, blocking until the
    /// buffer is full. Bytes already latched in RXBUF when this is called
    /// count toward the total (they are consumed by CPU — their trigger edge
    /// is already spent, see the section comment).
    ///
    /// **Error visibility is reduced on the DMA path**: the DMA's RXBUF
    /// reads clear the STATW error flags without anyone looking at them, so
    /// framing/parity/overrun in DMA-moved bytes goes unreported. Only the
    /// CPU-consumed stragglers get the full error decode. Use the polled or
    /// interrupt path when per-byte error accounting matters.
    pub fn read_exact_dma<const N: u8>(
        &mut self,
        ch: &mut crate::dma::Channel<N>,
        buf: &mut [u8],
    ) -> Result<(), Error> {
        let mut filled = 0;
        while filled < buf.len() {
            // Consume anything already latched: no future edge belongs to it.
            while filled < buf.len() && rx_ready::<USCI>() {
                match try_read_byte::<USCI>() {
                    Ok(b) => {
                        buf[filled] = b;
                        filled += 1;
                    }
                    Err(nb::Error::WouldBlock) => break,
                    Err(nb::Error::Other(e)) => return Err(e),
                }
            }
            if filled == buf.len() {
                break;
            }
            let count = (buf.len() - filled) as u16;
            unsafe {
                ch.arm_single_bytes(
                    USCI::DMA_RX_TRIGGER,
                    (USCI::BASE + RXBUF) as *const u8,
                    crate::dma::AddrMode::Fixed,
                    buf.as_mut_ptr().add(filled),
                    crate::dma::AddrMode::Increment,
                    count,
                );
            }
            // A byte that landed after the drain but before DMAEN went live
            // latched RXIFG with an edge the channel never saw — it would
            // stall the transfer forever. `remaining() == count` distinguishes
            // that stale byte from one the channel is already servicing (the
            // DMA wins the bus within a couple of cycles, before these reads).
            if rx_ready::<USCI>() && ch.remaining() == count {
                ch.disarm();
                continue; // go around: drain it by CPU, re-arm for the rest
            }
            ch.wait_done();
            filled = buf.len();
        }
        Ok(())
    }
}

/// A transmit half bonded to a DMA channel: [`embedded_io::Write`] whose
/// `write` moves the whole buffer per [`Tx::write_all_dma`]. The channel is
/// dedicated while bonded; [`release`](Self::release) gives both halves back.
#[cfg(feature = "critical-section")]
pub struct DmaTx<USCI, const N: u8> {
    tx: Tx<USCI>,
    ch: crate::dma::Channel<N>,
}

#[cfg(feature = "critical-section")]
impl<USCI: Instance> Tx<USCI> {
    /// Bond this transmit half to a DMA channel, yielding a [`DmaTx`].
    pub fn with_dma<const N: u8>(self, ch: crate::dma::Channel<N>) -> DmaTx<USCI, N> {
        DmaTx { tx: self, ch }
    }
}

#[cfg(feature = "critical-section")]
impl<USCI: Instance, const N: u8> DmaTx<USCI, N> {
    /// Take the transmit half and the DMA channel back.
    pub fn release(self) -> (Tx<USCI>, crate::dma::Channel<N>) {
        (self.tx, self.ch)
    }
}

#[cfg(feature = "critical-section")]
impl<USCI: Instance, const N: u8> embedded_io::ErrorType for DmaTx<USCI, N> {
    type Error = Error;
}

#[cfg(feature = "critical-section")]
impl<USCI: Instance, const N: u8> embedded_io::Write for DmaTx<USCI, N> {
    /// Never a short write: DMA pacing has no cheaper stopping point, so the
    /// whole buffer goes out (blocking) and `write_all` degenerates to one
    /// call.
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write_all_dma(&mut self.ch, buf)?;
        Ok(buf.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(try_flush::<USCI>())
    }
}

/// ISR-side: read `UCAxIV` — 0 = nothing, 0x02 = RX (`UCRXIFG`), 0x04 = TX
/// (`UCTXIFG`); reading clears the reported flag *except* RX, where reading
/// `RXBUF` (i.e. [`isr_read_byte`]) is what clears it.
///
/// A single-source handler (only `UCRXIE` enabled) can skip this and go
/// straight to [`isr_read_byte`]; it exists for when TX/error sources join.
pub fn read_iv<USCI: Instance>() -> u16 {
    unsafe { read_reg(USCI::BASE + IV) }
}

/// ISR-side: consume one received byte — status checked *before* `RXBUF`
/// (the `RXBUF` read clears the error flags along with `UCRXIFG`), same
/// decode as the thread-mode read path. `Err(WouldBlock)` on a spurious call
/// with nothing latched; `Err(Other)` reports a framing/parity/overrun/break
/// error (the corrupt byte is consumed to clear the flags).
pub fn isr_read_byte<USCI: Instance>() -> nb::Result<u8, Error> {
    try_read_byte::<USCI>()
}

/// Extension trait to turn a PAC eUSCI_A UART peripheral into a [`Serial`].
pub trait SerialExt {
    /// The eUSCI_A instance marker for this peripheral.
    type Instance: Instance;

    /// Consume the PAC peripheral and configure it as a UART.
    fn into_uart(self, config: Config) -> Serial<Self::Instance>;
}

impl SerialExt for pac::UsciA0UartMode {
    type Instance = UsciA0;
    fn into_uart(self, config: Config) -> Serial<UsciA0> {
        Serial::<UsciA0>::init(config)
    }
}

impl SerialExt for pac::UsciA1UartMode {
    type Instance = UsciA1;
    fn into_uart(self, config: Config) -> Serial<UsciA1> {
        Serial::<UsciA1>::init(config)
    }
}

// ---------------------------------------------------------------------------
// Low-level, instance-generic primitives shared by all the trait impls
// ---------------------------------------------------------------------------

/// Try to push one byte into the transmit buffer. `Err(WouldBlock)` if the TX
/// buffer is not yet empty.
fn try_write_byte<USCI: Instance>(byte: u8) -> nb::Result<(), Error> {
    let base = USCI::BASE;
    if unsafe { read_reg(base + IFG) } & UCTXIFG == 0 {
        return Err(nb::Error::WouldBlock);
    }
    unsafe { write_reg(base + TXBUF, byte as u16) };
    Ok(())
}

/// `Err(WouldBlock)` until the transmit shift register has fully drained.
fn try_flush<USCI: Instance>() -> nb::Result<(), Error> {
    let base = USCI::BASE;
    if unsafe { read_reg(base + STATW) } & UCBUSY != 0 {
        return Err(nb::Error::WouldBlock);
    }
    Ok(())
}

/// Try to read one received byte. `Err(WouldBlock)` if none is pending; a
/// receive error is reported as `Err(Error)` (and the byte is consumed to clear
/// the flags).
fn try_read_byte<USCI: Instance>() -> nb::Result<u8, Error> {
    let base = USCI::BASE;
    if unsafe { read_reg(base + IFG) } & UCRXIFG == 0 {
        return Err(nb::Error::WouldBlock);
    }
    // Read status before the buffer: reading RXBUF clears the error flags.
    let status = unsafe { read_reg(base + STATW) };
    let byte = unsafe { read_reg(base + RXBUF) } as u8;
    if status & UCRXERR != 0 {
        // Priority roughly follows severity; overrun is the most data-losing.
        let err = if status & UCOE != 0 {
            Error::Overrun
        } else if status & UCFE != 0 {
            Error::Framing
        } else if status & UCPE != 0 {
            Error::Parity
        } else if status & UCBRK != 0 {
            Error::Break
        } else {
            Error::Framing
        };
        return Err(nb::Error::Other(err));
    }
    Ok(byte)
}

#[inline]
fn tx_ready<USCI: Instance>() -> bool {
    (unsafe { read_reg(USCI::BASE + IFG) }) & UCTXIFG != 0
}

#[inline]
fn rx_ready<USCI: Instance>() -> bool {
    (unsafe { read_reg(USCI::BASE + IFG) }) & UCRXIFG != 0
}

// ---------------------------------------------------------------------------
// embedded-hal-nb: non-blocking serial traits
// ---------------------------------------------------------------------------

impl<USCI: Instance> embedded_hal_nb::serial::ErrorType for Serial<USCI> {
    type Error = Error;
}
impl<USCI: Instance> embedded_hal_nb::serial::ErrorType for Tx<USCI> {
    type Error = Error;
}
impl<USCI: Instance> embedded_hal_nb::serial::ErrorType for Rx<USCI> {
    type Error = Error;
}

impl<USCI: Instance> embedded_hal_nb::serial::Write<u8> for Serial<USCI> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        try_write_byte::<USCI>(word)
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        try_flush::<USCI>()
    }
}

impl<USCI: Instance> embedded_hal_nb::serial::Write<u8> for Tx<USCI> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        try_write_byte::<USCI>(word)
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        try_flush::<USCI>()
    }
}

impl<USCI: Instance> embedded_hal_nb::serial::Read<u8> for Serial<USCI> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        try_read_byte::<USCI>()
    }
}

impl<USCI: Instance> embedded_hal_nb::serial::Read<u8> for Rx<USCI> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        try_read_byte::<USCI>()
    }
}

// ---------------------------------------------------------------------------
// embedded-io: blocking byte-stream traits
// ---------------------------------------------------------------------------

impl<USCI: Instance> embedded_io::ErrorType for Serial<USCI> {
    type Error = Error;
}
impl<USCI: Instance> embedded_io::ErrorType for Tx<USCI> {
    type Error = Error;
}
impl<USCI: Instance> embedded_io::ErrorType for Rx<USCI> {
    type Error = Error;
}

/// Blocking write of an entire buffer, returning the number of bytes written.
fn io_write<USCI: Instance>(buf: &[u8]) -> Result<usize, Error> {
    for &byte in buf {
        nb::block!(try_write_byte::<USCI>(byte))?;
    }
    Ok(buf.len())
}

/// Blocking read: wait for at least one byte, then drain whatever else is
/// immediately available (without blocking) up to the buffer length.
fn io_read<USCI: Instance>(buf: &mut [u8]) -> Result<usize, Error> {
    if buf.is_empty() {
        return Ok(0);
    }
    buf[0] = nb::block!(try_read_byte::<USCI>())?;
    let mut n = 1;
    while n < buf.len() {
        match try_read_byte::<USCI>() {
            Ok(b) => {
                buf[n] = b;
                n += 1;
            }
            Err(nb::Error::WouldBlock) => break,
            Err(nb::Error::Other(e)) => return Err(e),
        }
    }
    Ok(n)
}

impl<USCI: Instance> embedded_io::Write for Serial<USCI> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        io_write::<USCI>(buf)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(try_flush::<USCI>())
    }
}

impl<USCI: Instance> embedded_io::Write for Tx<USCI> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        io_write::<USCI>(buf)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(try_flush::<USCI>())
    }
}

impl<USCI: Instance> embedded_io::Read for Serial<USCI> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        io_read::<USCI>(buf)
    }
}

impl<USCI: Instance> embedded_io::Read for Rx<USCI> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        io_read::<USCI>(buf)
    }
}

impl<USCI: Instance> embedded_io::WriteReady for Serial<USCI> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(tx_ready::<USCI>())
    }
}
impl<USCI: Instance> embedded_io::WriteReady for Tx<USCI> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(tx_ready::<USCI>())
    }
}

impl<USCI: Instance> embedded_io::ReadReady for Serial<USCI> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(rx_ready::<USCI>())
    }
}
impl<USCI: Instance> embedded_io::ReadReady for Rx<USCI> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(rx_ready::<USCI>())
    }
}

// ---------------------------------------------------------------------------
// core::fmt::Write — enables write!/writeln! on the transmit side
// ---------------------------------------------------------------------------
//
// NOTE: these impls are deliberately omitted. Pulling in `core::fmt::write`
// (the dynamic formatting engine behind `write!`) costs ~30 KB of `.rodata`,
// which does not fit alongside application code in this device's 48 KB of FRAM.
// Callers that need formatted output can use a stack buffer + `core::fmt` and
// then `embedded_io::Write::write_all`, or pull in a lightweight formatter.
