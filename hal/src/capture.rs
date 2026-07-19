//! Timer_A input capture — hardware timestamps of signal edges.
//!
//! [`crate::timer::Counter`] answers "how much time passed between these two
//! *instructions*?" — a software read of `TAxR`, so interrupt latency and
//! scheduling jitter land in the measurement. Capture mode answers the harder
//! question: **when exactly did that edge happen?** A capture/compare channel
//! with `CAP = 1` copies `TAxR` into its `TAxCCRn` register *in silicon* the
//! instant a selected edge appears on its input — the timestamp is frozen at
//! the event, and software can come collect it microseconds later without
//! losing a tick. Two timestamps and a subtraction turn that into period,
//! frequency, pulse width, or duty cycle, all measured to one timer tick.
//!
//! # Instances and channels (MSP430FR5969)
//!
//! The part has two pin-connected Timer_A3 blocks; this driver is generic over
//! both via [`Instance`] (the same raw-pointer pattern as [`crate::spi`],
//! since the PAC's per-instance register types cannot unify). [`Counter`]
//! usually owns TA0, which makes **TA1 the natural capture timer** — the
//! typestate keeps the two uses exclusive either way.
//!
//! Each block has three capture/compare channels, of which this driver exposes
//! **CCR1 and CCR2**. CCR0 is deliberately left out: it has its own interrupt
//! vector and is the compare/wake workhorse ([`Counter::schedule_wake_in`]),
//! and its TA1 capture pin (P1.7) is an eUSCI_B0 pin. What each channel can
//! watch is fixed by silicon (datasheet SLAS704G Tables 6-13/6-14):
//!
//! | Instance | Channel | Pin input (`CCIxA`)       | Internal input (`CCIxB`) |
//! |----------|---------|---------------------------|--------------------------|
//! | TA0      | CCR1    | P1.0 (green LED!)         | `COUT` (Comp_E output)   |
//! | TA0      | CCR2    | P1.1 (REFOUT / button S2) | `ACLK`                   |
//! | TA1      | CCR1    | **P1.2**                  | `COUT` (Comp_E output)   |
//! | TA1      | CCR2    | **P1.3**                  | `ACLK`                   |
//!
//! The two internal inputs are quietly excellent:
//!
//! - **`ACLK` on CCR2** means an SMCLK-clocked capture timer can timestamp
//!   crystal edges with no wiring at all — the measured tick-per-period ratio
//!   *is* the DCO's frequency error against the 32.768 kHz crystal (the math
//!   lives in `capture_math.rs`, host-tested).
//! - **`COUT` on CCR1** timestamps the analog comparator's output edges, so
//!   [`crate::comp_e`] events get hardware-accurate timing — and the
//!   REFOUT + ladder-step trick from the comparator fixture doubles as a
//!   hands-free capture stimulus.
//!
//! Every channel can also be fired **from software** ([`CaptureChannel::
//! fire_software`]): the `CCIS` field can select constant GND or VCC as the
//! input, and flipping GND→VCC manufactures a rising edge in hardware —
//! the no-wiring smoke test for the whole latch path.
//!
//! # Reading captures, and the overcapture flag
//!
//! `CCIFG` set means "a timestamp is waiting in `TAxCCRn`". If a *second* edge
//! is captured while `CCIFG` is still set, the hardware overwrites the
//! timestamp with the newer one and latches **`COV`** (overcapture) — the old
//! event's time is gone and the driver reports it ([`Error::Overcapture`])
//! rather than silently pairing mismatched edges. The polling budget is
//! therefore one full signal period: after seeing `CCIFG`, collect the value
//! before the next edge lands. At MCLK = 1 MHz a [`CaptureChannel::wait_edge`]
//! iteration costs ~15 µs, so polling tracks signals up to roughly 30 kHz
//! (ACLK, at 30.5 µs/edge, is right at the comfortable edge of that — which
//! is why [`CaptureChannel::measure_span_ticks`] exists: it services only the
//! two *bracketing* edges of a long span and counts the periods in between
//! arithmetically, `capture_math::periods_in_span`).
//!
//! All captures are synchronized to the timer clock (`SCS = 1`), per
//! SLAU367P's recommendation — an asynchronous capture racing the counter
//! increment can read a torn value.
//!
//! # Interrupts
//!
//! CCR1, CCR2, and the counter overflow share one vector per instance
//! (`TIMER0_A1` / `TIMER1_A1`), demuxed by `TAxIV`: [`read_iv`] returns
//! [`IV_CCR1`] (0x02), [`IV_CCR2`] (0x04), or [`IV_OVERFLOW`] (0x0E), and the
//! IV read **atomically clears the served flag in silicon** — the same
//! read-and-clear discipline as [`crate::gpio::read_iv`] / [`crate::dma::
//! read_iv`]. Because that clear happens in silicon, every thread-mode RMW of
//! a `TAxCCTLn` register in this driver runs inside `critical_section::with`
//! (mirroring the DMA driver's rule): a plain RMW could re-assert a `CCIFG`
//! the ISR consumed mid-sequence. A capture interrupt is a `wake_cpu` wake
//! source from LPM0 (SMCLK-clocked) or LPM3 (ACLK-clocked), like any other
//! Timer_A interrupt.
//!
//! # Example: measure a PWM signal on P1.2 (TA1, one jumper from P1.4)
//!
//! ```ignore
//! let capture = CaptureTimer::new_smclk(p.timer_1_a3, &clocks, Divider::Div1);
//! let p12 = port1.pin2.into_timer_a_capture();          // P1.2 → TA1.CCI1A
//! let mut ch = capture.capture_pin(p12, Edge::Rising);
//! let hz = ch.frequency_hz(8, 40_000)?;                 // 8 periods, ~40 ms timeout
//! ch.set_edge(Edge::Both);
//! let duty = ch.measure_duty_permille(40_000)?;         // needs Edge::Both
//! ```

use core::marker::PhantomData;

pub use crate::capture_math::{
    duty_permille, hz_from_period_ticks, periods_in_span, span_ratio_permille, within_permille,
};

use crate::capture_math;
use crate::clocks::Clocks;
use crate::gpio::{Pin, TimerA, P1};
use crate::pac;
use crate::timer::Divider;

// ---------------------------------------------------------------------------
// Register map (offsets from the instance base — identical for every Timer_A)
// ---------------------------------------------------------------------------

const CTL: usize = 0x00; // TAxCTL
const CCTL0: usize = 0x02; // TAxCCTL0 (CCTLn = CCTL0 + 2n)
const R: usize = 0x10; // TAxR
const CCR0: usize = 0x12; // TAxCCR0 (CCRn = CCR0 + 2n)
const IV: usize = 0x2E; // TAxIV

// TAxCTL bit fields.
const TASSEL_ACLK: u16 = 0x0100;
const TASSEL_SMCLK: u16 = 0x0200;
const MC_CONTINUOUS: u16 = 0x0020;
const TACLR: u16 = 0x0004;
#[cfg(feature = "critical-section")]
const TAIE: u16 = 0x0002;
#[cfg(feature = "critical-section")]
const TAIFG: u16 = 0x0001;

// TAxCCTLn bit fields.
const CM_MASK: u16 = 0xC000; // capture-mode (edge select)
const CM_RISING: u16 = 0x4000;
const CM_FALLING: u16 = 0x8000;
const CM_BOTH: u16 = 0xC000;
const CCIS_MASK: u16 = 0x3000; // capture input select
const CCIS_A: u16 = 0x0000; // CCIxA (the pin)
const CCIS_B: u16 = 0x1000; // CCIxB (internal: COUT on CCR1, ACLK on CCR2)
const CCIS_GND: u16 = 0x2000;
const CCIS_VCC: u16 = 0x3000;
const SCS: u16 = 0x0800; // synchronize capture to the timer clock
const CAP: u16 = 0x0100; // capture (not compare) mode
#[cfg(feature = "critical-section")]
const CCIE: u16 = 0x0010;
const CCI: u16 = 0x0008; // raw input level readback
const COV: u16 = 0x0002; // overcapture
const CCIFG: u16 = 0x0001;

#[inline(always)]
unsafe fn read_reg(addr: usize) -> u16 {
    (addr as *const u16).read_volatile()
}

#[inline(always)]
unsafe fn write_reg(addr: usize, val: u16) {
    (addr as *mut u16).write_volatile(val);
}

/// RMW a `TAxCCTLn`/`TAxCTL` word. The shared-vector ISR's `TAxIV` read
/// clears `CCIFG`/`TAIFG` **in silicon**, so a thread-mode read-modify-write
/// racing it could write back a flag the ISR just consumed — the same hazard
/// (and the same fix) as the DMA driver's `DMAxCTL` accesses: take a critical
/// section around the RMW whenever interrupts are possible.
#[inline]
fn rmw(addr: usize, f: impl Fn(u16) -> u16) {
    #[cfg(feature = "critical-section")]
    critical_section::with(|_| unsafe { write_reg(addr, f(read_reg(addr))) });
    #[cfg(not(feature = "critical-section"))]
    unsafe {
        write_reg(addr, f(read_reg(addr)))
    };
}

// ---------------------------------------------------------------------------
// Instances
// ---------------------------------------------------------------------------

mod sealed {
    pub trait Sealed {}
}

/// A pin-connected Timer_A3 block usable for input capture. Implemented for
/// the PAC's [`pac::Timer0A3`] and [`pac::Timer1A3`] (TA2/TA3 exist but have
/// internal-only inputs and only two CCRs; they are not covered here). The
/// [`CaptureTimer`] consumes the PAC peripheral, so capture-vs-[`crate::timer::
/// Counter`] on the same block is an exclusive choice by move semantics.
pub trait Instance: sealed::Sealed {
    /// Absolute base address of the `TAxCTL` register block.
    const BASE: usize;
}

impl sealed::Sealed for pac::Timer0A3 {}
impl Instance for pac::Timer0A3 {
    const BASE: usize = 0x0340;
}

impl sealed::Sealed for pac::Timer1A3 {}
impl Instance for pac::Timer1A3 {
    const BASE: usize = 0x0380;
}

// ---------------------------------------------------------------------------
// Typed capture pins
// ---------------------------------------------------------------------------

/// A pin routed to a Timer_A capture input (`TAx.CCInA`), in [`TimerA`] mode
/// via [`Pin::into_timer_a_capture`](crate::gpio::Pin::into_timer_a_capture).
/// The associated constant is the channel the silicon wires that pin to
/// (SLAS704G Tables 6-13/6-14) — routing a pin with no capture function is a
/// compile error. Sealed: the pin↔channel map is fixed by the package.
pub trait CapturePin<T: Instance>: sealed::Sealed {
    /// The `TAxCCRn` channel (1 or 2) this pin feeds.
    const CHANNEL: u8;
}

macro_rules! capture_pins {
    ($($Timer:ty: $Port:ident $N:literal => $ch:literal),+ $(,)?) => {$(
        impl sealed::Sealed for Pin<$Port, $N, TimerA> {}
        impl CapturePin<$Timer> for Pin<$Port, $N, TimerA> {
            const CHANNEL: u8 = $ch;
        }
    )+};
}

// TA0's capture pins double as the green LED (P1.0) and REFOUT/button-S2
// (P1.1) on the LaunchPad; TA1's P1.2/P1.3 are free — another reason TA1 is
// the natural capture instance.
capture_pins! {
    pac::Timer0A3: P1 0 => 1, // TA0.CCI1A (LaunchPad green LED)
    pac::Timer0A3: P1 1 => 2, // TA0.CCI2A (REFOUT / C1 / button S2)
    pac::Timer1A3: P1 2 => 1, // TA1.CCI1A
    pac::Timer1A3: P1 3 => 2, // TA1.CCI2A
}

// ---------------------------------------------------------------------------
// Configuration types
// ---------------------------------------------------------------------------

/// Which input transition latches a capture.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Edge {
    /// Low → high.
    Rising,
    /// High → low.
    Falling,
    /// Every transition. Required by [`CaptureChannel::measure_duty_permille`];
    /// note that with `Both`, one "period" of [`CaptureChannel::
    /// measure_period_ticks`] is edge-to-edge, i.e. half a signal period.
    Both,
}

impl Edge {
    const fn cm_bits(self) -> u16 {
        match self {
            Edge::Rising => CM_RISING,
            Edge::Falling => CM_FALLING,
            Edge::Both => CM_BOTH,
        }
    }
}

/// Which of the two exposed capture channels a software-fired capture uses.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Slot {
    /// `TAxCCR1` (interrupt demux code [`IV_CCR1`]).
    Ccr1 = 1,
    /// `TAxCCR2` (interrupt demux code [`IV_CCR2`]).
    Ccr2 = 2,
}

/// Capture measurement errors.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// No edge arrived within the caller's tick budget (dead input — an
    /// unjumpered pin, a stopped source). Bounded waits are a project-wide
    /// rule: a fixture must fail, not go dark.
    Timeout,
    /// A second edge was captured before the first was collected (`COV`): the
    /// earlier timestamp is gone and any pairing would be wrong. The signal is
    /// too fast for the polling loop — lower the signal rate or widen the
    /// polling budget.
    Overcapture,
    /// The input level after an edge did not match the edge direction the
    /// measurement expected (e.g. a pulse shorter than the level-readback
    /// latency during [`CaptureChannel::measure_duty_permille`]).
    LevelSync,
}

// ---------------------------------------------------------------------------
// The capture timer
// ---------------------------------------------------------------------------

/// A free-running Timer_A block dedicated to input capture. Owns the PAC
/// peripheral; vends per-CCR [`CaptureChannel`]s.
pub struct CaptureTimer<T: Instance> {
    _timer: T,
    tick_hz: u32,
}

impl<T: Instance> CaptureTimer<T> {
    /// Free-run the counter from **SMCLK** ÷ `div` (continuous mode).
    ///
    /// SMCLK is gated off in LPM3, so this variant is for awake / LPM0
    /// measurement — but its tick can be the full 8 MHz DCO (125 ns
    /// resolution, 8.19 ms wrap; see the table in [`crate::timer`]).
    pub fn new_smclk(timer: T, clocks: &Clocks, div: Divider) -> Self {
        Self::new(timer, TASSEL_SMCLK, clocks.smclk(), div)
    }

    /// Free-run the counter from **ACLK** ÷ `div` (continuous mode).
    ///
    /// With [`crate::clocks::configure_low_power`] this keeps counting (and
    /// capturing, and waking) through LPM3. Note the counter then ticks
    /// asynchronously to MCLK; [`now`](CaptureTimer::now) is a single
    /// unmajority-voted read, same as [`crate::timer::Counter::now`] — fine
    /// for polling loops, but don't lean on a lone read for exact math.
    pub fn new_aclk(timer: T, clocks: &Clocks, div: Divider) -> Self {
        Self::new(timer, TASSEL_ACLK, clocks.aclk(), div)
    }

    fn new(timer: T, tassel: u16, source_hz: u32, div: Divider) -> Self {
        let (id_bits, divisor) = match div {
            Divider::Div1 => (0u16 << 6, 1),
            Divider::Div2 => (1u16 << 6, 2),
            Divider::Div4 => (2u16 << 6, 4),
            Divider::Div8 => (3u16 << 6, 8),
        };
        // One full write: source, divider, continuous mode, and TACLR to
        // start counter + divider from a clean zero. (TAIE off, TAIFG clear.)
        unsafe { write_reg(T::BASE + CTL, tassel | id_bits | MC_CONTINUOUS | TACLR) };
        CaptureTimer {
            _timer: timer,
            tick_hz: source_hz / divisor,
        }
    }

    /// The counter's tick rate, in Hz.
    pub const fn tick_hz(&self) -> u32 {
        self.tick_hz
    }

    /// Snapshot the raw 16-bit counter (same modular-timestamp semantics as
    /// [`crate::timer::Counter::now`]).
    pub fn now(&self) -> u16 {
        unsafe { read_reg(T::BASE + R) }
    }

    /// Watch a capture **pin** with this timer. Consumes the routed
    /// [`TimerA`]-mode pin; the [`CapturePin`] bound fixes the channel to the
    /// one the silicon wires that pin to.
    pub fn capture_pin<P: CapturePin<T>>(&self, _pin: P, edge: Edge) -> CaptureChannel<T> {
        self.channel(P::CHANNEL, CCIS_A, edge.cm_bits(), false)
    }

    /// Watch the **Comp_E comparator output** (`COUT`, internal `CCI1B`) on
    /// channel 1 — hardware timestamps for analog threshold crossings, no pin
    /// spent. The comparator is configured independently via [`crate::comp_e`].
    pub fn capture_comparator(&self, edge: Edge) -> CaptureChannel<T> {
        self.channel(1, CCIS_B, edge.cm_bits(), false)
    }

    /// Watch **ACLK** (internal `CCI2B`) on channel 2. With ACLK on the LFXT
    /// crystal and the timer on SMCLK, the measured ticks-per-period ratio is
    /// the DCO's frequency error against the crystal
    /// (`capture_math::span_ratio_permille`).
    pub fn capture_aclk(&self, edge: Edge) -> CaptureChannel<T> {
        self.channel(2, CCIS_B, edge.cm_bits(), false)
    }

    /// A **software-fired** capture channel: the input is the constant-GND
    /// `CCIS` source, and each [`CaptureChannel::fire_software`] flips it
    /// GND→VCC→GND — a rising edge manufactured in hardware. The no-wiring
    /// smoke test for the capture latch, timestamp, `COV`, and interrupt path.
    pub fn capture_software(&self, slot: Slot) -> CaptureChannel<T> {
        self.channel(slot as u8, CCIS_GND, CM_RISING, true)
    }

    fn channel(&self, channel: u8, ccis: u16, cm: u16, software: bool) -> CaptureChannel<T> {
        // Full write: capture mode, synchronized, chosen input + edge, CCIE
        // off, CCIFG/COV cleared. Reconfiguring a channel is idempotent and
        // discards anything pending.
        unsafe { write_reg(T::BASE + CCTL0 + 2 * channel as usize, cm | ccis | SCS | CAP) };
        CaptureChannel {
            channel,
            software,
            tick_hz: self.tick_hz,
            fresh_arm: true,
            _instance: PhantomData,
        }
    }

    /// Enable the counter-overflow interrupt (`TAIE`): each 0xFFFF→0 rollover
    /// then fires the shared `TIMERx_A1` vector with [`read_iv`] =
    /// [`IV_OVERFLOW`] (the IV read clears it). Any **stale** `TAIFG` is
    /// dropped first — the counter has been wrapping since construction, so
    /// without that the enable would fire instantly for a rollover nobody was
    /// watching.
    #[cfg(feature = "critical-section")]
    pub fn enable_overflow_interrupt(&self) {
        rmw(T::BASE + CTL, |v| (v | TAIE) & !TAIFG);
    }

    /// Disable the counter-overflow interrupt and drop any pending `TAIFG`.
    #[cfg(feature = "critical-section")]
    pub fn disable_overflow_interrupt(&self) {
        rmw(T::BASE + CTL, |v| v & !(TAIE | TAIFG));
    }

    /// Release the underlying timer peripheral (left running).
    pub fn free(self) -> T {
        self._timer
    }
}

// ---------------------------------------------------------------------------
// A capture channel
// ---------------------------------------------------------------------------

/// One Timer_A capture channel (`TAxCCRn`, n = 1 or 2), created by the
/// [`CaptureTimer`] `capture_*` constructors.
///
/// Holds only the channel index and tick rate and reaches its own
/// `TAxCCTLn`/`TAxCCRn` through the instance's base address — sound because it
/// touches **only its own** channel registers, disjoint from the control word
/// and the sibling channel (the [`crate::pwm::PwmChannel`] pattern).
pub struct CaptureChannel<T: Instance> {
    channel: u8,
    software: bool,
    tick_hz: u32,
    /// Set by (re)arming, cleared by the first collected capture. The first
    /// real edge after arming can double-latch (`CCIFG` + immediate `COV`
    /// from one physical transition — the capture synchronizer picking up
    /// its initial state; HW-observed 2026-07-18, deterministic per input
    /// phase at arming). Both stamps are genuine times of that same edge, so
    /// [`wait_edge`](CaptureChannel::wait_edge) tolerates `COV` exactly once
    /// after an arm instead of failing the measurement.
    fresh_arm: bool,
    _instance: PhantomData<T>,
}

impl<T: Instance> CaptureChannel<T> {
    fn cctl(&self) -> usize {
        T::BASE + CCTL0 + 2 * self.channel as usize
    }

    fn ccr(&self) -> usize {
        T::BASE + CCR0 + 2 * self.channel as usize
    }

    /// The timer's tick rate, in Hz (for converting timestamp deltas).
    pub const fn tick_hz(&self) -> u32 {
        self.tick_hz
    }

    /// Whether an uncollected capture is waiting (`CCIFG`).
    pub fn pending(&self) -> bool {
        (unsafe { read_reg(self.cctl()) }) & CCIFG != 0
    }

    /// Collect the pending capture, if any: returns the latched timestamp and
    /// clears `CCIFG`. `COV` is left alone — check
    /// [`overcaptured`](CaptureChannel::overcaptured) if pairing matters.
    pub fn take(&mut self) -> Option<u16> {
        if !self.pending() {
            return None;
        }
        let ts = unsafe { read_reg(self.ccr()) };
        rmw(self.cctl(), |v| v & !CCIFG);
        Some(ts)
    }

    /// Whether a capture was overwritten before it was collected (`COV`).
    pub fn overcaptured(&self) -> bool {
        (unsafe { read_reg(self.cctl()) }) & COV != 0
    }

    /// Clear the overcapture flag.
    pub fn clear_overcapture(&mut self) {
        rmw(self.cctl(), |v| v & !COV);
    }

    /// The channel input's current level (`CCI` readback): with the fixed V+/V−
    /// convention this is the raw pin / `COUT` / `ACLK` state right now. After
    /// an [`Edge::Both`] capture it tells which direction the edge went —
    /// provided the level is read back before the signal moves again.
    pub fn input_level(&self) -> bool {
        (unsafe { read_reg(self.cctl()) }) & CCI != 0
    }

    /// Change which edge captures. Discards any pending capture and clears
    /// `COV` (an edge armed differently must not be paired with one from the
    /// old arming).
    ///
    /// The change passes through `CM = 0` (capture disabled): SLAU367P's
    /// "Changing Capture Inputs" note requires capture configuration changes
    /// to happen only with `CM = 0` or `CAP = 0` — rewriting `CMx` on an
    /// armed channel can latch an unintended capture event (HW-observed
    /// 2026-07-18: a live 1 kHz input measured cleanly under either arming,
    /// but a rising→both rewrite mid-stream poisoned the next paired
    /// measurement with a spurious `COV`).
    pub fn set_edge(&mut self, edge: Edge) {
        rmw(self.cctl(), |v| v & !CM_MASK);
        rmw(self.cctl(), |v| {
            (v & !(CCIFG | COV)) | edge.cm_bits()
        });
        self.fresh_arm = true;
    }

    /// Manufacture one rising edge in hardware by flipping the `CCIS` input
    /// GND→VCC, then park it back at GND to re-arm (the falling flip is
    /// ignored under the rising-edge arming). Only meaningful on a channel
    /// from [`CaptureTimer::capture_software`]; on any other channel this is
    /// a no-op, so a mis-wired call cannot corrupt a live capture source.
    pub fn fire_software(&mut self) {
        if !self.software {
            return;
        }
        rmw(self.cctl(), |v| (v & !CCIS_MASK) | CCIS_VCC);
        rmw(self.cctl(), |v| (v & !CCIS_MASK) | CCIS_GND);
    }

    /// Block until a **fresh** edge is captured and return its timestamp.
    ///
    /// Anything already pending is discarded first (`CCIFG` + `COV` cleared),
    /// so the returned stamp is from an edge that happened *after* the call
    /// began — the property period pairing depends on. Fails with
    /// [`Error::Timeout`] after `timeout_ticks` of the capture timer's own
    /// clock (must be less than one counter wrap), or
    /// [`Error::Overcapture`] if a second edge overwrote the first before the
    /// poll loop could collect it.
    pub fn wait_edge(&mut self, timeout_ticks: u16) -> Result<u16, Error> {
        rmw(self.cctl(), |v| v & !(CCIFG | COV));
        let start = unsafe { read_reg(T::BASE + R) };
        loop {
            let cctl = unsafe { read_reg(self.cctl()) };
            if cctl & CCIFG != 0 {
                let ts = unsafe { read_reg(self.ccr()) };
                // COV here normally means a second edge landed before this
                // read: `ts` is the *second* edge and any pairing math would
                // be wrong. Exception: the first capture after (re)arming may
                // double-latch a SINGLE edge (see `fresh_arm`) — there `ts`
                // is still that edge's genuine time, so it is returned.
                if unsafe { read_reg(self.cctl()) } & COV != 0 {
                    rmw(self.cctl(), |v| v & !(CCIFG | COV));
                    if !self.fresh_arm {
                        return Err(Error::Overcapture);
                    }
                    self.fresh_arm = false;
                    return Ok(ts);
                }
                rmw(self.cctl(), |v| v & !CCIFG);
                self.fresh_arm = false;
                return Ok(ts);
            }
            let now = unsafe { read_reg(T::BASE + R) };
            if now.wrapping_sub(start) >= timeout_ticks {
                return Err(Error::Timeout);
            }
        }
    }

    /// Measure `periods` consecutive same-edge intervals by servicing **every**
    /// edge, returning the summed tick count (each interval must be shorter
    /// than one counter wrap; the poll budget per edge is one signal period —
    /// see the module docs). With a single-edge arming each interval is one
    /// full signal period.
    pub fn measure_period_ticks(
        &mut self,
        periods: u16,
        per_edge_timeout: u16,
    ) -> Result<u32, Error> {
        let mut prev = self.wait_edge(per_edge_timeout)?;
        let mut total: u32 = 0;
        for _ in 0..periods {
            let t = self.wait_edge(per_edge_timeout)?;
            total += t.wrapping_sub(prev) as u32;
            prev = t;
        }
        Ok(total)
    }

    /// Measure the input's frequency in Hz over `periods` periods
    /// (single-edge arming; rounding in `capture_math::hz_from_period_ticks`).
    pub fn frequency_hz(&mut self, periods: u16, per_edge_timeout: u16) -> Result<u32, Error> {
        let total = self.measure_period_ticks(periods, per_edge_timeout)?;
        Ok(capture_math::hz_from_period_ticks(
            self.tick_hz,
            total,
            periods as u32,
        ))
    }

    /// Timestamp one edge, busy-wait at least `min_wait_ticks`, timestamp the
    /// next fresh edge, and return the raw delta — a span over *many* periods
    /// with only the two bracketing edges serviced.
    ///
    /// This is how a poll loop measures a signal near (or beyond) its own
    /// servicing rate: the edges in between are allowed to overwrite each
    /// other freely, and the caller recovers the period count arithmetically
    /// with `capture_math::periods_in_span` (exact while the clocks' relative
    /// error stays under half a period over the span). The whole span must fit
    /// in one counter wrap.
    ///
    /// Unlike [`wait_edge`](CaptureChannel::wait_edge), the bracketing reads
    /// here **tolerate overcapture**: every stamp in `TAxCCRn` is a genuine
    /// edge time, and a span only needs *a* fresh edge at each end — if a
    /// neighbor overwrote the one first seen, the delta still measures an
    /// integer number of periods and the rounding recovers the count. (This
    /// matters exactly when spans are the right tool: a signal near the
    /// polling rate, like ACLK under a 1 MHz MCLK.)
    pub fn measure_span_ticks(
        &mut self,
        min_wait_ticks: u16,
        per_edge_timeout: u16,
    ) -> Result<u16, Error> {
        let t0 = self.wait_fresh_stamp(per_edge_timeout)?;
        while unsafe { read_reg(T::BASE + R) }.wrapping_sub(t0) < min_wait_ticks {}
        let t1 = self.wait_fresh_stamp(per_edge_timeout)?;
        Ok(t1.wrapping_sub(t0))
    }

    /// Overcapture-tolerant fresh-edge wait for span bracketing: any edge
    /// stamped after entry qualifies, so `COV` is cleared rather than treated
    /// as an error (see `measure_span_ticks` for why that is sound).
    fn wait_fresh_stamp(&mut self, timeout_ticks: u16) -> Result<u16, Error> {
        rmw(self.cctl(), |v| v & !(CCIFG | COV));
        let start = unsafe { read_reg(T::BASE + R) };
        loop {
            if (unsafe { read_reg(self.cctl()) }) & CCIFG != 0 {
                let ts = unsafe { read_reg(self.ccr()) };
                rmw(self.cctl(), |v| v & !(CCIFG | COV));
                return Ok(ts);
            }
            let now = unsafe { read_reg(T::BASE + R) };
            if now.wrapping_sub(start) >= timeout_ticks {
                return Err(Error::Timeout);
            }
        }
    }

    /// Measure duty cycle in permille from three consecutive edges
    /// (rise → fall → rise). **Requires an [`Edge::Both`] arming**; the level
    /// readback after each edge identifies its direction, so pulses must be
    /// long relative to the poll latency ([`Error::LevelSync`] otherwise).
    /// A rail-parked signal (0%/100%) has no edges and reports
    /// [`Error::Timeout`].
    pub fn measure_duty_permille(&mut self, per_edge_timeout: u16) -> Result<u16, Error> {
        // Sync to a rising edge (at most one retry: whichever edge comes
        // first, the one after a falling edge is rising).
        let mut t_rise = self.wait_edge(per_edge_timeout)?;
        if !self.input_level() {
            t_rise = self.wait_edge(per_edge_timeout)?;
            if !self.input_level() {
                return Err(Error::LevelSync);
            }
        }
        let t_fall = self.wait_edge(per_edge_timeout)?;
        if self.input_level() {
            return Err(Error::LevelSync);
        }
        let t_rise2 = self.wait_edge(per_edge_timeout)?;
        let high = t_fall.wrapping_sub(t_rise) as u32;
        let period = t_rise2.wrapping_sub(t_rise) as u32;
        Ok(capture_math::duty_permille(high, period))
    }

    /// Enable this channel's capture interrupt (`CCIE`): each capture then
    /// fires the shared `TIMERx_A1` vector, demuxed by [`read_iv`].
    #[cfg(feature = "critical-section")]
    pub fn enable_interrupt(&mut self) {
        rmw(self.cctl(), |v| v | CCIE);
    }

    /// Disable this channel's capture interrupt and drop any pending `CCIFG`.
    #[cfg(feature = "critical-section")]
    pub fn disable_interrupt(&mut self) {
        rmw(self.cctl(), |v| v & !(CCIE | CCIFG));
    }
}

// ---------------------------------------------------------------------------
// ISR-side helpers
// ---------------------------------------------------------------------------

/// `TAxIV` demux value for a CCR1 capture.
pub const IV_CCR1: u16 = 0x0002;
/// `TAxIV` demux value for a CCR2 capture.
pub const IV_CCR2: u16 = 0x0004;
/// `TAxIV` demux value for a counter overflow (`TAIFG`).
pub const IV_OVERFLOW: u16 = 0x000E;

/// Read `TAxIV` from the instance's shared `TIMERx_A1` ISR: returns the
/// highest-priority pending source ([`IV_CCR1`] / [`IV_CCR2`] /
/// [`IV_OVERFLOW`], 0 if none) and **atomically clears that source's flag in
/// silicon** — the lost-flag-proof consumption pattern, same as
/// [`crate::gpio::read_iv`]. A free function because the ISR does not own the
/// [`CaptureTimer`]; it reads a read-to-clear register and nothing else.
pub fn read_iv<T: Instance>() -> u16 {
    unsafe { read_reg(T::BASE + IV) }
}

/// Disarm a channel's capture interrupt (`CCIE`) **from the ISR**, which does
/// not own the [`CaptureChannel`] — the free-function pattern of
/// [`crate::timer::clear_wake_irq`].
///
/// This is not just a convenience: a capture source faster than its own ISR
/// (ACLK at 30.5 µs/edge against an ISR that costs more than that at
/// MCLK = 1 MHz) re-enters the vector back-to-back and **starves thread mode
/// forever** — main never reaches its own `disable_interrupt` call. The only
/// place the interrupt can be turned off is inside the handler, so one-shot
/// wake-on-capture handlers should tally, then call this. (Plain RMW, no
/// critical section: interrupts do not nest here, and thread-mode `CCTLn`
/// RMWs are already protected on their side.)
#[cfg(feature = "critical-section")]
pub fn isr_disable_interrupt<T: Instance>(slot: Slot) {
    let addr = T::BASE + CCTL0 + 2 * (slot as u8 as usize);
    unsafe { write_reg(addr, read_reg(addr) & !CCIE) };
}
