//! Capacitive Touch I/O (CAPTIO) — turn any port pad into a relaxation
//! oscillator whose frequency tracks the pad's capacitance, and count that
//! oscillation with the instance's dedicated internal timer.
//!
//! # How the hardware works
//!
//! A CAPTIO instance is a single register, `CAPTIOxCTL`, and no clock: while
//! `CAPTIOEN` is set, the selected pad's inverted Schmitt-trigger input is
//! fed back into its pull-up/pull-down control, so the pad charges toward one
//! rail, trips the trigger, and discharges toward the other — a relaxation
//! oscillator whose period is set by the pad's capacitance against the
//! internal pull resistors. A fingertip on (or near) the pad adds a few
//! picofarads, and the frequency drops measurably. The oscillation is routed
//! **internally** to a timer; touch sensing is then just frequency
//! measurement, and "touched" is a relative verdict: a count over a fixed
//! gate that lands well below the pad's untouched baseline.
//!
//! # The instance↔timer pairing (fixed in silicon)
//!
//! Per SLAS704G Tables 6-15/6-16, each CAPTIO instance feeds exactly one of
//! the two internal-only Timer_A blocks — as its external-clock input
//! (`INCLK`, `TASSEL = 3`) and also as capture input `CCI1A`:
//!
//! | Instance | `CAPTIOxCTL` | Paired timer | Timer base |
//! |----------|--------------|--------------|------------|
//! | CAPTIO0  | `0x043E`     | **TA2**      | `0x0400`   |
//! | CAPTIO1  | `0x047E`     | **TA3**      | `0x0440`   |
//!
//! TA2/TA3 have no package pins and only two CCRs, so spending them here
//! costs nothing the rest of the HAL wants. [`TouchSense`] consumes **both**
//! the CAPTIO instance and its paired timer, and runs the timer from `INCLK`
//! in continuous mode: `TAxR` literally counts oscillation cycles. Frequency
//! is a gated count against a [`crate::timer::Counter`] yardstick
//! ([`TouchSense::measure_hz`]); touch detection compares raw
//! [`TouchSense::count`] deltas against a baseline.
//!
//! Any pad on P1–P4 or PJ can be routed, one at a time per instance (the two
//! instances are independent and concurrent). Successive re-routing is how
//! multi-pad scanning works — route, gate, read, next pad. Routing overrides
//! the pad's other functions while enabled, but leave the pin unconfigured as
//! a driver: [`TouchSense::route_pin`] takes a floating **input** pin, since
//! an enabled output driver or GPIO pull resistor would fight the oscillator
//! (the module runs the pad's pulls itself). `route_raw` reaches pads with no
//! typed-pin coverage (PJ) — mind that PJ.4/PJ.5 are the LFXT crystal pads on
//! this board.
//!
//! While `CAPTIOEN` is clear the signal toward the timer is 0 (the count
//! freezes) and the `CAPTIO` state bit reads 0.
//!
//! The oscillator is self-clocked (no system clock involved), so counting
//! continues with the CPU asleep; a counter-overflow interrupt
//! ([`TouchSense::enable_overflow_interrupt`], `TIMERx_A1` vector, demuxed by
//! [`read_timer_iv`]) can wake LPM0 — HW-verified by the `captio` fixture.
//! The overflow recurs every 65536 oscillations (tens of ms), so a one-shot
//! wake handler must disarm **inside the ISR** via
//! [`isr_disable_overflow_interrupt`], the capture module's starvation rule
//! in its mildest form.
//!
//! The `CAPTIOxCTL` encoding and the gated-count→Hz rounding are pure math in
//! `captio_ctl.rs`, host-tested (unit module `captio_ctl`) and re-exported
//! here.

pub use crate::captio_ctl::{ctl_word, hz_from_gate, CAPTIOEN, CAPTIO_STATE};

use crate::captio_ctl;
use crate::gpio::{Floating, Input, Pin, P1, P2, P3, P4};
use crate::pac;
use crate::timer::Counter;

// ---------------------------------------------------------------------------
// Register map (Timer_A offsets, same layout as `capture`'s)
// ---------------------------------------------------------------------------

const CTL: usize = 0x00; // TAxCTL
const R: usize = 0x10; // TAxR
const IV: usize = 0x2E; // TAxIV

// TAxCTL fields for the counting configuration.
const TASSEL_INCLK: u16 = 0x0300; // external clock input = the CAPTIO signal
const MC_CONTINUOUS: u16 = 0x0020;
const TACLR: u16 = 0x0004;
#[cfg(feature = "critical-section")]
const TAIE: u16 = 0x0002;
const TAIFG: u16 = 0x0001;

/// The paired timer's running configuration: count the CAPTIO oscillation
/// (`INCLK`), free-running. Rewriting this word with [`TACLR`] restarts the
/// count from zero (and clears `TAIFG`/`TAIE`).
const TIMER_RUN: u16 = TASSEL_INCLK | MC_CONTINUOUS;

#[inline(always)]
unsafe fn read_reg(addr: usize) -> u16 {
    (addr as *const u16).read_volatile()
}

#[inline(always)]
unsafe fn write_reg(addr: usize, val: u16) {
    (addr as *mut u16).write_volatile(val);
}

// ---------------------------------------------------------------------------
// Instances
// ---------------------------------------------------------------------------

mod sealed {
    pub trait Sealed {}
}

/// A CAPTIO instance bonded to its silicon-fixed measurement timer.
/// Implemented for the PAC's [`pac::CapacitiveTouchIo0`] (paired with TA2)
/// and [`pac::CapacitiveTouchIo1`] (TA3).
pub trait Instance: sealed::Sealed {
    /// Absolute address of the `CAPTIOxCTL` register (the PAC models it as
    /// the peripheral's base — the instance is that one register).
    const CTL_ADDR: usize;
    /// Absolute base address of the paired timer's `TAxCTL` register block.
    const TIMER_BASE: usize;
    /// The paired timer's PAC type, consumed alongside the instance so the
    /// pairing can't be crossed and the timer can't be double-used.
    type Timer;
}

impl sealed::Sealed for pac::CapacitiveTouchIo0 {}
impl Instance for pac::CapacitiveTouchIo0 {
    const CTL_ADDR: usize = 0x043E;
    const TIMER_BASE: usize = 0x0400; // TA2
    type Timer = pac::Timer2A2;
}

impl sealed::Sealed for pac::CapacitiveTouchIo1 {}
impl Instance for pac::CapacitiveTouchIo1 {
    const CTL_ADDR: usize = 0x047E;
    const TIMER_BASE: usize = 0x0440; // TA3
    type Timer = pac::Timer3A2;
}

// ---------------------------------------------------------------------------
// Ports
// ---------------------------------------------------------------------------

/// Maps a GPIO port marker to its `CAPTIOPOSELx` code. Sealed: the code↔port
/// assignment is fixed by the register definition.
pub trait CaptioPort: sealed::Sealed {
    /// The 4-bit `CAPTIOPOSELx` value selecting this port.
    const POSEL: u8;
}

macro_rules! captio_ports {
    ($($Port:ty => $posel:literal),+ $(,)?) => {$(
        impl sealed::Sealed for $Port {}
        impl CaptioPort for $Port {
            const POSEL: u8 = $posel;
        }
    )+};
}

captio_ports! {
    P1 => 1,
    P2 => 2,
    P3 => 3,
    P4 => 4,
}

/// A port selectable through [`TouchSense::route_raw`] — the ports this
/// device bonds out (`CAPTIOPOSELx` addresses up to P15 on larger packages;
/// selecting an absent port "gives unpredictable results" per SLAU367P, so
/// the raw path is still enum-bounded to real ones).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Port {
    /// Port J (`POSEL = 0`). PJ.0–PJ.3 are the JTAG pads (free under
    /// Spy-Bi-Wire debug); **PJ.4/PJ.5 are the LFXT crystal pads** — routing
    /// them disturbs the 32.768 kHz oscillator.
    PJ = 0,
    /// Port 1.
    P1 = 1,
    /// Port 2.
    P2 = 2,
    /// Port 3.
    P3 = 3,
    /// Port 4.
    P4 = 4,
}

/// Rejected pad selection: the pin index exceeds the 3-bit `CAPTIOPISELx`
/// field (`Px.0`..`Px.7`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct InvalidPin;

// ---------------------------------------------------------------------------
// The driver
// ---------------------------------------------------------------------------

/// One CAPTIO instance plus its paired internal timer, counting the routed
/// pad's oscillation. Constructed disabled (no pad routed, count frozen at
/// zero); route a pad to start it oscillating.
pub struct TouchSense<C: Instance> {
    _captio: C,
    _timer: C::Timer,
}

impl<C: Instance> TouchSense<C> {
    /// Take the CAPTIO instance and its silicon-paired timer. The timer is
    /// started immediately — `INCLK` source, continuous mode, cleared — but
    /// with the CAPTIO disabled its input is a constant 0, so the count sits
    /// at zero until a pad is routed.
    pub fn new(captio: C, timer: C::Timer) -> Self {
        unsafe {
            write_reg(C::CTL_ADDR, 0); // disabled, no pad selected
            write_reg(C::TIMER_BASE + CTL, TIMER_RUN | TACLR);
        }
        TouchSense {
            _captio: captio,
            _timer: timer,
        }
    }

    /// Route the oscillator to a typed pin and enable it. Takes the pin as a
    /// floating **input** borrow: the module drives the pad through its own
    /// pull-resistor control, so a GPIO output driver or REN pull would fight
    /// the oscillation. A borrow rather than a move because scanning
    /// re-routes the one instance across many pads ([`ctl_word`]'s `+2`
    /// idiom in typed form).
    pub fn route_pin<PORT: CaptioPort, const N: u8>(
        &mut self,
        _pin: &Pin<PORT, N, Input<Floating>>,
    ) {
        // `gpio` only vends N = 0..=7 and POSEL is 4-bit by construction, so
        // the encoding cannot fail; 0 (disabled) keeps the fallback honest.
        let word = captio_ctl::ctl_word(PORT::POSEL, N).unwrap_or(0);
        unsafe { write_reg(C::CTL_ADDR, word) };
    }

    /// Route the oscillator to `port`.`pin` without a typed pin — the escape
    /// hatch for PJ pads and for table-driven scans. The caller owns the
    /// pad-state discipline the typed path enforces (no output driver, no
    /// GPIO pull). Rejects `pin > 7`.
    pub fn route_raw(&mut self, port: Port, pin: u8) -> Result<(), InvalidPin> {
        match captio_ctl::ctl_word(port as u8, pin) {
            Some(word) => {
                unsafe { write_reg(C::CTL_ADDR, word) };
                Ok(())
            }
            None => Err(InvalidPin),
        }
    }

    /// Disable the oscillator (`CAPTIOxCTL = 0`): the pad returns to its
    /// normal function, the signal toward the timer is 0, and the count
    /// freezes where it was.
    pub fn disable(&mut self) {
        unsafe { write_reg(C::CTL_ADDR, 0) };
    }

    /// The raw `CAPTIOxCTL` word (routing, enable, and live state together).
    pub fn ctl(&self) -> u16 {
        unsafe { read_reg(C::CTL_ADDR) }
    }

    /// The live oscillation state (`CAPTIO`, bit 9): the pad's current level
    /// as the oscillator sees it. Reads `false` while disabled; while
    /// enabled it flips at the oscillation rate, so repeated samples see
    /// both levels.
    pub fn state(&self) -> bool {
        self.ctl() & CAPTIO_STATE != 0
    }

    /// Snapshot the raw oscillation count (`TAxR`). Same modular semantics
    /// as [`crate::timer::Counter::now`]: subtract two snapshots with
    /// `u16::wrapping_sub`, valid while fewer than 65536 oscillations
    /// separate them. This is the touch-detection primitive — a fixed-gate
    /// count delta well below the pad's untouched baseline is a touch.
    ///
    /// The count clock is the oscillator itself — **asynchronous to MCLK** —
    /// so a lone read of a running counter can tear mid-ripple (SLAU367P's
    /// async-read caveat; majority-voting doesn't help at MHz rates, the
    /// counter outruns back-to-back reads). Fine for touch polling, where a
    /// rare glitched sample washes out; for a clean read, freeze the
    /// oscillation around it as [`measure_hz`](Self::measure_hz) does.
    pub fn count(&self) -> u16 {
        unsafe { read_reg(C::TIMER_BASE + R) }
    }

    /// Restart the count from zero (`TACLR`). Also cancels any armed
    /// overflow interrupt (the control word is rewritten whole).
    pub fn restart_count(&self) {
        unsafe { write_reg(C::TIMER_BASE + CTL, TIMER_RUN | TACLR) };
    }

    /// Measure the routed pad's oscillation frequency: restart the count,
    /// hold for `gate_ticks` of the `counter` yardstick, and convert
    /// ([`hz_from_gate`], rounding half-away-from-zero). Returns `None` if
    /// the count wrapped during the gate (`TAIFG` — the oscillation is too
    /// fast for the gate; shorten it), so an aliased count can't
    /// masquerade as a low frequency. Size the gate so the expected count
    /// stays under 65536 — and expect bare pads to be *fast*: HW-measured
    /// on the LaunchPad 2026-07-11, pads with board traces oscillate at
    /// 3.2–3.5 MHz and trace-less pads exceed 6.5 MHz (a 10 ms gate at a
    /// 1 MHz yardstick wraps on them; 2 ms resolves up to 32.7 MHz and is
    /// a good default). Busy-waits the gate; like
    /// [`restart_count`](Self::restart_count), it cancels an armed overflow
    /// interrupt.
    ///
    /// The count is read **frozen**: the CAPTIO is disabled for the read
    /// (its signal toward the timer goes to 0, stopping the count cleanly)
    /// and restored right after — the async-counter read-tear guard the raw
    /// [`count`](Self::count) can't have. Costs at most one oscillation
    /// cycle of dead time at the gate edge.
    pub fn measure_hz(&self, counter: &Counter, gate_ticks: u16) -> Option<u32> {
        self.restart_count();
        let t0 = counter.now();
        while counter.elapsed_since(t0) < gate_ticks {}
        // Freeze, stamp the gate end, read the (now stable) count, resume.
        let routed = self.ctl() & !CAPTIO_STATE;
        unsafe { write_reg(C::CTL_ADDR, 0) };
        let elapsed = counter.elapsed_since(t0);
        let counts = self.count();
        let wrapped = unsafe { read_reg(C::TIMER_BASE + CTL) } & TAIFG != 0;
        unsafe { write_reg(C::CTL_ADDR, routed) };
        if wrapped {
            return None;
        }
        Some(captio_ctl::hz_from_gate(counts, elapsed, counter.tick_hz()))
    }

    /// Enable the count-overflow interrupt (`TAIE`): every 65536
    /// oscillations `TAIFG` fires the paired timer's shared **`TIMERx_A1`**
    /// vector (TA2 → `TIMER2_A1`, TA3 → `TIMER3_A1`), demuxed by
    /// [`read_timer_iv`] — the CPU-asleep touch watch. Clears a stale
    /// `TAIFG` first so a wrap that predates the arming can't fire
    /// immediately. The wrap recurs; one-shot wake handlers disarm inside
    /// the ISR via [`isr_disable_overflow_interrupt`].
    #[cfg(feature = "critical-section")]
    pub fn enable_overflow_interrupt(&mut self) {
        // The ISR's TAxIV read clears TAIFG in silicon; RMW under a critical
        // section so this write can't resurrect a flag the ISR just consumed.
        critical_section::with(|_| unsafe {
            let v = read_reg(C::TIMER_BASE + CTL);
            write_reg(C::TIMER_BASE + CTL, (v | TAIE) & !TAIFG);
        });
    }

    /// Disable the count-overflow interrupt and drop any pending `TAIFG`.
    #[cfg(feature = "critical-section")]
    pub fn disable_overflow_interrupt(&mut self) {
        critical_section::with(|_| unsafe {
            let v = read_reg(C::TIMER_BASE + CTL);
            write_reg(C::TIMER_BASE + CTL, v & !(TAIE | TAIFG));
        });
    }

    /// Release the CAPTIO instance and timer (both left as configured; the
    /// oscillator keeps running if a pad is routed —
    /// [`disable`](Self::disable) first for a clean hand-back).
    pub fn free(self) -> (C, C::Timer) {
        (self._captio, self._timer)
    }
}

// ---------------------------------------------------------------------------
// ISR-side helpers
// ---------------------------------------------------------------------------

/// `TAxIV` demux value for the count overflow (`TAIFG`) — the only source
/// [`TouchSense`] arms.
pub const IV_OVERFLOW: u16 = 0x000E;

/// Read the paired timer's `TAxIV` from its `TIMERx_A1` ISR: returns the
/// highest-priority pending source ([`IV_OVERFLOW`], or 0x02 for the unused
/// CCR1; 0 if none) and atomically clears that source's flag in silicon —
/// the lost-flag-proof consumption pattern of [`crate::capture::read_iv`].
/// Keyed on the CAPTIO instance because the ISR does not own the
/// [`TouchSense`].
pub fn read_timer_iv<C: Instance>() -> u16 {
    unsafe { read_reg(C::TIMER_BASE + IV) }
}

/// Disarm the count-overflow interrupt **from the ISR**. The overflow
/// re-latches every 65536 oscillations, so a one-shot wake handler that
/// stays armed fires again tens of milliseconds later; the disarm belongs
/// inside the handler ([`crate::capture::isr_disable_interrupt`]'s rule).
/// Plain RMW, no critical section: interrupts do not nest here, and
/// thread-mode `TAxCTL` writes are protected on their side.
#[cfg(feature = "critical-section")]
pub fn isr_disable_overflow_interrupt<C: Instance>() {
    unsafe {
        let addr = C::TIMER_BASE + CTL;
        write_reg(addr, read_reg(addr) & !TAIE);
    }
}
