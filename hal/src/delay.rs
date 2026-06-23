//! Blocking delays via a software cycle-counting busy-wait.
//!
//! Implements [`embedded_hal::delay::DelayNs`] for the MSP430FR5969 without
//! consuming a hardware timer. The implementation simply burns CPU cycles in a
//! tight loop whose per-iteration cost is known, so the only thing it needs to
//! turn a *time* into a *cycle count* is the core clock frequency (MCLK).
//!
//! # Why this needs the clock frequency
//!
//! A delay is fundamentally "spin for N clock cycles." Converting a requested
//! duration into N requires knowing how many cycles elapse per second, i.e.
//! MCLK in Hz. After reset this device runs MCLK ≈ 1 MHz (DCO), but once you
//! bring up the clock system (the next milestone in this HAL) MCLK changes, and
//! a [`Delay`] built with the old frequency would be wrong. Always construct
//! [`Delay`] with the *current* MCLK frequency.
//!
//! # Accuracy
//!
//! This is a software delay, so it is **approximate** and biased slightly long:
//!
//! - The inner loop costs [`CYCLES_PER_ITER`] cycles per iteration; the
//!   requested cycle count is divided by that, so sub-`CYCLES_PER_ITER`
//!   remainders are rounded down.
//! - The per-call function/loop-setup overhead (a handful of cycles) is not
//!   subtracted, so very short delays overshoot proportionally.
//! - Interrupts, if enabled, extend the delay by their service time.
//!
//! For the intended use — an ad-hoc watchdog polling on the order of
//! milliseconds-to-seconds — this is well within tolerance. If you need precise
//! or interrupt-tolerant timing, drive a Timer_A/Timer_B channel instead.
//!
//! # Example: watchdog poll loop
//!
//! ```ignore
//! use hal::delay::Delay;
//! use embedded_hal::delay::DelayNs;
//! use embedded_hal::digital::{InputPin, OutputPin};
//!
//! let mut delay = Delay::new(1_000_000); // MCLK = 1 MHz after reset
//! loop {
//!     if heartbeat.is_low().unwrap() {   // external system stopped pinging
//!         reset_line.set_high().unwrap();
//!         delay.delay_ms(10);            // hold the reset pulse
//!         reset_line.set_low().unwrap();
//!     }
//!     delay.delay_ms(1000);              // check once per second
//! }
//! ```

use embedded_hal::delay::DelayNs;

/// CPU cycles consumed by one iteration of the inner busy-loop.
///
/// The loop body is `sub #1, Rn` (1 cycle, `#1` comes from the constant
/// generator) followed by `jne` (2 cycles whether or not the branch is taken on
/// the MSP430), for 3 cycles per iteration.
const CYCLES_PER_ITER: u64 = 3;

/// A blocking delay source backed by a calibrated software busy-loop.
///
/// Construct it with the current MCLK frequency in Hz. Cheap to copy, so you can
/// hand a `Delay` to several drivers.
#[derive(Clone, Copy, Debug)]
pub struct Delay {
    /// Core clock (MCLK) frequency in Hz.
    hz: u32,
}

impl Delay {
    /// Create a delay source for the given MCLK frequency, in Hz.
    pub const fn new(mclk_hz: u32) -> Self {
        Delay { hz: mclk_hz }
    }

    /// The MCLK frequency this delay was calibrated for, in Hz.
    pub const fn freq(&self) -> u32 {
        self.hz
    }

    /// Busy-wait for approximately `cycles` core clock cycles.
    ///
    /// Work is split into `u16`-sized chunks because the inner loop counts down
    /// a 16-bit register; the outer loop's own cost is negligible next to the
    /// chunk it dispatches.
    fn delay_cycles(&self, cycles: u64) {
        let mut iters = cycles / CYCLES_PER_ITER;
        while iters > 0 {
            let chunk = if iters > u16::MAX as u64 {
                u16::MAX
            } else {
                iters as u16
            };
            busy_loop(chunk);
            iters -= chunk as u64;
        }
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // cycles = ns * f / 1e9. Done in u64 so neither the multiply nor any
        // realistic (ns, f) pair overflows; the divisor keeps the result small.
        self.delay_cycles((ns as u64 * self.hz as u64) / 1_000_000_000);
    }

    fn delay_us(&mut self, us: u32) {
        self.delay_cycles((us as u64 * self.hz as u64) / 1_000_000);
    }

    fn delay_ms(&mut self, ms: u32) {
        self.delay_cycles((ms as u64 * self.hz as u64) / 1_000);
    }
}

/// Spin for exactly `iters` iterations of a 3-cycle countdown loop.
///
/// Written in assembly so the optimizer cannot fold the loop away or alter its
/// cycle count. `sub #1, {i}` decrements (setting Z when it reaches 0) and
/// `jne` branches back while nonzero.
#[inline]
fn busy_loop(iters: u16) {
    if iters == 0 {
        return;
    }
    // SAFETY: a pure register countdown — no memory or stack access.
    unsafe {
        core::arch::asm!(
            "2:",
            "sub #1, {i}",
            "jne 2b",
            i = inout(reg) iters => _,
            options(nomem, nostack),
        );
    }
}
