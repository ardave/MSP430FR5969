//! Free-running counter for measuring elapsed time (durations / intervals).
//!
//! This is the *measurement* counterpart to [`crate::delay`]: where `Delay`
//! spends a known number of cycles to make time pass, [`Counter`] reads a
//! hardware counter to find out how much time *has* passed between two points.
//!
//! # How a duration is measured
//!
//! A Timer_A block contains a 16-bit counter, `TAxR`, that increments on every
//! tick of a clock you select. Put it in **continuous mode** and it free-runs:
//! 0, 1, 2, … 0xFFFF, 0, 1, … forever, with no CPU involvement. To time
//! something you snapshot the counter before and after and subtract:
//!
//! ```ignore
//! let start = counter.now();
//! do_the_thing();
//! let elapsed_ticks = counter.now().wrapping_sub(start);
//! let us = counter.ticks_to_us(elapsed_ticks);
//! ```
//!
//! `wrapping_sub` is load-bearing: the counter is modular (16-bit), so as long
//! as the interval is shorter than one full period the subtraction is correct
//! *even when the counter wraps through zero in between*. `60000.wrapping_sub(
//! 65000) == 61072` ticks — exactly the right answer across one rollover.
//! [`Counter::elapsed_since`] wraps this for you.
//!
//! # Resolution vs. range — the one real decision
//!
//! The counter is 16 bits, so it rolls over every 65536 ticks. The tick rate is
//! the selected clock divided by [`Divider`], and that single choice trades
//! timing resolution against the longest interval you can measure before the
//! counter laps itself and the subtraction silently lies:
//!
//! | SMCLK | Divider | Tick rate | Resolution | Wraps after |
//! |-------|---------|-----------|------------|-------------|
//! | 8 MHz | ÷1      | 8 MHz     | 125 ns     | 8.19 ms     |
//! | 8 MHz | ÷8      | 1 MHz     | 1 µs       | 65.5 ms     |
//! | 1 MHz | ÷1      | 1 MHz     | 1 µs       | 65.5 ms     |
//!
//! This module measures **single intervals shorter than one wrap period**. To
//! time anything longer you must count overflows (the `TAIFG` flag / the
//! `TIMERx_A1` interrupt) and assemble a wider tick count — that is deliberately
//! left to a later milestone (see `TIMING-MEASUREMENT.md`), because it needs the
//! project's first real ISR.
//!
//! # Clock source
//!
//! [`Counter::new_smclk`] sources the counter from **SMCLK**, the same clock the
//! UART's BRCLK runs on. SMCLK is gated off in LPM3, so this counter does *not*
//! run in deep sleep — that is also a job for the later ACLK/overflow milestone.
//! It reads the tick rate from [`Clocks`] (single source of truth) exactly as
//! [`crate::delay::Delay`] reads MCLK, so the tick↔time math tracks whichever
//! clock profile you configured.

use crate::clocks::Clocks;
use crate::pac;

/// Counter input divider applied to the selected clock (the `ID` field of
/// `TAxCTL`). Lower divisors give finer resolution but a shorter span before the
/// 16-bit counter wraps; see the table in the module docs.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Divider {
    /// ÷1 — finest resolution, shortest range.
    Div1,
    /// ÷2.
    Div2,
    /// ÷4.
    Div4,
    /// ÷8 — coarsest resolution, longest range.
    Div8,
}

impl Divider {
    /// The integer divisor this variant represents.
    const fn value(self) -> u32 {
        match self {
            Divider::Div1 => 1,
            Divider::Div2 => 2,
            Divider::Div4 => 4,
            Divider::Div8 => 8,
        }
    }
}

/// A free-running 16-bit up-counter on Timer0_A3, used as a timestamp source.
///
/// Consumes the `Timer0A3` peripheral so the counter is owned and configured in
/// exactly one place. Snapshot it with [`now`](Counter::now); turn a tick delta
/// into real time with [`ticks_to_us`](Counter::ticks_to_us) /
/// [`ticks_to_ns`](Counter::ticks_to_ns).
pub struct Counter {
    timer: pac::Timer0A3,
    tick_hz: u32,
}

impl Counter {
    /// Configure Timer0_A3 as a free-running counter clocked from **SMCLK**
    /// divided by `div`, and start it.
    ///
    /// The resulting tick rate is `clocks.smclk() / div`, stored so the
    /// `ticks_to_*` helpers can convert without re-deriving it. Programs
    /// `TA0CTL` in one write: `TASSEL = SMCLK`, `ID = div`, `MC = continuous`,
    /// and `TACLR = 1` to reset the counter and the divider so timing starts
    /// from a clean zero.
    pub fn new_smclk(timer: pac::Timer0A3, clocks: &Clocks, div: Divider) -> Self {
        timer.ta0ctl().write(|w| {
            w.tassel().tassel_2(); // SMCLK
            match div {
                Divider::Div1 => w.id().id_0(),
                Divider::Div2 => w.id().id_1(),
                Divider::Div4 => w.id().id_2(),
                Divider::Div8 => w.id().id_3(),
            };
            w.mc().mc_2(); // continuous (free-run 0..=0xFFFF)
            w.taclr().set_bit() // reset counter + divider
        });

        Counter {
            timer,
            tick_hz: clocks.smclk() / div.value(),
        }
    }

    /// The counter's tick rate, in Hz.
    pub const fn tick_hz(&self) -> u32 {
        self.tick_hz
    }

    /// Snapshot the raw 16-bit counter value (a timestamp in ticks).
    ///
    /// Subtract two snapshots with [`u16::wrapping_sub`] (or use
    /// [`elapsed_since`](Counter::elapsed_since)) to get a tick delta that is
    /// correct across a single rollover.
    pub fn now(&self) -> u16 {
        self.timer.ta0r().read().bits()
    }

    /// Ticks elapsed since the `start` snapshot, valid for intervals shorter
    /// than one full counter period (see the resolution/range table).
    pub fn elapsed_since(&self, start: u16) -> u16 {
        self.now().wrapping_sub(start)
    }

    /// Convert a tick delta to microseconds.
    ///
    /// Done in `u64` so `ticks * 1_000_000` cannot overflow (the max,
    /// `65535 * 1_000_000`, exceeds `u32`); the divide brings it back into range.
    pub fn ticks_to_us(&self, ticks: u16) -> u32 {
        (ticks as u64 * 1_000_000 / self.tick_hz as u64) as u32
    }

    /// Convert a tick delta to nanoseconds.
    ///
    /// `ticks * 1_000_000_000` is up to ~6.5e13, hence `u64`. Resolution is
    /// still one tick — at 1 MHz this only ever reports whole microseconds.
    pub fn ticks_to_ns(&self, ticks: u16) -> u32 {
        (ticks as u64 * 1_000_000_000 / self.tick_hz as u64) as u32
    }

    /// Release the underlying timer peripheral (stops owning it). The counter is
    /// left running; stop it via the returned peripheral if desired.
    pub fn free(self) -> pac::Timer0A3 {
        self.timer
    }
}
