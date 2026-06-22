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
//! Out of the box [`now`](Counter::now) measures **single intervals shorter
//! than one wrap period**. To time anything longer, enable the overflow
//! interrupt with [`Counter::enable_overflow_interrupt`] and have the
//! `TIMER0_A1` ISR tally rollovers in a shared counter; then
//! [`Counter::now64`] assembles those tallies with `TAxR` into a 32-bit
//! timestamp (~71 minutes before *it* wraps, at the 1 MHz tick). The ISR, the
//! shared counter, and enabling interrupts globally live in the application —
//! see `hal_consumer` for a worked example.
//!
//! # Hardware capture
//!
//! A software [`now`](Counter::now) read is taken whenever the CPU reaches the
//! instruction — so interrupt latency and scheduling jitter land *in* the
//! measurement. A capture/compare channel in **capture mode** instead latches
//! `TAxR` into `TAxCCR1` the moment a selected edge arrives, in hardware, so the
//! timestamp reflects the *event* regardless of when software reads it.
//! [`configure_capture`](Counter::configure_capture) sets this up on CCR1, and
//! [`software_capture`](Counter::software_capture) triggers one without any
//! external pin by toggling the internal `CCIS` input GND→VCC. (A true external
//! edge would route a pin to the channel's `CCIxA`/`CCIxB` input instead — on
//! this part `TA0.1`/CCI1A is P1.0, which is the green LED, so the pin route is
//! left as a later exercise.)
//!
//! # The overflow read race
//!
//! Reading a 32-bit timestamp out of a 16-bit counter plus a software high word
//! is not atomic: the counter can roll over (setting `TAIFG`) in the window
//! between sampling the high word and the low word. [`now64`](Counter::now64)
//! must therefore be called inside a critical section (interrupts masked, so the
//! ISR cannot run mid-read) and reconciles a *pending-but-uncounted* overflow
//! itself by checking `TAIFG` — see its docs.
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
    /// than one full counter period (see the resolution/range table). For
    /// longer intervals use [`now64`](Counter::now64) snapshots instead.
    pub fn elapsed_since(&self, start: u16) -> u16 {
        self.now().wrapping_sub(start)
    }

    /// Convert a tick delta to microseconds.
    ///
    /// Takes a `u32` tick count so it serves both the 16-bit
    /// [`elapsed_since`](Counter::elapsed_since) path (widen the `u16`) and the
    /// 32-bit [`now64`](Counter::now64) path. Done in `u64` so `ticks *
    /// 1_000_000` cannot overflow; the divide brings it back into range. The
    /// `u32` result holds up to ~4295 s (≈71 min) of microseconds.
    pub fn ticks_to_us(&self, ticks: u32) -> u32 {
        (ticks as u64 * 1_000_000 / self.tick_hz as u64) as u32
    }

    /// Convert a tick delta to nanoseconds. Resolution is still one tick — at
    /// 1 MHz this only ever reports whole microseconds. The `u32` result holds
    /// only ~4.3 s of nanoseconds, so use it for short deltas.
    pub fn ticks_to_ns(&self, ticks: u32) -> u32 {
        (ticks as u64 * 1_000_000_000 / self.tick_hz as u64) as u32
    }

    /// Enable the counter-overflow interrupt (`TAIE`).
    ///
    /// Once enabled, each time `TAxR` rolls over 0xFFFF→0x0000 the hardware sets
    /// `TAIFG` and fires the **`TIMER0_A1`** vector. The application must define
    /// that ISR (msp430-rt `#[interrupt] fn TIMER0_A1`), clear the flag from it
    /// with [`clear_overflow_irq`], tally the rollover in a shared counter, and
    /// enable interrupts globally (set GIE). See [`now64`](Counter::now64).
    pub fn enable_overflow_interrupt(&self) {
        self.timer.ta0ctl().modify(|_, w| w.taie().set_bit());
    }

    /// Whether a counter overflow is pending (`TAIFG` set).
    pub fn overflow_pending(&self) -> bool {
        self.timer.ta0ctl().read().taifg().bit_is_set()
    }

    /// Assemble a 32-bit timestamp from the software `overflows` tally and the
    /// hardware counter. **Call inside a critical section**, passing the current
    /// value of the ISR-maintained overflow counter (read under the same CS).
    ///
    /// With interrupts masked the ISR cannot run, so a rollover that happens
    /// while we are reading would set `TAIFG` without being tallied. We detect
    /// that and fold it in: if `TAIFG` is set, an uncounted overflow occurred,
    /// so we add one to `overflows` and re-read `TAxR` *after* observing the
    /// flag — guaranteeing the low half is the post-wrap (small) value that
    /// matches the incremented high half. (The ISR will also count this same
    /// overflow once the CS ends; that is fine — it only updates the software
    /// tally for *next* time, it does not double-count this reading.)
    pub fn now64(&self, overflows: u16) -> u32 {
        let mut ovf = overflows;
        let mut cnt = self.now();
        if self.overflow_pending() {
            ovf = ovf.wrapping_add(1);
            cnt = self.now();
        }
        ((ovf as u32) << 16) | cnt as u32
    }

    /// Configure capture/compare channel **CCR1** for software-triggered
    /// capture (no external pin).
    ///
    /// In *capture* mode (`CAP=1`) the hardware copies `TAxR` into `TAxCCR1` the
    /// instant a selected edge appears on the channel's capture input — the
    /// timestamp is frozen at the *event*, not at whenever software gets around
    /// to reading it. Here the input is the internal `CCIS` source rather than a
    /// pin: parking it at GND (`CCIS=2`) and later flipping it to VCC
    /// (`CCIS=3`) manufactures a rising edge in hardware, which is exactly what
    /// [`software_capture`](Counter::software_capture) does. `CM=rising` so only
    /// the GND→VCC flip captures (the re-arming VCC→GND flip is ignored), and
    /// `SCS=1` synchronizes the capture to the timer clock to avoid a race.
    pub fn configure_capture(&self) {
        self.timer.ta0cctl1().write(|w| {
            w.cap().set_bit(); // capture (not compare) mode
            w.scs().set_bit(); // synchronous capture
            w.cm().cm_1(); // capture on a rising edge
            w.ccis().ccis_2() // input = GND (armed for a GND→VCC rising edge)
        });
    }

    /// Software-trigger a capture and return the latched `TAxR` value.
    ///
    /// Drives the internal capture input GND→VCC (a rising edge the hardware
    /// latches), then returns it to GND to re-arm for next time. The two field
    /// writes also give the synchronous capture (`SCS`) a clock edge to complete
    /// before [`capture_value`](Counter::capture_value) reads `TAxCCR1`. Requires
    /// [`configure_capture`] first.
    pub fn software_capture(&self) -> u16 {
        self.timer.ta0cctl1().modify(|_, w| w.ccis().ccis_3()); // → VCC: rising edge → capture
        self.timer.ta0cctl1().modify(|_, w| w.ccis().ccis_2()); // → GND: re-arm (falling, ignored)
        self.capture_value()
    }

    /// Read the most recently captured value from `TAxCCR1`.
    pub fn capture_value(&self) -> u16 {
        self.timer.ta0ccr1().read().bits()
    }

    /// Whether a capture overflow occurred (`COV`): a new edge was captured
    /// before the previous value was read, so an event was missed. Clear it with
    /// [`clear_capture_overflow`](Counter::clear_capture_overflow).
    pub fn capture_overflowed(&self) -> bool {
        self.timer.ta0cctl1().read().cov().bit_is_set()
    }

    /// Clear the capture-overflow flag (`COV`).
    pub fn clear_capture_overflow(&self) {
        self.timer.ta0cctl1().modify(|_, w| w.cov().clear_bit());
    }

    /// Release the underlying timer peripheral (stops owning it). The counter is
    /// left running; stop it via the returned peripheral if desired.
    pub fn free(self) -> pac::Timer0A3 {
        self.timer
    }
}

/// Clear a pending Timer0_A3 overflow interrupt (`TAIFG`).
///
/// Intended to be called from the `TIMER0_A1` ISR, which does not own the
/// [`Counter`]; it therefore reaches `TA0CTL` by raw address (consistent with
/// the rest of this HAL) and clears only `TAIFG`, leaving the source/divider/
/// mode/enable bits intact. The read-modify-write cannot race the hardware
/// re-setting `TAIFG` — the next overflow is a full counter period away.
pub fn clear_overflow_irq() {
    // TA0CTL = Timer0_A3 base (0x0340); TAIFG is bit 0.
    const TA0CTL: usize = 0x0340;
    const TAIFG: u16 = 0x0001;
    unsafe {
        let p = TA0CTL as *mut u16;
        p.write_volatile(p.read_volatile() & !TAIFG);
    }
}
