//! RTC_B real-time clock — a battery-/supercap-backed **calendar** in hardware.
//!
//! The RTC_B keeps wall-clock date and time (year, month, day, weekday, hour,
//! minute, second) in dedicated registers that advance once per second entirely
//! on their own — including through LPM3.5 deep sleep, on microamps — so the CPU
//! can be off for hours and still wake knowing the real time. It is the
//! *timekeeping* peer of [`crate::timer`] (which counts elapsed ticks, not
//! calendar time) and [`crate::delay`] (which just burns time).
//!
//! # The 32 768 Hz requirement
//!
//! The calendar advances by counting a **32 768 Hz** clock and dividing it by
//! 32 768 (a chain of prescalers) to get exactly one tick per second. On this
//! part that source is the **LFXT crystal on ACLK** — the datasheet is explicit
//! that "RTC is clocked by XT1". So the RTC only keeps correct time when ACLK is
//! the 32.768 kHz crystal: bring the clock tree up with
//! [`crate::clocks::configure_low_power`], which starts LFXT. If the crystal is
//! absent and ACLK falls back to the imprecise VLO, the seconds would be wrong —
//! so [`Rtc::new`] **refuses to start** unless `clocks.aclk()` is 32 768 Hz,
//! returning [`Error::ClockNot32768`]. (The default [`crate::clocks::configure`]
//! profile parks ACLK on the VLO, so it will *not* drive a correct RTC.)
//!
//! # Binary vs. BCD
//!
//! The calendar registers can hold their fields as **binary** or as packed
//! **BCD** (`RTCBCD`). This driver uses **binary** (`RTCBCD = 0`), so every field
//! is a plain integer: `second` is `0..=59`, `year` is the full number like
//! `2026`. The hour register is 24-hour (`0..=23`).
//!
//! # Reading without tearing
//!
//! The seven calendar registers update together once a second, so a naive read
//! can land mid-increment and return e.g. `01:59:60`. The hardware exposes
//! **`RTCRDY`**: it is 1 while the registers are static and safe to read, and
//! drops for one ACLK period around each one-second update. [`Rtc::now`] reads
//! only while `RTCRDY` stays set across the whole read, retrying otherwise, so it
//! never returns a torn value.
//!
//! # The alarm
//!
//! RTC_B has one hardware alarm: four byte-wide registers (minute, hour,
//! day-of-week, day-of-month), each with a per-field enable, compared against
//! the calendar at every minute increment — all enabled fields matching
//! latches `RTCAIFG` (once). The enabled subset picks the recurrence
//! (minute-only = hourly, minute+hour = daily, …). Program it with
//! [`Rtc::set_alarm`] ([`Alarm`]'s validation/encoding math lives in
//! `rtc_alarm.rs` and is host-tested), poll it with [`alarm_irq_pending`], or
//! get the `RTC` vector via [`Rtc::enable_alarm_interrupt`] ([`read_iv`]
//! returns `0x06`). Because the RTC counts LFXT on ACLK, an alarm wakes LPM3.
//!
//! # No `embedded-hal` trait?
//!
//! `embedded-hal` 1.0 has **no RTC/clock trait** — its modules are `digital`,
//! `i2c`, `spi`, `pwm`, and `delay` only. The ecosystem's de-facto abstraction
//! lives in the separate **`rtcc`** crate (`DateTimeAccess`/`Rtcc`), which is
//! built on `chrono`'s date types. We deliberately don't pull that in: `chrono`
//! is heavyweight for this part's 48 KB FRAM budget (the same reason
//! [`crate::serial`] avoids `core::fmt`). Instead this API mirrors the *shape*
//! `rtcc` would use — a plain [`DateTime`] value object and `set`/`now` — so a
//! `rtcc::Rtcc` impl could be added behind a feature later without changing it.
//!
//! # Example
//!
//! ```ignore
//! let clocks = hal::clocks::configure_low_power(p.cs); // ACLK = LFXT 32.768 kHz
//! hal::gpio::unlock_pins(&p.pmm);
//! let start = DateTime { year: 2026, month: 6, day: 27, weekday: 6,
//!                        hour: 9, minute: 30, second: 0 };
//! let rtc = Rtc::new(p.rtc_b_real_time_clock, &clocks, &start).unwrap();
//! loop {
//!     let t = rtc.now();           // never torn
//!     // print t.hour:t.minute:t.second ...
//! }
//! ```

use crate::clocks::Clocks;
use crate::pac;

// The alarm field-validation/register-encoding math lives in `rtc_alarm.rs`
// (dependency-free, host-tested in `unit_tests/`); re-exported here so
// consumers only ever see `hal::rtc::Alarm`.
pub use crate::rtc_alarm::{alarm_matches, Alarm, AlarmError};

/// A wall-clock date and time, all fields **binary** (not BCD).
///
/// Field ranges follow the RTC_B calendar: `weekday` is `0..=6` with whatever
/// convention you assign (the hardware just counts 0→6→0), `hour` is 24-hour.
/// The driver writes these verbatim and does not validate them, so pass legal
/// values.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DateTime {
    /// Full year, `0..=4095` (e.g. `2026`).
    pub year: u16,
    /// Month, `1..=12`.
    pub month: u8,
    /// Day of month, `1..=31`.
    pub day: u8,
    /// Day of week, `0..=6`.
    pub weekday: u8,
    /// Hour, `0..=23` (24-hour).
    pub hour: u8,
    /// Minute, `0..=59`.
    pub minute: u8,
    /// Second, `0..=59`.
    pub second: u8,
}

/// Why [`Rtc::new`] (or [`Rtc::attach`]) declined to hand over the clock.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// ACLK is not 32 768 Hz, so the calendar would not advance at one Hz.
    /// Configure the clock tree with [`crate::clocks::configure_low_power`] so
    /// ACLK is the LFXT crystal. See the module docs.
    ClockNot32768,
}

/// A periodic time event the RTC can interrupt on (`RTCTEV`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Event {
    /// At every change of minute (`00` seconds).
    MinuteChanged,
    /// At every change of hour (`00:00`).
    HourChanged,
    /// At midnight (`00:00:00`).
    Midnight,
    /// At noon (`12:00:00`).
    Noon,
}

/// The RTC_B calendar clock. Owns the PAC peripheral; configured once at
/// construction, then read with [`now`](Rtc::now).
pub struct Rtc {
    rtc: pac::RtcBRealTimeClock,
}

impl Rtc {
    /// Configure the RTC_B in binary calendar mode, set it to `init`, and start
    /// it.
    ///
    /// Returns [`Error::ClockNot32768`] unless ACLK is 32 768 Hz (see the module
    /// docs) — the one external precondition for correct timekeeping. The
    /// calendar is loaded while the counter is **held** (`RTCHOLD = 1`, which is
    /// also the reset state and the only time the registers are writable), then
    /// released so it begins advancing from `init`.
    pub fn new(
        rtc: pac::RtcBRealTimeClock,
        clocks: &Clocks,
        init: &DateTime,
    ) -> Result<Self, Error> {
        if clocks.aclk() != 32_768 {
            return Err(Error::ClockNot32768);
        }

        // Hold the counter and select binary mode while we load the calendar
        // (the time registers are writable only while held).
        rtc.rtcctl01()
            .modify(|_, w| w.rtchold().set_bit().rtcbcd().clear_bit());

        let this = Rtc { rtc };
        this.write_calendar(init);

        // Release: the calendar starts advancing from the loaded value.
        this.rtc.rtcctl01().modify(|_, w| w.rtchold().clear_bit());

        Ok(this)
    }

    /// Adopt the calendar that survived LPM3.5 and set it running again — the
    /// wake-path counterpart of [`Rtc::new`].
    ///
    /// The RTC_B lives in its own always-powered domain, so it keeps counting
    /// straight through LPM3.5's regulator-off sleep — but the wake is a BOR
    /// reset, so the rebooted program holds no [`Rtc`] value, and [`Rtc::new`]
    /// would destroy exactly what survived (it rewrites the calendar).
    /// `attach` adopts it instead, honoring what the wake actually does to the
    /// module (hardware-observed on this part, 2026-07-04): the **calendar
    /// contents survive frozen** — the wake re-asserts `RTCHOLD`, halting the
    /// count at the wake moment — the wake's interrupt flag (e.g. `RTCTEVIFG`,
    /// see [`event_irq_pending`]) stays latched as evidence, and the interrupt
    /// *enable* bits come back cleared. So `attach` verifies ACLK is 32 768 Hz
    /// again (re-run [`crate::clocks::configure_low_power`] first — the
    /// crystal never stopped, its pins being latched, so it settles at once)
    /// and **releases `RTCHOLD`** so time resumes.
    ///
    /// Two consequences to plan around:
    ///
    /// - The calendar stands still from the wake until this call — attach
    ///   early in boot, or each wake silently loses wall-clock time (up to
    ///   a second even when prompt, since the sub-second prescaler state is
    ///   not recoverable).
    /// - Re-arm any wake interrupt (e.g. [`enable_event_interrupt`]
    ///   (Rtc::enable_event_interrupt)) before the next LPM3.5 entry; the
    ///   enables do not survive the wake.
    ///
    /// Calling this on a genuinely cold RTC (nothing survived) is not
    /// detectable from the registers — `RTCHOLD` reads 1 in both cases — and
    /// would set a reset-value calendar running. Gate the call on the reset
    /// reason instead: only attach when `SYSRSTIV` reported
    /// [`Lpm5WakeUp`](crate::sys::ResetReason::Lpm5WakeUp).
    pub fn attach(rtc: pac::RtcBRealTimeClock, clocks: &Clocks) -> Result<Self, Error> {
        if clocks.aclk() != 32_768 {
            return Err(Error::ClockNot32768);
        }
        // Release the hold the wake re-asserted; the calendar resumes from
        // the preserved (frozen-at-wake) value. Binary/BCD mode survived the
        // wake, so unlike `new` there is nothing else to program.
        rtc.rtcctl01().modify(|_, w| w.rtchold().clear_bit());
        Ok(Rtc { rtc })
    }

    /// Overwrite the calendar with `dt` (e.g. to set the time from an external
    /// source). Holds the counter for the update and restarts it afterward.
    pub fn set(&mut self, dt: &DateTime) {
        self.rtc.rtcctl01().modify(|_, w| w.rtchold().set_bit());
        self.write_calendar(dt);
        self.rtc.rtcctl01().modify(|_, w| w.rtchold().clear_bit());
    }

    /// Read the current date and time, never torn by the one-second update.
    ///
    /// Spins until `RTCRDY` is set, takes one snapshot, and confirms `RTCRDY` is
    /// still set — guaranteeing no one-second increment occurred mid-read (see
    /// the module docs). At 32 768 Hz the unsafe window is ~30 µs per second, so
    /// this almost always reads on the first try.
    pub fn now(&self) -> DateTime {
        loop {
            if self.ready() {
                let snapshot = self.read_calendar();
                if self.ready() {
                    return snapshot;
                }
            }
        }
    }

    /// Whether the calendar registers are currently static and safe to read
    /// (`RTCRDY`).
    pub fn ready(&self) -> bool {
        self.rtc.rtcctl01().read().rtcrdy().bit_is_set()
    }

    /// Enable the **time-event** interrupt for `event` (`RTCTEV` + `RTCTEVIE`).
    ///
    /// Fires the single **`RTC`** interrupt vector (`pac::Interrupt::RTC`) at the
    /// chosen calendar boundary. The application defines that ISR, enables
    /// interrupts globally (GIE), and clears the flag — reading `RTCIV`
    /// ([`read_iv`]) returns the source and auto-clears it, or call
    /// [`clear_event_irq`].
    pub fn enable_event_interrupt(&self, event: Event) {
        self.rtc.rtcctl01().modify(|_, w| {
            match event {
                Event::MinuteChanged => w.rtctev().rtctev_0(),
                Event::HourChanged => w.rtctev().rtctev_1(),
                Event::Midnight => w.rtctev().rtctev_2(),
                Event::Noon => w.rtctev().rtctev_3(),
            };
            w.rtctevie().set_bit()
        });
    }

    /// Enable the **once-per-second** interrupt (`RTCRDYIE`), which fires on the
    /// `RTC` vector each time the calendar finishes an update. Useful for a
    /// 1 Hz tick (clock display, seconds blink).
    pub fn enable_second_interrupt(&self) {
        self.rtc.rtcctl01().modify(|_, w| w.rtcrdyie().set_bit());
    }

    /// Program the **alarm**: `RTCAIFG` latches when the calendar reaches an
    /// instant where every enabled field of `alarm` matches (see [`Alarm`] —
    /// the enabled subset picks the recurrence: minute-only = hourly,
    /// minute+hour = daily, and so on).
    ///
    /// Follows SLAU367P's reprogramming procedure: the alarm interrupt enable
    /// is cleared first (so no ISR can fire off a half-written alarm), the
    /// four alarm registers are written, and `RTCAIFG` is cleared **last** —
    /// a mix of old and new fields can spuriously match during the update (the
    /// comparison runs at each minute increment), and the trailing clear
    /// scrubs any such latch. Consequence: the alarm is armed for matches
    /// *after* this call returns; poll [`alarm_irq_pending`] or call
    /// [`enable_alarm_interrupt`](Rtc::enable_alarm_interrupt) to be notified.
    /// The calendar keeps running throughout (the alarm registers are not
    /// behind `RTCHOLD`).
    ///
    /// Returns the field that was out of range ([`AlarmError`]) with the
    /// hardware untouched; an alarm with no enabled field is rejected as
    /// [`AlarmError::NoFieldEnabled`] since it could never fire.
    pub fn set_alarm(&mut self, alarm: &Alarm) -> Result<(), AlarmError> {
        let regs = crate::rtc_alarm::encode_alarm(alarm)?;
        self.rtc.rtcctl01().modify(|_, w| w.rtcaie().clear_bit());
        self.rtc.rtcamin().write(|w| unsafe { w.bits(regs.minute) });
        self.rtc.rtcahour().write(|w| unsafe { w.bits(regs.hour) });
        self.rtc.rtcadow().write(|w| unsafe { w.bits(regs.weekday) });
        self.rtc.rtcaday().write(|w| unsafe { w.bits(regs.day) });
        self.rtc.rtcctl01().modify(|_, w| w.rtcaifg().clear_bit());
        Ok(())
    }

    /// Disarm the alarm entirely: clears the interrupt enable, every field's
    /// `AE` bit (so nothing is compared anymore), and a pending `RTCAIFG`.
    pub fn disable_alarm(&mut self) {
        self.rtc.rtcctl01().modify(|_, w| w.rtcaie().clear_bit());
        self.rtc.rtcamin().write(|w| unsafe { w.bits(0) });
        self.rtc.rtcahour().write(|w| unsafe { w.bits(0) });
        self.rtc.rtcadow().write(|w| unsafe { w.bits(0) });
        self.rtc.rtcaday().write(|w| unsafe { w.bits(0) });
        self.rtc.rtcctl01().modify(|_, w| w.rtcaifg().clear_bit());
    }

    /// Enable the **alarm** interrupt (`RTCAIE`): a programmed alarm match
    /// ([`set_alarm`](Rtc::set_alarm)) fires the single `RTC` vector, where
    /// [`read_iv`] returns `0x06` (hardware-observed 2026-07-07 — one slot
    /// below where a casual reading of the generic RTC_B chapter puts it; see
    /// [`read_iv`] for the full table). An already-latched `RTCAIFG` fires the ISR
    /// immediately on enable — call this right after `set_alarm` (which
    /// leaves the flag clean) to only hear about future matches. The RTC
    /// keeps its clock (LFXT on ACLK) in LPM3, so an alarm wakes LPM3 given a
    /// `#[interrupt(wake_cpu)]` handler.
    pub fn enable_alarm_interrupt(&self) {
        self.rtc.rtcctl01().modify(|_, w| w.rtcaie().set_bit());
    }

    /// Release the underlying peripheral. The calendar is left running.
    pub fn free(self) -> pac::RtcBRealTimeClock {
        self.rtc
    }

    // -- internals --

    /// Load all seven calendar registers. Caller must hold the counter
    /// (`RTCHOLD = 1`). Binary mode: every field is written verbatim; the year
    /// register is 16-bit, the rest 8-bit.
    fn write_calendar(&self, dt: &DateTime) {
        // SAFETY: `bits` on these calendar registers is an unsafe writer (the
        // upper bits are reserved); the values are plain binary fields.
        self.rtc.rtcsec().write(|w| unsafe { w.bits(dt.second) });
        self.rtc.rtcmin().write(|w| unsafe { w.bits(dt.minute) });
        self.rtc.rtchour().write(|w| unsafe { w.bits(dt.hour) });
        self.rtc.rtcdow().write(|w| unsafe { w.bits(dt.weekday) });
        self.rtc.rtcday().write(|w| unsafe { w.bits(dt.day) });
        self.rtc.rtcmon().write(|w| unsafe { w.bits(dt.month) });
        self.rtc.rtcyear().write(|w| unsafe { w.bits(dt.year) });
    }

    /// One raw snapshot of the calendar registers (may be torn — callers go
    /// through [`now`](Rtc::now), which guards it with `RTCRDY`).
    fn read_calendar(&self) -> DateTime {
        DateTime {
            second: self.rtc.rtcsec().read().bits(),
            minute: self.rtc.rtcmin().read().bits(),
            hour: self.rtc.rtchour().read().bits(),
            weekday: self.rtc.rtcdow().read().bits(),
            day: self.rtc.rtcday().read().bits(),
            month: self.rtc.rtcmon().read().bits(),
            year: self.rtc.rtcyear().read().bits(),
        }
    }
}

/// Read the RTC interrupt vector register `RTCIV` from inside the `RTC` ISR.
///
/// The RTC's several sources share one vector; reading `RTCIV` returns a small
/// number identifying the highest-priority pending source and **auto-clears that
/// flag**. Provided as a free function because the ISR does not own the [`Rtc`].
/// Values (TI's `msp430fr5969.h` `RTCIV_*` constants; the alarm slot
/// hardware-observed 2026-07-07): `0x02` = `RTCRDYIFG` (per-second), `0x04` =
/// `RTCTEVIFG` (time event), `0x06` = `RTCAIFG` (alarm), `0x08`/`0x0A` =
/// prescaler 0/1, `0x0C` = oscillator fault (the *lowest*-priority slot, not
/// the highest).
pub fn read_iv() -> u16 {
    // SAFETY: a stolen handle reading the self-clearing RTCIV — the architected
    // way to service the shared RTC vector; touches no state the Rtc owner writes.
    let rtc = unsafe { pac::RtcBRealTimeClock::steal() };
    rtc.rtciv().read().bits()
}

/// Is the time-event interrupt flag (`RTCTEVIFG`) latched?
///
/// The direct-flag sibling of [`read_iv`], for the one situation where the IV
/// register cannot answer: after an **LPM3.5 wake**, the event flag that woke
/// the part is still latched but its *enable* bit was cleared by the wake —
/// and `RTCIV` only reports enabled sources, so it reads 0 (hardware-observed
/// on this part). Non-destructive: reading the flag does not clear it (unlike
/// a `read_iv` consume), so it can be checked before deciding how to handle
/// the wake; clear it with [`clear_event_irq`] when done.
pub fn event_irq_pending() -> bool {
    // SAFETY: a stolen handle reading one interrupt flag — no state written.
    let rtc = unsafe { pac::RtcBRealTimeClock::steal() };
    rtc.rtcctl01().read().rtctevifg().bit_is_set()
}

/// Clear the time-event interrupt flag (`RTCTEVIFG`) from the `RTC` ISR, for
/// handlers that do not read [`read_iv`].
pub fn clear_event_irq() {
    // SAFETY: a stolen handle clearing only RTCTEVIFG via read-modify-write —
    // disjoint from the calendar/control bits the Rtc owner manages.
    let rtc = unsafe { pac::RtcBRealTimeClock::steal() };
    rtc.rtcctl01().modify(|_, w| w.rtctevifg().clear_bit());
}

/// Is the alarm interrupt flag (`RTCAIFG`) latched?
///
/// The alarm sibling of [`event_irq_pending`], serving the same two callers:
/// polled use (an alarm armed by [`Rtc::set_alarm`] with the interrupt left
/// off — the flag latches regardless of `RTCAIE`), and the post-LPM3.5-wake
/// check, where the flag that woke the part is still latched but its enable
/// was cleared by the wake so `RTCIV` reads 0. Non-destructive; clear with
/// [`clear_alarm_irq`].
pub fn alarm_irq_pending() -> bool {
    // SAFETY: a stolen handle reading one interrupt flag — no state written.
    let rtc = unsafe { pac::RtcBRealTimeClock::steal() };
    rtc.rtcctl01().read().rtcaifg().bit_is_set()
}

/// Clear the alarm interrupt flag (`RTCAIFG`) — for polled alarm consumers
/// and for `RTC` ISR handlers that do not read [`read_iv`]. The alarm stays
/// armed (the `AE` bits are untouched) and will latch again at the next
/// matching minute increment.
pub fn clear_alarm_irq() {
    // SAFETY: a stolen handle clearing only RTCAIFG via read-modify-write —
    // disjoint from the calendar/control bits the Rtc owner manages.
    let rtc = unsafe { pac::RtcBRealTimeClock::steal() };
    rtc.rtcctl01().modify(|_, w| w.rtcaifg().clear_bit());
}
