// Pure RTC_B alarm-register math for the `rtc` module.
//
// Like `mpu_seg.rs` and `capture_math.rs`, this file is intentionally
// dependency-free (pure `core` integer arithmetic, no PAC/HAL types) so the
// exact same source can be `include!`d by the host-side test crate in
// `unit_tests/`. The `rtc::Rtc` alarm methods are a thin wrapper over these
// conversions — a regression here (a shifted AE bit, an accepted out-of-range
// field) fails the host tests without a board on the desk. Do not add
// external `use`s. (Regular `//` comments, not `//!`, so the file can be
// `include!`d mid-crate.)
//
// # The hardware's alarm model (SLAU367P, "Real-Time Clock Alarm")
//
// RTC_B has one user-programmable alarm made of four byte-wide registers —
// `RTCAMIN`, `RTCAHOUR`, `RTCADOW`, `RTCADAY` — each holding a compare value
// in its low bits plus an **alarm enable** (`AE`) in bit 7. Every time the
// calendar's *minute* increments, the hardware compares each **enabled**
// field against the calendar; when *all* enabled fields match, `RTCAIFG`
// latches (once, at that increment — while the match persists no further
// minute-increment-into-match can occur, so it cannot refire until the time
// moves away and back). Fields with `AE` clear are wildcards, so the enabled
// subset picks the recurrence: minute-only = hourly, minute+hour = daily,
// +weekday = weekly, +day-of-month = monthly. An alarm with **no** field
// enabled can never fire — the driver rejects it rather than program a
// well-formed no-op.
//
// The registers hold **binary** values here because the driver runs the RTC
// in binary mode (`RTCBCD = 0`, see `rtc`); in BCD mode the same registers
// would hold packed BCD. The calendar has no notion of month lengths in the
// comparison: `day: Some(31)` simply never matches in a 30-day month.

/// One RTC_B alarm: a compare value per calendar field, each optionally
/// enabled. `None` = wildcard (field not compared), `Some(v)` = must equal
/// `v`. The enabled subset sets the recurrence — see the module docs. At
/// least one field must be `Some` or [`encode_alarm`] refuses.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Alarm {
    /// Minute to match, `0..=59`.
    pub minute: Option<u8>,
    /// Hour to match, `0..=23` (24-hour, matching [`DateTime`]'s convention).
    pub hour: Option<u8>,
    /// Day of week to match, `0..=6` (whatever convention the calendar's
    /// `weekday` was loaded with — the hardware just compares numbers).
    pub weekday: Option<u8>,
    /// Day of month to match, `1..=31`.
    pub day: Option<u8>,
}

impl Alarm {
    /// A daily alarm at `hour:minute:00` — the most common shape.
    pub const fn daily_at(hour: u8, minute: u8) -> Alarm {
        Alarm { minute: Some(minute), hour: Some(hour), weekday: None, day: None }
    }

    /// An hourly alarm at `minute` past every hour.
    pub const fn hourly_at(minute: u8) -> Alarm {
        Alarm { minute: Some(minute), hour: None, weekday: None, day: None }
    }
}

/// Why an [`Alarm`] cannot be programmed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AlarmError {
    /// `minute` is not `0..=59`.
    MinuteOutOfRange,
    /// `hour` is not `0..=23`.
    HourOutOfRange,
    /// `weekday` is not `0..=6`.
    WeekdayOutOfRange,
    /// `day` is not `1..=31`.
    DayOutOfRange,
    /// Every field is `None`. With no `AE` bit set the hardware can never
    /// fire, so this is a caller mistake, not a configuration.
    NoFieldEnabled,
}

// Bit 7 of each alarm register: this field participates in the comparison.
pub(crate) const AE: u8 = 0x80;

/// The four raw alarm-register values [`encode_alarm`] produces, in
/// hardware order: `RTCAMIN`, `RTCAHOUR`, `RTCADOW`, `RTCADAY`. A disabled
/// field encodes as `0x00` (AE clear; the compare value is then ignored).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct AlarmRegs {
    pub minute: u8,
    pub hour: u8,
    pub weekday: u8,
    pub day: u8,
}

/// Validate an [`Alarm`] and encode it into its four register values —
/// enabled fields get `AE | value`, disabled fields `0x00`.
pub(crate) fn encode_alarm(alarm: &Alarm) -> Result<AlarmRegs, AlarmError> {
    if alarm.minute.is_none()
        && alarm.hour.is_none()
        && alarm.weekday.is_none()
        && alarm.day.is_none()
    {
        return Err(AlarmError::NoFieldEnabled);
    }
    Ok(AlarmRegs {
        minute: encode_field(alarm.minute, 0, 59, AlarmError::MinuteOutOfRange)?,
        hour: encode_field(alarm.hour, 0, 23, AlarmError::HourOutOfRange)?,
        weekday: encode_field(alarm.weekday, 0, 6, AlarmError::WeekdayOutOfRange)?,
        day: encode_field(alarm.day, 1, 31, AlarmError::DayOutOfRange)?,
    })
}

// One register: None → 0 (AE clear), Some(in-range) → AE | value.
fn encode_field(value: Option<u8>, lo: u8, hi: u8, err: AlarmError) -> Result<u8, AlarmError> {
    match value {
        None => Ok(0),
        Some(v) if v >= lo && v <= hi => Ok(AE | v),
        Some(_) => Err(err),
    }
}

/// Would `alarm` match the calendar instant (`minute`, `hour`, `weekday`,
/// `day`)? The AND of all enabled fields, exactly as the hardware compares on
/// each minute increment; an alarm with nothing enabled matches nothing.
/// Useful for predicting when a programmed alarm will fire (the hardware
/// latches `RTCAIFG` on the minute increment where this first becomes true).
pub fn alarm_matches(alarm: &Alarm, minute: u8, hour: u8, weekday: u8, day: u8) -> bool {
    let any_enabled = alarm.minute.is_some()
        || alarm.hour.is_some()
        || alarm.weekday.is_some()
        || alarm.day.is_some();
    any_enabled
        && alarm.minute.map_or(true, |m| m == minute)
        && alarm.hour.map_or(true, |h| h == hour)
        && alarm.weekday.map_or(true, |w| w == weekday)
        && alarm.day.map_or(true, |d| d == day)
}
