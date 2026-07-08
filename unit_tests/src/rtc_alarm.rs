//! Host-side tests for `hal/src/rtc_alarm.rs` — the RTC_B alarm field
//! validation, `AE`-bit register encoding, and match predicate that
//! `hal::rtc::Rtc::set_alarm` delegates to.
//!
//! Anchors: each alarm register is its binary compare value with the alarm
//! enable in **bit 7** (SLAU367P, "Real-Time Clock Alarm"), a disabled field
//! encodes to `0x00`, and the hardware fires on the AND of all *enabled*
//! fields — so a shifted AE bit, an accepted out-of-range value, or
//! OR-instead-of-AND match semantics can't survive these tests.

include!("../../hal/src/rtc_alarm.rs");

// ---------------------------------------------------------------- encoding

#[test]
fn enabled_field_is_ae_plus_value() {
    // AE is bit 7: minute 30 encodes as 0x80 | 30 = 0x9E.
    let regs = encode_alarm(&Alarm { minute: Some(30), ..Default::default() }).unwrap();
    assert_eq!(regs.minute, 0x9E);
    // The other three fields are disabled: AE clear, value zero.
    assert_eq!(regs.hour, 0x00);
    assert_eq!(regs.weekday, 0x00);
    assert_eq!(regs.day, 0x00);
}

#[test]
fn each_field_lands_in_its_register() {
    // All four enabled at distinct values — a transposed register assignment
    // (minute value in the hour register, etc.) fails here.
    let alarm = Alarm { minute: Some(59), hour: Some(23), weekday: Some(6), day: Some(31) };
    let regs = encode_alarm(&alarm).unwrap();
    assert_eq!(regs.minute, 0x80 | 59); // 0xBB
    assert_eq!(regs.hour, 0x80 | 23); // 0x97
    assert_eq!(regs.weekday, 0x80 | 6); // 0x86
    assert_eq!(regs.day, 0x80 | 31); // 0x9F
}

#[test]
fn field_minimums_encode() {
    let alarm = Alarm { minute: Some(0), hour: Some(0), weekday: Some(0), day: Some(1) };
    let regs = encode_alarm(&alarm).unwrap();
    // Value 0 with AE set is a real alarm ("on the hour"), not a disabled
    // field — the AE bit is what distinguishes them.
    assert_eq!(regs.minute, 0x80);
    assert_eq!(regs.hour, 0x80);
    assert_eq!(regs.weekday, 0x80);
    assert_eq!(regs.day, 0x81);
}

#[test]
fn out_of_range_fields_are_rejected() {
    let ok = Alarm::daily_at(9, 30);
    assert!(encode_alarm(&ok).is_ok());
    assert_eq!(
        encode_alarm(&Alarm { minute: Some(60), ..ok }),
        Err(AlarmError::MinuteOutOfRange)
    );
    assert_eq!(
        encode_alarm(&Alarm { hour: Some(24), ..ok }),
        Err(AlarmError::HourOutOfRange)
    );
    assert_eq!(
        encode_alarm(&Alarm { weekday: Some(7), ..ok }),
        Err(AlarmError::WeekdayOutOfRange)
    );
    // Day of month is 1-based: 0 and 32 are both unrepresentable dates.
    assert_eq!(
        encode_alarm(&Alarm { day: Some(0), ..ok }),
        Err(AlarmError::DayOutOfRange)
    );
    assert_eq!(
        encode_alarm(&Alarm { day: Some(32), ..ok }),
        Err(AlarmError::DayOutOfRange)
    );
}

#[test]
fn all_disabled_is_rejected() {
    // No AE bit set can never fire — refused rather than programmed as a
    // well-formed no-op.
    assert_eq!(encode_alarm(&Alarm::default()), Err(AlarmError::NoFieldEnabled));
}

#[test]
fn convenience_constructors() {
    assert_eq!(
        Alarm::daily_at(9, 31),
        Alarm { minute: Some(31), hour: Some(9), weekday: None, day: None }
    );
    assert_eq!(
        Alarm::hourly_at(45),
        Alarm { minute: Some(45), hour: None, weekday: None, day: None }
    );
}

// ---------------------------------------------------------------- matching

#[test]
fn match_is_and_of_enabled_fields() {
    let alarm = Alarm::daily_at(9, 31);
    // Both enabled fields must match — one alone is not enough (OR semantics
    // would pass the middle two).
    assert!(alarm_matches(&alarm, 31, 9, 0, 1));
    assert!(!alarm_matches(&alarm, 31, 10, 0, 1));
    assert!(!alarm_matches(&alarm, 30, 9, 0, 1));
    assert!(!alarm_matches(&alarm, 30, 10, 0, 1));
}

#[test]
fn disabled_fields_are_wildcards() {
    // Hourly alarm: minute must match, everything else is ignored.
    let alarm = Alarm::hourly_at(45);
    assert!(alarm_matches(&alarm, 45, 0, 0, 1));
    assert!(alarm_matches(&alarm, 45, 23, 6, 31));
    assert!(!alarm_matches(&alarm, 44, 23, 6, 31));
}

#[test]
fn weekday_and_day_participate() {
    let weekly = Alarm { minute: Some(0), hour: Some(8), weekday: Some(1), day: None };
    assert!(alarm_matches(&weekly, 0, 8, 1, 15));
    assert!(!alarm_matches(&weekly, 0, 8, 2, 15));

    let monthly = Alarm { minute: Some(0), hour: Some(8), weekday: None, day: Some(15) };
    assert!(alarm_matches(&monthly, 0, 8, 2, 15));
    assert!(!alarm_matches(&monthly, 0, 8, 2, 16));
}

#[test]
fn empty_alarm_never_matches() {
    // Mirrors the hardware: with no AE bit set there is nothing to compare,
    // and the alarm never fires — not "everything matches".
    assert!(!alarm_matches(&Alarm::default(), 31, 9, 0, 1));
}
