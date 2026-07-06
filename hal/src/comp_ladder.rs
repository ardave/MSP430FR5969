// Pure Comp_E reference-ladder arithmetic.
//
// Like `baud.rs`, `ticks.rs`, and `adc_cal.rs`, this file is intentionally
// dependency-free (pure `core` integer arithmetic, no PAC/HAL types) so the
// exact same source can be `include!`d by the host-side test crate in
// `unit_tests/`. The `comp_e` threshold helpers are thin wrappers over these
// functions, so a regression in the math fails the host tests. Do not add
// external `use`s here.
// (Regular `//` comments, not `//!`, so the file can be `include!`d mid-crate.)

/// The number of taps on the Comp_E reference resistor string (`CEREF0`/
/// `CEREF1` are 5-bit fields).
pub const LADDER_TAPS: u8 = 32;

/// The voltage at ladder tap `tap` (0..=31), in millivolts, given the ladder's
/// source voltage in millivolts.
///
/// The resistor string divides its source into 32 equal steps and tap `n`
/// selects the **(n+1)/32** point (SLAU367 §26.2.3: `CEREF0 = 0` is 1/32, not
/// 0 V — a zero threshold would be useless, so the encoding starts one step
/// up; tap 31 is the full source voltage). Rounded to nearest (add half the
/// divisor before dividing). Widened to `u32`: the worst case,
/// `3700 mV × 32 ≈ 1.2e5`, is far inside `u32`.
pub fn ladder_millivolts(tap: u8, source_mv: u16) -> u16 {
    let tap = if tap >= LADDER_TAPS { LADDER_TAPS - 1 } else { tap };
    ((source_mv as u32 * (tap as u32 + 1) + LADDER_TAPS as u32 / 2) / LADDER_TAPS as u32) as u16
}

/// The ladder tap (0..=31) whose voltage is closest to `target_mv`, given the
/// ladder's source voltage in millivolts.
///
/// Inverts [`ladder_millivolts`]: `tap + 1 = round(32 × target / source)`,
/// clamped to the real tap range — a target at or below half a step selects
/// tap 0 (1/32), a target at or above the source selects tap 31 (32/32).
/// Ties round up (toward the higher tap). `source_mv == 0` (meaningless — the
/// ladder has no source) degenerates to tap 0 rather than dividing by zero.
pub fn tap_for_millivolts(target_mv: u16, source_mv: u16) -> u8 {
    if source_mv == 0 {
        return 0;
    }
    let steps = (target_mv as u32 * LADDER_TAPS as u32 + source_mv as u32 / 2) / source_mv as u32;
    let steps = steps.clamp(1, LADDER_TAPS as u32);
    (steps - 1) as u8
}
