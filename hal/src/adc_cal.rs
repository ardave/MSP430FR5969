// Pure ADC calibration / scaling arithmetic.
//
// Like `baud.rs` and `ticks.rs`, this file is intentionally dependency-free
// (pure `core` integer arithmetic, no PAC/HAL types) so the exact same source
// can be `include!`d by the host-side test crate in `adc-cal-test/`. The
// `tlv::AdcCal`/`tlv::RefCal` methods and `Adc::to_millivolts` are thin
// wrappers over these functions, so a regression in the math fails the host
// tests. Do not add external `use`s here.
// (Regular `//` comments, not `//!`, so the file can be `include!`d mid-crate.)

/// Scale a conversion result to millivolts against a known reference.
///
/// `counts / full_scale` is the fraction of VR+ the input sat at, so
/// `mV = counts * vref_mv / full_scale`, rounded to nearest (add half the
/// divisor before dividing). Widened to `u32`: the worst case,
/// `4095 * 2500 ≈ 1.0e7`, is far inside `u32`. `full_scale` is
/// `Resolution::max()` — never zero.
pub(crate) fn counts_to_millivolts(counts: u16, full_scale: u16, vref_mv: u16) -> u32 {
    (counts as u32 * vref_mv as u32 + full_scale as u32 / 2) / full_scale as u32
}

/// Interpolate the temperature sensor reading to **deci-°C** (tenths of a
/// degree: 273 = 27.3 °C) between the two factory calibration points.
///
/// TI characterizes every die at 30 °C and 85 °C and stores the *measured ADC
/// results* in the TLV (per reference voltage). Because those constants were
/// read through this chip's own ADC, they already embed its gain and offset
/// error — so the interpolation runs on the **raw, uncorrected** reading
/// (SLAU367 §28.2.8):
///
/// ```text
/// T = 30 + (raw − CAL_T30) · (85 − 30) / (CAL_T85 − CAL_T30)     [°C]
/// ```
///
/// Scaled here to deci-°C (`(85−30)·10 = 550`), computed in `i32`
/// (`4095 · 550 ≈ 2.3e6`), rounded half-away-from-zero so readings below
/// 30 °C round symmetrically. Returns `None` if `cal_t85 <= cal_t30` — the
/// sensor slope is positive on real silicon, so a non-positive span means the
/// calibration words are corrupt (or a blank TLV read as 0xFFFF/0x0000), not
/// a cold room.
pub(crate) fn temp_deci_celsius(raw: u16, cal_t30: u16, cal_t85: u16) -> Option<i16> {
    let span = cal_t85 as i32 - cal_t30 as i32;
    if span <= 0 {
        return None;
    }
    let num = (raw as i32 - cal_t30 as i32) * 550;
    let half = span / 2;
    let interp = if num >= 0 {
        (num + half) / span
    } else {
        (num - half) / span
    };
    Some((300 + interp) as i16)
}

/// Apply the factory ADC gain factor and offset to a raw conversion result.
///
/// The TLV gain factor is **1.15 fixed point** (2^15 = unity gain, so 33096
/// means ×1.01), the offset a signed count correction; the corrected result is
/// `raw · gain / 2^15 + offset` (SLAU367 §28.2.7.5). Computed in `u32`/`i32`
/// (worst case `4095 · 65535 ≈ 2.7e8`), clamped to `0..=u16::MAX` so a
/// pathological calibration word cannot wrap.
pub(crate) fn apply_gain_offset(raw: u16, gain_factor: u16, offset: i16) -> u16 {
    let corrected = ((raw as u32 * gain_factor as u32) >> 15) as i32 + offset as i32;
    corrected.clamp(0, u16::MAX as i32) as u16
}

/// Apply the factory reference-voltage factor to a conversion result.
///
/// Same 1.15 fixed-point convention as the gain factor: the TLV stores the
/// measured-vs-nominal ratio of each REF_A output, and
/// `counts · factor / 2^15` re-expresses the result as if the reference had
/// been exactly nominal. Worst case `4095 · 65535` fits `u32`; the result is
/// at most ~2× the input, still comfortably `u16`.
pub(crate) fn apply_ref_factor(counts: u16, factor: u16) -> u16 {
    ((counts as u32 * factor as u32) >> 15) as u16
}
