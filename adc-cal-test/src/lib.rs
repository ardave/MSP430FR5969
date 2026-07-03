//! Host-side tests for the ADC calibration / scaling math.
//!
//! This crate `include!`s the REAL driver source (`hal/src/adc_cal.rs`) so the
//! tests exercise the exact code that ships in firmware — editing the
//! temperature interpolation or the fixed-point corrections incorrectly will
//! fail these tests.
//!
//! The formulas under test are the SLAU367 ones: temperature is a linear
//! interpolation between the factory 30 °C / 85 °C TLV points (§28.2.8), and
//! the gain/offset/REF corrections are 1.15 fixed-point multiplies (§28.2.7.5).

#![allow(dead_code)]

// Pull in the actual driver math (pure, dependency-free core arithmetic).
include!("../../hal/src/adc_cal.rs");

#[cfg(test)]
mod tests {
    use super::*;

    /// 1.15 fixed-point unity: 2^15.
    const UNITY: u16 = 1 << 15;

    // --- temp_deci_celsius ------------------------------------------------

    /// With a span of exactly 550 counts, one count is one deci-degree, so
    /// every interpolated value is exact.
    #[test]
    fn temp_exact_at_cal_points_and_midpoint() {
        // span 550 -> 1 count per 0.1 degC
        assert_eq!(temp_deci_celsius(1000, 1000, 1550), Some(300)); // 30.0 at the 30 degC point
        assert_eq!(temp_deci_celsius(1550, 1000, 1550), Some(850)); // 85.0 at the 85 degC point
        assert_eq!(temp_deci_celsius(1275, 1000, 1550), Some(575)); // midpoint = 57.5
    }

    /// Readings below the 30 °C point extrapolate downward, through 0 °C and
    /// into negative temperatures, symmetrically.
    #[test]
    fn temp_extrapolates_below_30c() {
        assert_eq!(temp_deci_celsius(900, 1000, 1550), Some(200)); // 20.0 degC
        assert_eq!(temp_deci_celsius(700, 1000, 1550), Some(0)); // 0.0 degC
        assert_eq!(temp_deci_celsius(600, 1000, 1550), Some(-100)); // -10.0 degC
    }

    /// Rounding is to nearest, half away from zero, on both sides of the
    /// 30 °C point (a plain `/` would truncate toward zero and bias readings
    /// below 30 °C warm).
    #[test]
    fn temp_rounds_half_away_from_zero() {
        // span 1000 -> 0.55 deci-degC per count
        assert_eq!(temp_deci_celsius(2001, 2000, 3000), Some(301)); // +0.55 -> +1
        assert_eq!(temp_deci_celsius(2002, 2000, 3000), Some(301)); // +1.10 -> +1
        assert_eq!(temp_deci_celsius(2003, 2000, 3000), Some(302)); // +1.65 -> +2
        assert_eq!(temp_deci_celsius(1999, 2000, 3000), Some(299)); // -0.55 -> -1
        assert_eq!(temp_deci_celsius(1997, 2000, 3000), Some(298)); // -1.65 -> -2
    }

    /// Realistic FR5969 2.0 V-reference constants (sensor ~2.5 counts/°C):
    /// spot-check against the reference formula evaluated in floating point.
    #[test]
    fn temp_matches_reference_formula_on_realistic_cal() {
        let (t30, t85) = (1500u16, 1638u16);
        for raw in [1400u16, 1500, 1520, 1569, 1600, 1638, 1700] {
            let expected =
                300.0 + (raw as f64 - t30 as f64) * 550.0 / (t85 as f64 - t30 as f64);
            let got = temp_deci_celsius(raw, t30, t85).unwrap() as f64;
            assert!(
                (got - expected).abs() <= 0.5,
                "raw {raw}: got {got}, expected {expected}"
            );
        }
    }

    /// A non-ascending calibration pair (blank or corrupt TLV) is rejected,
    /// not divided by.
    #[test]
    fn temp_rejects_corrupt_cal_span() {
        assert_eq!(temp_deci_celsius(1500, 2000, 2000), None); // zero span
        assert_eq!(temp_deci_celsius(1500, 2000, 1500), None); // inverted
        assert_eq!(temp_deci_celsius(1500, 0xFFFF, 0xFFFF), None); // blank FRAM
    }

    // --- counts_to_millivolts ----------------------------------------------

    #[test]
    fn counts_scale_exactly_at_full_and_zero() {
        assert_eq!(counts_to_millivolts(4095, 4095, 2000), 2000);
        assert_eq!(counts_to_millivolts(0, 4095, 2000), 0);
        // 8-bit full scale works off the same formula.
        assert_eq!(counts_to_millivolts(255, 255, 1200), 1200);
    }

    #[test]
    fn counts_round_to_nearest() {
        // 2048/4095 * 2500 = 1250.30... -> 1250
        assert_eq!(counts_to_millivolts(2048, 4095, 2500), 1250);
        // 3/4095 * 2500 = 1.83... -> 2 (truncation would say 1)
        assert_eq!(counts_to_millivolts(3, 4095, 2500), 2);
    }

    // --- apply_gain_offset --------------------------------------------------

    #[test]
    fn gain_offset_identity_at_unity_and_zero() {
        for raw in [0u16, 1, 2048, 4095] {
            assert_eq!(apply_gain_offset(raw, UNITY, 0), raw);
        }
    }

    #[test]
    fn gain_and_offset_apply_in_order() {
        // gain 2^15 * 1.01 -> +1%; 4000 * 1.01 = 4040 (fixed-point floor)
        let gain_1pct = UNITY + UNITY / 100 + 1; // 33096
        assert_eq!(apply_gain_offset(4000, gain_1pct, 0), 4040);
        // offset applies after the gain
        assert_eq!(apply_gain_offset(4000, gain_1pct, -40), 4000);
        assert_eq!(apply_gain_offset(4000, UNITY, 5), 4005);
    }

    #[test]
    fn gain_offset_clamps_instead_of_wrapping() {
        assert_eq!(apply_gain_offset(0, UNITY, -5), 0); // would go negative
        assert_eq!(apply_gain_offset(0xFFFF, 0xFFFF, i16::MAX), 0xFFFF); // would overflow
    }

    // --- apply_ref_factor ----------------------------------------------------

    #[test]
    fn ref_factor_identity_at_unity() {
        for counts in [0u16, 1, 2048, 4095] {
            assert_eq!(apply_ref_factor(counts, UNITY), counts);
        }
    }

    #[test]
    fn ref_factor_scales_fixed_point() {
        // A reference measured 0.5% low stores factor ~= 2^15 * 0.995.
        let low_half_pct = (UNITY as u32 * 995 / 1000) as u16; // 32604
        // 4000 * 32604 / 2^15 = 3979.98 -> 3979 (fixed-point floor)
        assert_eq!(apply_ref_factor(4000, low_half_pct), 3979);
        // Doubling factor doubles counts without overflow.
        assert_eq!(apply_ref_factor(4095, 0xFFFF), 8189);
    }
}
