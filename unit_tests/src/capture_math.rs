//! Host-side tests for the Timer_A capture measurement math.
//!
//! This module `include!`s the REAL driver source (`hal/src/capture_math.rs`)
//! so the tests exercise the exact code that ships in firmware — editing the
//! frequency/duty/span arithmetic incorrectly fails these tests.
//!
//! The quantities under test are the ones the capture fixture leans on: a
//! 1 kHz PWM measured with a 1 MHz tick (exact), the 32.768 kHz ACLK crystal
//! spanned by a 1 MHz or 8 MHz DCO tick (inexact — rounding is the whole
//! point), and the DCO-vs-crystal ratio in permille.

// Pull in the actual driver math (pure, dependency-free core arithmetic).
include!("../../hal/src/capture_math.rs");

#[cfg(test)]
mod tests {
    use super::*;

    // --- hz_from_period_ticks ------------------------------------------------

    /// Exact case: 8 periods of a 1 kHz signal at a 1 MHz tick span exactly
    /// 8000 ticks.
    #[test]
    fn hz_exact() {
        assert_eq!(hz_from_period_ticks(1_000_000, 8_000, 8), 1_000);
    }

    /// Rounding: 8 periods spanning 8004 ticks is 1000.5 Hz worth of period —
    /// 999.5 Hz of frequency, which rounds to 1000; 8040 ticks (1005 ticks
    /// per period) is ~995.02 Hz and must round *down* to 995.
    #[test]
    fn hz_rounds() {
        assert_eq!(hz_from_period_ticks(1_000_000, 8_004, 8), 1_000);
        assert_eq!(hz_from_period_ticks(1_000_000, 8_040, 8), 995);
    }

    /// The u64 widening: 8 MHz tick × 1000 periods overflows u32 in the
    /// numerator (8e9); the math must still be exact.
    #[test]
    fn hz_widens_to_u64() {
        // 1000 periods of exactly 1 kHz at 8 MHz ticks = 8_000_000 ticks.
        assert_eq!(hz_from_period_ticks(8_000_000, 8_000_000, 1_000), 1_000);
    }

    /// Zero-span guard: no measurement, not a division panic.
    #[test]
    fn hz_zero_span_guard() {
        assert_eq!(hz_from_period_ticks(1_000_000, 0, 8), 0);
    }

    // --- duty_permille --------------------------------------------------------

    /// Exact quarters of a 1000-tick period.
    #[test]
    fn duty_exact() {
        assert_eq!(duty_permille(0, 1_000), 0);
        assert_eq!(duty_permille(250, 1_000), 250);
        assert_eq!(duty_permille(750, 1_000), 750);
        assert_eq!(duty_permille(1_000, 1_000), 1_000);
    }

    /// Half-away-from-zero rounding: 1/8000 of a period is 0.125 permille
    /// (→ 0), 4/8000 is exactly 0.5 (→ 1), 999/1000 stays 999.
    #[test]
    fn duty_rounds() {
        assert_eq!(duty_permille(1, 8_000), 0);
        assert_eq!(duty_permille(4, 8_000), 1);
        assert_eq!(duty_permille(999, 1_000), 999);
    }

    /// A high-time mismeasured past its own period clamps to 1000 instead of
    /// reporting an impossible duty; a zero period is a guard, not a panic.
    #[test]
    fn duty_clamps_and_guards() {
        assert_eq!(duty_permille(1_100, 1_000), 1_000);
        assert_eq!(duty_permille(123, 0), 0);
    }

    // --- periods_in_span -------------------------------------------------------

    /// The fixture's own numbers: ACLK (32768 Hz) spanned by a 1 MHz tick.
    /// One period is 30.52 ticks; a 16-period span is ~488.3 ticks, and every
    /// plausible measured delta around it must round to exactly 16.
    #[test]
    fn periods_aclk_at_1mhz() {
        for delta in 481..=496 {
            assert_eq!(periods_in_span(delta, 1_000_000, 32_768), 16, "delta {delta}");
        }
    }

    /// At an 8 MHz tick a period is 244.14 ticks; 4 periods ≈ 976.6 ticks.
    #[test]
    fn periods_aclk_at_8mhz() {
        assert_eq!(periods_in_span(977, 8_000_000, 32_768), 4);
        assert_eq!(periods_in_span(976, 8_000_000, 32_768), 4);
        // A ±5% DCO error moves a 4-period span by ±49 ticks — still 4.
        assert_eq!(periods_in_span(928, 8_000_000, 32_768), 4);
        assert_eq!(periods_in_span(1_025, 8_000_000, 32_768), 4);
    }

    /// Zero tick rate is a guard, not a panic.
    #[test]
    fn periods_zero_guard() {
        assert_eq!(periods_in_span(1_000, 0, 32_768), 0);
    }

    // --- span_ratio_permille ----------------------------------------------------

    /// Perfect clocks read exactly 1000 (the ideal 16-period ACLK span at
    /// 1 MHz is 488.28 ticks; 488 measured rounds to 999, 489 to 1001 — pin
    /// both, they bracket the ideal).
    #[test]
    fn ratio_brackets_ideal() {
        assert_eq!(span_ratio_permille(488, 16, 1_000_000, 32_768), 999);
        assert_eq!(span_ratio_permille(489, 16, 1_000_000, 32_768), 1_001);
    }

    /// An exactly-5%-fast tick clock (its ticks run fast, so more of them fit
    /// in the span) reads 1050.
    #[test]
    fn ratio_reads_clock_error() {
        // Ideal span for 32 periods at 8 MHz = 7812.5 ticks; 5% more = 8203.
        assert_eq!(span_ratio_permille(8_203, 32, 8_000_000, 32_768), 1_050);
    }

    /// Zero periods (or a zero tick rate) is a guard, not a panic.
    #[test]
    fn ratio_zero_guard() {
        assert_eq!(span_ratio_permille(1_000, 0, 1_000_000, 32_768), 0);
        assert_eq!(span_ratio_permille(1_000, 16, 0, 32_768), 0);
    }

    // --- within_permille ----------------------------------------------------------

    /// The boundary is inclusive: exactly 1% off passes a 10-permille gate,
    /// one more part in a thousand fails it. Symmetric in both directions.
    #[test]
    fn within_boundary_inclusive() {
        assert!(within_permille(1_010, 1_000, 10));
        assert!(within_permille(990, 1_000, 10));
        assert!(!within_permille(1_011, 1_000, 10));
        assert!(!within_permille(989, 1_000, 10));
    }

    /// Large values must not overflow: 4 billion vs itself at any tolerance.
    #[test]
    fn within_widens_to_u64() {
        assert!(within_permille(4_000_000_000, 4_000_000_000, 0));
        assert!(!within_permille(4_000_000_000, 3_900_000_000, 10));
    }
}
