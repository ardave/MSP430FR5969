//! Host-side tests for the timer tick<->time math.
//!
//! This module `include!`s the REAL conversion source (`hal/src/ticks.rs`) so the
//! tests exercise the exact code that ships in firmware — a bad edit to
//! `ticks_to_us` / `ticks_to_ns` / `assemble_now32` (e.g. dropping the `u64`
//! widening, or mis-packing the 32-bit timestamp) will fail these tests.

// Pull in the actual conversion math (pure, dependency-free core arithmetic).
include!("../../hal/src/ticks.rs");

#[cfg(test)]
mod tests {
    use super::*;

    /// Tick rates this project actually configures, for reference in the tables:
    /// SMCLK 8 MHz and 1 MHz (performance / low-power profiles), the 32.768 kHz
    /// LFXT crystal, and the ~9.4 kHz VLO fallback.
    const SMCLK_8M: u32 = 8_000_000;
    const SMCLK_1M: u32 = 1_000_000;
    const LFXT: u32 = 32_768;
    const VLO: u32 = 9_400;

    /// `ticks_to_us` must be exact for representative (ticks, rate) pairs.
    #[test]
    fn ticks_to_us_exact_at_common_rates() {
        // (ticks, tick_hz, expected_us)
        let cases: &[(u32, u32, u32)] = &[
            (0, SMCLK_1M, 0),
            (1, SMCLK_1M, 1),               // 1 MHz -> 1 tick = 1 µs
            (1_000, SMCLK_1M, 1_000),
            (1_000_000, SMCLK_1M, 1_000_000),
            (8, SMCLK_8M, 1),               // 8 MHz -> 8 ticks = 1 µs
            (8_000, SMCLK_8M, 1_000),
            (1, SMCLK_8M, 0),               // 125 ns truncates to 0 µs
            (LFXT, LFXT, 1_000_000),        // 32768 ticks = exactly 1 s
            (1, LFXT, 30),                  // 1 tick = 30.5 µs -> 30 (trunc)
            (VLO, VLO, 1_000_000),          // 1 s at the VLO rate, whatever it is
        ];
        for &(ticks, hz, want) in cases {
            assert_eq!(
                ticks_to_us(ticks, hz),
                want,
                "ticks_to_us({ticks}, {hz})"
            );
        }
    }

    /// The exact values Step 4 printed over UART, reproduced from the math:
    /// a ~1 s LPM3 sleep on the 32.768 kHz crystal measured 32771/32772 ticks.
    /// This ties the host arithmetic to the observed hardware behavior.
    #[test]
    fn reproduces_step4_hardware_readings() {
        assert_eq!(ticks_to_us(32_768, LFXT), 1_000_000); // exact 1 s target
        assert_eq!(ticks_to_us(32_771, LFXT), 1_000_091); // observed reading
        assert_eq!(ticks_to_us(32_772, LFXT), 1_000_122); // observed +1-tick reading
    }

    /// Regression guard for the `u64` widening: `ticks * 1_000_000` must not be
    /// computed in `u32`. With 32-bit intermediates this product overflows for
    /// ticks > ~4294 and the result is garbage; in `u64` it is exact.
    #[test]
    fn ticks_to_us_requires_u64_widening() {
        // 10_000 * 1_000_000 = 1e10, which wraps a u32 (max ~4.29e9).
        assert_eq!(ticks_to_us(10_000, SMCLK_1M), 10_000);
        // A large-but-in-range result (1e8 µs fits u32) still computes correctly.
        assert_eq!(ticks_to_us(100_000_000, SMCLK_1M), 100_000_000);
    }

    /// `ticks_to_ns` exactness, including the 8 MHz "125 ns/tick" case and the
    /// u64-widening guard (ticks * 1e9 overflows u32 almost immediately).
    #[test]
    fn ticks_to_ns_exact_and_wide() {
        assert_eq!(ticks_to_ns(1, SMCLK_1M), 1_000); // 1 µs
        assert_eq!(ticks_to_ns(1, SMCLK_8M), 125);   // 125 ns
        assert_eq!(ticks_to_ns(1, LFXT), 30_517);    // 1e9/32768 = 30517.6 -> trunc
        // 100 * 1e9 = 1e11 overflows u32; must be computed in u64.
        assert_eq!(ticks_to_ns(100, SMCLK_1M), 100_000);
    }

    /// Truncation is toward zero: a sub-tick remainder is dropped, never rounded
    /// up (the driver is documented as truncating).
    #[test]
    fn conversions_truncate_toward_zero() {
        // 3 ticks at 8 MHz = 0.375 µs -> 0.
        assert_eq!(ticks_to_us(3, SMCLK_8M), 0);
        // 7 ticks at 8 MHz = 0.875 µs -> 0.
        assert_eq!(ticks_to_us(7, SMCLK_8M), 0);
        // 9 ticks at 8 MHz = 1.125 µs -> 1.
        assert_eq!(ticks_to_us(9, SMCLK_8M), 1);
    }

    /// `assemble_now32` packs the overflow tally as the high 16 bits and the
    /// counter as the low 16 bits.
    #[test]
    fn assemble_now32_packs_high_low() {
        assert_eq!(assemble_now32(0, 0), 0);
        assert_eq!(assemble_now32(0, 0xFFFF), 0x0000_FFFF);
        assert_eq!(assemble_now32(1, 0), 0x0001_0000); // one wrap = 65536
        assert_eq!(assemble_now32(0x000F, 0x4242), 0x000F_4242);
        assert_eq!(assemble_now32(0xFFFF, 0xFFFF), 0xFFFF_FFFF);
    }

    /// A round-trip sanity check: assemble a wide timestamp, then convert the
    /// span between two of them. One full 16-bit wrap (65536 ticks) at the
    /// crystal rate is exactly 2 seconds.
    #[test]
    fn one_wrap_at_crystal_is_two_seconds() {
        let start = assemble_now32(0, 0);
        let end = assemble_now32(1, 0); // exactly one wrap later
        let span = end.wrapping_sub(start);
        assert_eq!(span, 65_536);
        assert_eq!(ticks_to_us(span, LFXT), 2_000_000); // 2 s
    }

    /// `us_to_ticks` is the inverse of `ticks_to_us` for representative
    /// durations, truncating toward zero just like the forward conversion.
    #[test]
    fn us_to_ticks_exact_at_common_rates() {
        // (us, tick_hz, expected_ticks)
        let cases: &[(u32, u32, u16)] = &[
            (0, LFXT, 0),
            (1_000_000, LFXT, 32_768),     // 1 s on the crystal = 32768 ticks
            (30, LFXT, 0),                 // < 1 tick (30.5 µs) -> 0
            (31, LFXT, 1),                 // first whole tick
            (1_000, SMCLK_1M, 1_000),      // 1 ms at 1 MHz = 1000 ticks
            (1, SMCLK_1M, 1),
            (1_000, LFXT, 32),             // 1 ms -> 32.768 -> 32 (trunc)
        ];
        for &(us, hz, want) in cases {
            assert_eq!(us_to_ticks(us, hz), Some(want), "us_to_ticks({us}, {hz})");
        }
    }

    /// The whole point of `us_to_ticks` over an `as u16` cast: it rejects any
    /// interval that does not fit one 16-bit wrap instead of silently
    /// truncating. The boundary is exactly 65536 ticks (a full wrap, which maps
    /// `now` onto itself) — `Some(65535)` just under, `None` at and above.
    #[test]
    fn us_to_ticks_rejects_intervals_past_one_wrap() {
        // At the crystal rate the last representable interval is 65535 ticks,
        // reached at 1_999_970 µs and held until just under 2_000_000 µs; one
        // full wrap (65536 ticks) lands exactly at 2_000_000 µs and is rejected.
        assert_eq!(us_to_ticks(1_999_970, LFXT), Some(65_535)); // first µs that floors to 65535
        assert_eq!(us_to_ticks(1_999_999, LFXT), Some(65_535)); // last µs before the wrap
        assert_eq!(us_to_ticks(2_000_000, LFXT), None); // exactly one wrap -> rejected
        assert_eq!(us_to_ticks(3_000_000, LFXT), None); // well past one wrap

        // At 1 MHz a 16-bit counter wraps every 65536 µs; this is exactly where
        // the old `tick_hz() as u16` cast silently truncated.
        assert_eq!(us_to_ticks(65_535, SMCLK_1M), Some(65_535));
        assert_eq!(us_to_ticks(65_536, SMCLK_1M), None);
        // 1 s at 1 MHz = 1_000_000 ticks: hopelessly past one wrap, must be None
        // (the cast would have yielded 16960).
        assert_eq!(us_to_ticks(1_000_000, SMCLK_1M), None);
    }

    /// Guard the `u64` widening on the multiply, mirroring the forward
    /// conversion's guard: `us * tick_hz` overflows `u32` long before the result
    /// would, so the product must be computed in `u64`.
    #[test]
    fn us_to_ticks_requires_u64_widening() {
        // 1_000_000 µs * 32_768 Hz = 3.27e13, far past u32; result 32768 fits u16.
        assert_eq!(us_to_ticks(1_000_000, LFXT), Some(32_768));
    }
}
