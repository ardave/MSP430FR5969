//! Host-side tests for the eUSCI_A baud-rate math.
//!
//! This module `include!`s the REAL driver source (`hal/src/baud.rs`) so the
//! tests exercise the exact code that ships in firmware — editing
//! `UCBRS_TABLE` or `compute_baud` incorrectly will fail these tests.
//!
//! The driver follows the datasheet *procedure* (SLAU367P §30.3.10 + the
//! Table 30-4 fractional lookup). The recommended-settings values in Table 30-5
//! come from a separate "lowest-error search", and the datasheet notes other
//! settings can yield the same/similar error — so the hard pass/fail criterion
//! is the resulting average bit-timing error, not an exact register match.
//! Exact-match statistics are checked separately where the procedure is
//! deterministic.

// Pull in the actual driver math (pure, dependency-free core arithmetic).
include!("../../hal/src/baud.rs");

#[cfg(test)]
mod tests {
    use super::*;

    /// Average BRCLK ticks per bit the hardware produces for a setting.
    /// `UCBRFx` contributes +UCBRFx ticks/bit (oversampling); `UCBRSx` is an
    /// 8-bit modulation pattern contributing `popcount(UCBRSx)/8` ticks/bit on
    /// average.
    fn avg_ticks_per_bit(os16: bool, br: u32, brf: u32, brs: u8) -> f64 {
        let brs_avg = brs.count_ones() as f64 / 8.0;
        if os16 {
            16.0 * br as f64 + brf as f64 + brs_avg
        } else {
            br as f64 + brs_avg
        }
    }

    /// (brclk, baud, ds_os16, ds_br, ds_brf, ds_brs) rows from SLAU367P
    /// Table 30-5 (recommended settings for common crystals / baud rates).
    const TABLE_30_5: &[(u32, u32, bool, u32, u32, u8)] = &[
        // 32768 Hz ACLK
        (32768, 1200, true, 1, 11, 0x25),
        (32768, 2400, false, 13, 0, 0xB6),
        (32768, 4800, false, 6, 0, 0xEE),
        (32768, 9600, false, 3, 0, 0x92),
        // 1 MHz
        (1_000_000, 9600, true, 6, 8, 0x20),
        (1_000_000, 19200, true, 3, 4, 0x02),
        (1_000_000, 38400, true, 1, 10, 0x00),
        (1_000_000, 57600, false, 17, 0, 0x4A),
        (1_000_000, 115200, false, 8, 0, 0xD6),
        // 1048576 Hz
        (1_048_576, 9600, true, 6, 13, 0x22),
        (1_048_576, 19200, true, 3, 6, 0xAD),
        (1_048_576, 38400, true, 1, 11, 0x25),
        (1_048_576, 57600, false, 18, 0, 0x11),
        (1_048_576, 115200, false, 9, 0, 0x08),
        // 4 MHz
        (4_000_000, 9600, true, 26, 0, 0xB6),
        (4_000_000, 19200, true, 13, 0, 0x84),
        (4_000_000, 38400, true, 6, 8, 0x20),
        (4_000_000, 57600, true, 4, 5, 0x55),
        (4_000_000, 115200, true, 2, 2, 0xBB),
        // 4194304 Hz
        (4_194_304, 9600, true, 27, 4, 0xFB),
        (4_194_304, 19200, true, 13, 10, 0x55),
        (4_194_304, 38400, true, 6, 13, 0x22),
        (4_194_304, 57600, true, 4, 8, 0xEE),
        (4_194_304, 115200, true, 2, 4, 0x92),
        (4_194_304, 230400, false, 18, 0, 0x11),
        // 8 MHz
        (8_000_000, 9600, true, 52, 1, 0x49),
        (8_000_000, 19200, true, 26, 0, 0xB6),
        (8_000_000, 38400, true, 13, 0, 0x84),
        (8_000_000, 57600, true, 8, 10, 0xF7),
        (8_000_000, 115200, true, 4, 5, 0x55),
        (8_000_000, 230400, true, 2, 2, 0xBB),
        (8_000_000, 460800, false, 17, 0, 0x4A),
        // 8388608 Hz
        (8_388_608, 9600, true, 54, 9, 0xEE),
        (8_388_608, 19200, true, 27, 4, 0xFB),
        (8_388_608, 38400, true, 13, 10, 0x55),
        (8_388_608, 57600, true, 9, 1, 0xB5),
        (8_388_608, 115200, true, 4, 8, 0xEE),
        (8_388_608, 230400, true, 2, 4, 0x92),
        (8_388_608, 460800, false, 18, 0, 0x11),
        // 12 MHz
        (12_000_000, 9600, true, 78, 2, 0x00),
        (12_000_000, 19200, true, 39, 1, 0x00),
        (12_000_000, 38400, true, 19, 8, 0x65),
        (12_000_000, 57600, true, 13, 0, 0x25),
        (12_000_000, 115200, true, 6, 8, 0x20),
        (12_000_000, 230400, true, 3, 4, 0x02),
        (12_000_000, 460800, true, 1, 10, 0x00),
        // 16 MHz
        (16_000_000, 9600, true, 104, 2, 0xD6),
        (16_000_000, 19200, true, 52, 1, 0x49),
        (16_000_000, 38400, true, 26, 0, 0xB6),
        (16_000_000, 57600, true, 17, 5, 0xDD),
        (16_000_000, 115200, true, 8, 10, 0xF7),
        (16_000_000, 230400, true, 4, 5, 0x55),
        (16_000_000, 460800, true, 2, 2, 0xBB),
        // 16777216 Hz
        (16_777_216, 9600, true, 109, 3, 0xB5),
        (16_777_216, 19200, true, 54, 9, 0xEE),
        (16_777_216, 38400, true, 27, 4, 0xFB),
        (16_777_216, 115200, true, 9, 1, 0xB5),
        // 20 MHz
        (20_000_000, 9600, true, 130, 3, 0x25),
        (20_000_000, 19200, true, 65, 1, 0xD6),
        (20_000_000, 38400, true, 32, 8, 0xEE),
        (20_000_000, 115200, true, 10, 13, 0xAD),
        (20_000_000, 460800, true, 2, 11, 0x92),
    ];

    /// Per-bit timing error must stay small enough that drift over a 10-bit
    /// character stays well under half a bit. 2% per bit -> ~20% accumulated
    /// worst case, comfortably inside the ~50% sampling budget.
    const TOL_PCT: f64 = 2.0;

    /// Every recommended (BRCLK, baud) pair must produce a setting whose
    /// average bit-timing error is within tolerance — the real "is this a
    /// usable UART" criterion.
    #[test]
    fn table_30_5_timing_within_tolerance() {
        let mut worst = 0.0f64;
        for &(brclk, baud, _, _, _, _) in TABLE_30_5 {
            let r = compute_baud(brclk, baud);
            let n_ideal = brclk as f64 / baud as f64;
            let avg = avg_ticks_per_bit(r.oversampling, r.ucbr as u32, r.ucbrf as u32, r.ucbrs);
            let err = (avg - n_ideal) / n_ideal * 100.0;
            assert!(
                err.abs() < TOL_PCT,
                "{} Hz @ {} baud: {:+.3}% exceeds {:.1}% (os16={}, br={}, brf={}, brs={:#04x})",
                brclk, baud, err, TOL_PCT, r.oversampling, r.ucbr, r.ucbrf, r.ucbrs
            );
            worst = worst.max(err.abs());
        }
        // Sanity: the worst case across the whole table should be the coarse
        // 32768 Hz / 4800 baud point (~1.12%), not something larger sneaking in.
        assert!(worst < 1.2, "worst-case timing error grew to {:.3}%", worst);
    }

    /// Where our table-lookup `UCBRSx` differs from Table 30-5's searched value,
    /// it must not be *worse* on average timing (allowing a tiny float margin).
    #[test]
    fn divergent_ucbrs_is_not_worse() {
        for &(brclk, baud, ds_os16, ds_br, ds_brf, ds_brs) in TABLE_30_5 {
            let r = compute_baud(brclk, baud);
            let n_ideal = brclk as f64 / baud as f64;
            let mine =
                (avg_ticks_per_bit(r.oversampling, r.ucbr as u32, r.ucbrf as u32, r.ucbrs)
                    - n_ideal)
                    .abs();
            let ds = (avg_ticks_per_bit(ds_os16, ds_br, ds_brf, ds_brs) - n_ideal).abs();
            assert!(
                mine <= ds + 1e-6,
                "{} Hz @ {} baud: our avg error {:.4} ticks worse than datasheet {:.4}",
                brclk, baud, mine, ds
            );
        }
    }

    /// When our procedure picks the same mode as Table 30-5, the deterministic
    /// registers (UCBRx, and UCBRFx in oversampling mode) must match exactly.
    /// (Mode itself can legitimately differ when N is near the ÷16 threshold;
    /// those rows are covered by the timing and "not worse" tests.)
    ///
    /// At least most of the table shares the datasheet's mode, so this also
    /// guards that we're not silently diverging everywhere.
    #[test]
    fn deterministic_registers_match_when_mode_agrees() {
        let mut same_mode = 0;
        for &(brclk, baud, ds_os16, ds_br, ds_brf, _) in TABLE_30_5 {
            let r = compute_baud(brclk, baud);
            if r.oversampling != ds_os16 {
                continue;
            }
            same_mode += 1;
            assert_eq!(r.ucbr as u32, ds_br, "{} Hz @ {} baud: UCBRx", brclk, baud);
            if ds_os16 {
                assert_eq!(r.ucbrf as u32, ds_brf, "{} Hz @ {} baud: UCBRFx", brclk, baud);
            }
        }
        assert!(
            same_mode >= TABLE_30_5.len() - 6,
            "mode diverged from datasheet in too many rows: only {}/{} agreed",
            same_mode,
            TABLE_30_5.len()
        );
    }

    /// The default the demo uses (1 MHz BRCLK, 9600 baud) must match Table 30-5
    /// exactly: oversampling, UCBRx=6, UCBRFx=8, UCBRSx=0x20.
    #[test]
    fn default_1mhz_9600_exact() {
        let r = compute_baud(1_000_000, 9600);
        assert!(r.oversampling);
        assert_eq!(r.ucbr, 6);
        assert_eq!(r.ucbrf, 8);
        assert_eq!(r.ucbrs, 0x20);
    }

    /// Spot-check the Table 30-4 lookup at and around bucket boundaries.
    #[test]
    fn ucbrs_lookup_boundaries() {
        assert_eq!(ucbrs_lookup(0), 0x00); // exactly the first threshold
        assert_eq!(ucbrs_lookup(528), 0x00); // just below 0.0529
        assert_eq!(ucbrs_lookup(529), 0x01); // exactly 0.0529
        assert_eq!(ucbrs_lookup(1666), 0x20); // 1 MHz/9600 frac, below 0.1670
        assert_eq!(ucbrs_lookup(9288), 0xFE); // last bucket
        assert_eq!(ucbrs_lookup(9999), 0xFE); // saturates at the top bucket
    }

    /// The lookup table must be sorted by threshold (the linear scan in
    /// `ucbrs_lookup` relies on monotonic thresholds to stop early).
    #[test]
    fn ucbrs_table_thresholds_monotonic() {
        for w in UCBRS_TABLE.windows(2) {
            assert!(w[0].0 < w[1].0, "thresholds not strictly increasing: {:?}", w);
        }
        assert_eq!(UCBRS_TABLE[0].0, 0, "table must start at fractional 0");
    }
}
