//! Host-side tests for the Comp_E reference-ladder math.
//!
//! This module `include!`s the REAL driver source (`hal/src/comp_ladder.rs`)
//! so the tests exercise the exact code that ships in firmware — editing the
//! tap↔millivolt scaling incorrectly fails these tests.
//!
//! The formula under test is SLAU367's ladder geometry: `CEREF0`/`CEREF1`
//! tap `n` selects the **(n+1)/32** point of the ladder source (tap 0 is
//! 1/32, *not* zero; tap 31 is the full source), and `tap_for_millivolts` is
//! its rounding inverse.

// Pull in the actual driver math (pure, dependency-free core arithmetic).
include!("../../hal/src/comp_ladder.rs");

#[cfg(test)]
mod tests {
    use super::*;

    // --- ladder_millivolts --------------------------------------------------

    /// A 3200 mV source makes every step an exact 100 mV, so every tap is
    /// exact: tap n = (n+1) x 100 mV.
    #[test]
    fn ladder_exact_steps() {
        for tap in 0..32u8 {
            assert_eq!(ladder_millivolts(tap, 3200), (tap as u16 + 1) * 100);
        }
    }

    /// The endpoints pin the (n+1)/32 encoding: tap 0 is one step up (not
    /// 0 V — a zero threshold would be useless), tap 31 is the full source.
    #[test]
    fn ladder_endpoints() {
        assert_eq!(ladder_millivolts(0, 3200), 100);
        assert_eq!(ladder_millivolts(31, 3200), 3200);
        assert_eq!(ladder_millivolts(31, 1200), 1200);
    }

    /// Rounding is to nearest: 3630/32 = 113.4375, so tap 0 rounds to 113,
    /// and tap 15 (16/32 = exactly half) is 1815 exactly.
    #[test]
    fn ladder_rounds_to_nearest() {
        assert_eq!(ladder_millivolts(0, 3630), 113);
        assert_eq!(ladder_millivolts(15, 3630), 1815);
        // 20/32 x 3630 = 2268.75 -> 2269.
        assert_eq!(ladder_millivolts(19, 3630), 2269);
    }

    /// Out-of-range taps clamp to tap 31 instead of scaling past the source.
    #[test]
    fn ladder_clamps_tap() {
        assert_eq!(ladder_millivolts(32, 3200), ladder_millivolts(31, 3200));
        assert_eq!(ladder_millivolts(255, 3200), ladder_millivolts(31, 3200));
    }

    // --- tap_for_millivolts ------------------------------------------------

    /// Round-trip: the tap chosen for a tap's own voltage is that tap, for
    /// every tap at several realistic sources (the LaunchPad's ~3.63 V USB
    /// rail, a nominal 3.3 V, and the 2.0 V bandgap ladder).
    #[test]
    fn tap_round_trips_through_millivolts() {
        for source in [3630u16, 3300, 2000, 1200] {
            for tap in 0..32u8 {
                let mv = ladder_millivolts(tap, source);
                assert_eq!(
                    tap_for_millivolts(mv, source),
                    tap,
                    "round-trip failed at tap {tap}, source {source} mV"
                );
            }
        }
    }

    /// Nearest-tap selection: targets just below/above a tap midpoint pick
    /// the taps either side. With a 3200 mV source (100 mV steps), 149 mV is
    /// nearer tap 0 (100 mV) and 151 mV nearer tap 1 (200 mV); the 150 mV
    /// tie rounds up.
    #[test]
    fn tap_picks_nearest() {
        assert_eq!(tap_for_millivolts(149, 3200), 0);
        assert_eq!(tap_for_millivolts(150, 3200), 1);
        assert_eq!(tap_for_millivolts(151, 3200), 1);
    }

    /// Clamping: a target at/below half a step still selects tap 0 (the
    /// ladder cannot express 0 V), and a target at or beyond the source
    /// selects tap 31 (it cannot express more than the source).
    #[test]
    fn tap_clamps_to_real_range() {
        assert_eq!(tap_for_millivolts(0, 3200), 0);
        assert_eq!(tap_for_millivolts(1, 3200), 0);
        assert_eq!(tap_for_millivolts(3200, 3200), 31);
        assert_eq!(tap_for_millivolts(5000, 3200), 31);
    }

    /// A zero source is meaningless (no ladder input); it degenerates to
    /// tap 0 rather than dividing by zero.
    #[test]
    fn tap_zero_source_degenerates() {
        assert_eq!(tap_for_millivolts(1000, 0), 0);
    }

    /// The comp fixture's own prediction, pinned: with AVCC = 3630 mV, a
    /// 2.0 V threshold sits nearest tap 17 (18/32 x 3630 = 2042) and 1.2 V
    /// nearest tap 10 (11/32 x 3630 = 1248 vs 10/32 = 1134... 1200 is
    /// 10.58 steps -> tap 10).
    #[test]
    fn launchpad_flip_taps() {
        assert_eq!(tap_for_millivolts(2000, 3630), 17);
        assert_eq!(tap_for_millivolts(1200, 3630), 10);
    }
}
