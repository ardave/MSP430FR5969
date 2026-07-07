// Pure arithmetic for Timer_A input capture (no PAC/HAL types, `//` comments
// only, so unit_tests can `include!` this file and exercise the exact shipping
// code — see CLAUDE.md "Testing").
//
// Everything here turns raw capture-timestamp deltas (timer ticks) into
// physical quantities (Hz, duty permille) or back. All division rounds
// half-away-from-zero via the usual `(num + den/2) / den` trick, and every
// product is widened to `u64` first: at the largest legal inputs (8 MHz tick
// rate, u32 tick spans) the intermediates overflow `u32` by orders of
// magnitude.

/// Frequency in Hz, rounded, of a signal whose `periods` consecutive full
/// periods spanned `total_ticks` timer ticks at `tick_hz`.
///
/// Returns 0 if `total_ticks` is 0 (no span measured — a division guard, not
/// a meaningful frequency).
pub const fn hz_from_period_ticks(tick_hz: u32, total_ticks: u32, periods: u32) -> u32 {
    if total_ticks == 0 {
        return 0;
    }
    let num = tick_hz as u64 * periods as u64;
    ((num + total_ticks as u64 / 2) / total_ticks as u64) as u32
}

/// Duty cycle in permille (0..=1000), rounded, from a high-time of
/// `high_ticks` out of a period of `period_ticks`.
///
/// Returns 0 if `period_ticks` is 0 (guard); clamps to 1000 so a high-time
/// mismeasured slightly past its own period cannot report an impossible duty.
pub const fn duty_permille(high_ticks: u32, period_ticks: u32) -> u16 {
    if period_ticks == 0 {
        return 0;
    }
    let permille = (high_ticks as u64 * 1000 + period_ticks as u64 / 2) / period_ticks as u64;
    if permille > 1000 {
        1000
    } else {
        permille as u16
    }
}

/// How many whole periods of a `signal_hz` signal fit in `delta_ticks` ticks
/// at `tick_hz`, rounded to the nearest integer.
///
/// This is the "count periods by time" inverse used with
/// `measure_span_ticks`: capture one edge, wait a known-ish time, capture a
/// later edge — the rounded count is exact as long as the *relative* error
/// between the two clocks is under half a period over the span (for an n-period
/// span, a clock error below 1/(2n)). Returns 0 if `tick_hz` is 0 (guard).
pub const fn periods_in_span(delta_ticks: u32, tick_hz: u32, signal_hz: u32) -> u32 {
    if tick_hz == 0 {
        return 0;
    }
    let num = delta_ticks as u64 * signal_hz as u64;
    ((num + tick_hz as u64 / 2) / tick_hz as u64) as u32
}

/// The measured-to-ideal ratio, in permille, of a `delta_ticks` span that
/// should cover exactly `periods` periods of `signal_hz` at `tick_hz`.
///
/// 1000 means the two clocks agree perfectly; 1050 means the span measured 5%
/// long (the tick clock runs 5% fast relative to the signal, or vice versa).
/// This is the DCO-versus-crystal verdict in one number: with the signal on
/// the LFXT crystal (exact) and the ticks on the DCO, the deviation from 1000
/// *is* the DCO's frequency error. Returns 0 on a zero denominator (guard).
pub const fn span_ratio_permille(
    delta_ticks: u32,
    periods: u32,
    tick_hz: u32,
    signal_hz: u32,
) -> u32 {
    // ideal_ticks = periods * tick_hz / signal_hz, so
    // ratio = delta * 1000 / ideal = delta * signal_hz * 1000 / (periods * tick_hz).
    let den = periods as u64 * tick_hz as u64;
    if den == 0 {
        return 0;
    }
    let num = delta_ticks as u64 * signal_hz as u64 * 1000;
    ((num + den / 2) / den) as u32
}

/// Whether `actual` lies within `tol_permille` permille of `expected`
/// (i.e. |actual − expected| · 1000 ≤ expected · tol_permille).
pub const fn within_permille(actual: u32, expected: u32, tol_permille: u32) -> bool {
    let diff = if actual > expected {
        actual - expected
    } else {
        expected - actual
    };
    diff as u64 * 1000 <= expected as u64 * tol_permille as u64
}
