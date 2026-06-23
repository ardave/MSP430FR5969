// Pure tick<->time arithmetic for the timer Counter.
//
// Like `baud.rs`, this file is intentionally dependency-free (pure `core`
// integer arithmetic, no PAC/HAL types) so the exact same source can be
// `include!`d by the host-side test crate in `timer-test/`. The `Counter`
// methods are thin wrappers over these functions, so a regression in the
// conversion math fails the host tests. Do not add external `use`s here.
// (Regular `//` comments, not `//!`, so the file can be `include!`d mid-crate.)

/// Convert a tick delta to microseconds at `tick_hz` ticks/second.
///
/// Computed in `u64` so `ticks * 1_000_000` cannot overflow (the worst case,
/// `u32::MAX * 1_000_000 ≈ 4.3e15`, far exceeds `u32`); the divide brings it
/// back into range. The `u32` result therefore holds up to ~4295 s of µs.
/// Truncates toward zero (one tick of resolution at the configured rate).
pub(crate) fn ticks_to_us(ticks: u32, tick_hz: u32) -> u32 {
    (ticks as u64 * 1_000_000 / tick_hz as u64) as u32
}

/// Convert a tick delta to nanoseconds at `tick_hz` ticks/second.
///
/// `ticks * 1_000_000_000` reaches ~4.3e18, still within `u64`. The `u32` result
/// only spans ~4.3 s of ns, so this is for short deltas; resolution is one tick.
pub(crate) fn ticks_to_ns(ticks: u32, tick_hz: u32) -> u32 {
    (ticks as u64 * 1_000_000_000 / tick_hz as u64) as u32
}

/// Convert microseconds to ticks at `tick_hz` ticks/second, or `None` if the
/// interval is longer than one 16-bit counter wrap (≥ 65536 ticks).
///
/// This is the inverse of [`ticks_to_us`] and the safe replacement for an
/// `as u16` narrowing: the multiply is widened to `u64` (worst case
/// `u32::MAX * 1_000_000`), and the result is rejected rather than truncated
/// when it will not fit a single `u16` CCR compare. `Some` therefore always
/// carries a value that a 16-bit compare can represent unambiguously (a full
/// 65536-tick wrap maps `now` onto itself, so it is excluded). Truncates toward
/// zero like the forward conversion.
pub(crate) fn us_to_ticks(us: u32, tick_hz: u32) -> Option<u16> {
    let ticks = us as u64 * tick_hz as u64 / 1_000_000;
    if ticks > u16::MAX as u64 {
        None
    } else {
        Some(ticks as u16)
    }
}

/// Assemble a 32-bit timestamp from the software overflow tally (`overflows`,
/// the high 16 bits) and the hardware counter (`cnt`, the low 16 bits).
///
/// The caller is responsible for having already folded in any pending overflow
/// (see `Counter::now64`); this is just the bit-packing half, isolated so it is
/// testable without hardware.
pub(crate) fn assemble_now64(overflows: u16, cnt: u16) -> u32 {
    ((overflows as u32) << 16) | cnt as u32
}
