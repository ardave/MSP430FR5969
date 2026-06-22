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

/// Assemble a 32-bit timestamp from the software overflow tally (`overflows`,
/// the high 16 bits) and the hardware counter (`cnt`, the low 16 bits).
///
/// The caller is responsible for having already folded in any pending overflow
/// (see `Counter::now64`); this is just the bit-packing half, isolated so it is
/// testable without hardware.
pub(crate) fn assemble_now64(overflows: u16, cnt: u16) -> u32 {
    ((overflows as u32) << 16) | cnt as u32
}
