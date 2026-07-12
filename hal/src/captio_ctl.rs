// Pure Capacitive Touch I/O register math for the `captio` module.
//
// Like `rtc_tick.rs`, this file is intentionally dependency-free (pure `core`
// integer arithmetic, no PAC/HAL types) so the exact same source can be
// `include!`d by the host-side test crate in `unit_tests/`. The
// `captio::TouchSense` routing methods are a thin wrapper over these
// conversions — a regression here (a transposed port/pin field, an off-by-one
// shift, broken frequency rounding) fails the host tests without a board on
// the desk. Do not add external `use`s. (Regular `//` comments, not `//!`,
// so the file can be `include!`d mid-crate.)
//
// # The hardware's one register (SLAU367P, "Capacitive Touch I/O")
//
// A CAPTIO instance is a single 16-bit register, `CAPTIOxCTL`:
//
// - bits 7:4 `CAPTIOPOSELx` — port select (`0000` = PJ, `0001` = P1, …).
// - bits 3:1 `CAPTIOPISELx` — pin select within that port (`Px.0`..`Px.7`).
// - bit 8 `CAPTIOEN` — enable. While set, the selected pad is switched into
//   the capacitive-touch state (a relaxation oscillator built from the pin's
//   Schmitt trigger and pull resistors) and the oscillation is routed to the
//   instance's paired timer. While clear, "the signal toward timers is 0".
// - bit 9 `CAPTIO` — read-only live state of the oscillation (reads 0 while
//   disabled).
// - bit 0 is reserved-reads-zero, which is why TI's register map note about
//   scanning successive pins says to add **2** to the low byte: the pin
//   select starts at bit 1, so pin n → pin n+1 is `+2` on the encoded word.

/// `CAPTIOEN` — bit 8 of `CAPTIOxCTL`. Set in every [`ctl_word`] encoding;
/// a disabled instance is encoded as the whole word being 0.
pub const CAPTIOEN: u16 = 1 << 8;

/// `CAPTIO` — bit 9 of `CAPTIOxCTL`, the read-only live oscillation state.
pub const CAPTIO_STATE: u16 = 1 << 9;

/// Encode an **enabled** `CAPTIOxCTL` word routing port `posel`, pin `pisel`.
///
/// `posel` is the raw 4-bit port-select code (`0` = PJ, `1`..=`4` = P1..P4 on
/// this device; the field itself spans `0..=15` for larger packages) and
/// `pisel` the 3-bit pin index. Returns `None` when either exceeds its field
/// width — the caller decides which subset of encodable ports its device
/// actually bonds out. Successive pins differ by exactly 2 (the datasheet's
/// low-byte `+2` scanning idiom).
pub fn ctl_word(posel: u8, pisel: u8) -> Option<u16> {
    if posel > 0x0F || pisel > 0x07 {
        return None;
    }
    Some(CAPTIOEN | ((posel as u16) << 4) | ((pisel as u16) << 1))
}

/// Oscillation frequency from a gated count: `counts` timer increments seen
/// across `gate_ticks` ticks of a `tick_hz` yardstick clock, rounded
/// half-away-from-zero. Returns 0 for a zero-length gate rather than
/// dividing by it (project-wide zero-denominator rule).
///
/// `u64`-widened: the worst case (65535 counts against one tick of a 16 MHz
/// yardstick) overflows `u32` mid-product but not the widened form.
pub fn hz_from_gate(counts: u16, gate_ticks: u16, tick_hz: u32) -> u32 {
    if gate_ticks == 0 {
        return 0;
    }
    let num = counts as u64 * tick_hz as u64;
    let den = gate_ticks as u64;
    ((num + den / 2) / den) as u32
}
