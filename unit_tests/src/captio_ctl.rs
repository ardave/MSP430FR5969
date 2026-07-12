//! Host-side tests for `hal/src/captio_ctl.rs` — the Capacitive Touch I/O
//! `CAPTIOxCTL` encoding and gated-count→frequency math that
//! `hal::captio::TouchSense` delegates to.
//!
//! Anchors: the SLAU367P register layout (`CAPTIOPISELx` at bits 3:1,
//! `CAPTIOPOSELx` at bits 7:4, `CAPTIOEN` = bit 8, the read-only `CAPTIO`
//! state = bit 9, bit 0 reserved) and its scanning note — successive pins
//! differ by exactly 2 in the low byte, which is only true if the pin field
//! really starts at bit 1. A transposed field, an off-by-one shift, or
//! broken rounding can't survive these tests.

include!("../../hal/src/captio_ctl.rs");

// ------------------------------------------------------------ bit positions

#[test]
fn enable_and_state_bits_match_slau367() {
    assert_eq!(CAPTIOEN, 0x0100, "CAPTIOEN is bit 8");
    assert_eq!(CAPTIO_STATE, 0x0200, "CAPTIO (state) is bit 9");
}

// ------------------------------------------------------------ encoding

#[test]
fn known_words_pin_the_field_layout() {
    // PJ.0: both fields zero — the word is nothing but the enable bit.
    assert_eq!(ctl_word(0, 0), Some(0x0100));
    // P1.0: port field alone (POSEL = 1 at bits 7:4).
    assert_eq!(ctl_word(1, 0), Some(0x0110));
    // PJ.2: pin field alone (PISEL = 2 at bits 3:1).
    assert_eq!(ctl_word(0, 2), Some(0x0104));
    // P4.5: both fields populated (0x40 | 0x0A | EN).
    assert_eq!(ctl_word(4, 5), Some(0x014A));
    // P3.3: the fixture's scan territory.
    assert_eq!(ctl_word(3, 3), Some(0x0136));
    // P15.7: the field maxima still encode (larger-package headroom).
    assert_eq!(ctl_word(15, 7), Some(0x01FE));
}

#[test]
fn bit0_is_never_set() {
    // Bit 0 is reserved-reads-zero; no encoding may touch it.
    for posel in 0..=15 {
        for pisel in 0..=7 {
            assert_eq!(ctl_word(posel, pisel).unwrap() & 1, 0);
        }
    }
}

#[test]
fn successive_pins_differ_by_two() {
    // TI's scanning idiom: "increment the low byte of CAPTIOxCTL_L by 2" to
    // step to the next pin — the documented consequence of PISEL at bits 3:1.
    for posel in 0..=15 {
        for pisel in 0..=6 {
            assert_eq!(
                ctl_word(posel, pisel + 1).unwrap(),
                ctl_word(posel, pisel).unwrap() + 2,
                "P{posel}.{pisel} -> .{} must be +2",
                pisel + 1
            );
        }
    }
}

#[test]
fn out_of_field_selections_are_rejected() {
    assert_eq!(ctl_word(16, 0), None, "POSEL is 4 bits");
    assert_eq!(ctl_word(0, 8), None, "PISEL is 3 bits");
    assert_eq!(ctl_word(255, 255), None);
}

// ------------------------------------------------------------ frequency

#[test]
fn exact_gates_convert_exactly() {
    // 10_000 counts across a 10_000-tick gate of a 1 MHz yardstick: 1 MHz.
    assert_eq!(hz_from_gate(10_000, 10_000, 1_000_000), 1_000_000);
    // 1450 counts in 1 ms at 1 MHz: 1.45 MHz (the ballpark of a real pad).
    assert_eq!(hz_from_gate(1_450, 1_000, 1_000_000), 1_450_000);
    // Zero counts is zero hertz, not an error.
    assert_eq!(hz_from_gate(0, 10_000, 1_000_000), 0);
}

#[test]
fn rounding_is_half_away_from_zero() {
    // 5 counts over 3 ticks of a 1 Hz yardstick = 1.666… → 2.
    assert_eq!(hz_from_gate(5, 3, 1), 2);
    // 4/3 = 1.333… → 1.
    assert_eq!(hz_from_gate(4, 3, 1), 1);
    // Exactly .5 rounds away: 3/2 = 1.5 → 2.
    assert_eq!(hz_from_gate(3, 2, 1), 2);
}

#[test]
fn zero_gate_returns_zero_not_a_panic() {
    assert_eq!(hz_from_gate(1_000, 0, 1_000_000), 0);
}

#[test]
fn widening_survives_the_worst_case() {
    // 65535 counts * 16 MHz overflows a u32 product mid-computation (≈1e12);
    // the u64 widening must carry it through the divide. 40960 gate ticks at
    // 16 MHz is a 2.56 ms gate: 65535/0.00256 s = 25.599… MHz, exact in u32.
    assert_eq!(hz_from_gate(65_535, 40_960, 16_000_000), 25_599_609);
    // And the driver's realistic ceiling: 65535 counts in a 10 ms gate at
    // 1 MHz = 6.5535 MHz exactly.
    assert_eq!(hz_from_gate(65_535, 10_000, 1_000_000), 6_553_500);
}
