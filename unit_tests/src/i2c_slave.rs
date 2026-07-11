//! Host-side tests for `hal/src/i2c_slave.rs` — the eUSCI_B I2C slave-mode
//! own-address validation/encoding and the `UCB0IV` decode table that
//! `hal::i2c::I2cSlave` delegates to.
//!
//! Anchors: `UCOAEN` is **bit 10** and `UCGCEN` **bit 15** of `UCBxI2COA0`
//! (SLAU367P), the accepted own-address range is the spec-unreserved
//! `0x08..=0x77` (exactly the range the HAL's master bus scan probes), and
//! the IV slots are TI's `USCI_I2C_*` values from msp430fr5969.h. The IV
//! table is *documentation-pinned, not hardware-pinned* — this project has
//! already caught one vendor IV table wrong on silicon (the RTCIV alarm
//! slot), so these tests hold the driver to the header until the hardware
//! fixture can confirm each slot.

include!("../../hal/src/i2c_slave.rs");

// ---------------------------------------------------------------- encoding

#[test]
fn own_address_is_value_plus_oaen() {
    // 0x48 (the classic TMP102/register-file demo address): address in bits
    // 9:0, UCOAEN (bit 10) set, nothing else.
    assert_eq!(encode_own_address(0x48, false), Ok(0x0448));
}

#[test]
fn general_call_sets_ucgcen() {
    // UCGCEN is bit 15 of I2COA0 (and only I2COA0).
    assert_eq!(encode_own_address(0x48, true), Ok(0x8448));
}

#[test]
fn bit_positions_are_pinned() {
    // A shifted enable bit would still round-trip through encode/decode-style
    // tests; pin the raw positions to SLAU367P outright.
    assert_eq!(OA_ENABLE, 1 << 10);
    assert_eq!(OA_GENERAL_CALL, 1 << 15);
}

#[test]
fn accepted_range_boundaries() {
    // 0x08 and 0x77 are the first/last spec-unreserved 7-bit addresses —
    // also exactly the range the HAL's master bus scanner probes.
    assert_eq!(encode_own_address(0x08, false), Ok(0x0408));
    assert_eq!(encode_own_address(0x77, false), Ok(0x0477));
}

#[test]
fn reserved_addresses_rejected() {
    // Low block: general call/START byte (0x00), CBUS, HS-mode prefix...
    assert_eq!(encode_own_address(0x00, false), Err(AddressError::Reserved));
    assert_eq!(encode_own_address(0x07, false), Err(AddressError::Reserved));
    // High block: 10-bit prefix, device ID.
    assert_eq!(encode_own_address(0x78, false), Err(AddressError::Reserved));
    assert_eq!(encode_own_address(0x7F, false), Err(AddressError::Reserved));
}

#[test]
fn non_seven_bit_rejected() {
    assert_eq!(encode_own_address(0x80, false), Err(AddressError::OutOfRange));
    assert_eq!(encode_own_address(0xFF, false), Err(AddressError::OutOfRange));
}

// ---------------------------------------------------------------- IV decode

#[test]
fn iv_table_matches_the_header() {
    // The full priority-ordered eUSCI_B I2C table (msp430fr5969.h
    // USCI_I2C_*). A transposed pair — the classic off-by-one-slot error —
    // fails here.
    assert_eq!(decode_iv(0x00), SlaveIv::None);
    assert_eq!(decode_iv(0x02), SlaveIv::ArbitrationLost);
    assert_eq!(decode_iv(0x04), SlaveIv::Nack);
    assert_eq!(decode_iv(0x06), SlaveIv::Start);
    assert_eq!(decode_iv(0x08), SlaveIv::Stop);
    assert_eq!(decode_iv(0x16), SlaveIv::Rx);
    assert_eq!(decode_iv(0x18), SlaveIv::Tx);
    assert_eq!(decode_iv(0x1A), SlaveIv::ByteCounter);
    assert_eq!(decode_iv(0x1C), SlaveIv::ClockLowTimeout);
    assert_eq!(decode_iv(0x1E), SlaveIv::NinthBit);
}

#[test]
fn secondary_address_slots_are_grouped() {
    // Own addresses 1–3 (which the driver never enables) occupy 0x0A..=0x14;
    // they decode to SecondaryAddress carrying the raw slot for diagnostics.
    for iv in [0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14] {
        assert_eq!(decode_iv(iv), SlaveIv::SecondaryAddress(iv));
    }
}

#[test]
fn off_table_values_are_unknown_not_misattributed() {
    // The RTCIV lesson in miniature: if the silicon ever hands back a value
    // the table doesn't predict, it must surface as Unknown — loudly
    // loggable — rather than being folded into the nearest neighbor.
    assert_eq!(decode_iv(0x20), SlaveIv::Unknown(0x20));
    assert_eq!(decode_iv(0x03), SlaveIv::Unknown(0x03));
    assert_eq!(decode_iv(0xFFFF), SlaveIv::Unknown(0xFFFF));
}

#[test]
fn named_constants_match_their_slots() {
    assert_eq!(IV_NONE, 0x00);
    assert_eq!(IV_ARBITRATION_LOST, 0x02);
    assert_eq!(IV_NACK, 0x04);
    assert_eq!(IV_START, 0x06);
    assert_eq!(IV_STOP, 0x08);
    assert_eq!(IV_RX, 0x16);
    assert_eq!(IV_TX, 0x18);
    assert_eq!(IV_BYTE_COUNTER, 0x1A);
    assert_eq!(IV_CLOCK_LOW_TIMEOUT, 0x1C);
    assert_eq!(IV_NINTH_BIT, 0x1E);
}
