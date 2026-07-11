// Pure eUSCI_B I2C **slave-mode** register math for the `i2c` module.
//
// Like `rtc_alarm.rs` and `mpu_seg.rs`, this file is intentionally
// dependency-free (pure `core` integer arithmetic, no PAC/HAL types) so the
// exact same source can be `include!`d by the host-side test crate in
// `unit_tests/`. The `i2c::I2cSlave` driver is a thin wrapper over these
// conversions — a regression here (a shifted `UCOAEN` bit, an accepted
// reserved address, a transposed IV slot) fails the host tests without a
// board on the desk. Do not add external `use`s. (Regular `//` comments, not
// `//!`, so the file can be `include!`d mid-crate.)
//
// # The hardware's own-address model (SLAU367P, eUSCI_B I2C mode)
//
// A slave answers the addresses programmed into `UCBxI2COA0..3`. This driver
// uses only `I2COA0`: bits 9:0 hold the address, bit 10 (`UCOAEN`) enables
// matching, and — on `I2COA0` only — bit 15 (`UCGCEN`) additionally answers
// the I2C general call (address 0x00). The other three own-address registers
// and the `ADDMASK` range-matching stay at their reset values (disabled /
// match-all-bits) so exactly one address is answered.
//
// # The interrupt-vector table (UCBxIV)
//
// The `USCI_B0` vector demuxes through `UCBxIV`, whose read atomically clears
// the served (highest-priority pending) flag. The slot values below are TI's
// `USCI_I2C_*` constants from `msp430fr5969.h`. **They are not yet verified
// on hardware** — and this project has already caught one vendor-table
// discrepancy the hard way (the RTCIV alarm slot, documented 0x08, observed
// 0x06), so treat `decode_iv` as the documented-table reference that the
// eventual hardware fixture must confirm slot by slot.

/// `UCOAEN` — own-address-enable bit in `UCBxI2COA0..3` (bit 10).
pub const OA_ENABLE: u16 = 0x0400;

/// `UCGCEN` — general-call-enable bit, `UCBxI2COA0` only (bit 15).
pub const OA_GENERAL_CALL: u16 = 0x8000;

/// Reasons a 7-bit own address is rejected.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AddressError {
    /// Not a 7-bit address (`> 0x7F`).
    OutOfRange,
    /// A 7-bit address the I2C-bus specification reserves (`0x00..=0x07`:
    /// general call, START byte, CBUS, high-speed prefix; `0x78..=0x7F`:
    /// 10-bit prefix, device ID). Masters — including this HAL's bus
    /// scanner — only probe `0x08..=0x77`, so a slave parked on a reserved
    /// address would be unreachable by well-behaved bus citizens.
    Reserved,
}

/// Validate a 7-bit own address and encode the `UCBxI2COA0` register word:
/// the address in bits 9:0, `UCOAEN` set, and `UCGCEN` when the slave should
/// also answer the general call (address 0x00).
pub fn encode_own_address(address: u8, general_call: bool) -> Result<u16, AddressError> {
    if address > 0x7F {
        return Err(AddressError::OutOfRange);
    }
    if !(0x08..=0x77).contains(&address) {
        return Err(AddressError::Reserved);
    }
    let mut word = address as u16 | OA_ENABLE;
    if general_call {
        word |= OA_GENERAL_CALL;
    }
    Ok(word)
}

// ---------------------------------------------------------------------------
// UCBxIV decode
// ---------------------------------------------------------------------------

// TI's USCI_I2C_* interrupt-vector values (msp430fr5969.h). The eUSCI_B I2C
// IV is priority-ordered: bus errors first, then framing, then the four
// receive/transmit pairs from own-address 3 down to own-address 0, then the
// byte counter and timeout sources.
pub const IV_NONE: u16 = 0x00;
pub const IV_ARBITRATION_LOST: u16 = 0x02; // UCALIFG
pub const IV_NACK: u16 = 0x04; // UCNACKIFG
pub const IV_START: u16 = 0x06; // UCSTTIFG (own address + R/W received)
pub const IV_STOP: u16 = 0x08; // UCSTPIFG
pub const IV_RX3: u16 = 0x0A; // UCRXIFG3
pub const IV_TX3: u16 = 0x0C; // UCTXIFG3
pub const IV_RX2: u16 = 0x0E; // UCRXIFG2
pub const IV_TX2: u16 = 0x10; // UCTXIFG2
pub const IV_RX1: u16 = 0x12; // UCRXIFG1
pub const IV_TX1: u16 = 0x14; // UCTXIFG1
pub const IV_RX: u16 = 0x16; // UCRXIFG0 (data received, own address 0)
pub const IV_TX: u16 = 0x18; // UCTXIFG0 (TXBUF empty, own address 0)
pub const IV_BYTE_COUNTER: u16 = 0x1A; // UCBCNTIFG
pub const IV_CLOCK_LOW_TIMEOUT: u16 = 0x1C; // UCCLTOIFG
pub const IV_NINTH_BIT: u16 = 0x1E; // UCBIT9IFG

/// A decoded `UCBxIV` value, from the slave's point of view.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SlaveIv {
    /// 0x00 — no interrupt pending.
    None,
    /// 0x02 — arbitration lost (multi-master; not produced in a
    /// single-master-plus-this-slave topology).
    ArbitrationLost,
    /// 0x04 — NACK received. As a slave transmitter this is the master's
    /// normal "that was the last byte" signal, not an error.
    Nack,
    /// 0x06 — START + own address received; direction is in `UCTR`.
    Start,
    /// 0x08 — STOP received.
    Stop,
    /// 0x16 — a data byte is in RXBUF (own address 0).
    Rx,
    /// 0x18 — TXBUF is empty and the master wants a byte (own address 0).
    Tx,
    /// 0x0A..=0x14 — receive/transmit for secondary own addresses 1–3,
    /// which this driver never enables.
    SecondaryAddress(u16),
    /// 0x1A — byte-counter threshold (`UCBCNTIFG`; unused by this driver).
    ByteCounter,
    /// 0x1C — clock-low timeout (`UCCLTOIFG`; the slave driver leaves the
    /// timeout disabled — a slave stretching SCL is legitimate).
    ClockLowTimeout,
    /// 0x1E — ninth-bit position (`UCBIT9IFG`; unused by this driver).
    NinthBit,
    /// Anything outside the documented table.
    Unknown(u16),
}

/// Decode a `UCBxIV` read. Total (any `u16` maps to something), so an ISR can
/// log an off-table value instead of misattributing it.
pub fn decode_iv(iv: u16) -> SlaveIv {
    match iv {
        IV_NONE => SlaveIv::None,
        IV_ARBITRATION_LOST => SlaveIv::ArbitrationLost,
        IV_NACK => SlaveIv::Nack,
        IV_START => SlaveIv::Start,
        IV_STOP => SlaveIv::Stop,
        IV_RX3 | IV_TX3 | IV_RX2 | IV_TX2 | IV_RX1 | IV_TX1 => SlaveIv::SecondaryAddress(iv),
        IV_RX => SlaveIv::Rx,
        IV_TX => SlaveIv::Tx,
        IV_BYTE_COUNTER => SlaveIv::ByteCounter,
        IV_CLOCK_LOW_TIMEOUT => SlaveIv::ClockLowTimeout,
        IV_NINTH_BIT => SlaveIv::NinthBit,
        other => SlaveIv::Unknown(other),
    }
}
