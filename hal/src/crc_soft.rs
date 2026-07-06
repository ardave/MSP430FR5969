// Pure software CRC16-CCITT bit models — the reference the CRC16 hardware
// module is checked against.
//
// Like `baud.rs` and `ticks.rs`, this file is intentionally dependency-free
// (pure `core` integer arithmetic, no PAC/HAL types) so the exact same source
// can be `include!`d by the host-side test crate in `unit_tests/`. The
// hardware driver in `crc.rs` re-exports these functions, and the on-device
// `accel_test_runner` fixture feeds identical data through both the silicon
// and this model — so the model is pinned twice: against the published CRC
// catalog constants on the host, and against the real LFSR on hardware.
// (Regular `//` comments, not `//!`, so the file can be `include!`d mid-crate.)
//
// Both functions implement the CRC-16 polynomial x^16 + x^12 + x^5 + 1
// (0x1021), the only polynomial the MSP430 CRC16 module computes. The two
// bit-endianness conventions in circulation are both here because the module
// exposes both: `CRCDIRB`/`CRCINIRES` speak MSB-first ("CCITT-FALSE",
// "XMODEM"), `CRCDI`/`CRCRESR` speak LSB-first ("KERMIT", "X-25").

/// MSB-first (non-reflected) CRC-16 update over `data`, polynomial 0x1021.
///
/// `seed` is the initial shift-register value; the return value is the final
/// register, with no output reflection or final XOR. Named variants:
/// CRC-16/CCITT-FALSE is `crc16_ccitt_msb(0xFFFF, data)`; CRC-16/XMODEM is
/// `crc16_ccitt_msb(0x0000, data)`. Check value: over the ASCII bytes of
/// `"123456789"`, CCITT-FALSE = 0x29B1 and XMODEM = 0x31C3.
pub fn crc16_ccitt_msb(seed: u16, data: &[u8]) -> u16 {
    let mut crc = seed;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// LSB-first (reflected) CRC-16 update over `data` — polynomial 0x1021 run in
/// its bit-reversed form 0x8408.
///
/// `seed` is the initial shift-register value in the reflected orientation;
/// no final XOR is applied. Named variants: CRC-16/KERMIT is
/// `crc16_ccitt_lsb(0x0000, data)`; CRC-16/X-25 is
/// `crc16_ccitt_lsb(0xFFFF, data) ^ 0xFFFF`. Check value: over the ASCII
/// bytes of `"123456789"`, KERMIT = 0x2189 and X-25 = 0x906E.
pub fn crc16_ccitt_lsb(seed: u16, data: &[u8]) -> u16 {
    let mut crc = seed;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0x8408 } else { crc >> 1 };
        }
    }
    crc
}
