//! Host-side tests for the software CRC16-CCITT reference model.
//!
//! This module `include!`s the REAL source (`hal/src/crc_soft.rs`) — the model
//! that `hal::crc` re-exports and that the on-device `accel_test_firmware`
//! fixture compares the CRC16 silicon against. The catalog check values here
//! are the published ones (reveng's CRC catalogue), so the model is anchored
//! to the outside world; the fixture then anchors the hardware to the model.
//! A bad edit to either update loop fails these tests without a board.

// Pull in the actual model (pure, dependency-free core arithmetic).
include!("../../hal/src/crc_soft.rs");

#[cfg(test)]
mod tests {
    use super::*;

    /// The standard 9-byte check string used by every CRC catalogue.
    const CHECK: &[u8] = b"123456789";

    /// The four 0x1021-polynomial catalog variants the hardware module (and
    /// `hal::crc`'s convenience methods) expose, against their published
    /// check values.
    #[test]
    fn catalog_check_values() {
        // Non-reflected family (hardware: CRCDIRB in, CRCINIRES out).
        assert_eq!(crc16_ccitt_msb(0xFFFF, CHECK), 0x29B1); // CRC-16/CCITT-FALSE
        assert_eq!(crc16_ccitt_msb(0x0000, CHECK), 0x31C3); // CRC-16/XMODEM

        // Reflected family (hardware: CRCDI in, CRCRESR out).
        assert_eq!(crc16_ccitt_lsb(0x0000, CHECK), 0x2189); // CRC-16/KERMIT
        assert_eq!(crc16_ccitt_lsb(0xFFFF, CHECK) ^ 0xFFFF, 0x906E); // CRC-16/X-25
    }

    /// An empty message leaves the register at the seed — both conventions.
    #[test]
    fn empty_input_is_identity() {
        for seed in [0x0000, 0xFFFF, 0x1D0F, 0xB2AA] {
            assert_eq!(crc16_ccitt_msb(seed, &[]), seed);
            assert_eq!(crc16_ccitt_lsb(seed, &[]), seed);
        }
    }

    /// Feeding a message in pieces must equal feeding it whole — this is the
    /// property `Crc::begin`/`write_*`/`result` streaming relies on.
    #[test]
    fn incremental_equals_one_shot() {
        let data: Vec<u8> = (0u16..=255).map(|b| b as u8).collect();
        for split in [0, 1, 7, 128, 255, 256] {
            let (a, b) = data.split_at(split);
            assert_eq!(
                crc16_ccitt_msb(crc16_ccitt_msb(0xFFFF, a), b),
                crc16_ccitt_msb(0xFFFF, &data)
            );
            assert_eq!(
                crc16_ccitt_lsb(crc16_ccitt_lsb(0xFFFF, a), b),
                crc16_ccitt_lsb(0xFFFF, &data)
            );
        }
    }

    /// The two conventions are bit-reversals of one another: LSB-first over
    /// `data` == bit-reversed MSB-first over bit-reversed bytes (with the
    /// seed reversed too). This is exactly the CRCDI↔CRCDIRB /
    /// CRCINIRES↔CRCRESR relationship the hardware register pairs implement,
    /// so the fixture's cross-checks lean on it.
    #[test]
    fn conventions_are_bit_reversals() {
        let data: Vec<u8> = (0u16..=255).map(|b| (b as u8).wrapping_mul(37).wrapping_add(11)).collect();
        for seed in [0x0000u16, 0xFFFF, 0x8005, 0x1D0F] {
            let reversed: Vec<u8> = data.iter().map(|b| b.reverse_bits()).collect();
            assert_eq!(
                crc16_ccitt_lsb(seed, &data),
                crc16_ccitt_msb(seed.reverse_bits(), &reversed).reverse_bits()
            );
        }
    }

    /// A single-bit change anywhere in the message changes the checksum
    /// (linearity sanity — CRCs guarantee detection of all 1-bit errors).
    #[test]
    fn single_bit_errors_detected() {
        let base = crc16_ccitt_msb(0xFFFF, CHECK);
        for byte in 0..CHECK.len() {
            for bit in 0..8 {
                let mut corrupted = CHECK.to_vec();
                corrupted[byte] ^= 1 << bit;
                assert_ne!(crc16_ccitt_msb(0xFFFF, &corrupted), base);
            }
        }
    }
}
