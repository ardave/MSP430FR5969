//! CRC16 hardware module (CRC-CCITT, polynomial 0x1021).
//!
//! The MSP430FR5969 carries a 16-bit CRC accelerator: a hardware LFSR for the
//! CRC-CCITT polynomial x^16 + x^12 + x^5 + 1 (0x1021). Feeding it costs one
//! register write per byte (or word) with **no cycles spent on the bit math**
//! — the LFSR settles combinationally before the next write can retire, so
//! there is no busy flag and nothing to poll. That makes it worthwhile even
//! for short frames, and it is the natural integrity check for the FRAM
//! persistence this project already does (e.g. guarding an Info-FRAM config
//! block against a torn update).
//!
//! # One polynomial, two bit orders, four registers
//!
//! The module computes only the 0x1021 polynomial, but exposes **both bit
//! conventions** in which that polynomial is used in the wild (SLAU367P ch.
//! 16). The four data/result registers pair up:
//!
//! - [`write_msb_first`](Crc::write_msb_first) (`CRCDIRB`) +
//!   [`result`](Crc::result) (`CRCINIRES`) — the *non-reflected* ("MSB-first")
//!   family: CRC-16/CCITT-FALSE, CRC-16/XMODEM.
//! - [`write_lsb_first`](Crc::write_lsb_first) (`CRCDI`) +
//!   [`result_bit_reversed`](Crc::result_bit_reversed) (`CRCRESR`) — the
//!   *reflected* ("LSB-first") family: CRC-16/KERMIT, CRC-16/X-25.
//!
//! The hardware holds a single shift register; `CRCDIRB` bit-reverses data on
//! the way in and `CRCRESR` bit-reverses the register on the way out, which
//! is exactly the textbook relationship between the two conventions. The four
//! catalog variants ship as one-shot convenience methods
//! ([`ccitt_false`](Crc::ccitt_false), [`xmodem`](Crc::xmodem),
//! [`kermit`](Crc::kermit), [`x25`](Crc::x25)); their published check values
//! over `"123456789"` (0x29B1 / 0x31C3 / 0x2189 / 0x906E) are pinned by the
//! host unit tests against the software model in `crc_soft.rs` and by the
//! `accel_test_firmware` fixture against the silicon.
//!
//! # Byte writes through raw pointers
//!
//! Byte-granular input requires **byte-wide** stores to the data registers'
//! low byte (`CRCDI_L`/`CRCDIRB_L`): an 8-bit store clocks 8 bits into the
//! LFSR, a 16-bit store clocks 16. The PAC's register API only emits 16-bit
//! accesses, so [`write_msb_first`]/[`write_lsb_first`] go through raw
//! volatile byte pointers derived from the owned peripheral's base address —
//! same approach as the MSP430X byte primitives in [`fram`](crate::fram).
//! Word-at-a-time input is available too ([`write_words_msb_first`]): within
//! a 16-bit write to `CRCDIRB` the hardware processes the **lower byte
//! first** (HW-verified 2026-07-05), so the word path equals the byte path
//! over the words' little-endian serialization — i.e. checksumming a
//! `&[u16]` gives the same result as checksumming the same bytes in memory
//! order on this little-endian CPU.
//!
//! [`write_msb_first`]: Crc::write_msb_first
//! [`write_lsb_first`]: Crc::write_lsb_first
//! [`write_words_msb_first`]: Crc::write_words_msb_first
//!
//! # Example: guard a config block
//!
//! ```ignore
//! let mut crc = hal::crc::Crc::new(p.crc16);
//! let stored = crc.ccitt_false(&config_bytes);
//! // ... persist config_bytes + stored in Info FRAM; on boot, recompute and compare.
//! ```

pub use crate::crc_soft::{crc16_ccitt_lsb, crc16_ccitt_msb};

/// Byte offsets of the CRC16 registers from the module base (0x0150).
const CRCDI_OFFSET: usize = 0; // CRC data in
const CRCDIRB_OFFSET: usize = 2; // CRC data in, bit order reversed

/// Owned driver for the CRC16 module.
///
/// Owning the PAC peripheral makes concurrent feeders a compile error — the
/// module is a single shared LFSR, so an interleaved writer would silently
/// corrupt both checksums.
pub struct Crc {
    crc: pac::Crc16,
}

use crate::pac;

impl Crc {
    /// Take ownership of the CRC16 module.
    ///
    /// No configuration exists (or is needed): the polynomial is fixed and
    /// there are no clocks to enable — the module is combinational logic on
    /// the peripheral bus.
    pub fn new(crc: pac::Crc16) -> Self {
        Crc { crc }
    }

    /// Release the underlying PAC peripheral.
    pub fn free(self) -> pac::Crc16 {
        self.crc
    }

    /// Start a new checksum: load `seed` as the initial shift-register value
    /// (`CRCINIRES`). 0xFFFF and 0x0000 — the seeds of all four catalog
    /// variants — read the same in both bit conventions.
    pub fn begin(&mut self, seed: u16) {
        self.crc.crcinires().write(|w| unsafe { w.bits(seed) });
    }

    /// Feed bytes in the **non-reflected (MSB-first)** convention: each byte's
    /// most significant bit enters the LFSR first (`CRCDIRB_L` byte writes).
    /// Pair with [`result`](Crc::result).
    pub fn write_msb_first(&mut self, data: &[u8]) {
        for &b in data {
            // SAFETY: byte store to CRCDIRB_L within the owned register block;
            // an 8-bit access is architecturally defined to clock exactly 8
            // bits (SLAU367P 16.2.1). Volatile so every write is performed in
            // order — each one advances the LFSR.
            unsafe { byte_reg(&self.crc, CRCDIRB_OFFSET).write_volatile(b) };
        }
    }

    /// Feed bytes in the **reflected (LSB-first)** convention: each byte's
    /// least significant bit enters the LFSR first (`CRCDI_L` byte writes).
    /// Pair with [`result_bit_reversed`](Crc::result_bit_reversed).
    pub fn write_lsb_first(&mut self, data: &[u8]) {
        for &b in data {
            // SAFETY: as in `write_msb_first`, but CRCDI_L.
            unsafe { byte_reg(&self.crc, CRCDI_OFFSET).write_volatile(b) };
        }
    }

    /// Feed 16-bit words in the MSB-first convention. A word write to
    /// `CRCDIRB` processes the **lower byte first**, so this equals
    /// [`write_msb_first`](Crc::write_msb_first) over the words'
    /// little-endian byte serialization (= memory order on this CPU) — at
    /// half the store count.
    pub fn write_words_msb_first(&mut self, data: &[u16]) {
        for &w in data {
            self.crc.crcdirb().write(|r| unsafe { r.bits(w) });
        }
    }

    /// The running checksum in the non-reflected orientation (`CRCINIRES`).
    /// This is the result register for [`write_msb_first`](Crc::write_msb_first)
    /// data (CCITT-FALSE / XMODEM family). Reading does not disturb the LFSR.
    pub fn result(&self) -> u16 {
        self.crc.crcinires().read().bits()
    }

    /// The running checksum bit-reversed (`CRCRESR`) — the result register
    /// for [`write_lsb_first`](Crc::write_lsb_first) data (KERMIT / X-25
    /// family, before any final XOR). Reading does not disturb the LFSR.
    pub fn result_bit_reversed(&self) -> u16 {
        self.crc.crcresr().read().bits()
    }

    /// One-shot CRC-16/CCITT-FALSE: seed 0xFFFF, MSB-first, no final XOR.
    /// `"123456789"` → 0x29B1.
    pub fn ccitt_false(&mut self, data: &[u8]) -> u16 {
        self.begin(0xFFFF);
        self.write_msb_first(data);
        self.result()
    }

    /// One-shot CRC-16/XMODEM: seed 0x0000, MSB-first, no final XOR.
    /// `"123456789"` → 0x31C3.
    pub fn xmodem(&mut self, data: &[u8]) -> u16 {
        self.begin(0x0000);
        self.write_msb_first(data);
        self.result()
    }

    /// One-shot CRC-16/KERMIT: seed 0x0000, reflected in and out, no final
    /// XOR. `"123456789"` → 0x2189.
    pub fn kermit(&mut self, data: &[u8]) -> u16 {
        self.begin(0x0000);
        self.write_lsb_first(data);
        self.result_bit_reversed()
    }

    /// One-shot CRC-16/X-25: seed 0xFFFF, reflected in and out, final XOR
    /// 0xFFFF. `"123456789"` → 0x906E.
    pub fn x25(&mut self, data: &[u8]) -> u16 {
        self.begin(0xFFFF);
        self.write_lsb_first(data);
        self.result_bit_reversed() ^ 0xFFFF
    }
}

/// Byte-wide pointer to the low byte of a CRC data register.
///
/// Borrowing `&pac::Crc16` (rather than using `Crc16::ptr()` freestanding)
/// ties the raw access to the owned peripheral for the reader, even though
/// the address is the same either way.
fn byte_reg(crc: &pac::Crc16, offset: usize) -> *mut u8 {
    let _ = crc;
    (pac::Crc16::ptr() as *mut u8).wrapping_add(offset)
}
