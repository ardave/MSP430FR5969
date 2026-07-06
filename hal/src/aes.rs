//! AES256 hardware accelerator (AES-128/-192/-256 encrypt/decrypt).
//!
//! The MSP430FR5969's AES accelerator is a full FIPS-197 block cipher engine:
//! it performs the key expansion **and** all cipher rounds in silicon — an
//! AES-128 block encryption completes in 168 MCLK cycles (SLAU367P Table
//! 21-1), where a software AES on this CPU costs thousands of cycles and a
//! ~2 KB table against a 2 KB RAM budget. The driver below is deliberately a
//! *blocking* driver: at 168 cycles an operation finishes in ~21 µs at 1 MHz
//! — far cheaper to poll (`AESBUSY`) than to arrange any interrupt or DMA
//! hand-off, and this device routes no AES interrupt to a CPU vector anyway.
//!
//! # What the hardware does and doesn't do
//!
//! The engine is a **single-block ECB primitive**: load a key, write 16
//! bytes, poll busy, read 16 bytes. Everything above that is software:
//!
//! - [`encrypt_blocks`](Aes::encrypt_blocks) / [`decrypt_blocks`](Aes::decrypt_blocks)
//!   — raw ECB over a multiple of 16 bytes. ECB leaks equal-block structure;
//!   it is exposed because it *is* the primitive, but prefer CBC for data.
//! - [`encrypt_cbc`](Aes::encrypt_cbc) / [`decrypt_cbc`](Aes::decrypt_cbc) —
//!   CBC chaining done in software around the hardware block op (a 16-byte
//!   XOR is not worth the `AESAXDIN` register choreography the hardware's
//!   own cipher-mode support requires — that support, `AESCMEN`, exists for
//!   DMA-driven operation and is not used here).
//!
//! No padding is applied anywhere: lengths must be block multiples, callers
//!   own the padding scheme (checked, [`Error::NotBlockAligned`]).
//!
//! # Key handling
//!
//! The key is written once into `AESAKEY` and the engine keeps its expanded
//! schedule between blocks, so per-block cost is just data I/O. Two wrinkles
//! the driver hides:
//!
//! - **Direction changes reload the key.** The operation mode (`AESOPx`,
//!   encrypt vs decrypt) must be configured *before* the key is loaded, and
//!   rewriting `AESACTL0` invalidates the loaded key state — so the driver
//!   tracks which direction the hardware currently holds and transparently
//!   re-loads the key (from its own copy) when you switch between encrypt
//!   and decrypt calls.
//! - **Decrypt uses `AESOPx = 01`** ("decrypt with the encryption key"): the
//!   engine runs the reverse key schedule itself each block (274 cycles for
//!   AES-256 instead of 234 pre-expanded — not worth the extra
//!   generate-first-round-key state machine at these speeds).
//!
//! The driver therefore keeps a copy of the key material in the struct (the
//! hardware offers no key readback — `AESAKEY` reads as zero). Drop the
//! driver (or overwrite with [`set_key`](Aes::set_key)) when the key's
//! lifetime ends; [`free`](Aes::free) software-resets the module, which
//! clears the loaded key and state out of the silicon.
//!
//! # Byte order
//!
//! FIPS-197 defines the state as a byte sequence; the 16-bit registers take
//! it little-endian: byte *i* of the block is the low byte of word *i/2*
//! written to `AESAKEY`/`AESADIN` (and read from `AESADOUT`). Verified
//! against the FIPS-197 appendix-C known-answer vectors by the
//! `accel_test_runner` fixture.
//!
//! # Example
//!
//! ```ignore
//! use hal::aes::{Aes, Key};
//!
//! let mut aes = Aes::new(p.aes_accelerator, Key::Aes128(*b"an example key!!"));
//! let mut msg = *b"hello, aes block"; // exactly one 16-byte block
//! aes.encrypt_blocks(&mut msg).unwrap();
//! aes.decrypt_blocks(&mut msg).unwrap(); // round-trips
//! ```

use crate::pac;

/// AES block size in bytes — fixed by FIPS-197 for every key length.
pub const BLOCK_SIZE: usize = 16;

/// An AES key, sized by variant. Owned (copied into the driver): the driver
/// must be able to re-load it when switching cipher direction, and the
/// hardware cannot read keys back.
#[derive(Clone)]
pub enum Key {
    /// AES-128 (10 rounds, 168-cycle encrypt).
    Aes128([u8; 16]),
    /// AES-192 (12 rounds, 204-cycle encrypt).
    Aes192([u8; 24]),
    /// AES-256 (14 rounds, 234-cycle encrypt).
    Aes256([u8; 32]),
}

impl Key {
    /// The raw key bytes.
    fn bytes(&self) -> &[u8] {
        match self {
            Key::Aes128(k) => k,
            Key::Aes192(k) => k,
            Key::Aes256(k) => k,
        }
    }
}

/// Errors from the multi-block operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// Data length is not a multiple of the 16-byte AES block size. The
    /// hardware only speaks whole blocks; padding is the caller's protocol
    /// decision, not something a HAL should invent.
    NotBlockAligned,
}

/// Cipher direction currently loaded into the engine (`AESOPx`).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Op {
    Encrypt, // AESOPx = 00
    Decrypt, // AESOPx = 01 (decrypt using the encryption key)
}

/// Owned blocking driver for the AES accelerator.
pub struct Aes {
    aes: pac::AesAccelerator,
    key: Key,
    /// Which direction the hardware currently holds a loaded key for.
    /// `None` until the first operation (and after a key change).
    loaded: Option<Op>,
}

impl Aes {
    /// Take ownership of the accelerator and set the initial key.
    ///
    /// Performs a module software reset (`AESSWRST`) so no state from a
    /// previous owner — including a stale `AESERRFG` from an aborted
    /// key/data sequence — survives into this driver.
    pub fn new(aes: pac::AesAccelerator, key: Key) -> Self {
        let mut this = Aes {
            aes,
            key,
            loaded: None,
        };
        this.reset();
        this
    }

    /// Replace the key. Takes effect on the next operation (lazy load, like
    /// the initial key).
    pub fn set_key(&mut self, key: Key) {
        self.key = key;
        self.loaded = None;
    }

    /// Software-reset the module and release the underlying PAC peripheral.
    /// The reset clears the loaded key schedule out of the engine.
    pub fn free(mut self) -> pac::AesAccelerator {
        self.reset();
        self.aes
    }

    /// Encrypt `data` in place in ECB mode. Length must be a multiple of 16.
    ///
    /// Every block is enciphered independently — equal plaintext blocks give
    /// equal ciphertext blocks. For anything longer than one block prefer
    /// [`encrypt_cbc`](Aes::encrypt_cbc).
    pub fn encrypt_blocks(&mut self, data: &mut [u8]) -> Result<(), Error> {
        for block in blocks_mut(data)? {
            self.process_block(Op::Encrypt, block);
        }
        Ok(())
    }

    /// Decrypt ECB-mode `data` in place. Length must be a multiple of 16.
    pub fn decrypt_blocks(&mut self, data: &mut [u8]) -> Result<(), Error> {
        for block in blocks_mut(data)? {
            self.process_block(Op::Decrypt, block);
        }
        Ok(())
    }

    /// Encrypt `data` in place in CBC mode with the given initialization
    /// vector. Length must be a multiple of 16. The IV should be unique per
    /// message (a repeated IV leaks shared prefixes); it is not secret and
    /// is typically transmitted alongside the ciphertext.
    pub fn encrypt_cbc(&mut self, iv: &[u8; BLOCK_SIZE], data: &mut [u8]) -> Result<(), Error> {
        let mut chain = *iv;
        for block in blocks_mut(data)? {
            for (b, c) in block.iter_mut().zip(chain.iter()) {
                *b ^= c;
            }
            self.process_block(Op::Encrypt, block);
            chain = *block;
        }
        Ok(())
    }

    /// Decrypt CBC-mode `data` in place with the given initialization vector.
    /// Length must be a multiple of 16.
    pub fn decrypt_cbc(&mut self, iv: &[u8; BLOCK_SIZE], data: &mut [u8]) -> Result<(), Error> {
        let mut chain = *iv;
        for block in blocks_mut(data)? {
            let ct = *block; // next block chains off the *ciphertext*
            self.process_block(Op::Decrypt, block);
            for (b, c) in block.iter_mut().zip(chain.iter()) {
                *b ^= c;
            }
            chain = ct;
        }
        Ok(())
    }

    /// Run one 16-byte block through the engine in the given direction,
    /// (re)loading the key first if the hardware holds the other direction.
    fn process_block(&mut self, op: Op, block: &mut [u8; BLOCK_SIZE]) {
        if self.loaded != Some(op) {
            self.load_key(op);
        }
        // Write the state; the operation starts itself on the 8th word
        // (AESDINWR). Little-endian: byte 0 is the low byte of the first word.
        for chunk in block.chunks_exact(2) {
            self.aes
                .aesadin()
                .write(|w| unsafe { w.bits(u16::from_le_bytes([chunk[0], chunk[1]])) });
        }
        while self.aes.aesastat().read().aesbusy().bit_is_set() {}
        // Read the full result back; the counters (AESDOUTCNT) require the
        // words in order, and leaving any unread would poison the next block.
        for chunk in block.chunks_exact_mut(2) {
            let word = self.aes.aesadout().read().bits();
            chunk.copy_from_slice(&word.to_le_bytes());
        }
    }

    /// Configure `AESOPx` + key length and load the key. Must not run while
    /// the engine is busy (SLAU367P forbids touching `AESACTL0` then).
    fn load_key(&mut self, op: Op) {
        while self.aes.aesastat().read().aesbusy().bit_is_set() {}
        self.aes.aesactl0().write(|w| {
            let w = match op {
                Op::Encrypt => w.aesop().aesop_0(),
                Op::Decrypt => w.aesop().aesop_1(),
            };
            match self.key {
                Key::Aes128(_) => w.aeskl().aeskl_0(),
                Key::Aes192(_) => w.aeskl().aeskl_1(),
                Key::Aes256(_) => w.aeskl().aeskl_2(),
            }
        });
        for chunk in self.key.bytes().chunks_exact(2) {
            self.aes
                .aesakey()
                .write(|w| unsafe { w.bits(u16::from_le_bytes([chunk[0], chunk[1]])) });
        }
        // AESKEYWR confirms the engine saw a complete key of the configured
        // length — a sequencing bug here would otherwise surface as garbage
        // ciphertext three calls later.
        while self.aes.aesastat().read().aeskeywr().bit_is_clear() {}
        self.loaded = Some(op);
    }

    /// Pulse `AESSWRST`: aborts any operation, clears the loaded key, the
    /// byte counters, and `AESERRFG`. The subsequent zero write is implicit —
    /// `write` clears every other bit, and the reset bit itself self-clears.
    fn reset(&mut self) {
        self.aes.aesactl0().write(|w| w.aesswrst().set_bit());
        self.loaded = None;
    }
}

/// View `data` as exact 16-byte blocks, or fail if it doesn't divide evenly.
fn blocks_mut(data: &mut [u8]) -> Result<&mut [[u8; BLOCK_SIZE]], Error> {
    let (blocks, rest) = data.as_chunks_mut::<BLOCK_SIZE>();
    if rest.is_empty() {
        Ok(blocks)
    } else {
        Err(Error::NotBlockAligned)
    }
}
