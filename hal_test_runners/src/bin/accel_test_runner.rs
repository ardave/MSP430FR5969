#![no_std]
#![no_main]

//! Hardware-accelerator integration fixture (CRC16 + AES256) — **no wiring at
//! all**, driven by the host-side `accel_tests` runner. Both modules are pure
//! bus peripherals: data in, data out, nothing off-chip.
//!
//! ```text
//! cargo +nightly build --bin accel_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/accel_test_runner
//! ```
//!
//! # What it checks
//!
//! - **CRC MODEL** — the CRC16 silicon against the software reference model
//!   (`hal::crc::crc16_ccitt_msb`/`_lsb`, the same `crc_soft.rs` source the
//!   host unit tests pin to the published catalog constants): three byte
//!   patterns (the standard `"123456789"` check string, a 256-byte ramp
//!   covering every byte value, and an odd 7-byte pattern) × both seeds
//!   (0xFFFF / 0x0000) × both bit-order register pairs, plus the
//!   `CRCRESR == bitrev16(CRCINIRES)` cross-check and the word-write fast
//!   path (`CRCDIRB` processes the lower byte of a word first) against the
//!   byte path.
//! - **CRC CATALOG** — the four named one-shots over `"123456789"` against
//!   their published check values (CCITT-FALSE 0x29B1, XMODEM 0x31C3,
//!   KERMIT 0x2189, X-25 0x906E), hardcoded — *not* derived from the model,
//!   so a systematically wrong model cannot vouch for itself.
//! - **AES128/192/256 KAT** — the FIPS-197 appendix-C known-answer vectors
//!   (one plaintext, three key lengths), encrypt direction. These pin the
//!   register byte order and the key-length configuration.
//! - **AES DECRYPT** — each appendix-C ciphertext decrypted back to the
//!   plaintext, alternating direction per key so every call crosses the
//!   encrypt↔decrypt boundary and exercises the driver's transparent key
//!   reload.
//! - **AES CBC** — the SP800-38A F.2.1 CBC-AES128 two-block encrypt vector,
//!   then decrypt round-trip (chaining, IV handling).
//! - **AES ROUNDTRIP** — 48 bytes ECB-encrypted with AES-256, verified
//!   changed, decrypted, verified restored (multi-block sequencing beyond
//!   the single-block KATs).
//!
//! # Framed output for the host runner
//!
//! ```text
//! accel crc ccitt=29B1 xmodem=31C3 kermit=2189 x25=906E aes128ct=69C4E0D8
//! ACCEL_TEST_BEGIN
//! ACCEL CRC MODEL OK
//! ACCEL CRC CATALOG OK
//! ACCEL AES128 KAT OK
//! ACCEL AES192 KAT OK
//! ACCEL AES256 KAT OK
//! ACCEL AES DECRYPT OK
//! ACCEL AES CBC OK
//! ACCEL AES ROUNDTRIP OK
//! ACCEL_TEST_END
//! ```
//!
//! **GREEN** LED while all checks pass, **RED** otherwise. The info line
//! carries the raw catalog values so a routing mix-up between the two CRC
//! bit-order register pairs is diagnosable from one line of output.

use hal::aes::{Aes, Key};
use hal::crc::{crc16_ccitt_lsb, crc16_ccitt_msb, Crc};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// The standard CRC-catalogue check string.
const CHECK: &[u8] = b"123456789";

/// FIPS-197 appendix C: the one plaintext all three key lengths share.
const FIPS_PT: [u8; 16] = [
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee,
    0xff,
];
/// FIPS-197 C.1: AES-128 key 000102...0f and its ciphertext.
const FIPS_K128: [u8; 16] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
    0x0f,
];
const FIPS_CT128: [u8; 16] = [
    0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5,
    0x5a,
];
/// FIPS-197 C.2: AES-192.
const FIPS_K192: [u8; 24] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
    0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
];
const FIPS_CT192: [u8; 16] = [
    0xdd, 0xa9, 0x7c, 0xa4, 0x86, 0x4c, 0xdf, 0xe0, 0x6e, 0xaf, 0x70, 0xa0, 0xec, 0x0d, 0x71,
    0x91,
];
/// FIPS-197 C.3: AES-256.
const FIPS_K256: [u8; 32] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
    0x0f, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d,
    0x1e, 0x1f,
];
const FIPS_CT256: [u8; 16] = [
    0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf, 0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60,
    0x89,
];

/// NIST SP800-38A F.2.1 CBC-AES128.Encrypt: key, IV, and the first two
/// plaintext/ciphertext block pairs.
const CBC_KEY: [u8; 16] = [
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f,
    0x3c,
];
const CBC_IV: [u8; 16] = [
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
    0x0f,
];
const CBC_PT: [u8; 32] = [
    0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17,
    0x2a, 0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf,
    0x8e, 0x51,
];
const CBC_CT: [u8; 32] = [
    0x76, 0x49, 0xab, 0xac, 0x81, 0x19, 0xb2, 0x46, 0xce, 0xe9, 0x8e, 0x9b, 0x12, 0xe9, 0x19,
    0x7d, 0x50, 0x86, 0xcb, 0x9b, 0x50, 0x72, 0x19, 0xee, 0x95, 0xdb, 0x11, 0x3a, 0x91, 0x76,
    0x78, 0xb2,
];

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (SMCLK feeds the UART BRCLK below).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the UART pin mux takes effect. (Neither
    // accelerator has pins.)
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1).
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut crc = Crc::new(p.crc16);
    let mut aes = Aes::new(p.aes_accelerator, Key::Aes128(FIPS_K128));

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"MSP430FR5969 hardware accelerators: CRC16 + AES256 (no wiring)\r\n")
        .ok();

    loop {
        // ---- CRC vs the software model, both bit orders, both seeds -----
        let mut ramp = [0u8; 256];
        for (i, b) in ramp.iter_mut().enumerate() {
            *b = i as u8;
        }
        let odd: [u8; 7] = [0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x80, 0x7F];

        let mut model_ok = true;
        for data in [CHECK, &ramp[..], &odd[..]] {
            for seed in [0xFFFFu16, 0x0000] {
                // MSB-first pair: CRCDIRB in, CRCINIRES out.
                crc.begin(seed);
                crc.write_msb_first(data);
                let inires = crc.result();
                let resr = crc.result_bit_reversed();
                model_ok &= inires == crc16_ccitt_msb(seed, data);
                // The two result registers are bit-reversals of one register.
                model_ok &= resr == inires.reverse_bits();

                // LSB-first pair: CRCDI in, CRCRESR out.
                crc.begin(seed);
                crc.write_lsb_first(data);
                model_ok &= crc.result_bit_reversed() == crc16_ccitt_lsb(seed, data);
            }
        }
        // Word fast path: "12345678" packed little-endian must match the
        // byte path (a CRCDIRB word write processes the lower byte first —
        // HW-established here 2026-07-05: the check string via word writes
        // reads 0x3D14, the lower-byte-first value, not upper-first 0xA12B).
        let words: [u16; 4] = [0x3231, 0x3433, 0x3635, 0x3837];
        crc.begin(0xFFFF);
        crc.write_words_msb_first(&words);
        let word_result = crc.result();
        model_ok &= word_result == crc16_ccitt_msb(0xFFFF, b"12345678");

        // Diagnostic pair for the info line: both result registers after one
        // MSB-first feed of the check string (expected 29B1 / bitrev 8D94).
        crc.begin(0xFFFF);
        crc.write_msb_first(CHECK);
        let diag_inires = crc.result();
        let diag_resr = crc.result_bit_reversed();

        // ---- CRC catalog one-shots vs published constants ---------------
        let ccitt = crc.ccitt_false(CHECK);
        let xmodem = crc.xmodem(CHECK);
        let kermit = crc.kermit(CHECK);
        let x25 = crc.x25(CHECK);
        let catalog_ok =
            ccitt == 0x29B1 && xmodem == 0x31C3 && kermit == 0x2189 && x25 == 0x906E;

        // ---- AES known-answer tests (FIPS-197 appendix C), encrypt ------
        let mut kat = |key: Key, ct: &[u8; 16]| -> bool {
            aes.set_key(key);
            let mut block = FIPS_PT;
            aes.encrypt_blocks(&mut block).unwrap();
            block == *ct
        };
        let kat128_ok = kat(Key::Aes128(FIPS_K128), &FIPS_CT128);
        let kat192_ok = kat(Key::Aes192(FIPS_K192), &FIPS_CT192);
        let kat256_ok = kat(Key::Aes256(FIPS_K256), &FIPS_CT256);

        // ---- AES decrypt: each ciphertext back to the plaintext ----------
        // Each set_key + decrypt crosses the encrypt↔decrypt direction
        // boundary, exercising the driver's transparent key reload.
        let mut dec = |key: Key, ct: &[u8; 16]| -> bool {
            aes.set_key(key);
            let mut block = *ct;
            aes.decrypt_blocks(&mut block).unwrap();
            block == FIPS_PT
        };
        let decrypt_ok = dec(Key::Aes128(FIPS_K128), &FIPS_CT128)
            && dec(Key::Aes192(FIPS_K192), &FIPS_CT192)
            && dec(Key::Aes256(FIPS_K256), &FIPS_CT256);

        // ---- AES CBC: SP800-38A two-block vector + round trip ------------
        aes.set_key(Key::Aes128(CBC_KEY));
        let mut cbc = CBC_PT;
        aes.encrypt_cbc(&CBC_IV, &mut cbc).unwrap();
        let cbc_enc_ok = cbc == CBC_CT;
        aes.decrypt_cbc(&CBC_IV, &mut cbc).unwrap();
        let cbc_ok = cbc_enc_ok && cbc == CBC_PT;

        // ---- AES multi-block ECB round trip ------------------------------
        aes.set_key(Key::Aes256(FIPS_K256));
        let mut long = [0u8; 48];
        for (i, b) in long.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(23).wrapping_add(5);
        }
        let original = long;
        aes.encrypt_blocks(&mut long).unwrap();
        let changed = long != original;
        aes.decrypt_blocks(&mut long).unwrap();
        let roundtrip_ok = changed && long == original;

        // Human-readable info line (host skips everything up to BEGIN).
        tx.write_all(b"accel crc ccitt=").ok();
        write_hex16(&mut tx, ccitt);
        tx.write_all(b" xmodem=").ok();
        write_hex16(&mut tx, xmodem);
        tx.write_all(b" kermit=").ok();
        write_hex16(&mut tx, kermit);
        tx.write_all(b" x25=").ok();
        write_hex16(&mut tx, x25);
        tx.write_all(b" word=").ok();
        write_hex16(&mut tx, word_result);
        tx.write_all(b" ires=").ok();
        write_hex16(&mut tx, diag_inires);
        tx.write_all(b" resr=").ok();
        write_hex16(&mut tx, diag_resr);
        tx.write_all(b" aes128ct=").ok();
        {
            aes.set_key(Key::Aes128(FIPS_K128));
            let mut block = FIPS_PT;
            aes.encrypt_blocks(&mut block).unwrap();
            for &b in &block[..4] {
                write_hex8(&mut tx, b);
            }
        }
        tx.write_all(b"\r\n").ok();

        // The framed verdict burst.
        tx.write_all(b"ACCEL_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"ACCEL CRC MODEL", model_ok);
        verdict(&mut tx, b"ACCEL CRC CATALOG", catalog_ok);
        verdict(&mut tx, b"ACCEL AES128 KAT", kat128_ok);
        verdict(&mut tx, b"ACCEL AES192 KAT", kat192_ok);
        verdict(&mut tx, b"ACCEL AES256 KAT", kat256_ok);
        verdict(&mut tx, b"ACCEL AES DECRYPT", decrypt_ok);
        verdict(&mut tx, b"ACCEL AES CBC", cbc_ok);
        verdict(&mut tx, b"ACCEL AES ROUNDTRIP", roundtrip_ok);
        tx.write_all(b"ACCEL_TEST_END\r\n").ok();

        let all_ok = model_ok
            && catalog_ok
            && kat128_ok
            && kat192_ok
            && kat256_ok
            && decrypt_ok
            && cbc_ok
            && roundtrip_ok;
        if all_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write `name` + ` OK`/` FAIL` + CRLF.
fn verdict<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], ok: bool) {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" as &[u8] } else { b" FAIL\r\n" })
        .ok();
}

/// Write a byte as two uppercase hex digits. `core::fmt` is deliberately
/// avoided project-wide (FRAM budget).
fn write_hex8<W: hal::embedded_io::Write>(tx: &mut W, b: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    tx.write_all(&[HEX[(b >> 4) as usize], HEX[(b & 0xF) as usize]])
        .ok();
}

/// Write a u16 as four uppercase hex digits.
fn write_hex16<W: hal::embedded_io::Write>(tx: &mut W, v: u16) {
    write_hex8(tx, (v >> 8) as u8);
    write_hex8(tx, v as u8);
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// compiler-builtins' memcpy/memcmp reference `abort` on their safety paths.
// Provide a minimal one so we don't link newlib's libc (and its syscall stubs).
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
