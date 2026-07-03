#![no_std]
#![no_main]

//! I2C bus-scan fixture for the eUSCI_B0 I2C master driver, framed for the
//! host-side `i2c_tests` runner.
//!
//! This is the hardware-in-the-loop counterpart to the interactive
//! `i2c_scan_test_runner` demo: instead of re-printing the live scan forever, it
//! scans **once** at startup and then re-emits a fixed pass/fail verdict burst,
//! exactly like the `timer`/`deep_sleep` fixtures, so the host can flash it and
//! assert one framed result. Because eUSCI_B0 is *either* SPI or I2C (one
//! register block, shared P1.6/P1.7 pins), only one of those binaries can run on
//! the board at a time.
//!
//! ```text
//! cargo +nightly build --bin i2c_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/i2c_test_runner
//! ```
//!
//! # Wiring
//!
//! Driven by `i2c_tests`, which prints the BME280 hookup and waits for the
//! operator before flashing. I2C is open-drain, so **external pull-ups are
//! required**: ~4.7 kΩ from each of SDA (P1.6) and SCL (P1.7) up to 3V3 (most
//! breakouts include their own). A BME280 answers at 0x76 or 0x77, so a correct
//! hookup makes the scan find at least one device.
//!
//! # What it checks
//!
//! Probes every 7-bit address in `0x08..=0x77` with a zero-length write
//! (START + address + STOP). Finding **at least one** ACKing device passes
//! (`I2C SCAN OK`, GREEN LED); an empty bus fails (`I2C SCAN FAIL`, RED LED) —
//! the usual sign of a missing device, missing pull-ups, or an unremoved SPI
//! loopback jumper shorting SDA to SCL. Reaching the verdict at all also proves
//! the driver does not hang on a NACKing/empty bus.
//!
//! # Framed output for the host runner
//!
//! ```text
//! i2c found=76                       (human-readable info, skipped by host)
//! I2C_TEST_BEGIN
//! I2C SCAN OK                        (or `I2C SCAN FAIL`)
//! I2C_TEST_END
//! ```

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::i2c::{Config as I2cConfig, I2cExt};
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Most addresses we bother to remember for the human-readable info line.
const MAX_FOUND: usize = 8;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds both the UART BRCLK and the I2C
    // bit-rate generator below.
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO pins (clear LOCKLPM5) so the UART and I2C pin muxes take effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0) for printing results: 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1). P1.6/P1.7 are now I2C pins.
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());

    // eUSCI_B0 as an I2C master: 100 kHz standard mode (SMCLK 8 MHz / 80).
    // SDA = P1.6, SCL = P1.7 (external pull-ups required).
    let mut i2c = p
        .usci_b0_i2c_mode
        .into_i2c(I2cConfig::new(clocks.smclk()).scl_freq(100_000));

    // --- Scan once at startup; the loop below re-emits a stable verdict. -----
    let mut found = [0u8; MAX_FOUND];
    let mut count = 0usize;
    for addr in 0x08u8..=0x77 {
        if i2c.probe(addr) {
            if count < MAX_FOUND {
                found[count] = addr;
            }
            count += 1;
        }
    }
    let scan_ok = count > 0;

    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"i2c found=").ok();
        if count == 0 {
            tx.write_all(b"none").ok();
        } else {
            for (i, addr) in found.iter().take(count.min(MAX_FOUND)).enumerate() {
                if i > 0 {
                    tx.write_all(b",").ok();
                }
                write_hex(&mut tx, &[*addr]);
            }
        }
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"I2C_TEST_BEGIN\r\n").ok();
        tx.write_all(if scan_ok {
            b"I2C SCAN OK\r\n" as &[u8]
        } else {
            b"I2C SCAN FAIL\r\n"
        })
        .ok();
        tx.write_all(b"I2C_TEST_END\r\n").ok();

        // Green = at least one device answered, red = empty bus.
        if scan_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write a byte slice as space-separated uppercase hex over the UART.
fn write_hex<W: hal::embedded_io::Write>(tx: &mut W, bytes: &[u8]) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for &b in bytes {
        let pair = [HEX[(b >> 4) as usize], HEX[(b & 0x0F) as usize]];
        tx.write_all(&pair).ok();
    }
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
