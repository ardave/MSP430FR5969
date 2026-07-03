#![no_std]
#![no_main]

//! SPI loopback fixture for the eUSCI_B0 SPI master driver, framed for the
//! host-side `spi_tests` runner.
//!
//! This is the hardware-in-the-loop counterpart to the interactive SPI loopback
//! demo in `src/main.rs`: instead of re-printing the live `sent…/recv…` line
//! forever, it transfers a known pattern **once** at startup and then re-emits a
//! fixed pass/fail verdict burst, exactly like the `i2c`/`timer`/`deep_sleep`
//! fixtures, so the host can flash it and assert one framed result. Because
//! eUSCI_B0 is *either* SPI or I2C (one register block, shared P1.6/P1.7 pins),
//! only one of those binaries can run on the board at a time.
//!
//! ```text
//! cargo +nightly build --bin spi_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/spi_test_runner
//! ```
//!
//! # Wiring
//!
//! Driven by `spi_tests`, which prints the jumper hookup and waits for the
//! operator before flashing. The eUSCI_B0 SPI master needs its own transmit line
//! looped back to its receive line: a **jumper wire from P1.6 (SIMO) to P1.7
//! (SOMI)**. With the jumper every transmitted byte is clocked straight back in;
//! without it SOMI floats and the received bytes do not match.
//!
//! # What it checks
//!
//! `transfer_in_place`s a 6-byte pattern with a mix of bit positions and checks
//! that every byte round-trips (`SPI LOOPBACK OK`, GREEN LED). A mismatch fails
//! (`SPI LOOPBACK FAIL`, RED LED) — the usual sign of a missing jumper or a
//! floating SOMI. Reaching the verdict at all also proves the driver completes
//! the transfer rather than hanging.
//!
//! # Framed output for the host runner
//!
//! ```text
//! spi sent=A53CFF0055AA recv=A53CFF0055AA   (human-readable info, skipped by host)
//! SPI_TEST_BEGIN
//! SPI LOOPBACK OK                           (or `SPI LOOPBACK FAIL`)
//! SPI_TEST_END
//! ```

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_hal::spi::SpiBus as _;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::spi::{Config as SpiConfig, SpiExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// A pattern with a mix of bit positions, so a stuck/floating SOMI line is
/// obvious in the received bytes.
const PATTERN: [u8; 6] = [0xA5, 0x3C, 0xFF, 0x00, 0x55, 0xAA];

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds both the UART BRCLK and the SPI
    // bit-rate generator below.
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO pins (clear LOCKLPM5) so the UART and SPI pin muxes take effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0) for printing results: 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1). P1.6/P1.7 are now SPI pins.
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());

    // eUSCI_B0 as a 3-pin SPI master: 1 MHz SPI clock (SMCLK 8 MHz / 8), Mode 0,
    // MSB first. SIMO = P1.6, SOMI = P1.7, CLK = P2.2.
    let mut spi = p
        .usci_b0_spi_mode
        .into_spi(SpiConfig::new(clocks.smclk()).bit_rate(1_000_000));

    // --- Transfer once at startup; the loop below re-emits a stable verdict. ---
    // transfer_in_place sends each byte and overwrites it with the byte received
    // in its place. With the loopback jumper, received == sent.
    let mut buf = PATTERN;
    spi.transfer_in_place(&mut buf).ok();
    let loopback_ok = buf == PATTERN;

    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"spi sent=").ok();
        write_hex(&mut tx, &PATTERN);
        tx.write_all(b" recv=").ok();
        write_hex(&mut tx, &buf);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"SPI_TEST_BEGIN\r\n").ok();
        tx.write_all(if loopback_ok {
            b"SPI LOOPBACK OK\r\n" as &[u8]
        } else {
            b"SPI LOOPBACK FAIL\r\n"
        })
        .ok();
        tx.write_all(b"SPI_TEST_END\r\n").ok();

        // Green = loopback verified, red = mismatch (no jumper / floating SOMI).
        if loopback_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write a byte slice as uppercase hex over the UART.
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
