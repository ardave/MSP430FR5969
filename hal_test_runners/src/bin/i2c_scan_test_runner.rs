#![no_std]
#![no_main]

//! I2C bus-scanner demo for the eUSCI_B0 I2C master driver.
//!
//! This is the I2C counterpart to the SPI loopback demo in `main.rs`. Because
//! eUSCI_B0 is *either* SPI or I2C (one register block, shared P1.6/P1.7 pins),
//! only one of these binaries can run on the board at a time — flash this one to
//! exercise the I2C driver:
//!
//! ```text
//! cargo +nightly build --bin i2c_scan
//! DSLite load ... -f target/msp430-none-elf/debug/i2c_scan
//! ```
//!
//! # Wiring
//!
//! I2C is open-drain, so **external pull-ups are required**: ~4.7 kΩ from each
//! of SDA (P1.6) and SCL (P1.7) up to 3V3. Hang any I2C device(s) on those two
//! lines (plus shared ground). With no device and no pull-ups every address
//! NACKs; with pull-ups but no device every address still NACKs (cleanly); with
//! a device present its address ACKs and shows up in the scan.
//!
//! # What it does
//!
//! Probes every 7-bit address in the conventional range `0x08..=0x77` with a
//! zero-length write (START + address + STOP) and reports which ones ACK over
//! the UART (9600 8N1 on eUSCI_A0). Green LED = at least one device answered;
//! red = the bus is empty. Re-scans once a second.

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

    tx.write_all(b"MSP430FR5969 I2C bus scan\r\n").ok();
    tx.write_all(b"SDA=P1.6 SCL=P1.7, pull-ups required. Probing 0x08..0x77.\r\n")
        .ok();

    loop {
        tx.write_all(b"found:").ok();
        let mut count = 0u8;
        for addr in 0x08u8..=0x77 {
            if i2c.probe(addr) {
                tx.write_all(b" ").ok();
                write_hex(&mut tx, &[addr]);
                count += 1;
            }
        }
        if count == 0 {
            tx.write_all(b" (none)").ok();
        }
        tx.write_all(b"\r\n").ok();

        // Green = at least one device answered, red = empty bus.
        if count > 0 {
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
