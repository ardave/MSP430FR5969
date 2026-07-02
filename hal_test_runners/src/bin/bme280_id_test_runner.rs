#![no_std]
#![no_main]

//! BME280 chip-ID validation for the eUSCI_B0 I2C master driver.
//!
//! Where `i2c_scan` only proves the address/ACK path (zero-length writes), this
//! binary exercises the **full `embedded_hal::i2c::I2c::write_read` path** — a
//! write run (the register pointer), a **repeated START** turning the bus
//! around, then a read run with the correct last-byte NACK/STOP. That is the
//! transaction shape every real I2C device driver relies on, so it is the
//! meaningful end-to-end validation of the driver.
//!
//! # The check
//!
//! Every BME280 answers register **0xD0** ("id") with the fixed value **0x60**.
//! Reading it back proves we addressed the device, wrote the register pointer,
//! issued a repeated START, and read the byte the slave clocked out. A wrong or
//! absent value fails the check.
//!
//! # Wiring (Adafruit BME280 STEMMA QT)
//!
//! Red→3V3, Black→GND, Blue→P1.6 (SDA), Yellow→P1.7 (SCL). The breakout carries
//! its own pull-ups, so no external resistors are needed. The board answers at
//! **0x77** by default (0x76 if its address jumper is bridged); this demo tries
//! 0x77 then 0x76 so either works.
//!
//! ```text
//! cargo +nightly build --bin bme280_id
//! DSLite load ... -f target/msp430-none-elf/debug/bme280_id
//! ```
//!
//! Green LED + UART `PASS` = id 0x60 read back; red + `FAIL` = no device or
//! wrong id (the printed value tells you which).

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_hal::i2c::I2c as _;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::i2c::{Config as I2cConfig, I2cExt};
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

// Watchdog Timer Password / Hold.
const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

// BME280: the "id" register and the value every BME280 returns from it.
const BME280_REG_ID: u8 = 0xD0;
const BME280_CHIP_ID: u8 = 0x60;

// Candidate 7-bit addresses (SDO low / SDO high).
const ADDRS: [u8; 2] = [0x77, 0x76];

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog before anything else (default timeout ~32 ms, and
    // Peripherals::take() enters a critical section).
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    let p = hal::pac::Peripherals::take().unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds the UART BRCLK and the I2C bit clock.
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
    let mut i2c = p
        .usci_b0_i2c_mode
        .into_i2c(I2cConfig::new(clocks.smclk()).scl_freq(100_000));

    tx.write_all(b"MSP430FR5969 BME280 chip-ID check (reg 0xD0 == 0x60)\r\n")
        .ok();

    loop {
        let mut pass = false;
        for &addr in ADDRS.iter() {
            // The classic register read: write the register pointer, repeated
            // START, read one byte back. This is the path under test.
            let mut id = [0u8; 1];
            match i2c.write_read(addr, &[BME280_REG_ID], &mut id) {
                Ok(()) => {
                    tx.write_all(b"addr ").ok();
                    write_hex(&mut tx, &[addr]);
                    tx.write_all(b" id ").ok();
                    write_hex(&mut tx, &id);
                    if id[0] == BME280_CHIP_ID {
                        tx.write_all(b" PASS\r\n").ok();
                        pass = true;
                        break;
                    } else {
                        tx.write_all(b" (unexpected)\r\n").ok();
                    }
                }
                Err(_) => {
                    // No device acknowledged at this address — try the next.
                    tx.write_all(b"addr ").ok();
                    write_hex(&mut tx, &[addr]);
                    tx.write_all(b" no-ack\r\n").ok();
                }
            }
        }

        if pass {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write a byte slice as `0x`-prefixed uppercase hex over the UART.
fn write_hex<W: hal::embedded_io::Write>(tx: &mut W, bytes: &[u8]) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for &b in bytes {
        let out = [b'0', b'x', HEX[(b >> 4) as usize], HEX[(b & 0x0F) as usize]];
        tx.write_all(&out).ok();
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
