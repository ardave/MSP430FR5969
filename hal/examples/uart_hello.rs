#![no_std]
#![no_main]

//! Print a counter line over the UART backchannel once per second.
//!
//! The eUSCI_A0 UART rides the eZ-FET debug probe's virtual serial port —
//! 9600 8N1, e.g. `screen /dev/cu.usbmodem<N> 9600` on macOS (the eZ-FET gates
//! TX on DTR, which `screen` asserts).
//!
//! Note the hand-rolled decimal formatting: this project deliberately avoids
//! `core::fmt` (its formatting engine costs ~30 KB of the part's 48 KB FRAM),
//! which is also why `hal::serial` implements `embedded_io::Write` and not
//! `core::fmt::Write`.
//!
//! ```text
//! cargo +nightly build --example uart_hello --features rt,critical-section
//! tools/flash.sh target/msp430-none-elf/debug/examples/uart_hello
//! ```

use msp430fr5969_hal as hal;

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_io::Write as _;
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

#[entry]
fn main() -> ! {
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz — SMCLK is the UART's BRCLK below.
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let mut delay = Delay::new(clocks.mclk());
    let mut seconds: u32 = 0;

    tx.write_all(b"hello from the MSP430FR5969\r\n").ok();

    loop {
        tx.write_all(b"uptime: ").ok();
        write_dec(&mut tx, seconds);
        tx.write_all(b" s\r\n").ok();
        seconds += 1;
        delay.delay_ms(1000);
    }
}

/// Write an unsigned value as decimal ASCII (a u32 is at most 10 digits).
fn write_dec<W: hal::embedded_io::Write>(tx: &mut W, mut value: u32) {
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (value % 10) as u8;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    tx.write_all(&buf[i..]).ok();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// compiler-builtins' memcpy/memcmp reference `abort` on their safety paths.
#[no_mangle]
pub extern "C" fn abort() -> ! {
    loop {}
}
