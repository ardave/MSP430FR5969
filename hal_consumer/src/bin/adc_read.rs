#![no_std]
#![no_main]

//! ADC12_B single-channel demo for the `hal::adc` driver.
//!
//! Reads analog input **A4 = P1.4** once every half second, prints the raw count
//! and the equivalent millivolts over the UART backchannel, and uses the on-board
//! LEDs as a coarse level indicator. Unlike the eUSCI_B0 demos this needs no bus
//! wiring and does not conflict with them, so it can be flashed independently:
//!
//! ```text
//! cargo +nightly build --bin adc_read
//! DSLite load ... -f target/msp430-none-elf/debug/adc_read
//! ```
//!
//! # Wiring
//!
//! Drive **P1.4** with a voltage between AVSS (0 V) and AVCC (3.3 V) — a pot
//! wiper between 3V3 and GND is ideal. The reference is AVCC, so a count of 0 is
//! ~0 V and full scale (4095) is ~3.3 V. **Do not exceed AVCC** on the pin.
//!
//! # What you should see
//!
//! Over UART (9600 8N1 on eUSCI_A0): a line per reading like `A4: 2048 = 1650
//! mV`. Sweep the pot and the number tracks it. **GREEN** LED above mid-scale
//! (>1.65 V), **RED** at or below — a quick visual confirmation the conversion
//! follows the input. Floating the pin reads noise near mid-rail.

use hal::adc::{Adc, Config as AdcConfig};
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

// Watchdog Timer Password / Hold.
const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

// AVCC in millivolts — the ADC reference (VRSEL = 0), used to scale counts.
const AVCC_MV: u32 = 3300;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog before anything else (default timeout ~32 ms, and
    // Peripherals::take() enters a critical section).
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    let p = hal::pac::Peripherals::take().unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (SMCLK feeds the UART BRCLK below).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the UART and analog pin muxes take effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1). The analog input is P1.4, so
    // the green LED on P1.0 does not collide with it.
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    // P1.4 → analog input channel A4.
    let mut a4 = port1.pin4.into_analog();

    // ADC: 12-bit, MODOSC-clocked, AVCC reference (Config defaults).
    let mut adc = Adc::new(p.adc12, AdcConfig::default());

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"MSP430FR5969 ADC12_B demo: reading A4 (P1.4) vs AVCC\r\n")
        .ok();

    loop {
        let counts = adc.read(&mut a4);
        // counts/4095 * AVCC. counts*AVCC_MV is up to 4095*3300 ≈ 13.5M, so the
        // intermediate must be u32 (it would overflow u16).
        let millivolts = (counts as u32 * AVCC_MV) / 4095;

        tx.write_all(b"A4: ").ok();
        write_dec(&mut tx, counts as u32);
        tx.write_all(b" = ").ok();
        write_dec(&mut tx, millivolts);
        tx.write_all(b" mV\r\n").ok();

        // Green above mid-scale (~1.65 V), red at/below.
        if counts > 2047 {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(500);
    }
}

/// Write an unsigned value as decimal ASCII over the UART. `core::fmt` is
/// deliberately avoided project-wide (FRAM budget), so format by hand into a
/// small stack buffer — a u32 is at most 10 digits.
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
// Provide a minimal one so we don't link newlib's libc (and its syscall stubs).
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
