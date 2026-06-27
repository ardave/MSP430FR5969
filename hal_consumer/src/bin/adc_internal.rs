#![no_std]
#![no_main]

//! ADC12_B **internal-channel** demo — measures with NO external wiring.
//!
//! Unlike `adc_read` (which needs a voltage on P1.4), this reads the converter's
//! two on-chip sources, so the only thing you do is flash and watch the UART:
//!
//! ```text
//! cargo +nightly build --bin adc_internal
//! DSLite load ... -f target/msp430-none-elf/debug/adc_internal
//! ```
//!
//! # What it measures and what to expect
//!
//! - **(AVCC–AVSS)/2 supply monitor.** Measured against the AVCC reference this
//!   is ratiometric, so it must read ≈ **half full-scale (~2048 counts, ~1650
//!   mV)** regardless of the actual supply. That fixed, predictable value is the
//!   point: it confirms the ADC converts correctly with nothing connected.
//! - **Temperature sensor (raw).** Reads **~0** on this driver — the sensor is
//!   part of the REF_A module and is unpowered unless `REFON` is set, which we
//!   do not do (no REF_A support yet). Printed anyway to make that concrete; it
//!   will come alive once REF_A is brought up. (Verified on hardware 2026-06-27.)
//!
//! Over UART (9600 8N1 on eUSCI_A0) each line looks like
//! `AVCC/2: 2051 = 1652 mV   TEMP raw: 0`. **GREEN** LED if the supply-monitor
//! reading lands within ~10% of half-scale (the self-check passed), **RED**
//! otherwise. A long sample time is used because the supply divider is
//! high-impedance.

use hal::adc::{Adc, Config as AdcConfig, SampleTime};
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

// Half-scale at 12-bit, and a ±10% acceptance window for the supply self-check.
const HALF_SCALE: u16 = 2048;
const WINDOW: u16 = 205; // ~10% of 2048

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

    // Unlock GPIO (clear LOCKLPM5) so the UART pin mux takes effect. (The ADC
    // internal channels need no pins.)
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

    // ADC: 12-bit, MODOSC clock, AVCC reference, with a LONG sample time — the
    // internal supply divider and temperature sensor are high-impedance.
    let mut adc = Adc::new(
        p.adc12,
        AdcConfig::default().sample_time(SampleTime::Cycles256),
    );

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"MSP430FR5969 ADC12_B internal channels (no wiring)\r\n")
        .ok();

    loop {
        let supply = adc.read_supply_half();
        let supply_mv = (supply as u32 * AVCC_MV) / 4095;
        let temp = adc.read_temperature_raw();

        tx.write_all(b"AVCC/2: ").ok();
        write_dec(&mut tx, supply as u32);
        tx.write_all(b" = ").ok();
        write_dec(&mut tx, supply_mv);
        tx.write_all(b" mV   TEMP raw: ").ok();
        write_dec(&mut tx, temp as u32);
        tx.write_all(b"\r\n").ok();

        // Self-check: the supply monitor should sit within ~10% of half-scale.
        let ok = supply > HALF_SCALE - WINDOW && supply < HALF_SCALE + WINDOW;
        if ok {
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
