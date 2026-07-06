#![no_std]
#![no_main]

//! ADC12_B **internal-channel** integration fixture — validates with NO external
//! wiring, driven by the host-side `adc_tests` runner.
//!
//! Reads the converter's two on-chip sources, so the only thing the test harness
//! does is flash and listen on the UART backchannel:
//!
//! ```text
//! cargo +nightly build --bin adc_internal
//! DSLite load ... -f target/msp430-none-elf/debug/adc_internal
//! ```
//!
//! # What it measures and what to expect
//!
//! - **(AVCC–AVSS)/2 supply monitor.** Measured against the AVCC reference this
//!   is ratiometric, so it must read ≈ **half full-scale (~2048 counts)**
//!   regardless of the actual supply (which is ~3.6 V on this LaunchPad — see
//!   `ref_temp_test_runner`). That fixed, predictable value is the point: it
//!   confirms the ADC converts correctly with nothing connected. The reading is
//!   self-checked against a ±10% window of half-scale on-device.
//! - **Temperature sensor (raw).** Reads **~0** here — the sensor is part of
//!   the REF_A module and is unpowered unless `REFON` is set, which this
//!   fixture *deliberately never does*: the near-zero reading proves both that
//!   the ADC reports a dead channel honestly and that nothing brings REF_A up
//!   behind our back. (Verified on hardware 2026-06-27.) The powered, calibrated
//!   counterpart is the `ref_temp_test_runner` fixture.
//!
//! # Framed output for the host runner
//!
//! Like the `serial_uart` fixture, this emits a self-delimited burst once per
//! second, forever, so the host test can attach at any time and still catch a
//! complete cycle. Each cycle, over UART (9600 8N1 on eUSCI_A0):
//!
//! ```text
//! AVCC/2: 2051 = 1652 mV   TEMP raw: 0   (human-readable info, skipped by host)
//! ADC_INTERNAL_TEST_BEGIN
//! AVCC/2 SELF-CHECK OK                    (or `... FAIL` if outside the window)
//! TEMP SENSOR OFF                         (or `TEMP SENSOR ON` if it reads hot)
//! ADC_INTERNAL_TEST_END
//! ```
//!
//! The verdict lines inside the frame are fixed, greppable strings the host
//! asserts on; a FAIL/ON verdict makes the host mismatch and fail the test.
//! **GREEN** LED while the supply self-check passes, **RED** otherwise. A long
//! sample time is used because the supply divider is high-impedance.

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

// AVCC in millivolts — the ADC reference (VRSEL = 0), used to scale counts on
// the human info line only (the self-check verdict is ratiometric and does not
// depend on this). ~3.6 V on this LaunchPad: the eZ-FET LDO feeds the rail
// 3.6 V, not 3.3 V — measured 2026-07-03 via REF_A (`ref_temp_test_runner`).
const AVCC_MV: u32 = 3630;

// Half-scale at 12-bit, and a ±10% acceptance window for the supply self-check.
const HALF_SCALE: u16 = 2048;
const WINDOW: u16 = 205; // ~10% of 2048

// The temperature sensor is unpowered (REF_A off), so its raw reading sits near
// zero; anything below this counts as "off". Generous so sensor noise near the
// floor never flips the verdict.
const TEMP_OFF_MAX: u16 = 100;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (SMCLK feeds the UART BRCLK below).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the UART pin mux takes effect. (The ADC
    // internal channels need no pins.)
    hal::gpio::unlock_pins(&p.pmm);

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

    // One-time banner so a human watching `screen` sees what this binary is.
    tx.write_all(b"MSP430FR5969 ADC12_B internal channels (no wiring)\r\n")
        .ok();

    loop {
        let supply = adc.read_supply_half();
        let supply_mv = (supply as u32 * AVCC_MV) / 4095;
        let temp = adc.read_temperature_raw();

        // Self-check the supply monitor (within ±10% of half-scale) and that the
        // temperature sensor is unpowered (near zero, REF_A off).
        let supply_ok = supply > HALF_SCALE - WINDOW && supply < HALF_SCALE + WINDOW;
        let temp_off = temp <= TEMP_OFF_MAX;

        // Human-readable info line. The host runner skips everything up to the
        // BEGIN marker, so this is purely for someone watching over `screen`.
        tx.write_all(b"AVCC/2: ").ok();
        write_dec(&mut tx, supply as u32);
        tx.write_all(b" = ").ok();
        write_dec(&mut tx, supply_mv);
        tx.write_all(b" mV   TEMP raw: ").ok();
        write_dec(&mut tx, temp as u32);
        tx.write_all(b"\r\n").ok();

        // A self-delimited burst of fixed, greppable verdict lines. The
        // BEGIN/END markers let the host frame one cycle; the verdict strings
        // flip to FAIL/ON on a bad reading, which the host asserts against.
        tx.write_all(b"ADC_INTERNAL_TEST_BEGIN\r\n").ok();
        tx.write_all(if supply_ok {
            b"AVCC/2 SELF-CHECK OK\r\n" as &[u8]
        } else {
            b"AVCC/2 SELF-CHECK FAIL\r\n"
        })
        .ok();
        tx.write_all(if temp_off {
            b"TEMP SENSOR OFF\r\n" as &[u8]
        } else {
            b"TEMP SENSOR ON\r\n"
        })
        .ok();
        tx.write_all(b"ADC_INTERNAL_TEST_END\r\n").ok();

        // GREEN while the supply self-check passes, RED otherwise.
        if supply_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
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
