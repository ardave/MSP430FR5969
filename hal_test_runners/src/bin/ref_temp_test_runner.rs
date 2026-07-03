#![no_std]
#![no_main]

//! REF_A + ADC12_B **absolute measurement** integration fixture — validates
//! with NO external wiring, driven by the host-side `ref_a_tests` runner.
//!
//! This is the other half of the `adc_internal` fixture's story. That one
//! proves the converter works *ratiometrically* (AVCC reference) and that the
//! temperature sensor is dead while REF_A is off. This one brings REF_A up at
//! **2.0 V** and makes the two measurements that were impossible before:
//!
//! - **Die temperature in °C** — the sensor (powered by `REFON`, converted
//!   against VREF) interpolated between the factory 30 °C / 85 °C TLV points.
//!   2.0 V is used rather than the finer 1.2 V so a single reference setting
//!   also serves the supply measurement below; the TLV carries a calibration
//!   pair for every setting, so nothing is lost but resolution.
//! - **AVCC in millivolts** — the (AVCC–AVSS)/2 monitor against VREF, through
//!   the full calibration chain (ADC gain → offset → REF factor), then
//!   doubled. At 2.0 V the monitor covers supplies up to 4 V; against 1.2 V
//!   it would clip (AVCC/2 = 1.65 V at a 3.3 V supply).
//!
//! # What to expect (LaunchPad, USB-powered)
//!
//! Temperature a few degrees above ambient (the die self-heats slightly):
//! roughly 20–35 °C on a bench. AVCC ≈ 3300 mV from the eZ-FET's LDO. The
//! on-device plausibility windows are deliberately wide — 5–60 °C and
//! 2900–3600 mV — so the verdicts only flip on a broken reference, dead
//! sensor, or miscalibrated chain, not on a warm office.
//!
//! # Framed output for the host runner
//!
//! Like the `adc_internal` fixture, a self-delimited burst once per second,
//! forever, over UART (9600 8N1 on eUSCI_A0):
//!
//! ```text
//! TEMP: 27.3 C (raw 1497)   AVCC: 3312 mV      (human-readable, skipped by host)
//! REF_TEMP_TEST_BEGIN
//! TLV CAL OK                                   (or `TLV CAL MISSING`)
//! TEMP PLAUSIBLE OK                            (or `... FAIL`)
//! SUPPLY PLAUSIBLE OK                          (or `... FAIL`)
//! REF_TEMP_TEST_END
//! ```
//!
//! **GREEN** LED while all checks pass, **RED** otherwise.

use hal::adc::{Adc, Config as AdcConfig, SampleTime};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::ref_a::{Ref, ReferenceVoltage};
use hal::serial::{Config as UartConfig, SerialExt};
use hal::tlv;
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

// Plausibility windows for the verdicts, wide enough that only a real fault
// (not a warm room or a sagging USB port) flips them.
const TEMP_MIN_DECI_C: i16 = 50; // 5.0 °C
const TEMP_MAX_DECI_C: i16 = 600; // 60.0 °C
const SUPPLY_MIN_MV: u32 = 2900;
const SUPPLY_MAX_MV: u32 = 3600;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (SMCLK feeds the UART BRCLK below).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the UART pin mux takes effect. (REF_A
    // and the internal ADC channels need no pins.)
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

    // REF_A on at 2.0 V — settled when `new` returns; also powers the
    // temperature sensor. One setting serves both measurements (see module docs).
    let vref = Ref::new(p.shared_reference, ReferenceVoltage::V2_0);

    // ADC: 12-bit (the TLV temperature points are 12-bit results), MODOSC,
    // and a LONG sample time — both internal sources are high-impedance and
    // the sensor wants ≥ 30 µs of acquisition (256 cycles ≈ 53 µs).
    let mut adc = Adc::new(
        p.adc12,
        AdcConfig::default().sample_time(SampleTime::Cycles256),
    );

    // Factory calibration out of the device-descriptor table. Always present
    // on real silicon; `None` would mean a corrupt TLV, reported per-cycle
    // below so the host test fails visibly rather than silently.
    let cal = tlv::adc_cal();
    let ref_cal = tlv::ref_cal();

    let mut delay = Delay::new(clocks.mclk());

    // One-time banner so a human watching `screen` sees what this binary is.
    tx.write_all(b"MSP430FR5969 REF_A 2.0V: calibrated temperature & supply (no wiring)\r\n")
        .ok();

    loop {
        // Temperature: raw conversion interpolated with the factory 2.0 V pair.
        let temp_raw = adc.read_temperature(&vref);
        let temp_deci = cal.and_then(|c| c.temp_deci_celsius(vref.voltage(), temp_raw));

        // Supply: raw monitor reading through the full calibration chain
        // (gain -> offset -> REF factor), scaled to mV, doubled back to AVCC.
        let supply_raw = adc.read_supply_raw(&vref);
        let supply_mv = match (cal, ref_cal) {
            (Some(c), Some(r)) => {
                let corrected = r.correct(vref.voltage(), c.correct_gain_offset(supply_raw));
                adc.to_millivolts(corrected, &vref) * 2
            }
            // No calibration: nominal scaling still gives a usable number.
            _ => adc.read_supply_millivolts(&vref),
        };

        let cal_ok = cal.is_some() && ref_cal.is_some();
        let temp_ok = temp_deci
            .map(|t| (TEMP_MIN_DECI_C..=TEMP_MAX_DECI_C).contains(&t))
            .unwrap_or(false);
        let supply_ok = (SUPPLY_MIN_MV..=SUPPLY_MAX_MV).contains(&supply_mv);

        // Human-readable info line. The host runner skips everything up to the
        // BEGIN marker, so this is purely for someone watching over `screen`.
        tx.write_all(b"TEMP: ").ok();
        match temp_deci {
            Some(t) => write_deci(&mut tx, t),
            None => {
                tx.write_all(b"?").ok();
            }
        }
        tx.write_all(b" C (raw ").ok();
        write_dec(&mut tx, temp_raw as u32);
        tx.write_all(b")   AVCC: ").ok();
        write_dec(&mut tx, supply_mv);
        tx.write_all(b" mV\r\n").ok();

        // A self-delimited burst of fixed, greppable verdict lines (the same
        // contract as the adc_internal fixture — see its docs).
        tx.write_all(b"REF_TEMP_TEST_BEGIN\r\n").ok();
        tx.write_all(if cal_ok {
            b"TLV CAL OK\r\n" as &[u8]
        } else {
            b"TLV CAL MISSING\r\n"
        })
        .ok();
        tx.write_all(if temp_ok {
            b"TEMP PLAUSIBLE OK\r\n" as &[u8]
        } else {
            b"TEMP PLAUSIBLE FAIL\r\n"
        })
        .ok();
        tx.write_all(if supply_ok {
            b"SUPPLY PLAUSIBLE OK\r\n" as &[u8]
        } else {
            b"SUPPLY PLAUSIBLE FAIL\r\n"
        })
        .ok();
        tx.write_all(b"REF_TEMP_TEST_END\r\n").ok();

        // GREEN while everything passes, RED otherwise.
        if cal_ok && temp_ok && supply_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write a deci-value (tenths) as `-X.Y` decimal ASCII, e.g. 273 → `27.3`.
fn write_deci<W: hal::embedded_io::Write>(tx: &mut W, deci: i16) {
    let mut v = deci as i32;
    if v < 0 {
        tx.write_all(b"-").ok();
        v = -v;
    }
    write_dec(tx, (v / 10) as u32);
    tx.write_all(b".").ok();
    write_dec(tx, (v % 10) as u32);
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
