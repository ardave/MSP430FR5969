#![no_std]
#![no_main]

//! Read the die temperature (°C) and the supply rail (mV) once per second.
//!
//! No wiring — both are internal ADC12_B channels. REF_A comes up at 2.0 V
//! (one setting serves both: the temperature sensor is biased by REF_A, and
//! against 1.2 V the AVCC/2 monitor would clip on a 3+ V supply). The readings
//! go through the factory TLV calibration: the 30/85 °C interpolation points
//! for temperature, and the gain → offset → reference-factor chain for the
//! supply. Expect a few °C above ambient (die self-heating), and ≈3630 mV on a
//! USB-powered LaunchPad — the eZ-FET's LDO feeds it ~3.6 V, not 3.3 V.
//!
//! Output on the eUSCI_A0 backchannel UART, 9600 8N1.
//!
//! ```text
//! cargo +nightly build --example temperature --features rt,critical-section
//! tools/flash.sh target/msp430-none-elf/debug/examples/temperature
//! ```

use msp430fr5969_hal as hal;

use hal::adc::{Adc, Config as AdcConfig, SampleTime};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_io::Write as _;
use hal::ref_a::{Ref, ReferenceVoltage};
use hal::serial::{Config as UartConfig, SerialExt};
use hal::tlv;
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

#[entry]
fn main() -> ! {
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (SMCLK feeds the UART BRCLK).
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // REF_A on at 2.0 V — settled when `new` returns; also powers the
    // temperature sensor.
    let vref = Ref::new(p.shared_reference, ReferenceVoltage::V2_0);

    // 12-bit (the TLV temperature points are 12-bit results) with a LONG
    // sample time — the internal sources are high-impedance and the sensor
    // wants ≥ 30 µs of acquisition (256 cycles ≈ 53 µs on MODOSC).
    let mut adc = Adc::new(
        p.adc12,
        AdcConfig::default().sample_time(SampleTime::Cycles256),
    );

    // Factory calibration from the device-descriptor (TLV) table — always
    // present on real silicon.
    let cal = tlv::adc_cal();
    let ref_cal = tlv::ref_cal();

    let mut delay = Delay::new(clocks.mclk());

    loop {
        // Temperature: raw conversion interpolated between the factory
        // 30 °C / 85 °C points for the 2.0 V reference, in deci-°C.
        let temp_raw = adc.read_temperature(&vref);
        let temp_deci = cal.and_then(|c| c.temp_deci_celsius(vref.voltage(), temp_raw));

        // Supply: the (AVCC–AVSS)/2 monitor through the full calibration
        // chain, scaled to mV, doubled back to AVCC.
        let supply_raw = adc.read_supply_raw(&vref);
        let supply_mv = match (cal, ref_cal) {
            (Some(c), Some(r)) => {
                let corrected = r.correct(vref.voltage(), c.correct_gain_offset(supply_raw));
                adc.to_millivolts(corrected, &vref) * 2
            }
            // No calibration: nominal scaling still gives a usable number.
            _ => adc.read_supply_millivolts(&vref),
        };

        tx.write_all(b"temp: ").ok();
        match temp_deci {
            Some(t) => write_deci(&mut tx, t),
            None => {
                tx.write_all(b"?").ok();
            }
        }
        tx.write_all(b" C   AVCC: ").ok();
        write_dec(&mut tx, supply_mv);
        tx.write_all(b" mV\r\n").ok();

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
