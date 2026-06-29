#![no_std]
#![no_main]

//! BME280 environmental read-out: temperature, pressure, humidity.
//!
//! Unlike `bme280_id` (a raw chip-ID read), this binary drives the **third-party
//! `bme280` crate** — a real embedded-hal 1.0 ecosystem driver — through our
//! [`hal::i2c::I2c`]. That is the strongest validation of the I2C implementation:
//! if an independent driver written against the `embedded_hal::i2c::I2c` trait
//! can initialize the sensor and read compensated measurements, then our trait
//! impl is correct and interoperable, not merely self-consistent.
//!
//! The `bme280` crate performs the multi-register calibration burst reads,
//! oversampling/mode configuration, data burst reads, and Bosch fixed-point
//! compensation internally — all of it flowing through our driver's
//! `write`/`write_read` and our [`hal::delay::Delay`] (it implements `DelayNs`,
//! which the crate needs for the sensor's power-up and conversion timing).
//!
//! Wiring (Adafruit BME280 STEMMA QT): Red→3V3, Black→GND, Blue→P1.6 (SDA),
//! Yellow→P1.7 (SCL); the breakout has its own pull-ups. Output streams over the
//! eUSCI_A0 UART at 9600 8N1, once per second. The board's default address is
//! 0x77 (`new_secondary`).

use bme280::i2c::BME280;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::i2c::{Config as I2cConfig, I2cExt};
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

use msp430 as _;

const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

#[entry]
fn main() -> ! {
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    let p = hal::pac::Peripherals::take().unwrap();
    let clocks = hal::clocks::configure(p.cs);
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());

    // eUSCI_B0 as an I2C master at 100 kHz. The bme280 crate takes ownership of
    // it (a bus driver owns its bus), driving it through embedded_hal::i2c::I2c.
    let i2c = p
        .usci_b0_i2c_mode
        .into_i2c(I2cConfig::new(clocks.smclk()).scl_freq(100_000));

    tx.write_all(b"MSP430FR5969 BME280 via the `bme280` crate\r\n")
        .ok();

    // 0x77 = Adafruit default (SDO high); use new_primary for a 0x76 board.
    let mut sensor = BME280::new_secondary(i2c);
    if sensor.init(&mut delay).is_err() {
        tx.write_all(b"BME280 init failed (check wiring/address)\r\n")
            .ok();
        loop {
            red_led.set_high().ok();
            delay.delay_ms(1000);
        }
    }
    tx.write_all(b"init OK, streaming measurements:\r\n").ok();

    loop {
        match sensor.measure(&mut delay) {
            Ok(m) => {
                // The crate returns f32 °C / Pa / %RH. Render as fixed-point
                // (no core::fmt — FRAM budget): scale to integer hundredths.
                // f32 -> i32 uses the signed float-fix runtime routine
                // (__mspabi_fixfli); the unsigned one (__mspabi_fixful) isn't in
                // this multilib's libgcc, so convert via i32 then reinterpret
                // (these values are always positive). i32 -> u32 is free.
                let temp_c100 = (m.temperature * 100.0) as i32;
                let press_pa = (m.pressure as i32) as u32; // Pa
                let rh_centi = ((m.humidity * 100.0) as i32) as u32; // %RH * 100

                tx.write_all(b"T ").ok();
                write_signed_centi(&mut tx, temp_c100);
                tx.write_all(b" C  P ").ok();
                write_fixed2(&mut tx, press_pa / 100, press_pa % 100); // -> hPa
                tx.write_all(b" hPa  H ").ok();
                write_fixed2(&mut tx, rh_centi / 100, rh_centi % 100);
                tx.write_all(b" %\r\n").ok();

                green_led.set_high().ok();
                red_led.set_low().ok();
            }
            Err(_) => {
                tx.write_all(b"measure error\r\n").ok();
                red_led.set_high().ok();
                green_led.set_low().ok();
            }
        }
        delay.delay_ms(1000);
    }
}

// ---------------------------------------------------------------------------
// Minimal decimal formatting (no core::fmt — FRAM budget).
// ---------------------------------------------------------------------------

fn write_u32<W: hal::embedded_io::Write>(tx: &mut W, mut v: u32) {
    if v == 0 {
        tx.write_all(b"0").ok();
        return;
    }
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    while v > 0 {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    tx.write_all(&buf[i..]).ok();
}

/// Print `ip.fp` with exactly two fractional digits (`fp` in 0..=99).
fn write_fixed2<W: hal::embedded_io::Write>(tx: &mut W, ip: u32, fp: u32) {
    write_u32(tx, ip);
    let frac = [b'.', b'0' + (fp / 10) as u8, b'0' + (fp % 10) as u8];
    tx.write_all(&frac).ok();
}

/// Print a signed value expressed in hundredths (e.g. 2347 -> `23.47`).
fn write_signed_centi<W: hal::embedded_io::Write>(tx: &mut W, v: i32) {
    if v < 0 {
        tx.write_all(b"-").ok();
    }
    let a = v.unsigned_abs();
    write_fixed2(tx, a / 100, a % 100);
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
