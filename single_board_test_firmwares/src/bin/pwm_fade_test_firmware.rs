#![no_std]
#![no_main]

//! Timer_B0 PWM demo for the `hal::pwm` driver.
//!
//! Drives a ~1 kHz PWM on **TB0.1 = P1.4** and ramps its duty cycle 0 → 100 →
//! 0 % in a triangle, ~20 ms per step, so an LED on that pin **breathes**. The
//! current duty percent is printed over the UART backchannel, and the on-board
//! LEDs show the ramp direction (GREEN while brightening, RED while dimming).
//! Needs no bus wiring and does not conflict with the SPI/I2C demos:
//!
//! ```text
//! cargo +nightly build --bin pwm_fade
//! DSLite load ... -f target/msp430-none-elf/debug/pwm_fade
//! ```
//!
//! # Wiring
//!
//! Put an **LED in series with ~330 Ω from P1.4 to GND** (long leg / anode to
//! P1.4). As the duty climbs the average current rises and the LED brightens; as
//! it falls it dims — a smooth breathing effect at 1 kHz, far above flicker. A
//! scope on P1.4 instead shows the pulse width sweeping from 0 to 100 % of the
//! ~1 ms period. With nothing attached the demo still runs (watch the UART).
//!
//! # What you should see
//!
//! Over UART (9600 8N1 on eUSCI_A0): `duty: 0%`, `duty: 2%`, … `100%` … back to
//! `0%`, forever. The LED on P1.4 tracks it. The clean endpoints matter: at 0 %
//! the pin sits steady low (LED fully off), at 100 % steady high (LED full
//! brightness), with no flicker glitch at either rail.

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_hal::pwm::SetDutyCycle as _;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::pwm::Pwm;
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK clocks both the UART BRCLK and Timer_B0.
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, ramp up
    let mut red_led = port4.pin6.into_output(); // LED1, ramp down

    // P1.4 → TB0.1, driven by the timer.
    let p14 = port1.pin4.into_timer_b_output();
    let pwm = Pwm::new_smclk(p.timer_0_b7, &clocks, 1_000);
    let mut ch = pwm.channel(p14);

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"MSP430FR5969 Timer_B0 PWM demo: breathing TB0.1 (P1.4) @ ~1 kHz\r\n")
        .ok();

    loop {
        // Ramp up 0..=100 (green), then down 100..=0 (red). `_percent` maps the
        // percent onto the period and uses the clean-rail endpoints at 0/100.
        green_led.set_high().ok();
        red_led.set_low().ok();
        for percent in 0..=100u8 {
            ch.set_duty_cycle_percent(percent).ok();
            report(&mut tx, percent);
            delay.delay_ms(20);
        }

        green_led.set_low().ok();
        red_led.set_high().ok();
        for percent in (0..=100u8).rev() {
            ch.set_duty_cycle_percent(percent).ok();
            report(&mut tx, percent);
            delay.delay_ms(20);
        }
    }
}

/// Print `duty: N%` over the UART.
fn report<W: hal::embedded_io::Write>(tx: &mut W, percent: u8) {
    tx.write_all(b"duty: ").ok();
    write_dec(tx, percent as u32);
    tx.write_all(b"%\r\n").ok();
}

/// Write an unsigned value as decimal ASCII. `core::fmt` is avoided project-wide
/// (FRAM budget), so format by hand into a small stack buffer.
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
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
