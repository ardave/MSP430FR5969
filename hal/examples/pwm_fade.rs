#![no_std]
#![no_main]

//! Breathe an LED with ~1 kHz Timer_B0 PWM on TB0.1 (P1.4).
//!
//! Wiring: an LED in series with ~330 Ω from P1.4 to GND (or a scope on P1.4).
//! With nothing attached the demo still runs — the on-board LEDs show the ramp
//! direction (green while brightening, red while dimming).
//!
//! The duty endpoints are glitch-free by construction: 0 % parks the pin
//! steady low and 100 % steady high via `OUTMOD=0`; everything between uses
//! `OUTMOD=7` Reset/Set.
//!
//! ```text
//! cargo +nightly build --example pwm_fade --features rt,critical-section
//! tools/flash.sh target/msp430-none-elf/debug/examples/pwm_fade
//! ```

use msp430fr5969_hal as hal;

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_hal::pwm::SetDutyCycle as _;
use hal::gpio::GpioExt;
use hal::pwm::Pwm;
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

#[entry]
fn main() -> ! {
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK clocks Timer_B0.
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, ramp up
    let mut red_led = port4.pin6.into_output(); // LED1, ramp down

    // P1.4 → TB0.1, driven by the timer output unit.
    let p14 = port1.pin4.into_timer_b_output();
    let pwm = Pwm::new_smclk(p.timer_0_b7, &clocks, 1_000);
    let mut ch = pwm.channel(p14);

    let mut delay = Delay::new(clocks.mclk());

    loop {
        green_led.set_high().ok();
        red_led.set_low().ok();
        for percent in 0..=100u8 {
            ch.set_duty_cycle_percent(percent).ok();
            delay.delay_ms(20);
        }

        green_led.set_low().ok();
        red_led.set_high().ok();
        for percent in (0..=100u8).rev() {
            ch.set_duty_cycle_percent(percent).ok();
            delay.delay_ms(20);
        }
    }
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
