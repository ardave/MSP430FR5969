#![no_std]
#![no_main]

//! Starter firmware: blink the LaunchPad's green LED (LED2, P1.0) at ~2 Hz.

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::StatefulOutputPin;
use hal::gpio::GpioExt;
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in
    // that order — hal::init fuses them so the ordering can't be gotten wrong.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz.
    let clocks = hal::clocks::configure(p.cs);

    // Pins power up high-impedance behind the LOCKLPM5 latch; unlock them
    // after configuration.
    hal::gpio::unlock_pins(&p.pmm);

    let (port1, _port2) = p.port_1_2.split();
    let mut green_led = port1.pin0.into_output(); // LED2 = P1.0

    let mut delay = Delay::new(clocks.mclk());

    loop {
        green_led.toggle().ok();
        delay.delay_ms(250);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// compiler-builtins' memcpy/memcmp reference `abort` on their safety paths.
// Provide a minimal one so we don't link newlib's libc (and its syscall stubs).
#[no_mangle]
pub extern "C" fn abort() -> ! {
    loop {}
}
