#![no_std]
#![no_main]

use hal::embedded_hal::digital::{OutputPin, StatefulOutputPin};
use hal::gpio::GpioExt;

// The `msp430` crate provides the critical-section implementation for MSP430
// (acquire: read SR then DINT+NOP, release: restore GIE if it was set).
// Force-link it so the `set_impl!` symbols resolve for pac's Peripherals::take().
use msp430 as _;

const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

#[used]
#[unsafe(link_section = ".reset_vector")]
static RESET_VECTOR: unsafe extern "C" fn() -> ! = _start;

#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    // Stop the watchdog before anything else — must use raw access because
    // Peripherals::take() enters a critical section, and the default watchdog
    // timeout is ~32ms.
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    // Zero .bss — no runtime does this for us, and SRAM powers up with
    // indeterminate contents. DEVICE_PERIPHERALS (used by take()) lives here.
    unsafe {
        unsafe extern "C" {
            static mut __bss_start: u8;
            static mut __bss_end: u8;
        }
        let start = &raw mut __bss_start as u16;
        let end = &raw mut __bss_end as u16;
        let mut addr = start;
        while addr < end {
            (addr as *mut u8).write_volatile(0);
            addr += 1;
        }
    }

    let p = hal::pac::Peripherals::take().unwrap();

    // Unlock GPIO pins (clear LOCKLPM5 in PM5CTL0)
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // Split port peripherals into individual typed pins
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();

    // Configure LED pins as outputs via the HAL
    //   P1.0 = LED2 (red) on the MSP430FR5969 LaunchPad
    //   P4.6 = LED1 (green)
    let mut red_led = port1.pin0.into_output();
    let mut green_led = port4.pin6.into_output();

    // Start with red on, green off
    red_led.set_high().unwrap();
    green_led.set_low().unwrap();

    // Blink both LEDs in alternation using embedded-hal traits
    loop {
        red_led.toggle().unwrap();
        green_led.toggle().unwrap();
        delay(20_000);
    }
}

#[inline(never)]
fn delay(n: u32) {
    let mut i = n;
    while i > 0 {
        i -= 1;
        core::hint::black_box(i);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
