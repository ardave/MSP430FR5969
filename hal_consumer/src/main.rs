#![no_std]
#![no_main]

use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config, SerialExt};
use msp430_rt::entry;

// The `msp430` crate provides the critical-section implementation for MSP430
// (acquire: read SR then DINT+NOP, release: restore GIE if it was set).
// Force-link it so the `set_impl!` symbols resolve for pac's Peripherals::take().
use msp430 as _;

// Watchdog Timer Password
const WDTPW: u16 = 0x5A00;
// Watchdog Timer Hold.  Setting it stops (pauses) the watchdog timer.
const WDTHOLD: u16 = 0x0080;

/// Firmware entry point.
///
/// `#[entry]` (from msp430-rt) names the function the runtime calls after reset.
/// msp430-rt now owns everything `_start` used to do by hand: its `Reset`
/// handler loads the stack pointer from `_stack_start` (verify with `objdump -d`
/// that `Reset` opens with `mov #0x2400, r1`), zeroes `.bss` and copies `.data`,
/// then jumps here. The reset and interrupt vectors come from the PAC's `rt`
/// feature + msp430-rt's linker script, so there is no naked `_start`, no
/// `.reset_vector` static and no manual `.bss` loop in this crate anymore.
///
/// (An uninitialized stack pointer was the bug this used to guard against: it
/// masqueraded as a UART that hung on its first transmission only some of the
/// time. msp430-rt's Reset closes that hole for us now.)
#[entry]
fn main() -> ! {
    // Stop the watchdog before anything else. msp430-rt initializes RAM but does
    // not touch the WDT, and the default timeout is ~32 ms; Peripherals::take()
    // below also enters a critical section. Raw access because we don't hold the
    // peripheral singletons yet.
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    let p = hal::pac::Peripherals::take().unwrap();

    // Unlock GPIO pins (clear LOCKLPM5 in PM5CTL0) so the UART pin mux takes
    // effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // Configure eUSCI_A0 as a 9600 8N1 UART. After reset, SMCLK on this device
    // is the 8 MHz DCO divided by 8 = 1 MHz, which is the default BRCLK here.
    // UCA0TXD = P2.0, UCA0RXD = P2.1 (configured by the HAL).
    let serial = p.usci_a0_uart_mode.into_uart(Config::default().baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs on the MSP430FR5969 LaunchPad: P1.0 = LED2 (GREEN), P4.6 = LED1 (RED).
    // (Verified on hardware — the colours are the opposite of what's often
    // assumed; the UART labels would not match the LEDs if these were swapped.)
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    tx.write_all(b"MSP430FR5969 UART up @ 9600 8N1\r\n").ok();

    // Busy-loop iterations for a ~1 s hold between transitions. Calibrated
    // empirically at the reset MCLK (1 MHz): 150_000 iterations measured at
    // ~2.695 s, i.e. ~18 cycles/iter, so 1 s ≈ 55_300. This is a rough timing
    // loop, not a precise one — it drifts with optimization level and MCLK; a
    // hardware timer is the proper fix once the clock/timer HAL exists.
    const ONE_SECOND: u32 = 55_300;

    // Alternate the two LEDs, printing the colour of whichever just turned on.
    loop {
        red_led.set_high().ok();
        green_led.set_low().ok();
        tx.write_all(b"red\r\n").ok();
        delay(ONE_SECOND);

        green_led.set_high().ok();
        red_led.set_low().ok();
        tx.write_all(b"green\r\n").ok();
        delay(ONE_SECOND);
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

// compiler-builtins' memcpy/memcmp routines reference `abort` on their safety
// paths. Provide a minimal one so we don't have to link newlib's libc (which
// would in turn pull in unhosted syscall stubs: _exit, kill, getpid).
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
