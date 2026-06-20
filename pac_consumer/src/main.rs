#![no_std]
#![no_main]

use msp430_rt::entry;

// The `msp430` crate provides the critical-section implementation for MSP430
// (acquire: read SR then DINT+NOP, release: restore GIE if it was set).
// Force-link it so the `set_impl!` symbols resolve for pac's Peripherals::take().
use msp430 as _;

const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

/// Firmware entry point.
///
/// `#[entry]` (from msp430-rt) names the function the runtime calls after reset.
/// msp430-rt now owns everything `_start` used to do by hand: its `Reset`
/// handler loads the stack pointer from `_stack_start` (`mov #__stack_top, r1`
/// in spirit — verify with `objdump -d` that `Reset` opens with `mov #0x2400,
/// r1`), zeroes `.bss` and copies `.data`, then jumps here. The reset vector and
/// the interrupt vector table come from the PAC's `rt` feature + msp430-rt's
/// linker script, so there is no naked `_start`, no `.reset_vector` static and
/// no manual `.bss` loop in this crate anymore.
#[entry]
fn main() -> ! {
    // Stop the watchdog before anything else. msp430-rt initializes RAM but does
    // not touch the WDT, and the default timeout is ~32 ms; Peripherals::take()
    // below also enters a critical section. Raw access because we don't hold the
    // peripheral singletons yet.
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    // Take peripheral singletons — this exercises the critical-section impl
    // (disables interrupts via DINT+NOP, checks the static flag, re-enables).
    let p = pac::Peripherals::take().unwrap();

    // Unlock GPIO pins (clear LOCKLPM5 bit in PM5CTL0)
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // Set P1.0 (LED2, GREEN) and P4.6 (LED1, RED) as outputs.
    // (Colours verified on hardware — opposite of the commonly-assumed mapping.)
    p.port_1_2.p1dir().modify(|_, w| w.p1dir0().set_bit());
    p.port_3_4.p4dir().modify(|_, w| w.p4dir6().set_bit());

    // Blink forever
    loop {
        p.port_1_2.p1out().modify(|r, w| {
            if r.p1out0().bit() {
                w.p1out0().clear_bit()
            } else {
                w.p1out0().set_bit()
            }
        });
        p.port_3_4.p4out().modify(|r, w| {
            if r.p4out6().bit() {
                w.p4out6().clear_bit()
            } else {
                w.p4out6().set_bit()
            }
        });
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
