#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

// The `msp430` crate provides the critical-section implementation for MSP430
// (acquire: read SR then DINT+NOP, release: restore GIE if it was set).
// Force-link it so the `set_impl!` symbols resolve for pac's Peripherals::take().
use msp430 as _;

const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

#[used]
#[unsafe(link_section = ".reset_vector")]
static RESET_VECTOR: unsafe extern "C" fn() -> ! = _start;

/// Reset entry point.
///
/// PITFALL: the MSP430 does **not** initialize the stack pointer (R1) in
/// hardware on reset, and this project has no crt0/`msp430-rt` to do it for us.
/// If the very first thing that runs is ordinary compiled code, its prologue
/// will `push`/`sub` against whatever garbage R1 holds at reset, scribbling over
/// random memory. The symptom is *intermittent* corruption that changes shape
/// with each build and each reset (the reset-time SP varies), so it is easily
/// mistaken for a peripheral bug.
///
/// The cure: this is a `#[naked]` function, so the compiler emits no prologue.
/// Its first instruction loads SP with `__stack_top` (top of RAM, from
/// `memory.x`); only then do we tail-call the real entry, which never returns.
/// Every `_start` in this workspace must follow this pattern.
#[unsafe(naked)]
#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    core::arch::naked_asm!(
        "mov #__stack_top, r1",
        "br  #{main}",
        main = sym rust_main,
    )
}

extern "C" fn rust_main() -> ! {
    // Stop the watchdog before anything else — must use raw access because
    // Peripherals::take() itself enters a critical section, and the watchdog
    // will fire if we wait for the PAC setup.
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

    // Take peripheral singletons — this exercises the critical-section impl
    // (disables interrupts via DINT+NOP, checks the static flag, re-enables).
    let p = pac::Peripherals::take().unwrap();

    // Unlock GPIO pins (clear LOCKLPM5 bit in PM5CTL0)
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // Set P1.0 (LED2, red) and P4.6 (LED1, green) as outputs
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
