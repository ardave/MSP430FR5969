#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]

use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config, SerialExt};

// The `msp430` crate provides the critical-section implementation for MSP430
// (acquire: read SR then DINT+NOP, release: restore GIE if it was set).
// Force-link it so the `set_impl!` symbols resolve for pac's Peripherals::take().
use msp430 as _;

// Watchdog Timer Password
const WDTPW: u16 = 0x5A00;
// Watchdog Timer Hold.  Setting it stops (pauses) the watchdog timer.
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
/// mistaken for a peripheral bug — here it masqueraded as a UART that hung on
/// its first transmission only some of the time.
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
