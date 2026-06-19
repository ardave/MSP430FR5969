#![no_std]
#![no_main]

use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config, SerialExt};

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

    // Unlock GPIO pins (clear LOCKLPM5 in PM5CTL0) so the UART pin mux takes
    // effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // Configure eUSCI_A0 as a 9600 8N1 UART. After reset, SMCLK on this device
    // is the 8 MHz DCO divided by 8 = 1 MHz, which is the default BRCLK here.
    // UCA0TXD = P2.0, UCA0RXD = P2.1 (configured by the HAL).
    let serial = p.usci_a0_uart_mode.into_uart(Config::default().baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs on the MSP430FR5969 LaunchPad: P1.0 = LED2 (red), P4.6 = LED1 (green).
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut red_led = port1.pin0.into_output();
    let mut green_led = port4.pin6.into_output();

    tx.write_all(b"MSP430FR5969 UART up @ 9600 8N1\r\n").ok();

    // Alternate the two LEDs, printing the colour of whichever just turned on.
    loop {
        red_led.set_high().ok();
        green_led.set_low().ok();
        tx.write_all(b"red\r\n").ok();
        delay(150_000);

        green_led.set_high().ok();
        red_led.set_low().ok();
        tx.write_all(b"green\r\n").ok();
        delay(150_000);
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
