#![no_std]
#![no_main]

use hal::embedded_hal_nb::nb;
use hal::embedded_hal_nb::serial::Read as _;
use hal::embedded_hal_nb::serial::Write as NbWrite;
use hal::embedded_io::Write as _;
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
    let (mut tx, mut rx) = serial.split();

    // Greet over the serial link using the blocking embedded-io Write trait.
    tx.write_all(b"MSP430FR5969 UART up @ 9600 8N1\r\n").ok();

    // Echo received bytes back, blocking on each direction with the
    // non-blocking embedded-hal serial traits + nb::block!.
    loop {
        let byte = nb::block!(rx.read()).unwrap_or(b'?');
        nb::block!(NbWrite::write(&mut tx, byte)).ok();
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
