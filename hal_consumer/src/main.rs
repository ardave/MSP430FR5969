#![no_std]
#![no_main]

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
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

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

    loop {}
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
