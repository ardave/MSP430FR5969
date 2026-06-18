#![no_std]
#![no_main]

use pac::Peripherals;
// Force the msp430 rlib into the link so its critical-section `set_impl!`
// symbols (_critical_section_1_0_acquire/_release) resolve the references
// pulled in by pac's `Peripherals::take()`.
use msp430 as _;

// Watchdog password must be written to upper byte of WDTCTL
const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

// Place reset vector at 0xFFFE pointing to _start
#[used]
#[unsafe(link_section = ".reset_vector")]
static RESET_VECTOR: unsafe extern "C" fn() -> ! = _start;

#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    // Use raw register addresses to bypass Peripherals::take() / critical-section
    const WDTCTL: *mut u16 = 0x015C as *mut u16;
    const PM5CTL0: *mut u16 = 0x0130 as *mut u16;
    const P1OUT: *mut u8 = 0x0202 as *mut u8; // PORT_1_2 base 0x200 + offset 2
    const P1DIR: *mut u8 = 0x0204 as *mut u8; // PORT_1_2 base 0x200 + offset 4
    const P4OUT: *mut u8 = 0x0223 as *mut u8; // PORT_3_4 base 0x220 + offset 3
    const P4DIR: *mut u8 = 0x0225 as *mut u8; // PORT_3_4 base 0x220 + offset 5

    unsafe {
        // Stop the watchdog timer
        WDTCTL.write_volatile(WDTPW | WDTHOLD);

        // Unlock GPIO pins (clear LOCKLPM5 bit)
        let pm5 = PM5CTL0.read_volatile();
        PM5CTL0.write_volatile(pm5 & !0x0001);

        // Set P1.0 (LED2, red) as output
        P1DIR.write_volatile(P1DIR.read_volatile() | 0x01);

        // Set P4.6 (LED1, green) as output
        P4DIR.write_volatile(P4DIR.read_volatile() | 0x40);

        // Blink forever
        loop {
            P1OUT.write_volatile(P1OUT.read_volatile() ^ 0x01);
            P4OUT.write_volatile(P4OUT.read_volatile() ^ 0x40);
            delay(20_000);
        }
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
