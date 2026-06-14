#![no_std]
#![no_main]

use pac::Peripherals;

// Watchdog password must be written to upper byte of WDTCTL
const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

// Place reset vector at 0xFFFE pointing to _start
#[used]
#[unsafe(link_section = ".reset_vector")]
static RESET_VECTOR: unsafe extern "C" fn() -> ! = _start;

#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    let periph = unsafe { Peripherals::steal() };

    // Stop the watchdog timer
    periph
        .watchdog_timer
        .wdtctl()
        .write(|w| unsafe { w.bits(WDTPW | WDTHOLD) });

    // Unlock GPIO pins (clear LOCKLPM5)
    periph
        .pmm
        .pm5ctl0()
        .modify(|_, w| w.locklpm5().clear_bit());

    // Set P1.0 (LED2, red) as output
    periph
        .port_1_2
        .p1dir()
        .modify(|_, w| w.p1dir0().set_bit());

    // Set P4.6 (LED1, green) as output
    periph
        .port_3_4
        .p4dir()
        .modify(|_, w| w.p4dir6().set_bit());

    // Blink forever: toggle both LEDs with a long delay
    loop {
        // Toggle P1.0
        periph
            .port_1_2
            .p1out()
            .modify(|r, w| {
                if r.p1out0().bit_is_set() {
                    w.p1out0().clear_bit()
                } else {
                    w.p1out0().set_bit()
                }
            });

        // Toggle P4.6
        periph
            .port_3_4
            .p4out()
            .modify(|r, w| {
                if r.p4out6().bit_is_set() {
                    w.p4out6().clear_bit()
                } else {
                    w.p4out6().set_bit()
                }
            });

        // Busy-wait delay (~1s at default ~1 MHz DCO)
        delay(12_000);
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
