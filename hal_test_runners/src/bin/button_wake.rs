#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! Button-wake demo: sleep in **LPM4** until a LaunchPad button is pressed.
//!
//! LPM4 stops every clock on the part — no DCO, no SMCLK, not even the
//! 32.768 kHz crystal — so nothing scheduled can wake it. What *can* is a port
//! interrupt: `PxIFG` latches an edge asynchronously, no clock required. This
//! demo parks the CPU in LPM4 and wakes it only when a human presses a button:
//!
//! - **S2 (P1.1)** toggles the **GREEN** LED (P1.0),
//! - **S1 (P4.5)** toggles the **RED** LED (P4.6),
//!
//! and each press prints a line over the UART backchannel (eUSCI_A0, 9600 8N1,
//! `screen /dev/cu.usbmodem11203 9600`). Between presses the part draws
//! LPM4-grade current (sub-µA core, per datasheet — measurable across the
//! LaunchPad's current jumpers if you're curious).
//!
//! No wiring: the buttons are on the board. They short to GND with no external
//! resistor, so the pins use the internal pull-up and presses are falling edges.
//!
//! Two details worth noticing:
//!
//! - The ISRs are `#[interrupt(wake_cpu)]`: they clear the low-power bits in
//!   the *stacked* SR, so `enter_lpm4()` returns to `main` after the ISR
//!   instead of the CPU going straight back to sleep. Which button fired
//!   travels out of the ISR as the recorded `PxIV` value; `main` does the LED
//!   and UART work in thread mode, keeping the ISRs minimal.
//! - `tx.flush()` before `enter_lpm4()` is load-bearing: UART TX rides SMCLK,
//!   and LPM4 stops it. Sleeping while the last character is still in the
//!   shift register would truncate it mid-air; `flush` waits for `UCBUSY` to
//!   drop first. (On wake SMCLK simply resumes — the UART needs no reinit.)
//!
//! ```text
//! cargo +nightly build --bin button_wake
//! DSLite load ... -f target/msp430-none-elf/debug/button_wake
//! ```

use core::cell::Cell;

use critical_section::Mutex;
use hal::embedded_hal::digital::StatefulOutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::{self, Edge, GpioExt};
use hal::interrupt;
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Bitmask of buttons seen since main last drained it: bit 0 = S2 (P1.1),
/// bit 1 = S1 (P4.5). A mask (not a queue) — a bounce burst coalesces into one
/// wake, which is all a toggle demo wants.
static PRESSED: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));

/// S2 press: consume P1IV (clears the flag in silicon) and mark the button.
/// `wake_cpu` lets `main` resume after `enter_lpm4()`.
#[msp430_rt::interrupt(wake_cpu)]
fn PORT1() {
    if gpio::read_iv::<gpio::P1>() == 0x04 {
        critical_section::with(|cs| {
            let p = PRESSED.borrow(cs);
            p.set(p.get() | 0x01);
        });
    }
}

/// S1 press: same shape on P4IV (P4.5 → 0x0C).
#[msp430_rt::interrupt(wake_cpu)]
fn PORT4() {
    if gpio::read_iv::<gpio::P4>() == 0x0C {
        critical_section::with(|cs| {
            let p = PRESSED.borrow(cs);
            p.set(p.get() | 0x02);
        });
    }
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Performance profile: SMCLK = 8 MHz for the UART. The DCO restarting on
    // each wake is automatic; no clock is needed *during* LPM4.
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so pin muxes and pin interrupts take effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2 ← S2
    let mut red_led = port4.pin6.into_output(); // LED1 ← S1

    // Buttons: internal pull-up, interrupt on the falling (press) edge.
    let mut s2 = port1.pin1.into_pull_up_input(); // S2 = P1.1
    let mut s1 = port4.pin5.into_pull_up_input(); // S1 = P4.5
    s2.enable_interrupt(Edge::Falling);
    s1.enable_interrupt(Edge::Falling);

    tx.write_all(b"\r\nLPM4 button wake: S2 toggles GREEN, S1 toggles RED\r\n")
        .ok();

    loop {
        // Drain everything before sleeping: UART must be idle (SMCLK stops in
        // LPM4) and the pressed-mask empty (a press that landed between the
        // drain below and here left its wake pending — GIE is still off only
        // inside the ISRs; enter_lpm4 sets GIE atomically with sleeping, so a
        // latched PxIFG fires immediately and we fall straight through, which
        // is exactly right).
        tx.flush().ok();
        hal::power::enter_lpm4();

        let pressed = critical_section::with(|cs| {
            let p = PRESSED.borrow(cs);
            let v = p.get();
            p.set(0);
            v
        });
        if pressed & 0x01 != 0 {
            green_led.toggle().ok();
            tx.write_all(b"S2 -> GREEN\r\n").ok();
        }
        if pressed & 0x02 != 0 {
            red_led.toggle().ok();
            tx.write_all(b"S1 -> RED\r\n").ok();
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// compiler-builtins' memcpy/memcmp reference `abort` on their safety paths.
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
