#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits handlers with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! Sleep in **LPM4** until a LaunchPad button is pressed.
//!
//! LPM4 stops every clock on the part, so only an asynchronously-latched port
//! interrupt can wake it. S2 (P1.1) toggles the green LED, S1 (P4.5) toggles
//! the red one. No wiring — the buttons are on the board (they short to GND,
//! so the pins use the internal pull-up and a press is a falling edge).
//!
//! The interrupt recipe, end to end: handlers are `#[interrupt(wake_cpu)]` so
//! `power::enter_lpm4()` returns to `main` after the ISR; which button fired
//! travels out through a `critical_section::Mutex<Cell<u8>>` mask; and the
//! ISR consumes `PxIV` via `gpio::read_iv`, whose read clears the served flag
//! in silicon. `tx.flush()` before sleeping is load-bearing — UART TX rides
//! SMCLK, which LPM4 stops mid-character otherwise.
//!
//! ```text
//! cargo +nightly build --example button_wake --features rt,critical-section
//! tools/flash.sh target/msp430-none-elf/debug/examples/button_wake
//! ```

use msp430fr5969_hal as hal;

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

/// Buttons seen since main last drained the mask: bit 0 = S2, bit 1 = S1.
static PRESSED: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));

/// S2 press (P1.1 → P1IV = 0x04). `wake_cpu` lets `main` resume after LPM4.
#[msp430_rt::interrupt(wake_cpu)]
fn PORT1() {
    if gpio::read_iv::<gpio::P1>() == 0x04 {
        critical_section::with(|cs| {
            let p = PRESSED.borrow(cs);
            p.set(p.get() | 0x01);
        });
    }
}

/// S1 press (P4.5 → P4IV = 0x0C).
#[msp430_rt::interrupt(wake_cpu)]
fn PORT4() {
    if gpio::read_iv::<gpio::P4>() == 0x0C {
        critical_section::with(|cs| {
            let p = PRESSED.borrow(cs);
            p.set(p.get() | 0x02);
        });
    }
}

#[entry]
fn main() -> ! {
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // SMCLK = 8 MHz for the UART; no clock is needed *during* LPM4 (the DCO
    // restarts automatically on each wake).
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1.
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
        // Drain the UART before sleeping (LPM4 stops SMCLK). enter_lpm4 sets
        // GIE atomically with sleeping, so a press latched in the gap fires
        // immediately and we fall straight through — which is exactly right.
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
#[no_mangle]
pub extern "C" fn abort() -> ! {
    loop {}
}
