#![no_std]
#![no_main]

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, DataBits, Parity, SerialExt, StopBits};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

// Baud rate (and its matching confirmation line) are selected at *build time* so
// one fixture source covers multiple rates — see hal_test_runners's `[features]`.
// Default is 9600; `--features baud_115200` overrides it. The confirmation line
// is a fixed byte string per rate rather than a formatted integer, because
// pulling in `core::fmt` to print the number would not fit the FRAM budget.
#[cfg(feature = "baud_115200")]
const BAUD: u32 = 115200;
#[cfg(feature = "baud_115200")]
const UART_LINE: &[u8] = b"UART 115200 8N1 OK\r\n";

#[cfg(not(feature = "baud_115200"))]
const BAUD: u32 = 9600;
#[cfg(not(feature = "baud_115200"))]
const UART_LINE: &[u8] = b"UART 9600 8N1 OK\r\n";

/// Firmware entry point.
///
/// UART transmit fixture for automated integration testing. Brings up eUSCI_A0
/// at **`BAUD` 8N1** (the UART backchannel observed on `/dev/cu.usbmodem11203`),
/// where `BAUD` is chosen at build time — 9600 by default, or 115200 with
/// `--features baud_115200` — and emits a fixed, greppable sequence of
/// confirmation lines once per second, forever. The repetition makes the
/// host-side test runner robust to *when* it opens the port relative to the
/// board reset that `DSLite load` triggers — it can attach at any time and still
/// catch a full BEGIN..END cycle.
///
/// No external wiring is required: this exercises the TX path, baud-rate math,
/// and P2.0 pin mux end-to-end. The green LED (P1.0) toggles each cycle as a
/// visual heartbeat.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Clock profile: MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds the UART BRCLK below.
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): BAUD baud, 8 data bits, no parity, 1 stop bit, with
    // BRCLK = SMCLK = 8 MHz. The frame format is spelled out explicitly (rather
    // than relying on Config defaults) so this fixture documents the exact line
    // settings the host integration test must open the port with.
    let serial = p.usci_a0_uart_mode.into_uart(
        UartConfig::new(clocks.smclk())
            .baud(BAUD)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One),
    );
    let (mut tx, _rx) = serial.split();

    // Green LED (P1.0 = LED2) as a heartbeat so the board is visibly alive.
    let (port1, _port2) = p.port_1_2.split();
    let mut green_led = port1.pin0.into_output();

    let mut delay = Delay::new(clocks.mclk());

    // One-time banner so a human watching `screen` sees what this binary is.
    tx.write_all(b"MSP430FR5969 serial_uart integration fixture\r\n")
        .ok();

    let mut on = false;
    loop {
        // A self-delimited cycle of confirmation lines. The BEGIN/END markers
        // let the host runner frame one complete burst; the middle lines are
        // distinct, fixed strings it can assert on.
        tx.write_all(b"SERIAL_UART_TEST_BEGIN\r\n").ok();
        tx.write_all(UART_LINE).ok();
        tx.write_all(b"hello from msp430fr5969\r\n").ok();
        tx.write_all(b"SERIAL_UART_TEST_END\r\n").ok();

        on = !on;
        if on {
            green_led.set_high().ok();
        } else {
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// compiler-builtins' memcpy/memcmp reference `abort` on their safety paths.
// Provide a minimal one so we don't link newlib's libc (and its syscall stubs).
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
