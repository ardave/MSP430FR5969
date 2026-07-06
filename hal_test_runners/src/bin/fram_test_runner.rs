#![no_std]
#![no_main]

//! FRAM read/write integration fixture for the `hal::fram` storage backends.
//!
//! Exercises both FRAM regions and reports over the UART backchannel (eUSCI_A0,
//! 9600 8N1 on `/dev/cu.usbmodem11203`), driven by the host-side `fram_tests`
//! runner. Unlike the SPI/I2C demos this needs no wiring beyond the LaunchPad
//! itself — FRAM is on-chip.
//!
//! ```text
//! cargo +nightly build --bin fram_test
//! DSLite load ... -f target/msp430-none-elf/debug/fram_test
//! ```
//!
//! # What it does
//!
//! 1. **Persistent boot counter (Info FRAM, 16-bit access).** A 4-byte magic +
//!    4-byte counter live at the start of Information FRAM (0x1800). Each boot
//!    reads the counter, increments it, prints it, and writes it back. Because
//!    FRAM is non-volatile, the count **survives power-cycles and resets** — pull
//!    power, reconnect, and the number keeps climbing. That is the proof the
//!    write actually stuck in non-volatile memory.
//!
//! 2. **Upper-FRAM round-trip (0x10000, 20-bit access).** Writes a known pattern
//!    into the upper 16 KB — the storage "beyond the first 48 K," reachable only
//!    via the hand-emitted MSP430X instructions in `hal::fram` — then reads it
//!    back and compares. It also reads the region *before* writing, so on the
//!    second and later boots it can confirm the previous boot's pattern
//!    persisted.
//!
//! # Framed output for the host runner
//!
//! Like the `serial_uart` and `adc_internal` fixtures, this emits a self-delimited
//! burst once per second, forever, so the host test can attach at any time after
//! the `DSLite load` reset and still catch a complete cycle. Each cycle, over UART:
//!
//! ```text
//! boot count: 7 (persisted from last boot)   (human-readable info, skipped by host)
//! FRAM_TEST_BEGIN
//! INFO FRAM OK                                 (or `... FAIL` if the counter read-back differs)
//! HIGH FRAM OK                                 (or `... FAIL` if the upper-FRAM round-trip differs)
//! FRAM_TEST_END
//! ```
//!
//! The counter increment, the upper-FRAM round-trip, and their pass/fail verdicts
//! are all computed **once** at startup; the loop just re-emits the fixed verdict
//! lines and toggles the **GREEN** LED as a heartbeat. A steady **RED** LED means
//! a FRAM round-trip failed. The boot count in the info line still climbs by one
//! on every power-cycle — the persisted counter, observable over `screen`.

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::embedded_storage::{ReadStorage, Storage};
use hal::fram::{HighFram, InfoFram};
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Marks Info FRAM as "initialized by this firmware" so a blank/foreign chip is
/// detected and the counter starts from zero instead of garbage.
const MAGIC: u32 = 0xF5A1_0001;

/// 16-byte pattern written into upper FRAM; chosen so an all-00/all-FF region
/// (or a stuck address line) would not match by accident.
const PATTERN: [u8; 16] = [
    0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0xF0, 0x0D, 0xBA, 0xBE,
];

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds the UART BRCLK. MCLK at 1 MHz keeps
    // FRAM wait states (FRCTL0.NWAITS) at their reset default of 0.
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO pins (clear LOCKLPM5) so the UART pin mux takes effect.
    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1).
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());

    let mut info = InfoFram::new();
    let mut high = HighFram::new();

    tx.write_all(b"\r\nMSP430FR5969 FRAM demo\r\n").ok();

    // --- 1. Boot counter in Info FRAM (16-bit access) -----------------------
    let mut magic_buf = [0u8; 4];
    let mut count_buf = [0u8; 4];
    info.read(0, &mut magic_buf).ok();
    info.read(4, &mut count_buf).ok();

    let count = if u32::from_le_bytes(magic_buf) == MAGIC {
        u32::from_le_bytes(count_buf).wrapping_add(1)
    } else {
        // First run on this chip (or Info FRAM never initialized): seed it.
        info.write(0, &MAGIC.to_le_bytes()).ok();
        1
    };
    info.write(4, &count.to_le_bytes()).ok();

    // Read the counter back to confirm the write actually stuck — an in-boot
    // round-trip the host can assert on. (Cross-power-cycle persistence is the
    // human-observable extra reported in the info line below.)
    let mut readback = [0u8; 4];
    info.read(4, &mut readback).ok();
    let info_ok = u32::from_le_bytes(readback) == count;

    // --- 2. Upper-FRAM round-trip (20-bit access, "beyond 48K") -------------
    // Read first so we can tell if the previous boot's pattern persisted.
    let mut before = [0u8; 16];
    high.read(0, &mut before).ok();
    let persisted = before == PATTERN;

    // Write the pattern at the bottom of the region and one byte at the very top
    // (offset 0x3FFF) to exercise the full 16 KB span / the ADDA carry.
    high.write(0, &PATTERN).ok();
    high.write(0x3FFF, &[0x5A]).ok();

    let mut after = [0u8; 16];
    high.read(0, &mut after).ok();
    let mut top = [0u8; 1];
    high.read(0x3FFF, &mut top).ok();

    let high_ok = after == PATTERN && top[0] == 0x5A;

    // A self-delimited verdict burst, repeated once per second so the host runner
    // can attach at any time after the DSLite reset and still frame a full
    // BEGIN..END cycle. The GREEN LED toggles each cycle as a heartbeat; a steady
    // RED LED means a FRAM round-trip failed.
    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN): the
        // persisted boot count, which climbs by one on every power-cycle.
        tx.write_all(b"boot count: ").ok();
        write_u32(&mut tx, count);
        if persisted {
            tx.write_all(b" (persisted from last boot)").ok();
        }
        tx.write_all(b"\r\n").ok();

        // Fixed, greppable verdict lines framed by BEGIN/END. A bad round-trip
        // flips a verdict to FAIL, which the host asserts against.
        tx.write_all(b"FRAM_TEST_BEGIN\r\n").ok();
        tx.write_all(if info_ok {
            b"INFO FRAM OK\r\n" as &[u8]
        } else {
            b"INFO FRAM FAIL\r\n"
        })
        .ok();
        tx.write_all(if high_ok {
            b"HIGH FRAM OK\r\n" as &[u8]
        } else {
            b"HIGH FRAM FAIL\r\n"
        })
        .ok();
        tx.write_all(b"FRAM_TEST_END\r\n").ok();

        if info_ok && high_ok {
            red_led.set_low().ok();
            on = !on;
            if on {
                green_led.set_high().ok();
            } else {
                green_led.set_low().ok();
            }
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write a `u32` as decimal over the UART.
fn write_u32<W: hal::embedded_io::Write>(tx: &mut W, mut n: u32) {
    if n == 0 {
        tx.write_all(b"0").ok();
        return;
    }
    let mut buf = [0u8; 10]; // u32::MAX is 10 digits
    let mut i = buf.len();
    while n > 0 {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    tx.write_all(&buf[i..]).ok();
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
