#![no_std]
#![no_main]

//! FRAM read/write demo for the `hal::fram` storage backends.
//!
//! Exercises both FRAM regions and reports over the UART backchannel (eUSCI_A0,
//! 9600 8N1 on `/dev/cu.usbmodem11203`). Unlike the SPI/I2C demos this needs no
//! wiring beyond the LaunchPad itself — FRAM is on-chip.
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
//! The boot count and a `HIGH FRAM OK` / `FAIL` line are printed once over UART
//! at startup. The **GREEN** LED then blinks the boot count on a loop (so each
//! power-cycle adds one visible blink — the persisted counter without a UART); a
//! steady **RED** LED means the upper-FRAM round-trip failed.

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

// Watchdog Timer Password / Hold.
const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

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
    // Stop the watchdog before anything else (default timeout ~32 ms, and
    // Peripherals::take() enters a critical section).
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    let p = hal::pac::Peripherals::take().unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds the UART BRCLK. MCLK at 1 MHz keeps
    // FRAM wait states (FRCTL0.NWAITS) at their reset default of 0.
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO pins (clear LOCKLPM5) so the UART pin mux takes effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

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

    tx.write_all(b"boot count: ").ok();
    write_u32(&mut tx, count);
    tx.write_all(b" (survives power-cycle)\r\n").ok();

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

    let ok = after == PATTERN && top[0] == 0x5A;

    tx.write_all(b"high fram (0x10000): ").ok();
    if ok {
        tx.write_all(b"HIGH FRAM OK").ok();
    } else {
        tx.write_all(b"FAIL").ok();
    }
    if persisted {
        tx.write_all(b", persisted from last boot").ok();
    }
    tx.write_all(b"\r\n").ok();

    // Visual boot counter: on each cycle, blink the GREEN LED `count` times, then
    // hold a long gap so the groups are distinguishable. Power-cycle the board and
    // you'll literally see one more blink each time — the persisted count, no UART
    // needed. A steady RED LED means the high-FRAM round-trip failed.
    loop {
        if !ok {
            red_led.set_high().ok();
            green_led.set_low().ok();
            delay.delay_ms(1000);
            continue;
        }
        red_led.set_low().ok();

        let mut i = 0u32;
        while i < count {
            green_led.set_high().ok();
            delay.delay_ms(200);
            green_led.set_low().ok();
            delay.delay_ms(300);
            i += 1;
        }
        // Long dark gap between groups so the blink count reads cleanly.
        delay.delay_ms(2000);
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
