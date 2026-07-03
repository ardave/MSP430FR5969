#![no_std]
#![no_main]

//! RTC_B calendar demo for the `hal::rtc` driver.
//!
//! Sets the real-time clock to a fixed date/time, then prints the wall clock
//! once per second over the UART backchannel as `YYYY-MM-DD HH:MM:SS`, blinking
//! the GREEN LED on each tick. Needs no bus wiring and does not conflict with the
//! SPI/I2C demos:
//!
//! ```text
//! cargo +nightly build --bin rtc_clock
//! DSLite load ... -f target/msp430-none-elf/debug/rtc_clock
//! ```
//!
//! # Hardware requirement: the 32.768 kHz crystal
//!
//! The RTC_B counts the **LFXT watch crystal on ACLK** ("RTC is clocked by
//! XT1"), so this demo brings the clocks up with
//! [`hal::clocks::configure_low_power`], which starts LFXT. The
//! MSP-EXP430FR5969 LaunchPad populates that crystal, so it should start. If it
//! does **not** (e.g. a bare chip), ACLK falls back to the imprecise VLO,
//! [`Rtc::new`] returns [`hal::rtc::Error::ClockNot32768`], and this demo lights
//! the **RED** LED and prints `RTC: no 32768 Hz crystal` instead of counting —
//! a deliberate refusal rather than a clock that silently runs fast.
//!
//! # What you should see
//!
//! Over UART (9600 8N1 on eUSCI_A0): one line per second,
//! `2026-06-27 09:30:00`, `...01`, `...02`, … with the GREEN LED winking each
//! second. The cadence is crystal-accurate — compare it against a watch over a
//! minute and it should not drift.

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::rtc::{DateTime, Rtc};
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Low-power profile: ACLK on the 32.768 kHz LFXT crystal — required for the
    // RTC to keep correct time. SMCLK = 1 MHz still feeds the UART BRCLK.
    let clocks = hal::clocks::configure_low_power(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the pin muxes take effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 1 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, 1 Hz blink
    let mut red_led = port4.pin6.into_output(); // LED1, crystal-fault flag

    let mut delay = Delay::new(clocks.mclk());

    // Start the calendar at a fixed instant: Sat 2026-06-27 09:30:00.
    let start = DateTime {
        year: 2026,
        month: 6,
        day: 27,
        weekday: 6,
        hour: 9,
        minute: 30,
        second: 0,
    };

    let rtc = match Rtc::new(p.rtc_b_real_time_clock, &clocks, &start) {
        Ok(rtc) => rtc,
        Err(_) => {
            // No 32.768 kHz crystal — refuse rather than count at the wrong rate.
            red_led.set_high().ok();
            tx.write_all(b"RTC: no 32768 Hz crystal (ACLK on VLO) -- not started\r\n")
                .ok();
            loop {}
        }
    };

    tx.write_all(b"MSP430FR5969 RTC_B demo: wall clock at 1 Hz\r\n")
        .ok();

    // Poll for the second to change, then print and blink. (An ISR on the RTC
    // vector via rtc.enable_second_interrupt() would do this without polling.)
    let mut last = 255u8;
    loop {
        let now = rtc.now();
        if now.second != last {
            last = now.second;
            green_led.set_high().ok();
            print_datetime(&mut tx, &now);
            delay.delay_ms(50);
            green_led.set_low().ok();
        }
    }
}

/// Print `YYYY-MM-DD HH:MM:SS\r\n`.
fn print_datetime<W: hal::embedded_io::Write>(tx: &mut W, dt: &DateTime) {
    write_dec(tx, dt.year as u32);
    tx.write_all(b"-").ok();
    write_two(tx, dt.month);
    tx.write_all(b"-").ok();
    write_two(tx, dt.day);
    tx.write_all(b" ").ok();
    write_two(tx, dt.hour);
    tx.write_all(b":").ok();
    write_two(tx, dt.minute);
    tx.write_all(b":").ok();
    write_two(tx, dt.second);
    tx.write_all(b"\r\n").ok();
}

/// Write a value `0..=99` as exactly two zero-padded decimal digits.
fn write_two<W: hal::embedded_io::Write>(tx: &mut W, value: u8) {
    let buf = [b'0' + (value / 10) % 10, b'0' + value % 10];
    tx.write_all(&buf).ok();
}

/// Write an unsigned value as decimal ASCII (no padding); for the year.
fn write_dec<W: hal::embedded_io::Write>(tx: &mut W, mut value: u32) {
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (value % 10) as u8;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    tx.write_all(&buf[i..]).ok();
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
