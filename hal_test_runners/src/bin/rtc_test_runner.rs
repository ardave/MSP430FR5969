#![no_std]
#![no_main]

//! RTC_B integration fixture for the `hal::rtc` calendar driver.
//!
//! A self-checking sibling of the human-facing `rtc_clock` demo: instead of just
//! printing the wall clock, it runs a startup self-check and reports a framed
//! pass/fail verdict over the UART backchannel (eUSCI_A0, 9600 8N1 on
//! `/dev/cu.usbmodem11203`), driven by the host-side `rtc_tests` runner. Like the
//! demo it needs no wiring beyond the LaunchPad — RTC_B is on-chip — but it does
//! need the populated 32.768 kHz LFXT crystal (see below).
//!
//! ```text
//! cargo +nightly build --bin rtc_test
//! DSLite load ... -f target/msp430-none-elf/debug/rtc_test
//! ```
//!
//! # What it checks
//!
//! 1. **Load-and-read-back (`RTC SET`).** The calendar is started at a fixed
//!    instant (Sat 2026-06-27 09:30:00). An immediate `now()` must read back that
//!    instant (seconds 0..=1, since the read races the very first tick). This
//!    proves the BCD-vs-binary mode and the held-register write path landed the
//!    value we asked for.
//!
//! 2. **It actually advances (`RTC TICK`).** We snapshot `now()`, busy-wait ~3 s
//!    on the **MCU `Delay` (MCLK/DCO)**, then snapshot again. The elapsed RTC
//!    seconds must be ~3 (tolerance 2..=4). Because the delay is timed by the DCO
//!    while the calendar is timed by the **independent crystal on ACLK**, a clock
//!    that is stuck, running off the wrong source, or not counting at 1 Hz fails
//!    here — the two oscillators cross-check each other.
//!
//! Both verdicts are computed **once** at startup; the loop just re-emits the
//! fixed verdict lines and toggles the **GREEN** LED as a heartbeat. A steady
//! **RED** LED means a check failed.
//!
//! # Hardware requirement: the 32.768 kHz crystal
//!
//! RTC_B counts the LFXT watch crystal on ACLK, so this fixture brings the clocks
//! up with [`hal::clocks::configure_low_power`], which starts LFXT. If the crystal
//! does not start (e.g. a bare chip), ACLK falls back to the imprecise VLO,
//! [`Rtc::new`] returns [`hal::rtc::Error::ClockNot32768`], and the fixture lights
//! the **RED** LED and emits a `RTC CLOCK FAIL` burst (a deliberate refusal the
//! host reports cleanly, rather than a clock that silently runs fast).
//!
//! # Framed output for the host runner
//!
//! Like the `fram_test` and `adc_internal` fixtures, this emits a self-delimited
//! burst once per second, forever, so the host test can attach at any time after
//! the `DSLite load` reset and still catch a complete cycle. Each cycle, over UART:
//!
//! ```text
//! now: 2026-06-27 09:30:07     (human-readable info, skipped by host)
//! RTC_TEST_BEGIN
//! RTC SET OK                    (or `RTC SET FAIL` if the read-back differs)
//! RTC TICK OK                   (or `RTC TICK FAIL` if the calendar did not advance)
//! RTC_TEST_END
//! ```

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

/// The instant the calendar is started at: Sat 2026-06-27 09:30:00.
const START: DateTime = DateTime {
    year: 2026,
    month: 6,
    day: 27,
    weekday: 6,
    hour: 9,
    minute: 30,
    second: 0,
};

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Low-power profile: ACLK on the 32.768 kHz LFXT crystal — required for the
    // RTC to keep correct time. SMCLK = 1 MHz still feeds the UART BRCLK.
    let clocks = hal::clocks::configure_low_power(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 1 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, heartbeat
    let mut red_led = port4.pin6.into_output(); // LED1, failure flag

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 RTC_B self-check\r\n").ok();

    // Start the calendar; refuse (and report) if the crystal did not come up.
    let rtc = match Rtc::new(p.rtc_b_real_time_clock, &clocks, &START) {
        Ok(rtc) => rtc,
        Err(_) => {
            // No 32.768 kHz crystal — emit a framed FAIL burst forever so the host
            // gets a deterministic verdict instead of a timeout, and light RED.
            red_led.set_high().ok();
            loop {
                tx.write_all(b"RTC_TEST_BEGIN\r\n").ok();
                tx.write_all(b"RTC CLOCK FAIL\r\n").ok();
                tx.write_all(b"RTC_TEST_END\r\n").ok();
                delay.delay_ms(1000);
            }
        }
    };

    // --- 1. Load-and-read-back ----------------------------------------------
    // An immediate read should match START; seconds may have ticked to 1 if the
    // read lands just after the first crystal-driven update.
    let first = rtc.now();
    let set_ok = first.year == START.year
        && first.month == START.month
        && first.day == START.day
        && first.hour == START.hour
        && first.minute == START.minute
        && first.second <= 1;

    // --- 2. It actually advances --------------------------------------------
    // Cross-check the crystal (ACLK) against the DCO-timed MCU delay: wait ~3 s
    // and confirm the calendar advanced ~3 s. mod-60 handles a minute rollover.
    let before = rtc.now();
    delay.delay_ms(3000);
    let after = rtc.now();
    let elapsed = (after.second as i16 - before.second as i16).rem_euclid(60);
    let tick_ok = (2..=4).contains(&elapsed);

    // A self-delimited verdict burst, repeated once per second so the host runner
    // can attach at any time after the DSLite reset and still frame a full
    // BEGIN..END cycle. GREEN toggles each cycle as a heartbeat; steady RED means
    // a check failed.
    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN): the
        // live wall clock, observable over `screen`.
        let now = rtc.now();
        tx.write_all(b"now: ").ok();
        print_datetime(&mut tx, &now);

        // Fixed, greppable verdict lines framed by BEGIN/END.
        tx.write_all(b"RTC_TEST_BEGIN\r\n").ok();
        tx.write_all(if set_ok {
            b"RTC SET OK\r\n" as &[u8]
        } else {
            b"RTC SET FAIL\r\n"
        })
        .ok();
        tx.write_all(if tick_ok {
            b"RTC TICK OK\r\n" as &[u8]
        } else {
            b"RTC TICK FAIL\r\n"
        })
        .ok();
        tx.write_all(b"RTC_TEST_END\r\n").ok();

        if set_ok && tick_ok {
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
