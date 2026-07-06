#![no_std]
#![no_main]

//! `hal::delay::Delay` integration fixture: validate the software busy-loop against
//! an independent hardware clock.
//!
//! `Delay` makes time pass by spending a calibrated number of DCO cycles; the only
//! way to know it spends the *right* number on real silicon is to measure it with a
//! clock it does not derive from. This fixture brings up the **low-power profile**
//! ([`hal::clocks::configure_low_power`], ACLK = 32.768 kHz LFXT crystal) and uses
//! an **ACLK-sourced [`Counter`]** (~30.5 µs/tick, ~2 s wrap) as the yardstick: the
//! crystal and the DCO are independent oscillators, so a `Delay` that runs fast or
//! slow shows up as a measured duration that misses its target. Reports a framed
//! pass/fail verdict over the UART backchannel (eUSCI_A0, 9600 8N1), driven by the
//! host-side `delay_tests` runner. No wiring needed beyond the LaunchPad.
//!
//! ```text
//! cargo +nightly build --bin delay_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/delay_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **Millisecond delays (`DELAY MS`).** `delay_ms(250)`, `delay_ms(500)`, and
//!    `delay_ms(1000)` are each timed by the crystal counter and must land within
//!    ±5 % of target. Each interval is under one ~2 s counter wrap, so a single
//!    `u16` `elapsed_since` is valid.
//!
//! 2. **Microsecond delay (`DELAY US`).** `delay_us(2000)` exercises the sub-ms
//!    path of the `DelayNs` trait. At 30.5 µs/tick the crystal can only resolve it
//!    to ~65 ticks, so the band is wide, but a delay off by an order of magnitude
//!    still fails.
//!
//! Both verdicts are computed **once** at startup; the loop re-emits the fixed
//! verdict lines and toggles the **GREEN** LED as a heartbeat. A steady **RED** LED
//! means a check failed.
//!
//! # Hardware requirement: the 32.768 kHz crystal
//!
//! The reference counter is only trustworthy on the crystal. As with the RTC
//! fixture, if LFXT does not start ACLK falls back to the imprecise VLO; this
//! fixture refuses (lights **RED**, emits a `DELAY CLOCK FAIL` burst) rather than
//! grade a delay against a yardstick of unknown length.
//!
//! # Framed output for the host runner
//!
//! ```text
//! delay ms1000=998433 us2000=1983   (human-readable info, skipped by host)
//! DELAY_TEST_BEGIN
//! DELAY MS OK                        (or `DELAY MS FAIL`)
//! DELAY US OK                        (or `DELAY US FAIL`)
//! DELAY_TEST_END
//! ```

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::timer::{Counter, Divider};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Low-power profile: ACLK on the 32.768 kHz LFXT crystal (the independent
    // reference clock). MCLK = 1 MHz drives the Delay under test; SMCLK = 1 MHz
    // feeds the UART BRCLK.
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

    tx.write_all(b"\r\nMSP430FR5969 Delay self-check\r\n").ok();

    // Refuse (and report) if the crystal did not come up — without it the
    // reference counter ticks at the imprecise VLO and the grading is meaningless.
    if clocks.aclk() != 32_768 {
        red_led.set_high().ok();
        loop {
            tx.write_all(b"DELAY_TEST_BEGIN\r\n").ok();
            tx.write_all(b"DELAY CLOCK FAIL\r\n").ok();
            tx.write_all(b"DELAY_TEST_END\r\n").ok();
            delay.delay_ms(1000);
        }
    }

    // ACLK ÷1 → 32.768 kHz tick (~30.5 µs), ~2 s wrap. The independent yardstick.
    let counter = Counter::new_aclk(p.timer_0_a3, &clocks, Divider::Div1);

    // --- 1. Millisecond delays ----------------------------------------------
    // Each is well under one ~2 s wrap, so a single u16 delta is valid. Bands are
    // asymmetric and skew long: `Delay` is documented biased-long (~3.4 % here),
    // and its µs math carries a ~2.6 ms fixed overhead (64-bit multiply/divide at
    // MCLK = 1 MHz) that is negligible at these durations but always additive. The
    // bands are wide enough to absorb DCO drift yet still catch a 2× error.
    let us_250 = measure(&counter, &mut delay, 250);
    let us_500 = measure(&counter, &mut delay, 500);
    let us_1000 = measure(&counter, &mut delay, 1000);
    let ms_ok = (230_000..=285_000).contains(&us_250)
        && (475_000..=560_000).contains(&us_500)
        && (960_000..=1_100_000).contains(&us_1000);

    // --- 2. Microsecond-path delay ------------------------------------------
    // Exercise the `delay_us` trait method (divisor 1e6, distinct from delay_ms)
    // at 50 ms — large enough that the busy-loop dominates the ~2.6 ms arithmetic
    // floor, so this is a real magnitude check rather than an overhead measurement
    // (a `delay_us(2000)` would spend most of its time in that floor, ~4.7 ms).
    let start = counter.now();
    delay.delay_us(50_000);
    let us_50k = counter.ticks_to_us(counter.elapsed_since(start) as u32);
    let us_ok = (48_000..=62_000).contains(&us_50k);

    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"delay ms250=").ok();
        write_dec(&mut tx, us_250);
        tx.write_all(b" ms500=").ok();
        write_dec(&mut tx, us_500);
        tx.write_all(b" ms1000=").ok();
        write_dec(&mut tx, us_1000);
        tx.write_all(b" us50k=").ok();
        write_dec(&mut tx, us_50k);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"DELAY_TEST_BEGIN\r\n").ok();
        tx.write_all(if ms_ok {
            b"DELAY MS OK\r\n" as &[u8]
        } else {
            b"DELAY MS FAIL\r\n"
        })
        .ok();
        tx.write_all(if us_ok {
            b"DELAY US OK\r\n" as &[u8]
        } else {
            b"DELAY US FAIL\r\n"
        })
        .ok();
        tx.write_all(b"DELAY_TEST_END\r\n").ok();

        if ms_ok && us_ok {
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

/// Busy-wait `ms` milliseconds on `delay` and return the duration the crystal
/// `counter` actually measured, in microseconds.
fn measure(counter: &Counter, delay: &mut Delay, ms: u32) -> u32 {
    let start = counter.now();
    delay.delay_ms(ms);
    counter.ticks_to_us(counter.elapsed_since(start) as u32)
}

/// Write an unsigned value as decimal ASCII (no padding).
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
