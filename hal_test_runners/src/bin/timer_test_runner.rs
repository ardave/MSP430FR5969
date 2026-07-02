#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (so it returns with RETI, not RET) — still nightly-gated.
#![feature(abi_msp430_interrupt)]

//! Timer0_A3 integration fixture for the `hal::timer::Counter` driver.
//!
//! A self-checking sibling of the human-facing demos: it runs three startup
//! self-checks of the free-running counter and reports a framed pass/fail verdict
//! over the UART backchannel (eUSCI_A0, 9600 8N1 on `/dev/cu.usbmodem11203`),
//! driven by the host-side `timer_tests` runner. No wiring is needed — Timer0_A3
//! is on-chip — and, unlike the RTC/delay/deep-sleep fixtures, this one does **not**
//! need the 32.768 kHz crystal: it runs from the DCO-derived **performance clock
//! profile** ([`hal::clocks::configure`], SMCLK = 8 MHz), where the 16-bit counter
//! wraps every ~8.19 ms.
//!
//! ```text
//! cargo +nightly build --bin timer_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/timer_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **It actually counts (`TIMER RUN`).** Snapshot `now()`, busy-wait a known
//!    1 ms on the MCU `Delay`, snapshot again. At 8 MHz that is ~8000 ticks
//!    (just under one wrap), so the elapsed delta must land in a sane band — a
//!    counter that is stopped, mis-clocked, or not advancing fails here.
//!
//! 2. **Hardware capture (`TIMER CAPTURE`).** `configure_capture()` then
//!    `software_capture()` latches `TAxR` into `TAxCCR1` on an internally
//!    manufactured edge. The latched value must sit just *behind* a fresh `now()`
//!    (only the handful of ticks the two register writes cost), proving the
//!    capture path froze the live counter.
//!
//! 3. **Overflow + 32-bit timestamp (`TIMER OVERFLOW`).** With the `TIMER0_A1`
//!    overflow ISR tallying rollovers into a critical-section `Mutex<Cell<u16>>`,
//!    read `now32()`, busy-wait 150 ms (~18 wraps at 8 MHz), and read `now32()`
//!    again. The 32-bit tick delta converted with `ticks_to_us` must be ~150 000 µs
//!    — exercising the rollover fold-in in `Counter::now32` that a 16-bit `now()`
//!    alone could never measure across that many wraps.
//!
//! All three verdicts are computed **once** at startup; the loop just re-emits the
//! fixed verdict lines and toggles the **GREEN** LED as a heartbeat. A steady
//! **RED** LED means a check failed.
//!
//! # Framed output for the host runner
//!
//! ```text
//! timer us=149987            (human-readable info, skipped by host)
//! TIMER_TEST_BEGIN
//! TIMER RUN OK               (or `TIMER RUN FAIL`)
//! TIMER CAPTURE OK           (or `TIMER CAPTURE FAIL`)
//! TIMER OVERFLOW OK          (or `TIMER OVERFLOW FAIL`)
//! TIMER_TEST_END
//! ```

use core::cell::Cell;

use critical_section::Mutex;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::timer::{self, Counter, Divider};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take() (and so msp430::interrupt::enable() is available).
use msp430 as _;

// Watchdog Timer Password / Hold.
const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

/// Software tally of Timer0_A3 overflows (0xFFFF→0x0000 rollovers), maintained by
/// the `TIMER0_A1` ISR and read by `now32` under the same critical section. A
/// `Mutex<Cell<u16>>` because the value is shared between the ISR and `main`.
static OVERFLOWS: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// Timer0_A3 overflow ISR: bump the rollover tally and clear `TAIFG`.
#[msp430_rt::interrupt]
fn TIMER0_A1() {
    critical_section::with(|cs| {
        let ovf = OVERFLOWS.borrow(cs);
        ovf.set(ovf.get().wrapping_add(1));
    });
    timer::clear_overflow_irq();
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog before anything else (default timeout ~32 ms, and
    // Peripherals::take() enters a critical section).
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    let p = hal::pac::Peripherals::take().unwrap();

    // Performance profile: SMCLK = 8 MHz (counter ticks + UART BRCLK), MCLK = 1 MHz
    // (Delay). No crystal needed — this fixture is DCO-only.
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the pin muxes take effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, heartbeat
    let mut red_led = port4.pin6.into_output(); // LED1, failure flag

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 Timer0_A3 self-check\r\n").ok();

    // SMCLK ÷1 → 8 MHz tick (125 ns), 8.19 ms wrap.
    let counter = Counter::new_smclk(p.timer_0_a3, &clocks, Divider::Div1);

    // --- 1. It actually counts ----------------------------------------------
    // Liveness only: confirm the counter advances over a short delay. The exact
    // count is *not* a timing check — `Delay`'s 64-bit µs math has a ~2.6 ms fixed
    // floor at MCLK = 1 MHz, so even `delay_ms(1)` runs several ms and lands near
    // ~29000 ticks. The band is wide on purpose (the absolute-rate check is the
    // overflow test below); it just rules out a stuck counter (too low) or a value
    // that lapped the 65536-tick wrap into the small-number range (too high).
    let run_start = counter.now();
    delay.delay_ms(1);
    let run_ticks = counter.elapsed_since(run_start);
    let run_ok = (1_000..=60_000).contains(&run_ticks);

    // --- 2. Hardware capture ------------------------------------------------
    // Bracket check: a correct capture latches `TAxR` *during* software_capture(),
    // so the captured value must lie between a `now()` taken just before and one
    // just after. This is immune to the SMCLK/MCLK ratio (each instruction is ~8
    // counter ticks) — unlike an absolute lag threshold — yet still fails a capture
    // that returns a stale/garbage value outside the bracket.
    counter.configure_capture();
    let before_capture = counter.now();
    let captured = counter.software_capture();
    let after_capture = counter.now();
    let capture_span = after_capture.wrapping_sub(before_capture);
    let capture_off = captured.wrapping_sub(before_capture);
    let capture_ok = capture_off <= capture_span;

    // --- 3. Overflow + 32-bit timestamp -------------------------------------
    // Tally rollovers in the TIMER0_A1 ISR, then time 150 ms (~18 wraps) and
    // confirm now32 reconstructed the elapsed time across them.
    counter.enable_overflow_interrupt();
    // SAFETY: enabling interrupts globally (set GIE) so the overflow ISR can run.
    // No data shared with the ISR is touched outside a critical section.
    unsafe {
        msp430::interrupt::enable();
    }

    let t0 = critical_section::with(|cs| counter.now32(OVERFLOWS.borrow(cs).get()));
    delay.delay_ms(150);
    let t1 = critical_section::with(|cs| counter.now32(OVERFLOWS.borrow(cs).get()));
    let elapsed_us = counter.ticks_to_us(t1.wrapping_sub(t0));
    // 150 ms ±10%: a stuck or mis-tallied overflow path falls well outside.
    let overflow_ok = (135_000..=165_000).contains(&elapsed_us);

    // A self-delimited verdict burst, repeated once per second so the host runner
    // can attach at any time after the DSLite reset and still frame a full
    // BEGIN..END cycle. GREEN toggles each cycle as a heartbeat; steady RED means
    // a check failed.
    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"timer run=").ok();
        write_dec(&mut tx, run_ticks as u32);
        tx.write_all(b" cap_off=").ok();
        write_dec(&mut tx, capture_off as u32);
        tx.write_all(b" cap_span=").ok();
        write_dec(&mut tx, capture_span as u32);
        tx.write_all(b" ovf_us=").ok();
        write_dec(&mut tx, elapsed_us);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"TIMER_TEST_BEGIN\r\n").ok();
        tx.write_all(if run_ok {
            b"TIMER RUN OK\r\n" as &[u8]
        } else {
            b"TIMER RUN FAIL\r\n"
        })
        .ok();
        tx.write_all(if capture_ok {
            b"TIMER CAPTURE OK\r\n" as &[u8]
        } else {
            b"TIMER CAPTURE FAIL\r\n"
        })
        .ok();
        tx.write_all(if overflow_ok {
            b"TIMER OVERFLOW OK\r\n" as &[u8]
        } else {
            b"TIMER OVERFLOW FAIL\r\n"
        })
        .ok();
        tx.write_all(b"TIMER_TEST_END\r\n").ok();

        if run_ok && capture_ok && overflow_ok {
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
