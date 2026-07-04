#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (so it returns with RETI, not RET) — still nightly-gated.
#![feature(abi_msp430_interrupt)]

//! WDT_A interval-timer integration fixture: `Watchdog::start_interval` +
//! `enable_interval_interrupt` on the `WDT` vector.
//!
//! In interval mode the watchdog's countdown stops being a fuse and becomes a
//! metronome: every 2^N cycles it sets `WDTIFG` and fires the `WDT` vector —
//! **no PUC, ever**. That inversion is exactly what this fixture checks: that
//! the tick arrives at the configured cadence, and that the chip demonstrably
//! does *not* reset while the "watchdog" expires over and over. Reports a
//! framed pass/fail verdict over the UART backchannel (eUSCI_A0, 9600 8N1 on
//! `/dev/cu.usbmodem11203`), driven by the host-side `wdt_interval_tests`
//! runner. No wiring — WDT_A is on-chip.
//!
//! ```text
//! cargo +nightly build --bin wdt_interval_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/wdt_interval_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **Tick cadence (`WDT INTERVAL`).** `Cycles8192K` from SMCLK = 8 MHz is a
//!    ~1.05 s tick; the `WDT` ISR tallies expirations into a critical-section
//!    `Mutex<Cell<u16>>` while the fixture busy-waits a ~3.2 s window. The
//!    tally delta must be 2–4 — a dead interrupt (0), a mis-set divider, or a
//!    storm all fall outside. Note the ISR body has **no flag work**:
//!    servicing the dedicated `WDT` vector auto-resets `WDTIFG` in hardware.
//!
//! 2. **No reset (`WDT NORESET`).** `SYSRSTIV` (drained at boot via
//!    `hal::sys::ResetReasons`) must not report a watchdog timeout — and the
//!    burst's `wdt ticks=N n=M` info line carries a loop counter that climbs
//!    across bursts, so a latent PUC (which would restart `main` and zero it)
//!    is visible to anyone watching for >30 s even though each verdict is
//!    computed once.
//!
//! All verdicts are computed **once** at startup; the loop re-emits the fixed
//! verdict burst once per second, GREEN toggling as a heartbeat, steady RED on
//! failure. The interval keeps ticking in the loop — `ticks` keeps climbing.
//!
//! # Framed output for the host runner
//!
//! ```text
//! wdt ticks=7 n=4             (human-readable info, skipped by host)
//! WDT_INTERVAL_TEST_BEGIN
//! WDT INTERVAL OK             (or `WDT INTERVAL FAIL`)
//! WDT NORESET OK              (or `WDT NORESET FAIL`)
//! WDT_INTERVAL_TEST_END
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
use hal::sys::{ResetReason, ResetReasons};
use hal::watchdog::{ClockSource, Interval, Watchdog};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take() (and so msp430::interrupt::enable() is available).
use msp430 as _;

/// Interval-expiry tally, bumped by the WDT ISR, read by main under the same
/// critical section.
static TICKS: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// WDT interval ISR: count the tick. Nothing else — the dedicated vector's
/// service already auto-reset `WDTIFG` in hardware.
#[msp430_rt::interrupt]
fn WDT() {
    critical_section::with(|cs| {
        let t = TICKS.borrow(cs);
        t.set(t.get().wrapping_add(1));
    });
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Reset forensics *before* anything else touches the system module: a
    // watchdog-timeout cause here would mean interval mode PUC'd after all.
    let reset_reasons = ResetReasons::drain(&p.sys);
    let noreset_ok = !reset_reasons.contains(ResetReason::WatchdogTimeout);

    // Performance profile: SMCLK = 8 MHz (interval clock + UART BRCLK),
    // MCLK = 1 MHz (Delay).
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

    tx.write_all(b"\r\nMSP430FR5969 WDT_A interval-timer self-check\r\n")
        .ok();

    // Interval metronome: 2^23 cycles of SMCLK = 8 MHz -> ~1.049 s per tick.
    // From this write on the WDT can no longer reset the chip — that's the
    // property under test.
    let mut wdt = Watchdog::new(p.watchdog_timer);
    wdt.start_interval(ClockSource::Smclk, Interval::Cycles8192K);
    wdt.enable_interval_interrupt();

    // SAFETY: enabling interrupts globally (set GIE) so the WDT ISR can run.
    // The only shared state is the critical-section Mutex above.
    unsafe {
        msp430::interrupt::enable();
    }

    // --- 1. Tick cadence -----------------------------------------------------
    // ~3.2 s window at a ~1.049 s period: expect 3 ticks, accept 2–4 (the
    // window start is unsynchronized to the free-running interval phase).
    let t0 = critical_section::with(|cs| TICKS.borrow(cs).get());
    delay.delay_ms(3200);
    let t1 = critical_section::with(|cs| TICKS.borrow(cs).get());
    let window_ticks = t1.wrapping_sub(t0);
    let interval_ok = (2..=4).contains(&window_ticks);

    // A self-delimited verdict burst once per second. `ticks` and the loop
    // counter `n` both keep climbing — a PUC would restart main and zero them,
    // so a long watch (>30 s) also proves interval mode never resets.
    let mut on = false;
    let mut n: u32 = 0;
    loop {
        n = n.wrapping_add(1);
        let ticks = critical_section::with(|cs| TICKS.borrow(cs).get());
        tx.write_all(b"wdt ticks=").ok();
        write_dec(&mut tx, ticks as u32);
        tx.write_all(b" n=").ok();
        write_dec(&mut tx, n);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"WDT_INTERVAL_TEST_BEGIN\r\n").ok();
        tx.write_all(if interval_ok {
            b"WDT INTERVAL OK\r\n" as &[u8]
        } else {
            b"WDT INTERVAL FAIL\r\n"
        })
        .ok();
        tx.write_all(if noreset_ok {
            b"WDT NORESET OK\r\n" as &[u8]
        } else {
            b"WDT NORESET FAIL\r\n"
        })
        .ok();
        tx.write_all(b"WDT_INTERVAL_TEST_END\r\n").ok();

        if interval_ok && noreset_ok {
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
