#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! LPM3 deep-sleep wake fixture: `Counter::schedule_wake_in` (CCR0 compare) +
//! `hal::power::enter_lpm3`.
//!
//! Validates that the part can be put into deep sleep — CPU, MCLK, SMCLK, and the
//! DCO all gated — and woken on schedule by a timer running off the 32.768 kHz
//! crystal (ACLK keeps ticking in LPM3; see the `power` module table). An
//! **ACLK-sourced [`Counter`]** both schedules the wake (a CCR0 compare that fires
//! the `TIMER0_A0` interrupt) and, by snapshotting `now()` either side of the
//! sleep, measures how long the part was actually out. Reports a framed pass/fail
//! verdict over the UART backchannel (eUSCI_A0, 9600 8N1), driven by the host-side
//! `deep_sleep_test_orchestrator` runner. No wiring needed beyond the LaunchPad.
//!
//! ```text
//! cargo +nightly build --bin deep_sleep_test_firmware
//! DSLite load ... -f target/msp430-none-elf/debug/deep_sleep_test_firmware
//! ```
//!
//! # What it checks
//!
//! Four wake cycles, each scheduling a ~0.5 s wake (16384 ticks at 32.768 kHz),
//! entering LPM3, and measuring the slept interval with the crystal counter:
//!
//! 1. **It wakes at all (`SLEEP WAKE`).** A compare wake that never fired would
//!    leave the part asleep forever and this fixture would never emit a burst — so
//!    *reaching* the verdict already proves every `enter_lpm3()` returned (the same
//!    "doesn't hang" logic the SPI loopback demo relies on).
//!
//! 2. **It wakes on time (`SLEEP TIMING`).** Each measured interval must be within
//!    ±10 % of the scheduled 16384 ticks — a wake that fired early/late, or a
//!    counter that stopped in LPM3, falls outside.
//!
//! Both verdicts are computed **once** at startup (across the four cycles); the
//! loop then re-emits the fixed verdict lines and toggles the **GREEN** LED as a
//! heartbeat. A steady **RED** LED means a check failed.
//!
//! # Hardware requirement: the 32.768 kHz crystal
//!
//! LPM3 gates the DCO and SMCLK; only ACLK survives, and only as the LFXT crystal
//! (VLO is also gated when sourced for the RTC/sleep path). Without the crystal
//! there is no clock to wake on, so — as with the RTC fixture — this refuses
//! (lights **RED**, emits a `SLEEP CLOCK FAIL` burst) rather than risk sleeping
//! forever.
//!
//! # Framed output for the host runner
//!
//! ```text
//! sleep last_ticks=16390             (human-readable info, skipped by host)
//! SLEEP_TEST_BEGIN
//! SLEEP WAKE OK                      (always OK if reached — see above)
//! SLEEP TIMING OK                    (or `SLEEP TIMING FAIL`)
//! SLEEP_TEST_END
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
// pac's Peripherals::take().
use msp430 as _;

/// Set true by the TIMER0_A0 ISR each time the CCR0 compare fires, so an
/// active-mode probe can tell *when* the wake happened without sleeping.
static WOKE: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

/// Scheduled wake interval, in ACLK ticks: 16384 / 32768 Hz = 0.5 s.
const WAKE_TICKS: u16 = 16_384;

/// Number of sleep/wake cycles to run.
const CYCLES: usize = 4;

/// CCR0 compare-wake ISR. `wake_cpu` clears the low-power bits in the stacked SR
/// so the CPU resumes after `enter_lpm3()` (rather than sleeping again); the body
/// disarms the one-shot so the free-running counter's next wrap does not re-fire it.
#[msp430_rt::interrupt(wake_cpu)]
fn TIMER0_A0() {
    critical_section::with(|cs| WOKE.borrow(cs).set(true));
    timer::clear_wake_irq();
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Low-power profile: ACLK on the 32.768 kHz LFXT crystal, which keeps running
    // in LPM3 and clocks both the wake compare and the elapsed measurement.
    let clocks = hal::clocks::configure_low_power(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 1 MHz (active mode only — SMCLK is
    // gated in LPM3, so we transmit after waking, never during sleep).
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, heartbeat
    let mut red_led = port4.pin6.into_output(); // LED1, failure flag

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 LPM3 wake self-check\r\n").ok();

    // Refuse (and report) if the crystal did not come up: no surviving clock in
    // LPM3 means nothing to wake on, so sleeping could hang the part forever.
    if clocks.aclk() != 32_768 {
        red_led.set_high().ok();
        loop {
            tx.write_all(b"SLEEP_TEST_BEGIN\r\n").ok();
            tx.write_all(b"SLEEP CLOCK FAIL\r\n").ok();
            tx.write_all(b"SLEEP_TEST_END\r\n").ok();
            delay.delay_ms(1000);
        }
    }

    // ACLK ÷1 → 32.768 kHz tick; runs through LPM3.
    let counter = Counter::new_aclk(p.timer_0_a3, &clocks, Divider::Div1);

    // --- Active-mode counter sanity (diagnostic) ----------------------------
    // Confirm the ACLK counter advances at the expected rate while awake, before
    // we ask it to run through sleep: 100 ms should be ~3277 ticks at 32.768 kHz.
    // This separates "counter dead/mis-clocked" from "LPM3 wake misbehaves".
    let active_t0 = counter.now();
    delay.delay_ms(100);
    let active_ticks = counter.elapsed_since(active_t0);

    // --- Active-mode compare probe (diagnostic) -----------------------------
    // Arm the SAME CCR0 compare, enable GIE, but stay AWAKE and spin until the
    // ISR sets WOKE (bounded). If this reports ~16384 the compare itself is fine
    // and the bug is LPM3-specific; if it reports ~0 the compare/arming is wrong
    // independent of sleep.
    critical_section::with(|cs| WOKE.borrow(cs).set(false));
    let probe_t0 = counter.now();
    counter.schedule_wake_in(WAKE_TICKS);
    // SAFETY: enable GIE so the armed CCR0 compare can fire the ISR.
    unsafe {
        msp430::interrupt::enable();
    }
    let active_wake = loop {
        let woke = critical_section::with(|cs| WOKE.borrow(cs).get());
        let elapsed = counter.elapsed_since(probe_t0);
        if woke || elapsed > 40_000 {
            break elapsed;
        }
    };

    // --- Sleep/wake cycles --------------------------------------------------
    // Each cycle arms a CCR0 compare WAKE_TICKS ahead, drops to LPM3, and on wake
    // measures the interval the counter advanced. Reaching the end at all proves
    // every enter_lpm3() returned; the per-cycle band proves it returned on time.
    let mut timing_ok = true;
    let mut cycles = [0u16; CYCLES];
    for slot in cycles.iter_mut() {
        let t0 = counter.now();
        counter.schedule_wake_in(WAKE_TICKS);
        // Atomically set GIE + LPM3 bits; returns once the TIMER0_A0 ISR (which
        // cleared the low-power bits via wake_cpu) lets execution resume here.
        hal::power::enter_lpm3();
        let elapsed = counter.elapsed_since(t0);
        *slot = elapsed;
        // ±10 % of the scheduled interval.
        if !(14_745..=18_022).contains(&elapsed) {
            timing_ok = false;
        }
    }

    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"sleep active=").ok();
        write_dec(&mut tx, active_ticks as u32);
        tx.write_all(b" aw=").ok();
        write_dec(&mut tx, active_wake as u32);
        tx.write_all(b" cycles=").ok();
        for (i, c) in cycles.iter().enumerate() {
            if i > 0 {
                tx.write_all(b",").ok();
            }
            write_dec(&mut tx, *c as u32);
        }
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"SLEEP_TEST_BEGIN\r\n").ok();
        // Reaching here proves every enter_lpm3() returned, so WAKE always passes.
        tx.write_all(b"SLEEP WAKE OK\r\n").ok();
        tx.write_all(if timing_ok {
            b"SLEEP TIMING OK\r\n" as &[u8]
        } else {
            b"SLEEP TIMING FAIL\r\n"
        })
        .ok();
        tx.write_all(b"SLEEP_TEST_END\r\n").ok();

        if timing_ok {
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
