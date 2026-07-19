#![no_std]
#![no_main]

//! Timer_A PWM fixture (`hal::pwm::PwmTimerA` on **TA1 + TA0**): the same
//! up-mode/`OUTMOD` machinery as the Timer_B0 driver, on the blocks that
//! free TB0 — and with it the P1.4–P1.7 pins — from the PWM-vs-eUSCI_B0
//! conflict. Reports a framed pass/fail verdict over the UART backchannel
//! (eUSCI_A0, 9600 8N1), driven by the host-side `ta_pwm_tests` runner.
//!
//! **No wiring at all** — the observation trick: `P1IN` reflects the pad
//! level even while the secondary-function output unit drives it, so the
//! CPU can *sample its own PWM* right out of the input register. Duty is
//! the fraction of high samples (the ~15 µs sampling cadence walks the
//! 1000 µs waveform, so thousands of samples average to the duty), rails
//! are all-samples-one-level, and frequency comes from timestamping rising
//! transitions with the SMCLK [`Counter`]. No jumper, no second peripheral,
//! and an inverted, stuck, or mis-mapped output cannot fake any of it.
//!
//! ```text
//! cargo +nightly build --bin ta_pwm_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/ta_pwm_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **`TA PWM FREQ`** — 32 rising edges on TA1.1 (P1.2) timestamped by
//!    the TA0 [`Counter`]: the measured period must be within 2% of
//!    [`PwmTimerA::frequency`]'s readback (both ride SMCLK, so this is
//!    pure divider/period arithmetic made observable).
//! 2. **`TA PWM DUTY25`** — TA1.1 at 25%: high fraction 200‰..300‰.
//! 3. **`TA PWM DUTY75`** — TA1.2 (P1.3) at 75%, sampled in the same pass:
//!    700‰..800‰. Asymmetric points, so a transposed channel↔pin map or an
//!    inverted output fails loudly.
//! 4. **`TA PWM RAILS`** — TA1.1 parked at 0% reads all-low across 500
//!    samples, at 100% all-high: the `OUTMOD = 0` clean-rail contract, no
//!    one-tick glitches (a glitching park would eventually be caught
//!    mid-sample).
//! 5. **`TA PWM INDEP`** — after all that TA1.1 churn, TA1.2 still reads
//!    70–80%: channels touch only their own `CCRn`/`CCTLn`.
//! 6. **`TA PWM TA0`** — the [`Counter`] is freed and the *same block*
//!    rebuilt as a second PWM generator (instance-genericity on silicon):
//!    TA0.1 = P1.0 at 2 kHz / 50% reads 400‰..600‰. That pin is the green
//!    LED, which therefore glows at half brightness — the fixture's alive
//!    indicator (there is deliberately no green heartbeat; RED = failure).
//!
//! All verdicts are computed **once** at startup; the loop re-emits the
//! fixed verdict burst once per second. TA0.1 keeps dimly lighting LED2,
//! steady RED on failure.
//!
//! # Framed output for the host runner
//!
//! ```text
//! ta pwm period_us=1000 d25=249 d75=751 ta0=500   (info, skipped by host)
//! TA_PWM_TEST_BEGIN
//! TA PWM FREQ OK
//! TA PWM DUTY25 OK
//! TA PWM DUTY75 OK
//! TA PWM RAILS OK
//! TA PWM INDEP OK
//! TA PWM TA0 OK
//! TA_PWM_TEST_END
//! ```

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_hal::pwm::SetDutyCycle as _;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::pwm::PwmTimerA;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::timer::{Counter, Divider};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// P1IN — the port-1 input register (always reflects the pad, whatever
/// drives it). Raw address per the device header; read-only byte access.
fn p1in() -> u8 {
    unsafe { (0x0200 as *const u8).read_volatile() }
}

/// Fraction of `n` samples in which `mask` reads high, in permille. The
/// sampling loop is unsynchronized to the PWM (its ~15 µs cadence at 1 MHz
/// MCLK walks a 1000 µs waveform), so thousands of samples converge on the
/// duty cycle.
fn high_permille(mask: u8, n: u32) -> u32 {
    let mut hi = 0u32;
    for _ in 0..n {
        if p1in() & mask != 0 {
            hi += 1;
        }
    }
    hi * 1000 / n
}

/// Measure the mean PWM period on `mask` in µs: sync to a rising
/// transition, then time `periods` more with the counter. Bounded (~60 ms
/// per wait) — a dead output returns `None` instead of hanging the board.
fn measure_period_us(counter: &Counter, mask: u8, periods: u32) -> Option<u32> {
    let bound = counter.now();
    let mut prev = p1in() & mask != 0;
    // Sync edge.
    loop {
        let cur = p1in() & mask != 0;
        if cur && !prev {
            break;
        }
        prev = cur;
        if counter.elapsed_since(bound) > 60_000 {
            return None;
        }
    }
    let t0 = counter.now();
    let mut prev = true;
    let mut count = 0u32;
    loop {
        let cur = p1in() & mask != 0;
        if cur && !prev {
            count += 1;
            if count == periods {
                let ticks = counter.elapsed_since(t0);
                return Some(counter.ticks_to_us(ticks as u32) / periods);
            }
        }
        prev = cur;
        if counter.elapsed_since(t0) > 60_000 {
            return None;
        }
    }
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Performance profile: SMCLK = 8 MHz (PWM + Counter + UART BRCLK),
    // MCLK = 1 MHz (the sampling loops).
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut red_led = port4.pin6.into_output(); // LED1, failure flag
    // No green heartbeat here: P1.0 *is* TA0.1, the phase-6 output — the
    // dim half-brightness LED2 glow is the alive indicator.

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 Timer_A PWM self-check (no wiring)\r\n")
        .ok();

    // The frequency yardstick: TA0 as a Counter first (SMCLK/8 = 1 MHz tick,
    // 65 ms wrap — each bounded wait stays under it). Freed in phase 6 to
    // become the second PWM instance.
    let counter = Counter::new_smclk(p.timer_0_a3, &clocks, Divider::Div8);

    // TA1 PWM at ~1 kHz: TA1.1 = P1.2, TA1.2 = P1.3 (free pins — the reason
    // TA1 is the natural PWM block).
    let pwm1 = PwmTimerA::new_smclk(p.timer_1_a3, &clocks, 1_000);
    let mut ch1 = pwm1.channel(port1.pin2.into_timer_a_output());
    let mut ch2 = pwm1.channel(port1.pin3.into_timer_a_output());
    ch1.set_duty_cycle_percent(25).ok();
    ch2.set_duty_cycle_percent(75).ok();

    const P12: u8 = 1 << 2;
    const P13: u8 = 1 << 3;
    const P10: u8 = 1 << 0;

    // --- 1: frequency against the TA0 counter --------------------------------
    let expect_us = 1_000_000 / pwm1.frequency().max(1);
    let period_us = measure_period_us(&counter, P12, 32).unwrap_or(0);
    // Both timers ride SMCLK: 2% covers sampling quantization, not clocks.
    let freq_ok =
        period_us >= expect_us - expect_us / 50 && period_us <= expect_us + expect_us / 50;

    // --- 2+3: both duties in one sampling pass --------------------------------
    let d25 = high_permille(P12, 4_000);
    let d75 = high_permille(P13, 4_000);
    let duty25_ok = (200..=300).contains(&d25);
    let duty75_ok = (700..=800).contains(&d75);

    // --- 4: clean rails on TA1.1 ----------------------------------------------
    ch1.set_duty_cycle_fully_off().ok();
    let low_hi = high_permille(P12, 500);
    ch1.set_duty_cycle_fully_on().ok();
    let high_hi = high_permille(P12, 500);
    let rails_ok = low_hi == 0 && high_hi == 1000;

    // --- 5: TA1.2 untouched by all the TA1.1 churn -----------------------------
    let d75_after = high_permille(P13, 4_000);
    let indep_ok = (700..=800).contains(&d75_after);

    // --- 6: the freed TA0 as a second generator (instance-genericity) ----------
    let ta0 = counter.free();
    let pwm0 = PwmTimerA::new_smclk(ta0, &clocks, 2_000);
    let mut ch0 = pwm0.channel(port1.pin0.into_timer_a_output()); // green LED2
    ch0.set_duty_cycle_percent(50).ok();
    // Let the first period establish itself before sampling.
    delay.delay_ms(2);
    let ta0_permille = high_permille(P10, 4_000);
    let ta0_ok = (400..=600).contains(&ta0_permille);

    let all_ok = freq_ok && duty25_ok && duty75_ok && rails_ok && indep_ok && ta0_ok;

    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"ta pwm period_us=").ok();
        write_dec(&mut tx, period_us);
        tx.write_all(b" d25=").ok();
        write_dec(&mut tx, d25);
        tx.write_all(b" d75=").ok();
        write_dec(&mut tx, d75_after);
        tx.write_all(b" ta0=").ok();
        write_dec(&mut tx, ta0_permille);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"TA_PWM_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"TA PWM FREQ", freq_ok);
        verdict(&mut tx, b"TA PWM DUTY25", duty25_ok);
        verdict(&mut tx, b"TA PWM DUTY75", duty75_ok);
        verdict(&mut tx, b"TA PWM RAILS", rails_ok);
        verdict(&mut tx, b"TA PWM INDEP", indep_ok);
        verdict(&mut tx, b"TA PWM TA0", ta0_ok);
        tx.write_all(b"TA_PWM_TEST_END\r\n").ok();

        if all_ok {
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Emit one `NAME OK` / `NAME FAIL` verdict line.
fn verdict<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], ok: bool) {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" as &[u8] } else { b" FAIL\r\n" })
        .ok();
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
