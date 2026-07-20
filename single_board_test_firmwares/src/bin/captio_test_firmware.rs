#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! Capacitive Touch I/O (CAPTIO) fixture: each instance turns a port pad into
//! a relaxation oscillator and counts it on its silicon-paired internal timer
//! (CAPTIO0 → TA2, CAPTIO1 → TA3, both via `INCLK`). Reports a framed
//! pass/fail verdict over the UART backchannel (eUSCI_A0, 9600 8N1), driven
//! by the host-side `captio_tests` runner. **No wiring at all** — the
//! oscillator's stimulus is the pad's own parasitic capacitance, so a bare
//! pad oscillates in the ~3 MHz ballpark all by itself (HW-measured; faster
//! the less copper hangs off the pad).
//!
//! ```text
//! cargo +nightly build --bin captio_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/captio_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **`CAPTIO OSC`** — CAPTIO0 routed to the free header pad P3.0
//!    oscillates: a 2 ms gate against the SMCLK [`Counter`] lands in a
//!    generous 50 kHz–16 MHz plausibility window (an unrouted/broken
//!    oscillator reads 0; a wrapped count reads as a refusal, not a number).
//! 2. **`CAPTIO STATE`** — the read-only `CAPTIO` bit is the *live*
//!    oscillation: 64 polled samples see both levels.
//! 3. **`CAPTIO OFF`** — disabled, the signal toward the timer is 0 (the
//!    datasheet's words): the count is frozen across a full gate and the
//!    state bit reads 0.
//! 4. **`CAPTIO PAIR`** — with only CAPTIO0 enabled, **TA3** stays at zero
//!    across a gate: instance 0's signal must not reach instance 1's timer
//!    (a crossed `INCLK` pairing fails here).
//! 5. **`CAPTIO OSC1`** — CAPTIO1 on P1.3 oscillates in-window **while**
//!    CAPTIO0 keeps counting in the same gate window: the instances are
//!    independent and concurrent.
//! 6. **`CAPTIO SCAN`** — CAPTIO0 re-routed across P3.0→P3.3 (the typed-pin
//!    scanning idiom), every pad in-window; then P3.0 again through
//!    `route_raw`, which must agree with the typed measurement within ±10%
//!    (same pad, same capacitance — the two paths encode the same word).
//! 7. **`CAPTIO WAKE`** — the oscillator is self-clocked, so it keeps
//!    counting with the CPU asleep: the TA2 count-overflow interrupt (armed
//!    with GIE off, ≤ ~65 ms away) wakes the part from **LPM0**, the ISR
//!    sees `TAxIV` = 0x0E exactly once and disarms in-handler
//!    (`captio::isr_disable_overflow_interrupt` — the overflow re-latches
//!    every 65536 oscillations), and the tally holds at one across a
//!    further 100 ms.
//! 8. **`CAPTIO STOP`** — everything disabled: counts frozen and the ISR
//!    tally unmoved across 50 ms.
//!
//! All verdicts are computed **once** at startup; the loop re-emits the
//! fixed verdict burst once per second, GREEN toggling as a heartbeat,
//! steady RED on failure.
//!
//! # Framed output for the host runner
//!
//! ```text
//! captio p30=1450210 p31=1387500 p32=1512000 p33=1420800 raw30=1449100 f1=1305600 wakes=1
//! CAPTIO_TEST_BEGIN
//! CAPTIO OSC OK
//! ...
//! CAPTIO_TEST_END
//! ```

use core::cell::Cell;

use critical_section::Mutex;
use hal::captio::{self, TouchSense};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::pac;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::timer::{Counter, Divider};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Oscillation-frequency plausibility window for a bare pad, Hz. Wide on
/// purpose: the exact rate depends on trace + header capacitance, but a dead
/// pad (0) or an aliased/implausible reading can't land inside it.
/// HW-measured 2026-07-11: P3.0 ≈ 3.50 MHz and P1.3 ≈ 3.16 MHz (pads with
/// board traces); P3.1–P3.3 run *faster* still (trace-less pads, less
/// capacitance) — past the 6.55 MHz ceiling of the first attempt's 10 ms
/// gate, which is why the gate below is 2 ms.
const HZ_MIN: u32 = 50_000;
const HZ_MAX: u32 = 16_000_000;

/// 2 ms gate at the counter's 1 MHz tick: resolves up to 32.7 MHz without
/// wrapping the 16-bit oscillation count (a 10 ms gate wraps on the fastest
/// bare pads — measured, not theoretical), at a 500 Hz resolution that is
/// still four orders of magnitude below the signal.
const GATE_TICKS: u16 = 2_000;

/// ISR tallies: TA2 overflow wakes (`TAxIV` = 0x0E) and any other IV value.
static WAKES: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static OTHER_IV: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// TA2's shared vector: demux (the `TAxIV` read auto-clears the served
/// flag), tally, and disarm right here — the overflow re-latches every
/// 65536 oscillations (~45 ms at a real pad's rate), so a keep-armed
/// one-shot would re-fire forever. `wake_cpu` lets `main` resume after
/// `enter_lpm0()`.
#[msp430_rt::interrupt(wake_cpu)]
fn TIMER2_A1() {
    let iv = captio::read_timer_iv::<pac::CapacitiveTouchIo0>();
    critical_section::with(|cs| {
        if iv == captio::IV_OVERFLOW {
            let c = WAKES.borrow(cs);
            c.set(c.get().wrapping_add(1));
        } else {
            let c = OTHER_IV.borrow(cs);
            c.set(c.get().wrapping_add(1));
        }
    });
    captio::isr_disable_overflow_interrupt::<pac::CapacitiveTouchIo0>();
}

fn wakes() -> u16 {
    critical_section::with(|cs| WAKES.borrow(cs).get())
}
fn other_iv() -> u16 {
    critical_section::with(|cs| OTHER_IV.borrow(cs).get())
}

/// Whether a measured frequency is a plausible pad oscillation. `None`
/// (count wrapped during the gate) is implausible by definition.
fn in_window(hz: Option<u32>) -> bool {
    matches!(hz, Some(f) if (HZ_MIN..=HZ_MAX).contains(&f))
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Default profile: MCLK = 1 MHz, SMCLK = 8 MHz (the gate yardstick).
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, heartbeat
    let mut red_led = port4.pin6.into_output(); // LED1, failure flag

    // The pads under test: bare BoosterPack-header pins, floating inputs
    // (the module drives the pad through its own pulls; a GPIO driver or
    // REN pull would fight the oscillator).
    let pad30 = port3.pin0.into_floating_input();
    let pad31 = port3.pin1.into_floating_input();
    let pad32 = port3.pin2.into_floating_input();
    let pad33 = port3.pin3.into_floating_input();
    let pad13 = port1.pin3.into_floating_input();

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 Capacitive Touch I/O self-check (no wiring)\r\n")
        .ok();

    // The gate yardstick: SMCLK/8 = 1 MHz tick (1 us), 65.5 ms wrap — every
    // gate below stays well under one wrap.
    let counter = Counter::new_smclk(p.timer_0_a3, &clocks, Divider::Div8);

    // Each CAPTIO instance is bonded to its silicon-paired internal timer.
    let mut sense0 = TouchSense::new(p.capacitive_touch_io_0, p.timer_2_a2);
    let mut sense1 = TouchSense::new(p.capacitive_touch_io_1, p.timer_3_a2);

    // --- 1: a bare pad oscillates --------------------------------------------
    sense0.route_pin(&pad30);
    let f30 = sense0.measure_hz(&counter, GATE_TICKS);
    let osc_ok = in_window(f30);

    // --- 2: the CAPTIO state bit is the live oscillation ---------------------
    let (mut saw_high, mut saw_low) = (false, false);
    for _ in 0..64 {
        if sense0.state() {
            saw_high = true;
        } else {
            saw_low = true;
        }
        // Skew the sample phase against the MHz-class oscillation.
        delay.delay_us(7);
    }
    let state_ok = saw_high && saw_low;

    // --- 3: disabled means frozen --------------------------------------------
    sense0.disable();
    sense0.restart_count();
    let t0 = counter.now();
    while counter.elapsed_since(t0) < GATE_TICKS {}
    let off_ok = sense0.count() == 0 && !sense0.state();

    // --- 4: instance 0's signal must not reach instance 1's timer ------------
    sense0.route_pin(&pad30); // oscillating again; CAPTIO1 still disabled
    sense1.restart_count();
    let t0 = counter.now();
    while counter.elapsed_since(t0) < GATE_TICKS {}
    let pair_ok = sense1.count() == 0;

    // --- 5: both instances, independent and concurrent -----------------------
    sense1.route_pin(&pad13);
    let f1 = sense1.measure_hz(&counter, GATE_TICKS);
    sense0.restart_count();
    sense1.restart_count();
    let t0 = counter.now();
    while counter.elapsed_since(t0) < GATE_TICKS {}
    // Two reads each: the counters are running async to MCLK, and while a
    // single torn read returning 0 is already astronomically unlikely, the
    // max of two closes it entirely.
    let c0 = sense0.count().max(sense0.count());
    let c1 = sense1.count().max(sense1.count());
    let osc1_ok = in_window(f1) && c0 > 0 && c1 > 0;
    sense1.disable();

    // --- 6: scanning — one instance across four pads, plus the raw path ------
    let f31 = {
        sense0.route_pin(&pad31);
        sense0.measure_hz(&counter, GATE_TICKS)
    };
    let f32_ = {
        sense0.route_pin(&pad32);
        sense0.measure_hz(&counter, GATE_TICKS)
    };
    let f33 = {
        sense0.route_pin(&pad33);
        sense0.measure_hz(&counter, GATE_TICKS)
    };
    // The raw path must land on the same pad the typed path measured: same
    // capacitance, same frequency (gate-to-gate jitter is way under 10%).
    let raw30 = match sense0.route_raw(captio::Port::P3, 0) {
        Ok(()) => sense0.measure_hz(&counter, GATE_TICKS),
        Err(_) => None,
    };
    let raw_agrees = match (f30, raw30) {
        (Some(a), Some(b)) => {
            let diff = if a > b { a - b } else { b - a };
            diff <= a / 10
        }
        _ => false,
    };
    let scan_ok = in_window(f31)
        && in_window(f32_)
        && in_window(f33)
        && sense0.route_raw(captio::Port::P3, 9).is_err() // pin field is 3 bits
        && raw_agrees;

    // --- 7: the self-clocked count wakes LPM0 --------------------------------
    // GIE is off (never enabled in this fixture), so the arm -> sleep window
    // is race-free: an overflow landing early latches TAIFG and `enter_lpm0`
    // (which sets GIE atomically with sleeping) delivers it. P3.0 is still
    // routed and oscillating; the wrap is at most ~65 ms away.
    sense0.enable_overflow_interrupt();
    hal::power::enter_lpm0();
    // The ISR disarmed in-handler; across a further 100 ms (two-plus wrap
    // periods) the tally must hold at exactly one.
    delay.delay_ms(100);
    let wake_ok = wakes() == 1 && other_iv() == 0;

    // --- 8: everything off stays silent --------------------------------------
    sense0.disable();
    sense0.restart_count();
    let (w0, o0) = (wakes(), other_iv());
    delay.delay_ms(50);
    let stop_ok = sense0.count() == 0 && wakes() == w0 && other_iv() == o0;

    let all_ok =
        osc_ok && state_ok && off_ok && pair_ok && osc1_ok && scan_ok && wake_ok && stop_ok;

    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"captio p30=").ok();
        write_dec(&mut tx, f30.unwrap_or(0));
        tx.write_all(b" p31=").ok();
        write_dec(&mut tx, f31.unwrap_or(0));
        tx.write_all(b" p32=").ok();
        write_dec(&mut tx, f32_.unwrap_or(0));
        tx.write_all(b" p33=").ok();
        write_dec(&mut tx, f33.unwrap_or(0));
        tx.write_all(b" raw30=").ok();
        write_dec(&mut tx, raw30.unwrap_or(0));
        tx.write_all(b" f1=").ok();
        write_dec(&mut tx, f1.unwrap_or(0));
        tx.write_all(b" wakes=").ok();
        write_dec(&mut tx, wakes() as u32);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"CAPTIO_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"CAPTIO OSC", osc_ok);
        verdict(&mut tx, b"CAPTIO STATE", state_ok);
        verdict(&mut tx, b"CAPTIO OFF", off_ok);
        verdict(&mut tx, b"CAPTIO PAIR", pair_ok);
        verdict(&mut tx, b"CAPTIO OSC1", osc1_ok);
        verdict(&mut tx, b"CAPTIO SCAN", scan_ok);
        verdict(&mut tx, b"CAPTIO WAKE", wake_ok);
        verdict(&mut tx, b"CAPTIO STOP", stop_ok);
        tx.write_all(b"CAPTIO_TEST_END\r\n").ok();

        if all_ok {
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
