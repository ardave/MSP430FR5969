#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! Timer_A input-capture integration fixture — hands-free except for one
//! **optional** jumper, driven by the host-side `capture_test_orchestrator` runner.
//!
//! ```text
//! cargo +nightly build --bin capture_test_firmware
//! DSLite load ... -f target/msp430-none-elf/debug/capture_test_firmware
//! ```
//!
//! Capture timer: **TA1** on SMCLK (1 MHz under the low-power clock profile,
//! chosen so ACLK rides the 32.768 kHz LFXT crystal). Three stimuli, two of
//! them wiring-free thanks to TA1's internal capture inputs (SLAS704G
//! Table 6-14):
//!
//! 1. **Software** — the `CCIS` GND→VCC flip manufactures a rising edge in
//!    hardware: the no-wiring smoke test of the latch, timestamp, and `COV`.
//! 2. **ACLK on CCI2B** — the crystal timestamped by DCO-derived ticks; the
//!    span ratio *is* the DCO's frequency error against the crystal.
//! 3. **COUT on CCI1B** — Comp_E output edges made on demand with the
//!    comparator fixture's REFOUT + ladder-tap-step trick.
//!
//! The optional part: a **jumper from P1.4 (TB0.1 PWM out) to P1.2
//! (TA1.CCI1A capture in)** lets the fixture measure a PWM wave it generates
//! itself — frequency and 25%/75% duty through a real pad. The fixture
//! *detects* the jumper (P1.4 driven as GPIO, P1.2 read with a pull-down —
//! the level follows only if wired) and reports the PWM verdicts as `SKIP`
//! when it is absent, so the hands-free portion stays in the default suite.
//!
//! # What it checks
//!
//! - **CAPT SOFT FIRE** — a software-fired capture latches exactly once, and
//!   its timestamp lands inside the `now()` bracket taken around the fire
//!   (the "timestamp frozen at the event" property).
//! - **CAPT SOFT COV** — a second fire before collection latches `COV`, and
//!   clearing it clears it.
//! - **CAPT ACLK SPAN** — a ~16-period ACLK span, bracketed by two serviced
//!   edges with the period count recovered arithmetically, lands within
//!   ±5% of the ideal DCO/crystal ratio (the factory DCO trim is ±3.5%).
//! - **CAPT COUT EDGES** — comparator edges (ladder-tap steps across REFOUT)
//!   timestamped by CCR1: each stamp inside its `now()` bracket, `CCI` level
//!   readback matching the edge direction, both directions exercised.
//! - **CAPT IV DEMUX** — `TA1IV` demux: a comparator-edge capture reads 0x02
//!   exactly once; an ACLK capture reads 0x04 (one-shot: the ISR disarms
//!   itself — see below); a counter overflow reads 0x0E; nothing refires
//!   after disarm and no unexpected codes appear.
//! - **CAPT LPM0 WAKE** — with GIE off, an armed ACLK capture must wake
//!   `enter_lpm0` through the `wake_cpu` ISR (an edge is never more than
//!   30.5 µs away — the fixture sleeps *into* the stimulus).
//! - **CAPT PWM FREQ / CAPT PWM DUTY** — with the jumper: the measured
//!   frequency matches the PWM driver's reported frequency within 1%, and
//!   duty measures 25% and 75% within ±2% (two asymmetric duties, so an
//!   inverted or stuck line cannot pass). Without: `SKIP`.
//!
//! The ACLK-interrupt one-shot is load-bearing, not a convenience: at
//! MCLK = 1 MHz the ISR costs more than one 30.5 µs ACLK period, so a
//! keep-armed handler re-enters back-to-back and starves main forever. The
//! ISR tallies, then calls `capture::isr_disable_interrupt` — the only place
//! the disarm can happen.
//!
//! # Framed output for the host runner
//!
//! ```text
//! capture aclk=1 span=488 n=16 ratio=1001 f=1000 d25=249 d75=749 jumper=1 iv1=1 iv2=1 ovf=1 other=0
//! CAPT_TEST_BEGIN
//! CAPT SOFT FIRE OK
//! CAPT SOFT COV OK
//! CAPT ACLK SPAN OK
//! CAPT COUT EDGES OK
//! CAPT IV DEMUX OK
//! CAPT LPM0 WAKE OK
//! CAPT PWM FREQ OK        (or SKIP without the jumper)
//! CAPT PWM DUTY OK        (or SKIP)
//! CAPT_TEST_END
//! ```
//!
//! **GREEN** while everything present passes (SKIPs don't fail), **RED**
//! otherwise; the burst repeats once per second with frozen verdicts.

use core::cell::Cell;

use critical_section::Mutex;
use hal::capture::{self, CaptureTimer, Edge, Slot};
use hal::comp_e::{CompE, Config as CompConfig, FilterDelay, Threshold};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::{InputPin, OutputPin};
use hal::embedded_hal::pwm::SetDutyCycle as _;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::pac::Timer1A3;
use hal::power;
use hal::pwm::Pwm;
use hal::ref_a::{Ref, ReferenceVoltage};
use hal::serial::{Config as UartConfig, SerialExt};
use hal::timer::Divider;
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take() (and so msp430::interrupt::enable() is available).
use msp430 as _;

/// TA1IV tallies, shared ISR → main. `OTHER` counts unexpected codes.
static IV1: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static IV2: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static OVF: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static OTHER: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// TIMER1_A1 ISR: consume exactly one pending source via `TA1IV` (the read
/// clears the served flag in silicon) and tally it. A CCR2 (ACLK) capture
/// additionally **disarms itself** — at MCLK = 1 MHz this handler costs more
/// than one ACLK period, so leaving it armed would re-enter back-to-back and
/// starve main forever. `wake_cpu` so a wake from `enter_lpm0` resumes `main`.
#[msp430_rt::interrupt(wake_cpu)]
fn TIMER1_A1() {
    let iv = capture::read_iv::<Timer1A3>();
    if iv == capture::IV_CCR2 {
        capture::isr_disable_interrupt::<Timer1A3>(Slot::Ccr2);
    }
    critical_section::with(|cs| {
        let counter = match iv {
            capture::IV_CCR1 => IV1.borrow(cs),
            capture::IV_CCR2 => IV2.borrow(cs),
            capture::IV_OVERFLOW => OVF.borrow(cs),
            _ => OTHER.borrow(cs),
        };
        counter.set(counter.get().wrapping_add(1));
    });
}

/// Read the ISR tallies.
fn counts() -> (u16, u16, u16, u16) {
    critical_section::with(|cs| {
        (
            IV1.borrow(cs).get(),
            IV2.borrow(cs).get(),
            OVF.borrow(cs).get(),
            OTHER.borrow(cs).get(),
        )
    })
}

/// A ladder tap far below REFOUT at 2.0 V (~113 mV at 3.63 V AVCC) → COUT=1.
const TAP_LOW: u8 = 0;
/// A ladder tap far above REFOUT at 2.0 V (full AVCC) → COUT=0.
const TAP_HIGH: u8 = 31;

/// Per-edge poll budget for the ~1 kHz PWM (5 periods at the 1 MHz tick).
const PWM_TIMEOUT: u16 = 5_000;

/// Was a capture timestamp taken between the `before`/`after` counter
/// snapshots? (All three from the same free-running 16-bit counter, so the
/// wrapping-subtraction ordering trick applies.)
fn stamp_bracketed(ts: u16, before: u16, after: u16) -> bool {
    ts.wrapping_sub(before) <= after.wrapping_sub(before)
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Low-power profile: MCLK = SMCLK = 1 MHz, and — the part this fixture
    // exists for — ACLK on the 32.768 kHz LFXT crystal, so the ACLK-capture
    // verdict measures the DCO against a crystal-exact reference.
    let clocks = hal::clocks::configure_low_power(p.cs);
    let aclk_is_crystal = clocks.aclk_source() == hal::clocks::AclkSource::Lfxt;

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 1 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2
    let mut red_led = port4.pin6.into_output(); // LED1

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 Timer_A capture self-check (jumper optional)\r\n")
        .ok();

    // --- Jumper detection: P1.4 driven as GPIO, P1.2 read with a pull-down --
    // Done before either pin is handed to its timer. The level follows the
    // drive only if the jumper is installed (the pull-down pins a floating
    // P1.2 low, so noise cannot fake a jumper).
    let mut p14_probe = port1.pin4.into_output();
    let mut p12_probe = port1.pin2.into_pull_down_input();
    p14_probe.set_high().ok();
    delay.delay_us(100);
    let high_follows = p12_probe.is_high().unwrap_or(false);
    p14_probe.set_low().ok();
    delay.delay_us(100);
    let low_follows = p12_probe.is_low().unwrap_or(false);
    let jumper = high_follows && low_follows;

    // --- Capture timer: TA1 free-running on SMCLK ÷ 1 (1 µs tick, 65.5 ms wrap)
    let cap_timer = CaptureTimer::new_smclk(p.timer_1_a3, &clocks, Divider::Div1);

    // --- CAPT SOFT FIRE: latch path + timestamp-at-event ------------------
    let mut soft = cap_timer.capture_software(Slot::Ccr1);
    let before = cap_timer.now();
    soft.fire_software();
    let after = cap_timer.now();
    let first = soft.take();
    let soft_fire_ok = match first {
        Some(ts) => stamp_bracketed(ts, before, after) && soft.take().is_none(),
        None => false,
    };

    // --- CAPT SOFT COV: a second fire before collection is flagged --------
    soft.fire_software();
    soft.fire_software();
    let cov_latched = soft.overcaptured();
    soft.clear_overcapture();
    let soft_cov_ok = cov_latched && !soft.overcaptured() && soft.take().is_some();

    // --- CAPT ACLK SPAN: the DCO measured against the crystal -------------
    // ~16 crystal periods (488 ticks ideal) bracketed by two serviced edges;
    // the count is recovered by rounding, the ratio must sit within ±5%
    // (crystal exact, factory DCO trim ±3.5%). Requires the crystal.
    let mut aclk_ch = cap_timer.capture_aclk(Edge::Rising);
    let (span, n, ratio) = match aclk_ch.measure_span_ticks(480, 200) {
        Ok(delta) => {
            let n = capture::periods_in_span(delta as u32, cap_timer.tick_hz(), clocks.aclk());
            (
                delta,
                n,
                capture::span_ratio_permille(delta as u32, n, cap_timer.tick_hz(), clocks.aclk()),
            )
        }
        Err(_) => (0, 0, 0),
    };
    let aclk_ok = aclk_is_crystal && (12..=20).contains(&n) && (950..=1050).contains(&ratio);

    // --- CAPT COUT EDGES: comparator edges timestamped in hardware --------
    // REFOUT (2.0 V buffered onto P1.1 = C1) is the analog stimulus; stepping
    // the VCC ladder across it makes real COUT edges from software, exactly
    // as in the comparator fixture. (Keep button S2 released — it shorts
    // P1.1/REFOUT to ground.)
    let mut vref = Ref::new(p.shared_reference, ReferenceVoltage::V2_0);
    let p11 = port1.pin1.into_analog();
    vref.enable_output(&p11);
    let mut comp = CompE::new(
        p.comparator_e,
        CompConfig::default().filter(FilterDelay::Ns1800),
    );
    comp.watch_pin(&p11, Threshold::vcc_ladder(TAP_HIGH)); // V− far above → COUT=0
    delay.delay_ms(1); // REFOUT buffer + comparator settling

    let mut cout_ch = cap_timer.capture_comparator(Edge::Both);
    let mut cout_ok = true;
    let edge = |comp: &mut CompE,
                    ch: &mut capture::CaptureChannel<Timer1A3>,
                    delay: &mut Delay,
                    tap: u8,
                    expect_level: bool|
     -> bool {
        let _ = ch.take(); // discard anything stale
        let before = cap_timer.now();
        comp.set_taps(tap, tap);
        delay.delay_us(500); // comparator + filter settle, edge latches
        let after = cap_timer.now();
        match ch.take() {
            Some(ts) => stamp_bracketed(ts, before, after) && ch.input_level() == expect_level,
            None => false,
        }
    };
    // V− drops far below REFOUT → COUT rises; back above → COUT falls.
    cout_ok &= edge(&mut comp, &mut cout_ch, &mut delay, TAP_LOW, true);
    cout_ok &= edge(&mut comp, &mut cout_ch, &mut delay, TAP_HIGH, false);
    cout_ok &= edge(&mut comp, &mut cout_ch, &mut delay, TAP_LOW, true);
    comp.set_taps(TAP_HIGH, TAP_HIGH); // park COUT=0 for the IRQ phase
    delay.delay_us(500);

    // --- CAPT IV DEMUX phase A: comparator capture → IV 0x02, exactly once -
    let _ = cout_ch.take();
    cout_ch.enable_interrupt();
    // SAFETY: enabling interrupts globally (set GIE) so the TIMER1_A1 ISR can
    // run. All state shared with the ISR lives in critical-section Mutexes.
    unsafe {
        msp430::interrupt::enable();
    }
    comp.set_taps(TAP_LOW, TAP_LOW); // rising COUT edge → capture → interrupt
    delay.delay_ms(2);
    let after_ch1 = counts();
    cout_ch.disable_interrupt();

    // --- CAPT IV DEMUX phase B + LPM0 WAKE: sleep into an ACLK capture ----
    // GIE off first, so the interrupt cannot fire before the sleep;
    // enter_lpm0 sets GIE and the LPM bits in one instruction and the next
    // crystal edge (≤ 30.5 µs away) must deliver the wake. The ISR disarms
    // CCR2 itself (see its docs) — that is also what makes this exactly-once.
    msp430::interrupt::disable();
    let mut aclk_irq_ch = cap_timer.capture_aclk(Edge::Rising);
    aclk_irq_ch.enable_interrupt();
    power::enter_lpm0();
    let after_ch2 = counts();
    delay.delay_ms(2); // armed-no-more: nothing may accumulate
    let frozen_ch2 = counts();
    let lpm0_wake_ok = after_ch2.1 == 1 && frozen_ch2.1 == 1;

    // --- CAPT IV DEMUX phase C: counter overflow → IV 0x0E -----------------
    cap_timer.enable_overflow_interrupt();
    delay.delay_ms(70); // one wrap of the 65.5 ms counter
    cap_timer.disable_overflow_interrupt();
    let after_ovf = counts();
    delay.delay_ms(70);
    let frozen = counts();
    let iv_ok = after_ch1 == (1, 0, 0, 0)
        && after_ovf.0 == 1
        && after_ovf.2 >= 1
        && frozen == after_ovf
        && frozen.3 == 0;

    // --- CAPT PWM FREQ / DUTY: through a real pad, if the jumper is on -----
    // TB0.1 (P1.4) generates ~1 kHz; TA1.CCI1A (P1.2) measures it. Both
    // timers run from the same SMCLK, so frequency must agree to rounding;
    // the two asymmetric duty points catch an inverted or stuck line.
    let (mut f_meas, mut d25, mut d75) = (0u32, 0u16, 0u16);
    let (mut pwm_freq_ok, mut pwm_duty_ok) = (false, false);
    if jumper {
        let pwm = Pwm::new_smclk(p.timer_0_b7, &clocks, 1_000);
        let p14 = p14_probe.into_timer_b_output(); // P1.4 → TB0.1
        let mut pwm_ch = pwm.channel(p14);
        let p12 = p12_probe.into_timer_a_capture(); // P1.2 → TA1.CCI1A
        let mut cap_ch = cap_timer.capture_pin(p12, Edge::Rising);

        pwm_ch.set_duty_cycle_percent(25).ok();
        delay.delay_ms(5); // let the wave run before measuring
        f_meas = cap_ch.frequency_hz(8, PWM_TIMEOUT).unwrap_or(0);
        pwm_freq_ok = capture::within_permille(f_meas, pwm.frequency(), 10);

        cap_ch.set_edge(Edge::Both);
        d25 = cap_ch.measure_duty_permille(PWM_TIMEOUT).unwrap_or(0);
        pwm_ch.set_duty_cycle_percent(75).ok();
        delay.delay_ms(5);
        d75 = cap_ch.measure_duty_permille(PWM_TIMEOUT).unwrap_or(0);
        // set_duty_cycle_percent(25) programs floor(999·25/100) = 249‰.
        pwm_duty_ok = d25.abs_diff(250) <= 20 && d75.abs_diff(750) <= 20;
    }

    let hands_free_ok =
        soft_fire_ok && soft_cov_ok && aclk_ok && cout_ok && iv_ok && lpm0_wake_ok;
    let all_ok = hands_free_ok && (!jumper || (pwm_freq_ok && pwm_duty_ok));

    // Verdicts are frozen; re-emit the burst forever at 1 Hz.
    loop {
        let (iv1, iv2, ovf, other) = counts();
        tx.write_all(b"capture aclk=").ok();
        write_dec(&mut tx, aclk_is_crystal as u32);
        tx.write_all(b" span=").ok();
        write_dec(&mut tx, span as u32);
        tx.write_all(b" n=").ok();
        write_dec(&mut tx, n);
        tx.write_all(b" ratio=").ok();
        write_dec(&mut tx, ratio);
        tx.write_all(b" f=").ok();
        write_dec(&mut tx, f_meas);
        tx.write_all(b" d25=").ok();
        write_dec(&mut tx, d25 as u32);
        tx.write_all(b" d75=").ok();
        write_dec(&mut tx, d75 as u32);
        tx.write_all(b" jumper=").ok();
        write_dec(&mut tx, jumper as u32);
        tx.write_all(b" iv1=").ok();
        write_dec(&mut tx, iv1 as u32);
        tx.write_all(b" iv2=").ok();
        write_dec(&mut tx, iv2 as u32);
        tx.write_all(b" ovf=").ok();
        write_dec(&mut tx, ovf as u32);
        tx.write_all(b" other=").ok();
        write_dec(&mut tx, other as u32);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"CAPT_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"CAPT SOFT FIRE", soft_fire_ok);
        verdict(&mut tx, b"CAPT SOFT COV", soft_cov_ok);
        verdict(&mut tx, b"CAPT ACLK SPAN", aclk_ok);
        verdict(&mut tx, b"CAPT COUT EDGES", cout_ok);
        verdict(&mut tx, b"CAPT IV DEMUX", iv_ok);
        verdict(&mut tx, b"CAPT LPM0 WAKE", lpm0_wake_ok);
        if jumper {
            verdict(&mut tx, b"CAPT PWM FREQ", pwm_freq_ok);
            verdict(&mut tx, b"CAPT PWM DUTY", pwm_duty_ok);
        } else {
            tx.write_all(b"CAPT PWM FREQ SKIP\r\n").ok();
            tx.write_all(b"CAPT PWM DUTY SKIP\r\n").ok();
        }
        tx.write_all(b"CAPT_TEST_END\r\n").ok();

        if all_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write `name` + ` OK`/` FAIL` + CRLF.
fn verdict<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], ok: bool) {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" as &[u8] } else { b" FAIL\r\n" })
        .ok();
}

/// Write an unsigned value as decimal ASCII (no padding). `core::fmt` is
/// deliberately avoided project-wide (FRAM budget).
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
