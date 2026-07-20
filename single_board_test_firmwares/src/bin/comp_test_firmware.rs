#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! Comp_E analog-comparator integration fixture — **no wiring at all**,
//! driven by the host-side `comp_tests` runner.
//!
//! ```text
//! cargo +nightly build --bin comp_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/comp_test_runner
//! ```
//!
//! Two tricks make a comparator testable with nothing on the desk:
//!
//! 1. **REFOUT is an on-die signal generator.** The shared reference buffered
//!    onto P1.1 (= C1) presents a *known mid-rail analog voltage* to the
//!    comparator's V+ — the one way to get a real analog level onto a pad
//!    with no wiring. (Do not hold button S2 during the run: S2 shorts
//!    P1.1/REFOUT to ground.)
//! 2. **Stepping the ladder taps makes comparator edges on demand.** With V+
//!    parked at REFOUT, moving `CEREF0/1` from a tap above REFOUT to one
//!    below (and back) swings V− across V+ — a genuine analog crossing and a
//!    real `CEOUT` edge, produced entirely from software. It is the same
//!    mechanism the hysteresis hardware uses on every output flip. This
//!    drives the interrupt and LPM0-wake checks deterministically.
//!
//! (The obvious-seeming third trick — comparing a pad the firmware drives as
//! a digital *output* — does **not** work on this part: hardware-established
//! 2026-07-05, a pad only reaches the comparator through the pin's analog
//! function (`PxSEL = 11`), which disconnects the output driver. The fixture
//! keeps a two-tap probe of a driven P1.3/C3 as a *diagnostic*, reported in
//! the info line as `dig=`, expected dead — if a future die revision or
//! errata clarification changes this, the info line will say so.)
//!
//! # What it checks
//!
//! - **COMP RAILS** — with V+ = REFOUT at 2.0 V, taps far below (0, 5) read
//!   `output() == 1` and taps far above (25, 31) read 0 (mux routing, ladder
//!   ordering, `CEOUT` polarity).
//! - **COMP EXCHANGE** — `CEEX` swaps the input terminals *and* inverts the
//!   output, so the logical comparison must survive the exchange unchanged —
//!   asserted at both output states (a stuck-at output fails the 0 case).
//! - **COMP IRQ IV** — a tap step below REFOUT makes a rising `CEOUT` edge
//!   latching `CEIFG` (CEIV 0x02), a step back above makes a falling edge
//!   latching `CEIIFG` (0x04), each exactly once, demuxed by
//!   `comp_e::read_iv()` whose read auto-clears (counts stay put afterward).
//! - **COMP LPM0 WAKE** — with GIE off, a tap-step edge latches; `enter_lpm0`
//!   (GIE+LPM set atomically) must deliver it to the `wake_cpu` ISR and
//!   resume `main`.
//! - **COMP SWEEP MONO** — both 32-tap sweeps are monotone: all-1s then
//!   all-0s, exactly one transition (an unsettled or mis-ordered ladder
//!   fails).
//! - **COMP SWEEP 2V0 / 1V2** — the flip tap against REFOUT at 2.0 V and
//!   1.2 V lands within ±2 taps (±~225 mV) of the prediction from the
//!   ADC-measured AVCC (~3630 mV on this LaunchPad → expect ~17 and ~10) —
//!   the comparator run as a manual SAR, cross-validated against the ADC.
//!
//! # Framed output for the host runner
//!
//! ```text
//! comp avcc=3630 flip20=17 flip12=10 sweep20=0001FFFF sweep12=000003FF rose=2 fell=1 other=0 iv=2 dig=0
//! COMP_TEST_BEGIN
//! COMP RAILS OK
//! COMP EXCHANGE OK
//! COMP IRQ IV OK
//! COMP LPM0 WAKE OK
//! COMP SWEEP MONO OK
//! COMP SWEEP 2V0 OK
//! COMP SWEEP 1V2 OK
//! COMP_TEST_END
//! ```
//!
//! **GREEN** while all pass, **RED** otherwise; the burst repeats once per
//! second with frozen verdicts.

use core::cell::Cell;

use critical_section::Mutex;
use hal::adc::{Adc, Config as AdcConfig, SampleTime};
use hal::comp_e::{self, CompE, Config as CompConfig, FilterDelay, Threshold};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::power;
use hal::ref_a::{Ref, ReferenceVoltage};
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take() (and so msp430::interrupt::enable() is available).
use msp430 as _;

/// Edge tallies, shared ISR → main. `OTHER` counts unexpected CEIV values
/// (any nonzero value that is neither 0x02 nor 0x04) with the last one kept
/// for the info line.
static ROSE: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static FELL: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static OTHER: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static LAST_IV: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// COMP_E ISR: consume exactly one pending source via `CEIV` (the read
/// clears the served flag in silicon) and tally it. `wake_cpu` so a wake
/// from `enter_lpm0` resumes `main`.
#[msp430_rt::interrupt(wake_cpu)]
fn COMP_E() {
    let iv = comp_e::read_iv();
    critical_section::with(|cs| {
        LAST_IV.borrow(cs).set(iv);
        let counter = match iv {
            comp_e::IV_OUTPUT_ROSE => ROSE.borrow(cs),
            comp_e::IV_OUTPUT_FELL => FELL.borrow(cs),
            _ => OTHER.borrow(cs),
        };
        counter.set(counter.get().wrapping_add(1));
    });
}

/// Read the ISR tallies.
fn counts() -> (u16, u16, u16) {
    critical_section::with(|cs| {
        (
            ROSE.borrow(cs).get(),
            FELL.borrow(cs).get(),
            OTHER.borrow(cs).get(),
        )
    })
}

/// A tap far below REFOUT at either reference voltage (113 mV at 3.63 V VCC).
const TAP_LOW: u8 = 0;
/// A tap far above REFOUT at either reference voltage (full AVCC).
const TAP_HIGH: u8 = 31;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Performance profile: SMCLK = 8 MHz (UART BRCLK), MCLK = 1 MHz (Delay).
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2
    let mut red_led = port4.pin6.into_output(); // LED1

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 Comp_E analog comparator self-check (no wiring)\r\n")
        .ok();

    // Reference + ADC first: the ADC-measured supply is the sweep prediction's
    // input, REFOUT on P1.1 = C1 is the comparator's analog stimulus.
    let mut vref = Ref::new(p.shared_reference, ReferenceVoltage::V2_0);
    let mut adc = Adc::new(
        p.adc12,
        AdcConfig::default().sample_time(SampleTime::Cycles256),
    );
    let avcc_mv = adc.read_supply_millivolts(&vref) as u16;
    let p11 = port1.pin1.into_analog();
    vref.enable_output(&p11);

    // The output glitch filter smooths the slivers a slow analog crossing can
    // make, so each tap step produces exactly one counted edge.
    let mut comp = CompE::new(p.comparator_e, CompConfig::default().filter(FilterDelay::Ns1800));

    // --- Diagnostic (info line only): a digitally-driven pad ---------------
    // P1.3 as GPIO output high/low, watched as C3. Expected DEAD on this
    // part (the pad only reaches the comparator through PxSEL = 11, which
    // disconnects the driver) — recorded so the fact stays visible.
    let mut drive = port1.pin3.into_output();
    let mut dig_bits = 0u32;
    comp.watch_channel(3, Threshold::vcc_ladder(15));
    delay.delay_us(200);
    drive.set_high().ok();
    delay.delay_us(200);
    if comp.output() {
        dig_bits |= 1;
    }
    drive.set_low().ok();
    delay.delay_us(200);
    if comp.output() {
        dig_bits |= 2;
    }

    // --- COMP RAILS: REFOUT lands on the right side of far taps ------------
    comp.watch_pin(&p11, Threshold::vcc_ladder(TAP_HIGH));
    delay.delay_ms(1); // REFOUT buffer + comparator settling
    let mut rails_ok = true;
    for (tap, expect_above) in [(TAP_LOW, true), (5, true), (25, false), (TAP_HIGH, false)] {
        comp.set_taps(tap, tap);
        delay.delay_us(200);
        rails_ok &= comp.output() == expect_above;
    }

    // --- COMP EXCHANGE: CEEX's swap + invert cancel at both output states --
    // Exchanging the terminals also inverts the output, so the logical
    // comparison must read the SAME before and after — checked with the
    // output at 1 (tap far below REFOUT) and at 0 (tap far above), so a
    // stuck-at output cannot pass.
    let mut exchange_ok = true;
    for (tap, expect) in [(TAP_LOW, true), (TAP_HIGH, false)] {
        comp.set_taps(tap, tap);
        delay.delay_us(200);
        exchange_ok &= comp.output() == expect;
        comp.exchange_inputs(true);
        delay.delay_us(200);
        exchange_ok &= comp.output() == expect;
        comp.exchange_inputs(false);
        delay.delay_us(200);
    }

    // --- COMP IRQ IV: one tap-step edge = one CEIV code, exactly once ------
    comp.set_taps(TAP_HIGH, TAP_HIGH); // out = 0
    delay.delay_us(200);
    comp.enable_output_interrupts();
    // SAFETY: enabling interrupts globally (set GIE) so the COMP_E ISR can
    // run. All state shared with the ISR lives in critical-section Mutexes.
    unsafe {
        msp430::interrupt::enable();
    }
    comp.set_taps(TAP_LOW, TAP_LOW); // V− drops through V+ → rising CEOUT edge
    delay.delay_ms(2);
    let (rose1, fell1, other1) = counts();
    comp.set_taps(TAP_HIGH, TAP_HIGH); // V− climbs back over V+ → falling edge
    delay.delay_ms(2);
    let (rose2, fell2, other2) = counts();
    // Exactly-once and stays-cleared: nothing may refire from stale flags.
    delay.delay_ms(10);
    let (rose3, fell3, other3) = counts();
    let irq_ok = (rose1, fell1, other1) == (1, 0, 0)
        && (rose2, fell2, other2) == (1, 1, 0)
        && (rose3, fell3, other3) == (1, 1, 0);

    // --- COMP LPM0 WAKE: a latched comparator edge must end the sleep ------
    // Latch the edge with GIE off, so the ISR cannot run *before* the sleep:
    // enter_lpm0 sets GIE and the LPM bits in one instruction, the pending
    // CEIFG fires the wake_cpu ISR, and execution resumes here. Attempted
    // only if the interrupt path just proved itself — sleeping on an edge
    // the previous check could not produce would hang the fixture instead
    // of failing it.
    let wake_ok = if irq_ok {
        tx.write_all(b"comp entering LPM0, expecting comparator wake\r\n")
            .ok();
        msp430::interrupt::disable();
        comp.set_taps(TAP_LOW, TAP_LOW); // rising edge latches CEIFG, GIE off
        delay.delay_us(200);
        power::enter_lpm0();
        let (rose4, _, other4) = counts();
        rose4 == 2 && other4 == 0
    } else {
        tx.write_all(b"comp skipping LPM0 wake (irq path failed)\r\n")
            .ok();
        false
    };
    comp.disable_output_interrupts();

    // --- COMP SWEEP: the comparator as a manual SAR against REFOUT ---------
    // Sweep all 32 taps: output is 1 while REFOUT > tap. Bit n of the mask =
    // output at tap n, so a healthy sweep is a solid run of 1s from tap 0.
    let sweep = |comp: &mut CompE, delay: &mut Delay| -> u32 {
        let mut mask = 0u32;
        for tap in 0..32u8 {
            comp.set_taps(tap, tap);
            delay.delay_us(200);
            if comp.output() {
                mask |= 1u32 << tap;
            }
        }
        mask
    };
    let sweep20 = sweep(&mut comp, &mut delay);

    vref.set_voltage(ReferenceVoltage::V1_2); // REFOUT follows; REFGENRDY-gated
    delay.delay_ms(1);
    let sweep12 = sweep(&mut comp, &mut delay);

    // Flip tap = number of taps still below REFOUT; prediction from the
    // ADC-measured AVCC through the same host-tested ladder math.
    let flip20 = sweep20.trailing_ones() as u8;
    let flip12 = sweep12.trailing_ones() as u8;
    let expected_flip = |vref_mv: u16| -> u8 {
        let mut n = 0u8;
        while n < 32 && comp_e::ladder_millivolts(n, avcc_mv) < vref_mv {
            n += 1;
        }
        n
    };
    let mono_ok = is_single_transition(sweep20) && is_single_transition(sweep12);
    let sweep20_ok = flip_within(flip20, expected_flip(2000), 2);
    let sweep12_ok = flip_within(flip12, expected_flip(1200), 2);

    vref.disable_output();

    let all_ok =
        rails_ok && exchange_ok && irq_ok && wake_ok && mono_ok && sweep20_ok && sweep12_ok;

    // Verdicts are frozen; re-emit the burst forever at 1 Hz.
    loop {
        let (rose, fell, other) = counts();
        tx.write_all(b"comp avcc=").ok();
        write_dec(&mut tx, avcc_mv as u32);
        tx.write_all(b" flip20=").ok();
        write_dec(&mut tx, flip20 as u32);
        tx.write_all(b" flip12=").ok();
        write_dec(&mut tx, flip12 as u32);
        tx.write_all(b" sweep20=").ok();
        write_hex32(&mut tx, sweep20);
        tx.write_all(b" sweep12=").ok();
        write_hex32(&mut tx, sweep12);
        tx.write_all(b" rose=").ok();
        write_dec(&mut tx, rose as u32);
        tx.write_all(b" fell=").ok();
        write_dec(&mut tx, fell as u32);
        tx.write_all(b" other=").ok();
        write_dec(&mut tx, other as u32);
        tx.write_all(b" iv=").ok();
        write_dec(
            &mut tx,
            critical_section::with(|cs| LAST_IV.borrow(cs).get()) as u32,
        );
        tx.write_all(b" dig=").ok();
        write_dec(&mut tx, dig_bits);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"COMP_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"COMP RAILS", rails_ok);
        verdict(&mut tx, b"COMP EXCHANGE", exchange_ok);
        verdict(&mut tx, b"COMP IRQ IV", irq_ok);
        verdict(&mut tx, b"COMP LPM0 WAKE", wake_ok);
        verdict(&mut tx, b"COMP SWEEP MONO", mono_ok);
        verdict(&mut tx, b"COMP SWEEP 2V0", sweep20_ok);
        verdict(&mut tx, b"COMP SWEEP 1V2", sweep12_ok);
        tx.write_all(b"COMP_TEST_END\r\n").ok();

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

/// A healthy sweep mask is a (possibly empty) solid run of 1s starting at
/// bit 0, then 0s — i.e. `mask + 1` is a power of two (or mask is all-ones).
fn is_single_transition(mask: u32) -> bool {
    mask == u32::MAX || (mask.wrapping_add(1) & mask) == 0
}

/// |actual − expected| ≤ tol without underflow.
fn flip_within(actual: u8, expected: u8, tol: u8) -> bool {
    actual.abs_diff(expected) <= tol
}

/// Write `name` + ` OK`/` FAIL` + CRLF.
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

/// Write a u32 as eight uppercase hex digits. `core::fmt` is deliberately
/// avoided project-wide (FRAM budget).
fn write_hex32<W: hal::embedded_io::Write>(tx: &mut W, v: u32) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mut out = [0u8; 8];
    for (i, digit) in out.iter_mut().enumerate() {
        *digit = HEX[((v >> (28 - 4 * i)) & 0xF) as usize];
    }
    tx.write_all(&out).ok();
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
