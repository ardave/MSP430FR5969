#![no_std]
#![no_main]

//! High-speed clock-profile integration fixture — **no wiring at all**,
//! driven by the host-side `clock_speed_test_orchestrator` runner.
//!
//! ```text
//! cargo +nightly build --bin clock_speed_test_firmware
//! DSLite load ... -f target/msp430-none-elf/debug/clock_speed_test_firmware
//! ```
//!
//! Default build runs [`hal::clocks::configure_max_speed`] (MCLK = SMCLK =
//! 16 MHz, one FRAM wait state); with `--features clock_high_speed` it runs
//! [`hal::clocks::configure_high_speed`] (SMCLK = 16 MHz, MCLK = 8 MHz, zero
//! wait states) — one source, both new profiles.
//!
//! Verifying a clock change is subtle because almost everything scales
//! *together*: a Delay measured by a timer proves only that MCLK and SMCLK
//! kept their ratio, since both derive from the same DCO. The fixture
//! therefore combines four signals with different failure blind spots:
//!
//! - **CLKSPD FRCTL** — `FRCTL0.NWAITS` reads back as the profile programmed
//!   it (1 for max speed, 0 for high speed). Readback needs no password, so
//!   this directly proves the password-bracketed write landed. (And the code
//!   *running* at 16 MHz MCLK is itself the wait-state proof — instruction
//!   fetch outruns the FRAM without it.)
//! - **CLKSPD DELAY TIMER** — `delay_ms(10)` (MCLK-derived cycle counting)
//!   measured by a `Counter` on SMCLK ÷ 8: the MCLK↔SMCLK ratio check,
//!   which also pins the known Delay overhead at the new speed (~0.2–0.4 ms
//!   at 8–16 MHz, vs the ~2.5 ms it costs at 1 MHz).
//! - **CLKSPD VLO** — the capture module timestamps ACLK (= VLO in these
//!   profiles) against SMCLK ticks: the VLO is *independent of the DCO*, so
//!   a DCO stuck in the wrong range shifts the measured "VLO frequency" by
//!   2× and out of the 5–15 kHz gate. This is also the first real use of
//!   capture-as-clock-instrument on a non-crystal source.
//! - **CLKSPD FRAM RW** — an Info-FRAM round-trip (offset 0x80; lpmx5 owns
//!   0x60, mpu 0x70) exercises the FRAM *data* path under the new wait-state
//!   setting, complementing the fetch path the running code proves.
//!
//! The truly absolute check lives host-side: the runner wall-clocks the gap
//! between consecutive `CLKSPD_TEST_BEGIN` lines (nominally ~1.25 s: a 1 s
//! Delay plus ~0.2 s of burst at 9600 baud). A DCO in the wrong range would
//! stretch it past 1.8 s — and would also garble the 9600 baud UART, since
//! BRCLK rides the same SMCLK.
//!
//! # Framed output for the host runner
//!
//! ```text
//! clkspd mclk=16000000 smclk=16000000 nwaits=1 d10=10231 vlo=9382 vlotries=1 rst=...
//! CLKSPD_TEST_BEGIN
//! CLKSPD FRCTL OK
//! CLKSPD DELAY TIMER OK
//! CLKSPD VLO OK
//! CLKSPD FRAM RW OK
//! CLKSPD_TEST_END
//! ```
//!
//! **GREEN** while all pass, **RED** otherwise; the burst repeats with frozen
//! verdicts.

use hal::capture::{CaptureTimer, Edge};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::embedded_storage::{ReadStorage as _, Storage as _};
use hal::fram::InfoFram;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::sys::ResetReasons;
use hal::timer::{Counter, Divider};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// The NWAITS field (FRCTL0 low byte, bits 6:4) each profile must leave behind.
#[cfg(not(feature = "clock_high_speed"))]
const EXPECT_NWAITS: u8 = 0x10; // one wait state (MCLK 16 MHz)
#[cfg(feature = "clock_high_speed")]
const EXPECT_NWAITS: u8 = 0x00; // zero wait states (MCLK 8 MHz)

/// Info-FRAM scratch offset for the round-trip (0x60 = lpmx5, 0x70 = mpu).
const FRAM_SCRATCH: u32 = 0x80;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Reset reason before anything else: a PUC loop from a bad FRCTL/CS write
    // would show up here (e.g. "FRCTL password violation") instead of the
    // expected cold-boot causes.
    let reset = ResetReasons::drain(&p.sys);

    // The profile under test.
    #[cfg(not(feature = "clock_high_speed"))]
    let clocks = hal::clocks::configure_max_speed(p.cs);
    #[cfg(feature = "clock_high_speed")]
    let clocks = hal::clocks::configure_high_speed(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1 from BRCLK = SMCLK = 16 MHz. Readable output
    // is itself a verdict — the baud math has never run from a 16 MHz BRCLK
    // on hardware before.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2
    let mut red_led = port4.pin6.into_output(); // LED1

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 high-speed clock profile self-check (no wiring)\r\n")
        .ok();

    // --- CLKSPD FRCTL: the wait-state write landed --------------------------
    let nwaits = (unsafe { (0x0140 as *const u8).read_volatile() }) & 0x70;
    let frctl_ok = nwaits == EXPECT_NWAITS;

    // --- CLKSPD DELAY TIMER: MCLK-derived Delay vs SMCLK-derived Counter ----
    // Counter tick = SMCLK/8 = 2 MHz (32.7 ms wrap; a 10 ms interval fits).
    // Expect 10 ms + Delay's small fixed overhead + its ~1.7% long bias.
    let counter = Counter::new_smclk(p.timer_0_a3, &clocks, Divider::Div8);
    let t0 = counter.now();
    delay.delay_ms(10);
    let d10_us = counter.ticks_to_us(counter.elapsed_since(t0) as u32);
    let delay_ok = (9_800..=11_500).contains(&d10_us);

    // --- CLKSPD VLO: ACLK (= VLO) measured against the DCO-derived tick -----
    // The VLO doesn't follow the DCO, so a wrong DCO range doubles/halves the
    // measured frequency out of the gate. Period ~106 µs ≈ 213 ticks at the
    // 2 MHz capture tick; 4 periods with a 500 µs per-edge budget.
    //
    // Retried up to 5× (10 ms apart): rare boots come up with ACLK showing
    // no edges at the first sample (observed 2026-07-11 as a frozen `vlo=0`
    // FAIL; the `vlo_soak` fixture measures the rate and recovery profile).
    // Retrying doesn't weaken the verdict's real quarry — a DCO stuck in the
    // wrong range shifts *every* attempt 2× out of the gate — it only
    // absorbs a transiently dead ACLK. `vlotries` in the info line reports
    // which attempt delivered, so the flake stays visible in the transcript.
    let cap = CaptureTimer::new_smclk(p.timer_1_a3, &clocks, Divider::Div8);
    let mut aclk_ch = cap.capture_aclk(Edge::Rising);
    let mut vlo_hz = 0u32;
    let mut vlo_tries = 0u32;
    while vlo_tries < 5 {
        vlo_tries += 1;
        if let Ok(hz) = aclk_ch.frequency_hz(4, 1_000) {
            vlo_hz = hz;
            break;
        }
        delay.delay_ms(10);
    }
    let vlo_ok = (5_000..=15_000).contains(&vlo_hz);

    // --- CLKSPD FRAM RW: data path under the new wait-state setting ---------
    let mut info = InfoFram::new();
    let pattern: [u8; 8] = [
        0xC1, 0x0C, nwaits, !nwaits, 0x5A, 0xA5, 0x3C, 0xC3, // varies per profile
    ];
    let mut readback = [0u8; 8];
    let fram_ok = info.write(FRAM_SCRATCH, &pattern).is_ok()
        && info.read(FRAM_SCRATCH, &mut readback).is_ok()
        && readback == pattern;

    let all_ok = frctl_ok && delay_ok && vlo_ok && fram_ok;

    // Verdicts are frozen; re-emit the burst forever (the host wall-clocks
    // the BEGIN-to-BEGIN gap, so keep the loop shape stable).
    loop {
        tx.write_all(b"clkspd mclk=").ok();
        write_dec(&mut tx, clocks.mclk());
        tx.write_all(b" smclk=").ok();
        write_dec(&mut tx, clocks.smclk());
        tx.write_all(b" nwaits=").ok();
        write_dec(&mut tx, (nwaits >> 4) as u32);
        tx.write_all(b" d10=").ok();
        write_dec(&mut tx, d10_us);
        tx.write_all(b" vlo=").ok();
        write_dec(&mut tx, vlo_hz);
        tx.write_all(b" vlotries=").ok();
        write_dec(&mut tx, vlo_tries);
        tx.write_all(b" rst=").ok();
        tx.write_all(reset.primary().map_or("none", |r| r.as_str()).as_bytes())
            .ok();
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"CLKSPD_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"CLKSPD FRCTL", frctl_ok);
        verdict(&mut tx, b"CLKSPD DELAY TIMER", delay_ok);
        verdict(&mut tx, b"CLKSPD VLO", vlo_ok);
        verdict(&mut tx, b"CLKSPD FRAM RW", fram_ok);
        tx.write_all(b"CLKSPD_TEST_END\r\n").ok();

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
