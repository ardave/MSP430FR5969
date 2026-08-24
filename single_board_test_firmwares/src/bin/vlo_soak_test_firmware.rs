#![no_std]
#![no_main]

//! VLO/ACLK boot-race soak instrument — **no wiring at all**, driven by the
//! host-side `vlo_soak_test_orchestrator` runner (name-only suite: `cargo +nightly run
//! -- vlo_soak`; not in the default set — it reboots the chip 200 times).
//!
//! # Why this exists
//!
//! The `clock_speed` fixture occasionally boots with its VLO capture
//! measuring **zero ACLK edges** (observed 2026-07-11: frozen `vlo=0` FAIL,
//! roughly a few percent of boots), even though ACLK = VLO is selected and
//! the measurement runs ~60 ms after the profile lands with 4–5× timing
//! margin per edge. One boot gives one sample of that race; reflashing for
//! more samples costs ~40 s each. This fixture turns the chip into its own
//! statistics engine: each boot measures ACLK health once, records the
//! outcome in Info FRAM, and immediately reboots via
//! [`hal::watchdog::force_reset`] — ~200 boot samples in seconds, no host
//! involvement until the report.
//!
//! # Per-boot protocol
//!
//! 1. Drain `SYSRSTIV`: **WatchdogPassword** (what `force_reset` reports) +
//!    valid FRAM magic = soak continuation; **ResetPin/Brownout** (reflash,
//!    reset button, power cycle) = cold start, counters zeroed. Anything
//!    else also cold-starts (a surprise PUC must not corrupt the tally).
//! 2. Run [`hal::clocks::configure_max_speed`] — the profile both observed
//!    failures booted under — and immediately arm the same capture the
//!    clock fixture uses (TA1 on SMCLK÷8, ACLK on CCI2B, 4 periods, 500 µs
//!    per-edge budget). *No UART first*: this samples ACLK a few
//!    milliseconds after boot, earlier than the clock fixture does, to
//!    catch the widest version of the race.
//! 3. Retry `frequency_hz` up to 200× back-to-back (a failed attempt costs
//!    one 500 µs first-edge timeout, so the whole budget is ~0.1 s):
//!    - success on try 1 → clean boot;
//!    - success on try 2..=200 → **flaky** boot; the try number is the
//!      recovery profile (`maxtries` tracks the worst);
//!    - no success in 200 tries → **dead** boot (ACLK never showed an edge
//!      in ~0.1 s — the whole-boot-dead hypothesis).
//! 4. Update the FRAM record, and either `force_reset()` for the next
//!    sample or — after [`TARGET_BOOTS`] — bring up the UART and report
//!    forever.
//!
//! # Caveat on reset class
//!
//! Both field observations of the flake were **reset-pin** boots (DSLite
//! release after flashing). This soak necessarily samples **PUC** boots
//! (WDT-password resets) — the only reset the chip can give itself without
//! external help. A zero flake count here is therefore still informative:
//! it localizes the race to reset-pin/debugger-release boots rather than
//! all boots. (After the report phase, pressing the reset button cold-starts
//! a fresh soak.)
//!
//! # Framed output for the host runner (report phase only)
//!
//! ```text
//! vlosoak total=200 flaky=0 dead=0 maxtries=1 lasthz=9732 rst=WDT password violation
//! VLO_SOAK_BEGIN
//! SOAK COMPLETE OK
//! VLO_SOAK_END
//! ```
//!
//! `SOAK COMPLETE` is the only verdict — this is an instrument, not a
//! regression gate; the counts in the info line are the product. GREEN in
//! the report phase when the soak completed with zero dead boots, RED if
//! any boot's ACLK never recovered.

use hal::capture::{CaptureTimer, Edge};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::embedded_storage::{ReadStorage as _, Storage as _};
use hal::fram::InfoFram;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::sys::{ResetReason, ResetReasons};
use hal::timer::Divider;
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Info-FRAM offset for the soak record (0x60 = lpmx5, 0x70 = mpu, 0x80 =
/// clock_speed scratch).
const FRAM_OFFSET: u32 = 0x90;
/// Record layout marker; any other first two bytes = uninitialized.
const MAGIC: [u8; 2] = [0x50, 0xAC]; // "SoAC"

/// Boot samples per soak.
const TARGET_BOOTS: u16 = 200;
/// Measurement attempts per boot before declaring ACLK dead for the boot.
const MAX_TRIES: u16 = 200;

/// The persisted tally. 12 bytes at [`FRAM_OFFSET`]:
/// magic(2) total(2) flaky(2) dead(2) maxtries(2) lasthz(2).
struct Record {
    total: u16,
    flaky: u16,
    dead: u16,
    maxtries: u16,
    lasthz: u16,
}

fn load(fram: &mut InfoFram) -> Option<Record> {
    let mut raw = [0u8; 12];
    fram.read(FRAM_OFFSET, &mut raw).ok()?;
    if raw[0..2] != MAGIC {
        return None;
    }
    let word = |i: usize| u16::from_le_bytes([raw[i], raw[i + 1]]);
    Some(Record {
        total: word(2),
        flaky: word(4),
        dead: word(6),
        maxtries: word(8),
        lasthz: word(10),
    })
}

fn save(fram: &mut InfoFram, r: &Record) {
    let mut raw = [0u8; 12];
    raw[0..2].copy_from_slice(&MAGIC);
    raw[2..4].copy_from_slice(&r.total.to_le_bytes());
    raw[4..6].copy_from_slice(&r.flaky.to_le_bytes());
    raw[6..8].copy_from_slice(&r.dead.to_le_bytes());
    raw[8..10].copy_from_slice(&r.maxtries.to_le_bytes());
    raw[10..12].copy_from_slice(&r.lasthz.to_le_bytes());
    fram.write(FRAM_OFFSET, &raw).ok();
}

/// Firmware entry point — runs once per boot sample (and once more for the
/// report phase).
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Continuation only on the exact reset force_reset produces; everything
    // else (reflash, reset button, power-on, surprise PUC) starts cold.
    let reasons = ResetReasons::drain(&p.sys);
    let continuation = reasons.contains(ResetReason::WatchdogPassword);

    let mut fram = InfoFram::new();
    let mut rec = match load(&mut fram) {
        Some(r) if continuation => r,
        _ => Record {
            total: 0,
            flaky: 0,
            dead: 0,
            maxtries: 0,
            lasthz: 0,
        },
    };

    // The profile both observed failures booted under.
    let clocks = hal::clocks::configure_max_speed(p.cs);

    if rec.total < TARGET_BOOTS {
        // ---- Sampling boot: measure ACLK health NOW, no UART first --------
        let cap = CaptureTimer::new_smclk(p.timer_1_a3, &clocks, Divider::Div8);
        let mut aclk_ch = cap.capture_aclk(Edge::Rising);
        let mut tries: u16 = 0;
        let mut hz: u32 = 0;
        while tries < MAX_TRIES {
            tries += 1;
            if let Ok(f) = aclk_ch.frequency_hz(4, 1_000) {
                hz = f;
                break;
            }
        }

        rec.total += 1;
        if hz == 0 {
            rec.dead += 1;
        } else {
            if tries > 1 {
                rec.flaky += 1;
            }
            if tries > rec.maxtries {
                rec.maxtries = tries;
            }
            rec.lasthz = hz.min(u16::MAX as u32) as u16;
        }
        save(&mut fram, &rec);
        hal::watchdog::force_reset();
    }

    // ---- Report phase: the tally is in; bring up the UART and say so ------
    hal::gpio::unlock_pins(&p.pmm);

    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2
    let mut red_led = port4.pin6.into_output(); // LED1

    let mut delay = Delay::new(clocks.mclk());
    let complete = rec.total >= TARGET_BOOTS;

    loop {
        tx.write_all(b"vlosoak total=").ok();
        write_dec(&mut tx, rec.total as u32);
        tx.write_all(b" flaky=").ok();
        write_dec(&mut tx, rec.flaky as u32);
        tx.write_all(b" dead=").ok();
        write_dec(&mut tx, rec.dead as u32);
        tx.write_all(b" maxtries=").ok();
        write_dec(&mut tx, rec.maxtries as u32);
        tx.write_all(b" lasthz=").ok();
        write_dec(&mut tx, rec.lasthz as u32);
        tx.write_all(b" rst=").ok();
        tx.write_all(reasons.primary().map_or("none", |r| r.as_str()).as_bytes())
            .ok();
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"VLO_SOAK_BEGIN\r\n").ok();
        verdict(&mut tx, b"SOAK COMPLETE", complete);
        tx.write_all(b"VLO_SOAK_END\r\n").ok();

        if complete && rec.dead == 0 {
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
