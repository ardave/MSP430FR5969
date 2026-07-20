#![no_std]
#![no_main]

//! Watchdog + reset-reason integration fixture for `hal::watchdog` and
//! `hal::sys`.
//!
//! Reports over the UART backchannel (eUSCI_A0, 9600 8N1 on
//! `/dev/cu.usbmodem11203`), driven by the host-side `watchdog_tests` runner.
//! Needs no wiring beyond the LaunchPad itself — WDT_A, SYS, and FRAM are all
//! on-chip.
//!
//! ```text
//! cargo +nightly build --bin watchdog_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/watchdog_test_runner
//! ```
//!
//! # What it checks — a test that reboots itself, twice
//!
//! A watchdog test is unlike every other fixture here: the pass condition *is
//! a reset*, which wipes the program state that would normally carry the
//! verdict. So this fixture is a state machine that persists its phase in
//! Info FRAM (which PUC-class resets do not touch) and uses the drained
//! `SYSRSTIV` causes — the very feature under test — to tell its boots apart:
//!
//! 1. **FEEDING (fresh flash).** Arm WDT_A from SMCLK (8 MHz) with a ~1.05 s
//!    timeout (`Cycles8192K`) and feed it every 250 ms for 3 s — roughly
//!    three timeouts' worth. Surviving this proves [`Watchdog::feed`] really
//!    zeroes the count. Then record phase = STARVING and stop feeding.
//! 2. **The bite.** Within ~1.05 s the unfed dog issues a PUC. The next boot
//!    drains `SYSRSTIV` and must find **`0x16` (WDT timeout)**. Finding it
//!    while FRAM still says FEEDING means the dog bit *while being fed* —
//!    that is a feed failure, recorded as such (the chain still continues, so
//!    the host always gets a complete verdict burst). Either way the fixture
//!    records the timeout verdict, sets phase = AWAITING_KEY, and calls
//!    [`hal::watchdog::force_reset`].
//! 3. **The deliberate reboot.** `force_reset` is a wrong-password `WDTCTL`
//!    write, so this boot must drain **`0x18` (WDT password violation)** —
//!    proving firmware can tell "I rebooted on purpose" from "the guard shot
//!    me". The fixture then settles into the usual once-per-second framed
//!    verdict burst, forever.
//!
//! Any boot whose drained causes contain neither WDT cause (a DSLite flash
//! reset, the RST button, a power cycle) restarts the chain from FEEDING —
//! stale FRAM state cannot wedge the fixture, and re-running the host test
//! just re-runs the chain.
//!
//! # Framed output for the host runner
//!
//! The whole chain takes ~5 s from flash; the host only needs the steady
//! final burst. Each cycle, over UART:
//!
//! ```text
//! reasons: 0x18 (WDT password)      (human-readable info, skipped by host)
//! WDT_TEST_BEGIN
//! WDT FEED OK                        (survived 3 s of feeding a ~1 s dog)
//! WDT TIMEOUT RESET OK               (starved dog bit; SYSRSTIV said 0x16)
//! WDT KEY RESET OK                   (force_reset ran; SYSRSTIV said 0x18)
//! WDT_TEST_END
//! ```
//!
//! GREEN toggles each burst as a heartbeat; a steady RED means some verdict
//! is FAIL. Intermediate phases also narrate over UART (visible in `screen`,
//! usually lost to the host, which attaches whenever it likes — only the
//! final burst matters to it).

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::embedded_storage::{ReadStorage, Storage};
use hal::fram::InfoFram;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::sys::{ResetReason, ResetReasons};
use hal::watchdog::{ClockSource, Interval, Watchdog};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Info FRAM offset of this fixture's state block — well clear of the boot
/// counter the `fram_test_runner` fixture keeps at offset 0.
const STATE_OFFSET: u32 = 0x100;

/// Marks the state block as written by this firmware (layout version in the
/// low byte), so garbage or another fixture's leavings read as "no state".
const MAGIC: u32 = 0x57D6_0001;

/// Phase values persisted at `STATE_OFFSET + 4`.
const PHASE_FEEDING: u8 = 1;
const PHASE_STARVING: u8 = 2;
const PHASE_AWAITING_KEY: u8 = 3;

/// Verdict flag bits persisted at `STATE_OFFSET + 5`.
const FLAG_FEED_OK: u8 = 1 << 0;
const FLAG_TIMEOUT_OK: u8 = 1 << 1;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in
    // that order. Every boot of the chain starts held; only the FEEDING
    // phase re-arms, explicitly.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Drain the reset causes first thing — consume-on-read, first reader
    // wins. This is the feature under test.
    let reasons = ResetReasons::drain(&p.sys);

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds both the UART BRCLK and (in the
    // FEEDING phase) the watchdog countdown.
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO pins (clear LOCKLPM5) so the UART pin mux takes effect.
    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1).
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());
    let mut info = InfoFram::new();

    tx.write_all(b"\r\nMSP430FR5969 WDT + reset-reason test\r\n")
        .ok();
    print_reasons(&mut tx, &reasons);

    // Persisted state: magic word, phase byte, verdict flags byte.
    let mut state = [0u8; 6];
    info.read(STATE_OFFSET, &mut state).ok();
    let state_valid = u32::from_le_bytes([state[0], state[1], state[2], state[3]]) == MAGIC;
    let phase = state[4];
    let flags = state[5];

    let timed_out = reasons.contains(ResetReason::WatchdogTimeout);
    let key_violated = reasons.contains(ResetReason::WatchdogPassword);

    // --- Boot 2: the starved dog bit ----------------------------------------
    // A WDT-timeout cause while our state says the chain was in flight. If we
    // were still FEEDING, the dog bit through the feeds — feed FAIL — but the
    // chain continues either way so the host always gets a full burst.
    if state_valid && timed_out && (phase == PHASE_FEEDING || phase == PHASE_STARVING) {
        let mut new_flags = FLAG_TIMEOUT_OK;
        if phase == PHASE_STARVING {
            new_flags |= FLAG_FEED_OK;
        }
        write_state(&mut info, PHASE_AWAITING_KEY, new_flags);
        tx.write_all(b"timeout reset confirmed; forcing key reset...\r\n")
            .ok();
        hal::watchdog::force_reset();
    }

    // --- Boot 3: force_reset's password violation ---------------------------
    // Chain complete: report the accumulated verdicts forever.
    if state_valid && key_violated && phase == PHASE_AWAITING_KEY {
        let feed_ok = flags & FLAG_FEED_OK != 0;
        let timeout_ok = flags & FLAG_TIMEOUT_OK != 0;

        let mut on = false;
        loop {
            print_reasons(&mut tx, &reasons);

            tx.write_all(b"WDT_TEST_BEGIN\r\n").ok();
            tx.write_all(if feed_ok {
                b"WDT FEED OK\r\n" as &[u8]
            } else {
                b"WDT FEED FAIL\r\n"
            })
            .ok();
            tx.write_all(if timeout_ok {
                b"WDT TIMEOUT RESET OK\r\n" as &[u8]
            } else {
                b"WDT TIMEOUT RESET FAIL\r\n"
            })
            .ok();
            // Reaching this branch at all is the key-reset proof.
            tx.write_all(b"WDT KEY RESET OK\r\n").ok();
            tx.write_all(b"WDT_TEST_END\r\n").ok();

            if feed_ok && timeout_ok {
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

    // --- Boot 1: fresh start (flash, RST button, power cycle, or a state ----
    // mismatch above) — run the feed test, then starve the dog.
    write_state(&mut info, PHASE_FEEDING, 0);

    // ~1.05 s fuse: 2^23 cycles of SMCLK @ 8 MHz.
    let mut wdt = Watchdog::new(p.watchdog_timer);
    wdt.start(ClockSource::Smclk, Interval::Cycles8192K);

    // Feed every 250 ms for 3 s — three timeouts' worth. A broken feed
    // resets us mid-loop, which the next boot sees as FEEDING + timeout.
    tx.write_all(b"feeding a ~1 s watchdog for 3 s...\r\n").ok();
    for _ in 0..12 {
        delay.delay_ms(250);
        wdt.feed();
        green_led.set_high().ok();
        delay.delay_us(1000);
        green_led.set_low().ok();
    }

    // Survived. Now prove the dog actually bites: record the phase flip
    // *before* going quiet, then never feed again.
    write_state(&mut info, PHASE_STARVING, 0);
    tx.write_all(b"feed survived; starving the dog now...\r\n").ok();
    loop {
        delay.delay_ms(1000);
    }
}

/// Persist `{MAGIC, phase, flags}` at `STATE_OFFSET`.
fn write_state(info: &mut InfoFram, phase: u8, flags: u8) {
    let m = MAGIC.to_le_bytes();
    let state = [m[0], m[1], m[2], m[3], phase, flags];
    info.write(STATE_OFFSET, &state).ok();
}

/// Human-readable info line: `reasons: 0x04 (reset pin) 0x02 (brownout)`.
/// The host runner skips everything up to `WDT_TEST_BEGIN`.
fn print_reasons<W: hal::embedded_io::Write>(tx: &mut W, reasons: &ResetReasons) {
    tx.write_all(b"reasons:").ok();
    if reasons.is_empty() {
        tx.write_all(b" (none)").ok();
    }
    for &iv in reasons.raw() {
        let decoded = ResetReason::from_iv(iv).unwrap_or(ResetReason::Unknown(iv));
        tx.write_all(b" 0x").ok();
        write_hex_byte(tx, iv as u8);
        tx.write_all(b" (").ok();
        tx.write_all(decoded.as_str().as_bytes()).ok();
        tx.write_all(b")").ok();
    }
    tx.write_all(b"\r\n").ok();
}

/// Write one byte as two hex digits.
fn write_hex_byte<W: hal::embedded_io::Write>(tx: &mut W, byte: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let buf = [HEX[(byte >> 4) as usize], HEX[(byte & 0xF) as usize]];
    tx.write_all(&buf).ok();
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
