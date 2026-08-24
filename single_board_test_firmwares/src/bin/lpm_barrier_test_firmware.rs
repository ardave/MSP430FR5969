#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! LPM sleep compiler-barrier fixture: `power::enter_lpm0` must act as a
//! compiler barrier, because an ISR mutates memory *inside* the sleep.
//!
//! `enter_lpm0()` is documented to "return once an interrupt has woken the
//! CPU" — i.e. between the `bis` that sleeps and the instruction after it, a
//! whole ISR runs and writes memory. The compiler must therefore treat the
//! sleep as a point where memory can change: a value loaded *before* the sleep
//! may not be reused *after* it. If the sleep asm is marked `options(nomem)`,
//! the optimizer is told the opposite — and it may legally hoist a flag load
//! out of a sleep loop or CSE it with a pre-sleep load, so post-wake code
//! checks a stale register copy of a flag the ISR has already set.
//!
//! This fixture pins the barrier property *behaviorally*, on silicon: a flag
//! set by a wake ISR and checked after each wake. Both probes are deliberately
//! shaped so nothing between the sleep and the flag read is a call or critical
//! section (either would be its own barrier and mask the bug): the entries are
//! `#[inline]` over the HAL's single shared sleep asm site (`sleep_bis`,
//! `#[inline(always)]`), the flag is a plain (non-volatile — a volatile read
//! can never be reused, so it too would mask the bug) load from an
//! `UnsafeCell` static, and the loop bodies are otherwise register-only
//! arithmetic. That flag is sound by the same argument as
//! `critical_section::Mutex<Cell>` itself: on a single core the accesses are
//! temporally exclusive (main touches it with GIE off, or after the ISR that
//! woke it has fully returned), and the compiled code respects that order
//! exactly when the sleep is the compiler barrier it must be — which is the
//! property under test. The wake source is the WDT interval metronome, which
//! re-fires without any in-loop re-arm — re-arming a one-shot would be an
//! out-of-crate call, i.e. a barrier.
//!
//! The probe pair runs **twice**, through two different entries and wake
//! clocks: `enter_lpm0` woken by the metronome on SMCLK (8 MHz/8192 ≈ 1 ms —
//! SMCLK stays alive in LPM0) and `enter_lpm3` woken by it on ACLK (VLO under
//! the default profile, ~9.4 kHz/512 ≈ 54 ms — ACLK is precisely what LPM3
//! keeps running). The probes are expanded by macro, not shared through a
//! function, so no call boundary can mask a missing barrier. Since the HAL
//! routes every LPM entry through the one `sleep_bis` asm block, these two
//! rounds pin the options on the block that `enter_lpm4` and the LPMx.5
//! entries also execute; LPM4 has no hands-free scheduled wake (only async
//! pin edges — covered as a *wake* by `button_wake` and the two-board
//! `lpm4_wake`), and the x.5 entries never return, so a stale post-sleep load
//! cannot exist for them by construction (their reboot semantics are the
//! `lpmx5` fixture's business).
//!
//! # What it checks
//!
//! 1. **Sleep loop sees the ISR's flag (`BARRIER LPM0/LPM3 LOOP`).** With
//!    interrupts off, clear the flag, then sleep in a bounded loop, checking
//!    the flag after each wake. The only wake source is the WDT ISR, which
//!    sets the flag — so a correct build breaks out on wake 1. A build whose
//!    optimizer hoisted the load out of the loop spins through all 8 wakes on
//!    the stale `false` and fails.
//!
//! 2. **Pre-sleep load is not reused post-wake (`BARRIER LPM0/LPM3
//!    RELOAD`).** With interrupts off, clear the flag, load it (`false`),
//!    sleep once, load it again. The wake that returned control *is* the ISR
//!    that set the flag, so the second load must see `true`. Under `nomem`
//!    the optimizer may CSE the second load into the first (no visible memory
//!    write between them) and report the pre-sleep `false`.
//!
//! 3. **The metronome really fired (`BARRIER ISR`).** The ISR also tallies into
//!    a `Mutex<Cell>`; the tally must be nonzero. This separates "compiler
//!    reused a stale load" (probes fail, tally climbs) from "interrupts dead"
//!    (the fixture would hang before the burst — reaching it at all proves the
//!    wakes happened, like the deep-sleep fixture's WAKE verdict).
//!
//! All verdicts are computed **once** at startup; the loop re-emits the fixed
//! verdict burst once per second, GREEN toggling as a heartbeat, steady RED on
//! failure. No wiring — WDT_A, the VLO, and the CPU are on-chip.
//!
//! # Framed output for the host runner
//!
//! ```text
//! barrier w0=1 w3=1 fires=4 p0=0/1 p3=0/1  (info line, skipped by host)
//! BARRIER_TEST_BEGIN
//! BARRIER LPM0 LOOP OK                     (or `... FAIL`)
//! BARRIER LPM0 RELOAD OK
//! BARRIER LPM3 LOOP OK
//! BARRIER LPM3 RELOAD OK
//! BARRIER ISR OK
//! BARRIER_TEST_END
//! ```

use core::cell::{Cell, UnsafeCell};

use critical_section::Mutex;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::watchdog::{ClockSource, Interval, Watchdog};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take() (and so msp430::interrupt::disable() is available).
use msp430 as _;

/// Set by the WDT interval ISR; checked by main after each wake **without** a
/// critical section — the `Mutex` would wrap the read in DINT/EINT asm whose
/// own barrier would mask the property under test (and this target has no
/// `core` atomics). Plain accesses to an `UnsafeCell` static instead.
///
/// SAFETY (same argument as `critical_section::Mutex<Cell>`, minus the
/// `with`): on a single core the accesses never overlap in time — main reads
/// or writes it only while GIE is off, or after the wake ISR that set it has
/// fully returned. The compiled code respects that temporal order exactly when
/// `enter_lpm0` is the compiler barrier its contract requires; that this holds
/// is what the fixture exists to verify.
struct IsrFlag(UnsafeCell<bool>);
// SAFETY: see above — single-core temporal exclusion, ordered by the sleep
// barrier under test.
unsafe impl Sync for IsrFlag {}

impl IsrFlag {
    /// Plain (non-volatile, non-atomic) read — deliberately reusable by the
    /// optimizer, so a missing sleep barrier is observable.
    fn get(&self) -> bool {
        unsafe { *self.0.get() }
    }

    fn set(&self, value: bool) {
        unsafe { *self.0.get() = value }
    }
}

static WOKE: IsrFlag = IsrFlag(UnsafeCell::new(false));

/// ISR fire tally (control verdict): proves the metronome ran even if the
/// probes fail, so a barrier regression is distinguishable from dead wakes.
static FIRES: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// Maximum wakes probe 1 will sleep through before giving up on the flag.
const MAX_WAKES: u16 = 8;

/// Expand the LOOP + RELOAD probe pair around the given sleep entry.
///
/// A macro, not a function: a shared `fn` would put a call boundary — its own
/// compiler barrier — between the sleep and the flag reads, masking exactly
/// the property under test. Evaluates to `(loop_ok, reload_ok, wakes, pre,
/// post)`.
///
/// Interrupts are disabled up front (probe 1's clear-then-sleep must be
/// race-free, and round 2 starts with GIE still on from round 1's last wake);
/// the sleep entry itself re-enables GIE atomically with sleeping.
macro_rules! probe_pair {
    ($sleep:path) => {{
        // --- Probe 1: sleep loop must observe the wake ISR's flag ----------
        // The loop body is exactly: inlined sleep asm, one plain load,
        // register arithmetic — nothing else, so nothing but the sleep itself
        // can force the load to be redone per iteration.
        msp430::interrupt::disable();
        WOKE.set(false);
        let mut wakes: u16 = 0;
        let loop_ok = loop {
            $sleep();
            wakes += 1;
            if WOKE.get() {
                break true;
            }
            if wakes >= MAX_WAKES {
                break false;
            }
        };

        // --- Probe 2: a pre-sleep load must not be reused after the wake ---
        // The wake that lets this code continue is the ISR that set the flag,
        // so the post-wake load must read `true` — unless the optimizer
        // folded it into the pre-sleep load.
        msp430::interrupt::disable();
        WOKE.set(false);
        let pre = WOKE.get();
        $sleep();
        let post = WOKE.get();
        (loop_ok, !pre && post, wakes, pre, post)
    }};
}

/// WDT interval ISR: set the flag, count the fire. The dedicated `WDT`
/// vector's service auto-resets `WDTIFG` in hardware, so no flag work.
#[msp430_rt::interrupt(wake_cpu)]
fn WDT() {
    WOKE.set(true);
    critical_section::with(|cs| {
        let fires = FIRES.borrow(cs);
        fires.set(fires.get().saturating_add(1));
    });
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the boot watchdog fuse and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Default profile: MCLK = 1 MHz, SMCLK = 8 MHz.
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1 on the backchannel.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, heartbeat
    let mut red_led = port4.pin6.into_output(); // LED1, failure flag

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 LPM sleep barrier self-check\r\n")
        .ok();

    // --- Round 1: LPM0, WDT metronome on SMCLK (8 MHz/8192 ≈ 1.02 ms) ------
    // The interval re-fires on its own, so the probe loops need no in-loop
    // re-arm (which would be a call, i.e. a barrier that masks the property
    // under test). GIE is still off here — nothing fires until the first
    // sleep; each wake IS a WDT ISR having run, so a correct build sees the
    // flag on wake 1.
    let mut wdt = Watchdog::new(p.watchdog_timer);
    wdt.start_interval(ClockSource::Smclk, Interval::Cycles8192);
    wdt.enable_interval_interrupt();

    let (loop0_ok, reload0_ok, wakes0, pre0, post0) = probe_pair!(hal::power::enter_lpm0);

    // --- Round 2: LPM3, WDT metronome on ACLK (VLO ~9.4 kHz/512 ≈ 54 ms) ---
    // LPM3 gates SMCLK and the DCO; ACLK is what survives, so the same
    // metronome re-sourced from ACLK is the LPM3-capable wake
    // (`start_interval` rewrites WDTCTL whole, counter cleared).
    wdt.start_interval(ClockSource::Aclk, Interval::Cycles512);

    let (loop3_ok, reload3_ok, wakes3, pre3, post3) = probe_pair!(hal::power::enter_lpm3);

    // Probes done: stop the metronome (GIE is on again after the last wake).
    wdt.disable_interval_interrupt();
    wdt.stop();

    // --- Control: the metronome demonstrably fired --------------------------
    let fires = critical_section::with(|cs| FIRES.borrow(cs).get());
    let isr_ok = fires > 0;

    let all_ok = loop0_ok && reload0_ok && loop3_ok && reload3_ok && isr_ok;

    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"barrier w0=").ok();
        write_dec(&mut tx, wakes0 as u32);
        tx.write_all(b" w3=").ok();
        write_dec(&mut tx, wakes3 as u32);
        tx.write_all(b" fires=").ok();
        write_dec(&mut tx, fires as u32);
        tx.write_all(b" p0=").ok();
        write_dec(&mut tx, pre0 as u32);
        tx.write_all(b"/").ok();
        write_dec(&mut tx, post0 as u32);
        tx.write_all(b" p3=").ok();
        write_dec(&mut tx, pre3 as u32);
        tx.write_all(b"/").ok();
        write_dec(&mut tx, post3 as u32);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"BARRIER_TEST_BEGIN\r\n").ok();
        tx.write_all(if loop0_ok {
            b"BARRIER LPM0 LOOP OK\r\n" as &[u8]
        } else {
            b"BARRIER LPM0 LOOP FAIL\r\n"
        })
        .ok();
        tx.write_all(if reload0_ok {
            b"BARRIER LPM0 RELOAD OK\r\n" as &[u8]
        } else {
            b"BARRIER LPM0 RELOAD FAIL\r\n"
        })
        .ok();
        tx.write_all(if loop3_ok {
            b"BARRIER LPM3 LOOP OK\r\n" as &[u8]
        } else {
            b"BARRIER LPM3 LOOP FAIL\r\n"
        })
        .ok();
        tx.write_all(if reload3_ok {
            b"BARRIER LPM3 RELOAD OK\r\n" as &[u8]
        } else {
            b"BARRIER LPM3 RELOAD FAIL\r\n"
        })
        .ok();
        tx.write_all(if isr_ok {
            b"BARRIER ISR OK\r\n" as &[u8]
        } else {
            b"BARRIER ISR FAIL\r\n"
        })
        .ok();
        tx.write_all(b"BARRIER_TEST_END\r\n").ok();

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
