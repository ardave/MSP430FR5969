#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits handlers with the `extern "msp430-interrupt"`
// ABI (RETI, not RET) — still nightly-gated.
#![feature(abi_msp430_interrupt)]

//! LPMx.5 integration fixture: `power::enter_lpm4_5` / `enter_lpm3_5`, the
//! BOR-reset wake path, and what survives the dark interval.
//!
//! LPMx.5 turns the core regulator off, so "waking" is a reboot: every stage
//! of this test is a **separate life of `main`**, chained through a little
//! state machine persisted in Info FRAM (which, being nonvolatile, is exactly
//! the LPMx.5 state-keeping idiom the test also demonstrates). The host-side
//! `lpmx5_tests` module drives it over the UART backchannel (eUSCI_A0,
//! 9600 8N1) — including *waking the board*: the UART RX line is physically
//! P2.1, and a start bit is a falling edge, so the host wakes LPM4.5 by
//! sending a byte at a pin armed as a GPIO wake source. The byte itself is
//! sacrificial (it lands on unpowered silicon); only its edge matters.
//!
//! ```text
//! Life 1 (cold boot, any reset that is not an LPMx.5 wake):
//!   print LPMX5_READY until the host sends a go-byte
//!   state := AwaitPin; arm P2.1 (pull-up, falling); enter LPM4.5
//! Life 2 (host's wake byte -> BOR reset, SYSRSTIV = LPM5WU):
//!   re-arm P2.1 BEFORE clearing LOCKLPM5 -> latched wake P2IFG.1 delivered
//!   checks: cause == Lpm5WakeUp, P2IFG.1 pending          -> PIN verdict
//!   state := AwaitRtc; RTC := 00:00:55, minute-event wake; enter LPM3.5
//! Life 3 (~5 s later, RTC minute rollover -> BOR reset):
//!   checks: cause == Lpm5WakeUp, RTCIV == 0x06 (RTCTEVIFG survived),
//!           Rtc::attach + now() reads ~00:01:00 (calendar kept counting
//!           through the powered-off interval)                -> RTC/TIME
//!   state := Done; emit the framed verdict burst once per second forever
//! ```
//!
//! Verdict bits ride in FRAM too (earlier lives learn things the last life
//! must report). A reflash or reset-button press reads as a cold boot (the
//! debug probe's reset latches `ResetPin`), so the machine restarts cleanly
//! from any state.
//!
//! # Framed output for the host runner
//!
//! ```text
//! LPMX5_READY                (life 1, repeating until go-byte)
//! LPMX5_SLEEPING mode=4.5    (life 1, last words before power-off)
//! LPMX5_SLEEPING mode=3.5    (life 2)
//! LPMX5_TEST_BEGIN           (life 3+, once per second)
//! LPMX5 PIN OK               (or FAIL: pin-edge wake + latched-IFG delivery)
//! LPMX5 RTC OK               (or FAIL: RTC-event wake + RTCTEVIFG survival)
//! LPMX5 TIME OK              (or FAIL: calendar counted through power-off)
//! LPMX5_TEST_END
//! ```
//!
//! Info lines (`lpmx5 ...`) are human-readable diagnostics the host skips.

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_hal_nb::serial::Read as _;
use hal::embedded_io::Write as _;
use hal::embedded_storage::{ReadStorage as _, Storage as _};
use hal::fram::InfoFram;
use hal::gpio::{Edge, GpioExt};
use hal::interrupt;
use hal::rtc::{DateTime, Rtc};
use hal::serial::{Config as UartConfig, SerialExt};
use hal::sys::{ResetReason, ResetReasons};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Where the cross-reboot state lives in Info FRAM. Offset chosen away from 0
/// so a half-written buffer from some other fixture is unlikely to alias the
/// magic; the cold path re-initializes everything regardless.
const FRAM_OFFSET: u32 = 0x60;

/// State record: [magic0, magic1, state, flags]. The magic guards against
/// adopting garbage after a reflash; `flags` carries verdict bits forward
/// between lives (bit 0 = PIN, bit 1 = RTC, bit 2 = TIME).
const MAGIC: [u8; 2] = [b'L', b'5'];
const STATE_AWAIT_PIN: u8 = 1;
const STATE_AWAIT_RTC: u8 = 2;
const STATE_DONE: u8 = 3;

const FLAG_PIN: u8 = 1 << 0;
const FLAG_RTC: u8 = 1 << 1;
const FLAG_TIME: u8 = 1 << 2;

/// Pre-entry race insurance (see `power::enter_lpm3_5` docs): an event firing
/// between arming and the entry `bis` is serviced as a normal interrupt. These
/// handlers just consume the source so that path lands somewhere defined
/// instead of in `DefaultHandler`; the test's timing makes them dead code.
#[msp430_rt::interrupt]
fn PORT2() {
    let _ = hal::gpio::read_iv::<hal::gpio::P2>();
}

#[msp430_rt::interrupt]
fn RTC() {
    let _ = hal::rtc::read_iv();
}

/// Firmware entry point — runs once per *life* (cold boot or LPMx.5 wake).
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Why did this life begin? Drained once, destructively, before anything
    // else can touch it. A genuine LPMx.5 wake reports Lpm5WakeUp; a reflash
    // or reset-button press latches ResetPin (and a power cycle Brownout), so
    // those force the cold path even if stale FRAM state says otherwise.
    let reasons = ResetReasons::drain(&p.sys);
    let lpm5_wake = reasons.contains(ResetReason::Lpm5WakeUp)
        && !reasons.contains(ResetReason::ResetPin)
        && !reasons.contains(ResetReason::Brownout);

    // Load the cross-reboot state early so the wake path can order its pin
    // work correctly (see below).
    let mut fram = InfoFram::new();
    let mut record = [0u8; 4];
    fram.read(FRAM_OFFSET, &mut record).ok();
    let valid = record[0] == MAGIC[0] && record[1] == MAGIC[1];
    let state = if lpm5_wake && valid { record[2] } else { 0 };
    let mut flags = if valid { record[3] } else { 0 };

    // Ports split up front: the wake path needs P2.1 reconfigured *before*
    // LOCKLPM5 is cleared, or the latched wake IFG evaporates un-observed.
    let (port1, port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut wake_pin = port2.pin1.into_pull_up_input(); // P2.1 = UCA0RXD's pad
    if state == STATE_AWAIT_PIN {
        // Re-arm with the same edge as before sleeping: with PxIE set at the
        // moment LOCKLPM5 clears, the pin that woke us presents its IFG.
        wake_pin.enable_interrupt(Edge::Falling);
    }

    // Clock tree: ACLK on LFXT for the RTC stages. Across an LPM3.5 wake the
    // crystal never stopped (its PJ.4/PJ.5 mux is latched, and the RTC domain
    // kept it powered), so the fault-flag settle loop passes almost at once.
    let clocks = hal::clocks::configure_low_power(p.cs);

    // Unlock the I/O latch. On a cold boot this is the usual "make the pin
    // muxes take effect"; on an LPMx.5 wake it is the moment the latched pins
    // hand control back to the (now reconfigured) port registers — and the
    // moment the wake pin's IFG becomes visible.
    hal::gpio::unlock_pins(&p.pmm);

    // Capture the wake evidence immediately, then stand down the GPIO wake so
    // UART traffic on the same pad cannot latch stray flags.
    let pin_pending = wake_pin.interrupt_pending();
    wake_pin.disable_interrupt();
    wake_pin.clear_interrupt_pending();

    // UART up (this re-muxes P2.0/P2.1 to eUSCI_A0 — the GPIO wake pin reverts
    // to being the RX line). 9600 8N1, BRCLK = SMCLK = 1 MHz here.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, mut rx) = serial.split();

    let mut green_led = port1.pin0.into_output(); // LED2: pass/heartbeat
    let mut red_led = port4.pin6.into_output(); // LED1: failure
    let mut delay = Delay::new(clocks.mclk());

    // Boot diagnostics (host skips `lpmx5 ...` lines).
    tx.write_all(b"\r\nlpmx5 boot state=").ok();
    write_dec(&mut tx, state as u32);
    tx.write_all(b" causes=").ok();
    for (i, r) in reasons.iter().enumerate() {
        if i > 0 {
            tx.write_all(b",").ok();
        }
        tx.write_all(r.as_str().as_bytes()).ok();
    }
    tx.write_all(b"\r\n").ok();

    match state {
        STATE_AWAIT_PIN => {
            // ---- Life 2: the host's UART byte woke us out of LPM4.5 --------
            if lpm5_wake && pin_pending {
                flags |= FLAG_PIN;
            }
            tx.write_all(b"lpmx5 wake1 pin_pending=").ok();
            write_dec(&mut tx, pin_pending as u32);
            tx.write_all(b"\r\n").ok();

            // Stage the RTC test: LFXT must actually be the crystal.
            if clocks.aclk_source() != hal::clocks::AclkSource::Lfxt {
                fail_forever(&mut tx, &mut red_led, &mut delay, b"lfxt");
            }

            // Seconds start at 55 so the minute-changed event — the fastest
            // calendar event RTC_B offers — arrives ~5 s after entry.
            let start = DateTime {
                year: 2026,
                month: 7,
                day: 4,
                weekday: 6,
                hour: 0,
                minute: 0,
                second: 55,
            };
            let rtc = Rtc::new(p.rtc_b_real_time_clock, &clocks, &start).unwrap();
            rtc.enable_event_interrupt(hal::rtc::Event::MinuteChanged);

            save_state(&mut fram, STATE_AWAIT_RTC, flags);
            tx.write_all(b"LPMX5_SLEEPING mode=3.5\r\n").ok();
            tx.flush().ok();
            hal::power::enter_lpm3_5(&p.pmm);
        }

        STATE_AWAIT_RTC => {
            // ---- Life 3: the RTC minute event woke us out of LPM3.5 --------
            // RTCTEVIFG lives in the RTC's always-on domain and is the wake's
            // receipt: still latched after the reboot. (Its *enable* bit is
            // not — the wake clears RTC interrupt enables, which is why this
            // reads the flag directly instead of the IE-masked RTCIV.)
            let tev_pending = hal::rtc::event_irq_pending();
            if lpm5_wake && tev_pending {
                flags |= FLAG_RTC;
            }

            // The calendar must have kept counting while the core was dark:
            // we slept at 00:00:55, so the minute event that woke us pinned
            // it at exactly 00:01:00 — where the wake froze it (RTCHOLD) for
            // `attach` to release. Allow a little slack past :00 in case the
            // boot path ever slows down after a future attach-earlier change.
            let time_ok = match Rtc::attach(p.rtc_b_real_time_clock, &clocks) {
                Ok(rtc) => {
                    let now = rtc.now();
                    tx.write_all(b"lpmx5 wake2 tev=").ok();
                    write_dec(&mut tx, tev_pending as u32);
                    tx.write_all(b" rtc=").ok();
                    write_dec(&mut tx, now.hour as u32);
                    tx.write_all(b":").ok();
                    write_dec(&mut tx, now.minute as u32);
                    tx.write_all(b":").ok();
                    write_dec(&mut tx, now.second as u32);
                    tx.write_all(b"\r\n").ok();
                    now.hour == 0 && now.minute == 1 && now.second <= 5
                }
                Err(_) => {
                    tx.write_all(b"lpmx5 attach=clock\r\n").ok();
                    false
                }
            };
            if time_ok {
                flags |= FLAG_TIME;
            }

            save_state(&mut fram, STATE_DONE, flags);
            report_forever(&mut tx, flags, &mut green_led, &mut red_led, &mut delay);
        }

        STATE_DONE => {
            // A DONE-state LPMx.5 wake shouldn't happen (nothing is armed),
            // but if the host re-attaches mid-run, keep reporting the verdict.
            report_forever(&mut tx, flags, &mut green_led, &mut red_led, &mut delay);
        }

        _ => {
            // ---- Life 1: cold boot ------------------------------------------
            // Handshake so no protocol line is emitted before the host has the
            // port open (the eZ-FET gates TX on DTR; early lines are lost).
            loop {
                tx.write_all(b"LPMX5_READY\r\n").ok();
                let mut got_byte = false;
                for _ in 0..50 {
                    if rx.read().is_ok() {
                        got_byte = true;
                        break;
                    }
                    delay.delay_ms(10);
                }
                if got_byte {
                    break;
                }
            }

            save_state(&mut fram, STATE_AWAIT_PIN, 0);
            tx.write_all(b"LPMX5_SLEEPING mode=4.5\r\n").ok();
            tx.flush().ok();

            // Hand the RX pad back to GPIO and arm it: the next start bit the
            // host sends is a falling edge on a powered-down chip — the wake.
            let mut wp = wake_pin.into_pull_up_input();
            wp.enable_interrupt(Edge::Falling);
            hal::power::enter_lpm4_5(&p.pmm);
        }
    }
}

/// Persist [magic, state, flags] to Info FRAM.
fn save_state(fram: &mut InfoFram, state: u8, flags: u8) {
    let record = [MAGIC[0], MAGIC[1], state, flags];
    fram.write(FRAM_OFFSET, &record).ok();
}

/// Emit the framed verdict burst once per second forever, with the usual LED
/// convention (GREEN heartbeat on full pass, solid RED otherwise).
fn report_forever<W, G, R>(
    tx: &mut W,
    flags: u8,
    green_led: &mut G,
    red_led: &mut R,
    delay: &mut Delay,
) -> !
where
    W: hal::embedded_io::Write,
    G: OutputPin,
    R: OutputPin,
{
    let all_ok = flags & (FLAG_PIN | FLAG_RTC | FLAG_TIME) == (FLAG_PIN | FLAG_RTC | FLAG_TIME);
    let mut on = false;
    loop {
        tx.write_all(b"LPMX5_TEST_BEGIN\r\n").ok();
        tx.write_all(if flags & FLAG_PIN != 0 {
            b"LPMX5 PIN OK\r\n" as &[u8]
        } else {
            b"LPMX5 PIN FAIL\r\n"
        })
        .ok();
        tx.write_all(if flags & FLAG_RTC != 0 {
            b"LPMX5 RTC OK\r\n" as &[u8]
        } else {
            b"LPMX5 RTC FAIL\r\n"
        })
        .ok();
        tx.write_all(if flags & FLAG_TIME != 0 {
            b"LPMX5 TIME OK\r\n" as &[u8]
        } else {
            b"LPMX5 TIME FAIL\r\n"
        })
        .ok();
        tx.write_all(b"LPMX5_TEST_END\r\n").ok();

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

/// Refuse to continue: the fixture's own preconditions failed (not the thing
/// under test). Solid RED, one `lpmx5 fail ...` line per second.
fn fail_forever<W, R>(tx: &mut W, red_led: &mut R, delay: &mut Delay, what: &[u8]) -> !
where
    W: hal::embedded_io::Write,
    R: OutputPin,
{
    red_led.set_high().ok();
    loop {
        tx.write_all(b"lpmx5 fail ").ok();
        tx.write_all(what).ok();
        tx.write_all(b"\r\n").ok();
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
