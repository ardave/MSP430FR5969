#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! RTC_B integration fixture for the `hal::rtc` calendar + alarm driver.
//!
//! A self-checking sibling of the human-facing `rtc_clock` demo: instead of just
//! printing the wall clock, it runs a startup self-check and reports a framed
//! pass/fail verdict over the UART backchannel (eUSCI_A0, 9600 8N1 on
//! `/dev/cu.usbmodem11203`), driven by the host-side `rtc_tests` runner. Like the
//! demo it needs no wiring beyond the LaunchPad — RTC_B is on-chip — but it does
//! need the populated 32.768 kHz LFXT crystal (see below).
//!
//! ```text
//! cargo +nightly build --bin rtc_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/rtc_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **Load-and-read-back (`RTC SET`).** The calendar is started at a fixed
//!    instant (Sat 2026-06-27 09:30:00). An immediate `now()` must read back that
//!    instant (seconds 0..=1, since the read races the very first tick). This
//!    proves the BCD-vs-binary mode and the held-register write path landed the
//!    value we asked for.
//!
//! 2. **It actually advances (`RTC TICK`).** We snapshot `now()`, busy-wait ~3 s
//!    on the **MCU `Delay` (MCLK/DCO)**, then snapshot again. The elapsed RTC
//!    seconds must be ~3 (tolerance 2..=4). Because the delay is timed by the DCO
//!    while the calendar is timed by the **independent crystal on ACLK**, a clock
//!    that is stuck, running off the wrong source, or not counting at 1 Hz fails
//!    here — the two oscillators cross-check each other.
//!
//! 3. **Alarm, polled (`RTC ALARM ARM/EARLY/FIRE/ONCE`).** The clock is re-set
//!    to 09:30:56 and a daily alarm programmed for 09:31 — four seconds out, the
//!    fixture picking the time so the minute-boundary-granular alarm fires
//!    seconds (not minutes) later. `ARM`: `set_alarm` accepts the alarm and
//!    leaves `RTCAIFG` clean. `EARLY`: ~1.5 s later (09:30:58) the flag is
//!    *still* clear — an alarm that latches immediately (e.g. comparing against
//!    the wrong fields, or a stale flag surviving `set_alarm`) fails here.
//!    `FIRE`: polling `alarm_irq_pending()` sees the flag latch within the ~8 s
//!    budget (expected ~2.5 s in, at the 09:31:00 minute increment). `ONCE`:
//!    after `clear_alarm_irq()` the flag stays clear through the rest of the
//!    matched minute — the hardware latches on the increment-into-match, not
//!    continuously while matched.
//!
//! 4. **Alarm interrupt + LPM3 wake (`RTC ALARM IRQ`).** The clock is re-set to
//!    09:31:56, the alarm to 09:32, `RTCAIE` enabled, and the part dropped into
//!    **LPM3** (DCO and SMCLK gated; only the crystal runs). The alarm is the
//!    only interrupt armed, so *returning at all* proves it woke the part; the
//!    `RTC` ISR (`#[interrupt(wake_cpu)]`) records `rtc::read_iv()`, which must
//!    be `0x06` (`RTCAIFG`'s slot per TI's `RTCIV_RTCAIFG`, hardware-observed
//!    2026-07-07) exactly once, and the IV read must have auto-cleared the
//!    flag. This is the flagship alarm use — wake from deep sleep at a
//!    wall-clock time — end to end.
//!
//! All verdicts are computed **once** at startup; the loop just re-emits the
//! fixed verdict lines and toggles the **GREEN** LED as a heartbeat. A steady
//! **RED** LED means a check failed. If the alarm interrupt never fires, the
//! part stays in LPM3 and no burst is ever emitted — the host runner's deadline
//! turns that hang into a clean failure (the same "reaching the verdict proves
//! it returned" logic as the deep-sleep fixture).
//!
//! # Hardware requirement: the 32.768 kHz crystal
//!
//! RTC_B counts the LFXT watch crystal on ACLK, so this fixture brings the clocks
//! up with [`hal::clocks::configure_low_power`], which starts LFXT. If the crystal
//! does not start (e.g. a bare chip), ACLK falls back to the imprecise VLO,
//! [`Rtc::new`] returns [`hal::rtc::Error::ClockNot32768`], and the fixture lights
//! the **RED** LED and emits a `RTC CLOCK FAIL` burst (a deliberate refusal the
//! host reports cleanly, rather than a clock that silently runs fast).
//!
//! # Framed output for the host runner
//!
//! Like the `fram_test` and `adc_internal` fixtures, this emits a self-delimited
//! burst once per second, forever, so the host test can attach at any time after
//! the `DSLite load` reset and still catch a complete cycle. Each cycle, over UART:
//!
//! ```text
//! now: 2026-06-27 09:32:07      (human-readable info, skipped by host)
//! alarm fire_ms=2600 iv=8 n=1   (human-readable info, skipped by host)
//! RTC_TEST_BEGIN
//! RTC SET OK                    (or `... FAIL`)
//! RTC TICK OK
//! RTC ALARM ARM OK
//! RTC ALARM EARLY OK
//! RTC ALARM FIRE OK
//! RTC ALARM ONCE OK
//! RTC ALARM IRQ OK
//! RTC_TEST_END
//! ```

use core::cell::Cell;

use critical_section::Mutex;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::rtc::{Alarm, DateTime, Rtc};
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// The #[msp430_rt::interrupt] macro validates the handler name against an
// `interrupt::NAME` path.
use hal::interrupt;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// The instant the calendar is started at: Sat 2026-06-27 09:30:00.
const START: DateTime = DateTime {
    year: 2026,
    month: 6,
    day: 27,
    weekday: 6,
    hour: 9,
    minute: 30,
    second: 0,
};

/// START's date at a different time of day — the alarm phases re-set the clock
/// to land just shy of a minute boundary so the minute-granular alarm fires
/// seconds later.
const fn start_date_at(hour: u8, minute: u8, second: u8) -> DateTime {
    DateTime { hour, minute, second, ..START }
}

/// What the `RTC` ISR observed: the last `RTCIV` value read, and how many
/// times the vector fired. Only the alarm interrupt is ever enabled, so one
/// firing with IV = 0x08 is the exactly-once pass state.
static ALARM_IV: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static ALARM_COUNT: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));

/// RTC interrupt: consume the IV (the RTCIV read auto-clears the served
/// flag — RTCAIFG here) and record it. `wake_cpu` clears the LPM bits in the
/// stacked SR so `enter_lpm3()` returns to main.
#[msp430_rt::interrupt(wake_cpu)]
fn RTC() {
    let iv = hal::rtc::read_iv();
    critical_section::with(|cs| {
        ALARM_IV.borrow(cs).set(iv);
        let count = ALARM_COUNT.borrow(cs);
        count.set(count.get().saturating_add(1));
    });
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Low-power profile: ACLK on the 32.768 kHz LFXT crystal — required for the
    // RTC to keep correct time. SMCLK = 1 MHz still feeds the UART BRCLK.
    let clocks = hal::clocks::configure_low_power(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 1 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, heartbeat
    let mut red_led = port4.pin6.into_output(); // LED1, failure flag

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 RTC_B self-check\r\n").ok();

    // Start the calendar; refuse (and report) if the crystal did not come up.
    let mut rtc = match Rtc::new(p.rtc_b_real_time_clock, &clocks, &START) {
        Ok(rtc) => rtc,
        Err(_) => {
            // No 32.768 kHz crystal — emit a framed FAIL burst forever so the host
            // gets a deterministic verdict instead of a timeout, and light RED.
            red_led.set_high().ok();
            loop {
                tx.write_all(b"RTC_TEST_BEGIN\r\n").ok();
                tx.write_all(b"RTC CLOCK FAIL\r\n").ok();
                tx.write_all(b"RTC_TEST_END\r\n").ok();
                delay.delay_ms(1000);
            }
        }
    };

    // --- 1. Load-and-read-back ----------------------------------------------
    // An immediate read should match START; seconds may have ticked to 1 if the
    // read lands just after the first crystal-driven update.
    let first = rtc.now();
    let set_ok = first.year == START.year
        && first.month == START.month
        && first.day == START.day
        && first.hour == START.hour
        && first.minute == START.minute
        && first.second <= 1;

    // --- 2. It actually advances --------------------------------------------
    // Cross-check the crystal (ACLK) against the DCO-timed MCU delay: wait ~3 s
    // and confirm the calendar advanced ~3 s. mod-60 handles a minute rollover.
    let before = rtc.now();
    delay.delay_ms(3000);
    let after = rtc.now();
    let elapsed = (after.second as i16 - before.second as i16).rem_euclid(60);
    let tick_ok = (2..=4).contains(&elapsed);

    // --- 3. Alarm, polled ----------------------------------------------------
    // The alarm compares at each *minute* increment, so a naive test would wait
    // up to a minute; instead re-set the clock to 4 s before the boundary the
    // alarm names. Daily alarm (hour + minute enabled) so a stuck wildcard
    // can't pass by matching some other hour.
    rtc.set(&start_date_at(9, 30, 56));
    let arm_ok = rtc.set_alarm(&Alarm::daily_at(9, 31)).is_ok()
        && !hal::rtc::alarm_irq_pending();

    // ~1.5 s in (09:30:58) the boundary has not been reached: the flag must
    // still be clear. Catches an alarm latched by the arming itself.
    delay.delay_ms(1500);
    let early_ok = !hal::rtc::alarm_irq_pending();

    // Poll for the latch. Expected ~2.5 s in (the 09:31:00 increment); the 8 s
    // budget is deliberately generous — FIRE checks *that* it latches, EARLY
    // already checked it wasn't instant, and TICK already timed the crystal.
    let mut fire_ms: u32 = 0;
    let mut fired = false;
    while fire_ms < 8000 {
        delay.delay_ms(100);
        fire_ms += 100;
        if hal::rtc::alarm_irq_pending() {
            fired = true;
            break;
        }
    }
    let fire_ok = fired;

    // Exactly-once: the latch happens on the increment *into* the match, not
    // continuously while the minute matches. Cleared, it must stay clear for
    // the rest of the matched minute.
    hal::rtc::clear_alarm_irq();
    delay.delay_ms(2500);
    let once_ok = fired && !hal::rtc::alarm_irq_pending();

    // --- 4. Alarm interrupt + LPM3 wake --------------------------------------
    // Same trick, next boundary: alarm at 09:32 with the clock at 09:31:56,
    // this time delivered as an interrupt to a sleeping part. The alarm is the
    // only enabled interrupt, so returning from enter_lpm3() at all proves the
    // wake; the ISR's IV must be RTCAIFG's slot (0x06), exactly once.
    rtc.set(&start_date_at(9, 31, 56));
    rtc.set_alarm(&Alarm::daily_at(9, 32)).ok();
    rtc.enable_alarm_interrupt();
    // SMCLK stops in LPM3 — flush so the sleep doesn't cut a character short.
    tx.flush().ok();
    hal::power::enter_lpm3();
    let (irq_iv, irq_count) =
        critical_section::with(|cs| (ALARM_IV.borrow(cs).get(), ALARM_COUNT.borrow(cs).get()));
    // 0x06 = RTCIV_RTCAIFG (TI msp430fr5969.h; first HW-observed 2026-07-07 —
    // the run that corrected the driver's earlier off-by-one-slot IV table).
    // The IV read in the ISR auto-clears RTCAIFG; a flag still pending here
    // would mean the demux read didn't consume it.
    let irq_ok = irq_iv == 0x06 && irq_count == 1 && !hal::rtc::alarm_irq_pending();
    // Disarm so the daily alarm can't refire into the report loop.
    rtc.disable_alarm();

    let all_ok = set_ok && tick_ok && arm_ok && early_ok && fire_ok && once_ok && irq_ok;

    // A self-delimited verdict burst, repeated once per second so the host runner
    // can attach at any time after the DSLite reset and still frame a full
    // BEGIN..END cycle. GREEN toggles each cycle as a heartbeat; steady RED means
    // a check failed.
    let mut on = false;
    loop {
        // Human-readable info lines (the host skips everything up to BEGIN): the
        // live wall clock and the alarm observations, observable over `screen`.
        let now = rtc.now();
        tx.write_all(b"now: ").ok();
        print_datetime(&mut tx, &now);
        tx.write_all(b"alarm fire_ms=").ok();
        write_dec(&mut tx, fire_ms);
        tx.write_all(b" iv=").ok();
        write_dec(&mut tx, irq_iv as u32);
        tx.write_all(b" n=").ok();
        write_dec(&mut tx, irq_count as u32);
        tx.write_all(b"\r\n").ok();

        // Fixed, greppable verdict lines framed by BEGIN/END.
        tx.write_all(b"RTC_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"RTC SET", set_ok);
        verdict(&mut tx, b"RTC TICK", tick_ok);
        verdict(&mut tx, b"RTC ALARM ARM", arm_ok);
        verdict(&mut tx, b"RTC ALARM EARLY", early_ok);
        verdict(&mut tx, b"RTC ALARM FIRE", fire_ok);
        verdict(&mut tx, b"RTC ALARM ONCE", once_ok);
        verdict(&mut tx, b"RTC ALARM IRQ", irq_ok);
        tx.write_all(b"RTC_TEST_END\r\n").ok();

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

/// Write one `<name> OK\r\n` / `<name> FAIL\r\n` verdict line.
fn verdict<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], ok: bool) {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" as &[u8] } else { b" FAIL\r\n" }).ok();
}

/// Print `YYYY-MM-DD HH:MM:SS\r\n`.
fn print_datetime<W: hal::embedded_io::Write>(tx: &mut W, dt: &DateTime) {
    write_dec(tx, dt.year as u32);
    tx.write_all(b"-").ok();
    write_two(tx, dt.month);
    tx.write_all(b"-").ok();
    write_two(tx, dt.day);
    tx.write_all(b" ").ok();
    write_two(tx, dt.hour);
    tx.write_all(b":").ok();
    write_two(tx, dt.minute);
    tx.write_all(b":").ok();
    write_two(tx, dt.second);
    tx.write_all(b"\r\n").ok();
}

/// Write a value `0..=99` as exactly two zero-padded decimal digits.
fn write_two<W: hal::embedded_io::Write>(tx: &mut W, value: u8) {
    let buf = [b'0' + (value / 10) % 10, b'0' + value % 10];
    tx.write_all(&buf).ok();
}

/// Write an unsigned value as decimal ASCII (no padding); for the year.
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
