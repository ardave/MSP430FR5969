#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! RTC_B **prescaler tick** fixture: sub-second, crystal-accurate periodic
//! interrupts from the RTC's own divider chain (`RT0PS`/`RT1PS`, `RTxIP`
//! interval taps), demuxed on the shared `RTC` vector and waking the part
//! from **LPM3**. Reports a framed pass/fail verdict over the UART
//! backchannel (eUSCI_A0, 9600 8N1), driven by the host-side `rtc_tick_tests`
//! runner. **No wiring** — but, like every RTC fixture, it **requires the
//! 32.768 kHz LFXT crystal** (populated on the LaunchPad): the prescalers
//! divide ACLK's crystal, which is exactly why their ticks survive LPM3.
//!
//! ```text
//! cargo +nightly build --bin rtc_tick_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/rtc_tick_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **`RTC TICK RATE`** — 32 ticks at 128 Hz (RT0PS's slowest tap)
//!    wall-clocked by the SMCLK-driven [`Counter`]: 250 ms nominal, gated
//!    ±5%. The tick source is the crystal and the yardstick is the DCO —
//!    the same two-clocks-cross-check the calendar fixture uses at 1 Hz,
//!    here at sub-second resolution.
//! 2. **`RTC TICK IV0`** — every firing in the rate phase reported
//!    `RTCIV` = **0x08** (`RTCIV_RT0PSIFG`); nothing else fired.
//! 3. **`RTC TICK IV1`** — 64 Hz from the *other* bank (RT1PS's fastest
//!    tap) fires ≥ 8 times, every one reporting **0x0A**
//!    (`RTCIV_RT1PSIFG`), with the RT0PS tally frozen.
//! 4. **`RTC TICK BOTH`** — both banks armed concurrently (128 + 64 Hz),
//!    both tallies advance: the prescalers are independent.
//! 5. **`RTC TICK WAKE`** — a 32 Hz tick armed with GIE off, then
//!    `enter_lpm3()`: the tick (≤ 31.25 ms away, with MCLK/SMCLK/DCO all
//!    stopped) wakes the part. The ISR disarms via
//!    `rtc::isr_disable_tick_interrupts()`, and the tally holds at exactly
//!    **one** across a further 100 ms — the prescaler re-latches every
//!    period, so a keep-armed one-shot would keep firing.
//! 6. **`RTC TICK STOP`** — 50 ms after everything is disarmed, no tally
//!    has moved: disables really disable, no flag refires.
//!
//! All verdicts are computed **once** at startup; the loop re-emits the
//! fixed verdict burst once per second, GREEN toggling as a heartbeat,
//! steady RED on failure.
//!
//! # Framed output for the host runner
//!
//! ```text
//! rtc tick c0=37 c1=14 other=0 rate_us=250122   (info, skipped by host)
//! RTC_TICK_TEST_BEGIN
//! RTC TICK RATE OK
//! RTC TICK IV0 OK
//! RTC TICK IV1 OK
//! RTC TICK BOTH OK
//! RTC TICK WAKE OK
//! RTC TICK STOP OK
//! RTC_TICK_TEST_END
//! ```

use core::cell::Cell;

use critical_section::Mutex;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::rtc::{self, DateTime, Rtc, TickRate};
use hal::serial::{Config as UartConfig, SerialExt};
use hal::timer::{Counter, Divider};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Per-source ISR tallies: RTCIV 0x08 (RT0PS), 0x0A (RT1PS), anything else.
static C0: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static C1: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static OTHER: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
/// When set, the next firing disarms both tick banks inside the ISR (the
/// one-shot wake contract) and clears itself.
static DISARM: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));

/// RTC ISR: demux the shared vector (the RTCIV read auto-clears the reported
/// flag), tally per source, and — in one-shot mode — disarm right here: a
/// prescaler re-latches its flag every period, so a keep-armed wake handler
/// re-fires forever (at the fast taps, faster than a 1 MHz MCLK can leave
/// the ISR). `wake_cpu` lets `main` resume after `enter_lpm3()`.
#[msp430_rt::interrupt(wake_cpu)]
fn RTC() {
    let iv = rtc::read_iv();
    critical_section::with(|cs| {
        match iv {
            0x08 => {
                let c = C0.borrow(cs);
                c.set(c.get().wrapping_add(1));
            }
            0x0A => {
                let c = C1.borrow(cs);
                c.set(c.get().wrapping_add(1));
            }
            _ => {
                let c = OTHER.borrow(cs);
                c.set(c.get().wrapping_add(1));
            }
        }
        if DISARM.borrow(cs).get() {
            rtc::isr_disable_tick_interrupts();
            DISARM.borrow(cs).set(false);
        }
    });
}

fn c0() -> u16 {
    critical_section::with(|cs| C0.borrow(cs).get())
}
fn c1() -> u16 {
    critical_section::with(|cs| C1.borrow(cs).get())
}
fn other() -> u16 {
    critical_section::with(|cs| OTHER.borrow(cs).get())
}

/// Spin until `done()` or `max_ticks` of the counter elapse (bounded — a
/// dead tick source becomes a FAIL verdict, not a dark board). Returns
/// whether `done()` was reached.
fn wait_until(counter: &Counter, max_ticks: u16, mut done: impl FnMut() -> bool) -> bool {
    let start = counter.now();
    while !done() {
        if counter.elapsed_since(start) >= max_ticks {
            return false;
        }
    }
    true
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Low-power profile: ACLK = LFXT 32.768 kHz crystal (the RTC's clock and
    // the whole point), MCLK = SMCLK = 1 MHz.
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

    tx.write_all(b"\r\nMSP430FR5969 RTC_B prescaler tick self-check (no wiring)\r\n")
        .ok();

    // The DCO-side yardstick: SMCLK/8 = 125 kHz tick (8 us), 524 ms wrap —
    // every bounded wait below stays under one wrap.
    let counter = Counter::new_smclk(p.timer_0_a3, &clocks, Divider::Div8);

    // The calendar itself is irrelevant here (any load works); constructing
    // the Rtc proves ACLK is the crystal and releases RTCHOLD, which also
    // starts the prescaler chain.
    let start = DateTime {
        year: 2026,
        month: 7,
        day: 8,
        weekday: 3,
        hour: 12,
        minute: 0,
        second: 0,
    };
    let rtc = match Rtc::new(p.rtc_b_real_time_clock, &clocks, &start) {
        Ok(rtc) => rtc,
        Err(_) => {
            // No crystal: refuse loudly (the calendar fixture's convention).
            loop {
                red_led.set_high().ok();
                tx.write_all(b"RTC TICK REFUSED: ACLK is not the 32768 Hz crystal\r\n")
                    .ok();
                delay.delay_ms(1000);
            }
        }
    };

    // Ticks are the only enabled RTC sources; GIE on for the counting phases.
    unsafe { msp430::interrupt::enable() };

    // --- 1+2: 128 Hz rate against the DCO + IV 0x08 identity ----------------
    rtc.enable_tick_interrupt(TickRate::Hz128);
    // Sync to a tick edge, then time 32 full periods (250 ms nominal).
    let synced = wait_until(&counter, 20_000, || c0() >= 1);
    let t0 = counter.now();
    let base = c0();
    let counted = synced && wait_until(&counter, 50_000, || c0() >= base + 32);
    let rate_ticks = counter.elapsed_since(t0);
    rtc.disable_tick_interrupt(TickRate::Hz128);
    let rate_us = counter.ticks_to_us(rate_ticks as u32);
    // 250 ms +/- 5%: crystal-sourced ticks against the +/-3.5% DCO yardstick.
    let rate_ok = counted && (237_500..=262_500).contains(&rate_us);
    let iv0_ok = counted && c1() == 0 && other() == 0;

    // --- 3: 64 Hz from the other bank, IV 0x0A ------------------------------
    let c0_frozen = c0();
    rtc.enable_tick_interrupt(TickRate::Hz64);
    // >= 8 ticks is 125 ms nominal; bound at ~400 ms.
    let counted1 = wait_until(&counter, 50_000, || c1() >= 8);
    rtc.disable_tick_interrupt(TickRate::Hz64);
    let iv1_ok = counted1 && c0() == c0_frozen && other() == 0;

    // --- 4: both banks concurrently ------------------------------------------
    let (b0, b1) = (c0(), c1());
    rtc.enable_tick_interrupt(TickRate::Hz128);
    rtc.enable_tick_interrupt(TickRate::Hz64);
    let both_ok = wait_until(&counter, 50_000, || c0() >= b0 + 4 && c1() >= b1 + 4)
        && other() == 0;
    rtc.disable_tick_interrupt(TickRate::Hz128);
    rtc.disable_tick_interrupt(TickRate::Hz64);

    // --- 5: one 32 Hz tick wakes LPM3, exactly once --------------------------
    // GIE off so the arm -> sleep window is race-free: a tick landing early
    // latches and `enter_lpm3` (which sets GIE atomically with sleeping)
    // delivers it. In LPM3 the DCO/MCLK/SMCLK are all stopped — only the
    // crystal-fed prescaler can bring the part back.
    msp430::interrupt::disable();
    let w0 = c1(); // Hz32 lives in the RT1PS bank
    critical_section::with(|cs| DISARM.borrow(cs).set(true));
    rtc.enable_tick_interrupt(TickRate::Hz32);
    hal::power::enter_lpm3();
    // The ISR disarmed both banks; across a further 100 ms (3+ periods) the
    // tally must hold at exactly one.
    delay.delay_ms(100);
    let wake_ok = c1() == w0 + 1 && other() == 0;

    // --- 6: everything disarmed stays silent --------------------------------
    let (s0, s1, so) = (c0(), c1(), other());
    delay.delay_ms(50);
    let stop_ok = c0() == s0 && c1() == s1 && other() == so;

    let all_ok = rate_ok && iv0_ok && iv1_ok && both_ok && wake_ok && stop_ok;
    let _ = rtc; // calendar left running; ticks disarmed

    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"rtc tick c0=").ok();
        write_dec(&mut tx, c0() as u32);
        tx.write_all(b" c1=").ok();
        write_dec(&mut tx, c1() as u32);
        tx.write_all(b" other=").ok();
        write_dec(&mut tx, other() as u32);
        tx.write_all(b" rate_us=").ok();
        write_dec(&mut tx, rate_us);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"RTC_TICK_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"RTC TICK RATE", rate_ok);
        verdict(&mut tx, b"RTC TICK IV0", iv0_ok);
        verdict(&mut tx, b"RTC TICK IV1", iv1_ok);
        verdict(&mut tx, b"RTC TICK BOTH", both_ok);
        verdict(&mut tx, b"RTC TICK WAKE", wake_ok);
        verdict(&mut tx, b"RTC TICK STOP", stop_ok);
        tx.write_all(b"RTC_TICK_TEST_END\r\n").ok();

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
