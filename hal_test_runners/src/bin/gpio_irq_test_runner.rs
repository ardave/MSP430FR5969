#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (so it returns with RETI, not RET) — still nightly-gated.
#![feature(abi_msp430_interrupt)]

//! GPIO port-interrupt integration fixture: `Pin::enable_interrupt` +
//! `gpio::read_iv` on the `PORT1`/`PORT4` vectors.
//!
//! A self-checking sibling of the human-facing demos. The trick that makes it
//! hands-free: **`PxIFG` is software-settable, and a software-set flag goes
//! through the same latch → vector → `PxIV` path as a real pin edge** — so the
//! fixture can arm the LaunchPad buttons' pins (S2 = P1.1, S1 = P4.5, both
//! pull-up/falling) and then "press" them from software via
//! `set_interrupt_pending()`, verifying the whole ISR chain with nobody in the
//! room. Reports a framed pass/fail verdict over the UART backchannel
//! (eUSCI_A0, 9600 8N1 on `/dev/cu.usbmodem11203`), driven by the host-side
//! `gpio_tests` runner. No wiring needed beyond the LaunchPad itself.
//!
//! ```text
//! cargo +nightly build --bin gpio_irq_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/gpio_irq_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **The vector demux (`GPIO IV`).** Software-latch P1IFG.1; the `PORT1`
//!    ISR must fire exactly once and its `read_iv::<P1>()` must return 0x04
//!    (= 2·(pin+1) for pin 1) — proving enable, vectoring, and the IV
//!    priority encoding.
//!
//! 2. **The IV read-and-clear (`GPIO CLEAR`).** After the ISR, P1IFG.1 must
//!    read 0 (the `PxIV` read cleared it in silicon — the fixture never
//!    touches the flag) and the ISR count must still be 1 a delay later (no
//!    refire from a stale flag).
//!
//! 3. **A second port (`GPIO P4`).** The same two assertions on P4.5/`PORT4`,
//!    expecting IV = 0x0C — proving the per-port addressing (the interleaved
//!    odd-byte registers) isn't right by accident on port 1 only.
//!
//! All verdicts are computed **once** at startup; the loop re-emits the fixed
//! verdict burst once per second with GREEN toggling as a heartbeat (steady
//! RED = a check failed). The buttons stay armed: real presses bump the same
//! counters, which the info line reports (`gpio p1=… p4=…`) — press S2/S1 and
//! watch the counts climb for a live interrupt demo. (The wake-from-LPM4
//! button demo is `--bin button_wake`; this fixture stays awake so the burst
//! keeps a 1 Hz cadence for the host runner.)
//!
//! # Framed output for the host runner
//!
//! ```text
//! gpio p1=1 p4=1              (human-readable info, skipped by host)
//! GPIO_TEST_BEGIN
//! GPIO IV OK                  (or `GPIO IV FAIL`)
//! GPIO CLEAR OK               (or `GPIO CLEAR FAIL`)
//! GPIO P4 OK                  (or `GPIO P4 FAIL`)
//! GPIO_TEST_END
//! ```

use core::cell::Cell;

use critical_section::Mutex;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::{self, Edge, GpioExt};
use hal::interrupt;
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take() (and so msp430::interrupt::enable() is available).
use msp430 as _;

/// Event tally + last observed PxIV per port, shared ISR → main.
static P1_COUNT: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static P1_LAST_IV: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static P4_COUNT: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static P4_LAST_IV: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// PORT1 ISR: consume the highest-priority pending pin via `PxIV` (the read
/// clears its IFG bit in silicon — no manual flag handling) and record it.
#[msp430_rt::interrupt]
fn PORT1() {
    let iv = gpio::read_iv::<gpio::P1>();
    critical_section::with(|cs| {
        P1_LAST_IV.borrow(cs).set(iv);
        let c = P1_COUNT.borrow(cs);
        c.set(c.get().wrapping_add(1));
    });
}

/// PORT4 ISR: same shape as PORT1.
#[msp430_rt::interrupt]
fn PORT4() {
    let iv = gpio::read_iv::<gpio::P4>();
    critical_section::with(|cs| {
        P4_LAST_IV.borrow(cs).set(iv);
        let c = P4_COUNT.borrow(cs);
        c.set(c.get().wrapping_add(1));
    });
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Performance profile: SMCLK = 8 MHz (UART BRCLK), MCLK = 1 MHz (Delay).
    let clocks = hal::clocks::configure(p.cs);

    // Port interrupt config written while pins are locked would not reach the pads.
    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2, heartbeat
    let mut red_led = port4.pin6.into_output(); // LED1, failure flag

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"\r\nMSP430FR5969 GPIO port-interrupt self-check\r\n")
        .ok();

    // The LaunchPad buttons short their pin to GND and the board populates no
    // external resistor, so the internal pull-up is mandatory; press = falling.
    let mut s2 = port1.pin1.into_pull_up_input(); // S2 = P1.1
    let mut s1 = port4.pin5.into_pull_up_input(); // S1 = P4.5
    s2.enable_interrupt(Edge::Falling);
    s1.enable_interrupt(Edge::Falling);

    // SAFETY: enabling interrupts globally (set GIE) so the port ISRs can run.
    // All state shared with the ISRs lives in critical-section Mutexes.
    unsafe {
        msp430::interrupt::enable();
    }

    // --- 1+2. PORT1: software "press", IV demux, IV auto-clear ---------------
    // set_interrupt_pending latches P1IFG.1 exactly as a falling edge would;
    // the ISR runs before the delay below finishes (interrupt latency is a
    // handful of cycles — the delay is only to catch *extra* firings).
    s2.set_interrupt_pending();
    delay.delay_ms(10);
    let (p1_count, p1_iv) =
        critical_section::with(|cs| (P1_COUNT.borrow(cs).get(), P1_LAST_IV.borrow(cs).get()));
    let iv_ok = p1_count == 1 && p1_iv == 0x04;
    // The fixture never cleared the flag — only the ISR's PxIV read did. A
    // still-set flag (or a count that kept climbing) fails here.
    delay.delay_ms(10);
    let p1_count_after = critical_section::with(|cs| P1_COUNT.borrow(cs).get());
    let clear_ok = !s2.interrupt_pending() && p1_count_after == 1;

    // --- 3. PORT4: same via the other register bank --------------------------
    s1.set_interrupt_pending();
    delay.delay_ms(10);
    let (p4_count, p4_iv) =
        critical_section::with(|cs| (P4_COUNT.borrow(cs).get(), P4_LAST_IV.borrow(cs).get()));
    let p4_ok = p4_count == 1 && p4_iv == 0x0C && !s1.interrupt_pending();

    // Verdicts are frozen; the buttons stay armed so live presses keep bumping
    // the counters shown in the info line.
    let mut on = false;
    loop {
        let (c1, c4) =
            critical_section::with(|cs| (P1_COUNT.borrow(cs).get(), P4_COUNT.borrow(cs).get()));
        tx.write_all(b"gpio p1=").ok();
        write_dec(&mut tx, c1 as u32);
        tx.write_all(b" p4=").ok();
        write_dec(&mut tx, c4 as u32);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"GPIO_TEST_BEGIN\r\n").ok();
        tx.write_all(if iv_ok {
            b"GPIO IV OK\r\n" as &[u8]
        } else {
            b"GPIO IV FAIL\r\n"
        })
        .ok();
        tx.write_all(if clear_ok {
            b"GPIO CLEAR OK\r\n" as &[u8]
        } else {
            b"GPIO CLEAR FAIL\r\n"
        })
        .ok();
        tx.write_all(if p4_ok {
            b"GPIO P4 OK\r\n" as &[u8]
        } else {
            b"GPIO P4 FAIL\r\n"
        })
        .ok();
        tx.write_all(b"GPIO_TEST_END\r\n").ok();

        if iv_ok && clear_ok && p4_ok {
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
