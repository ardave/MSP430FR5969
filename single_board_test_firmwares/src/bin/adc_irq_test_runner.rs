#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! ADC12_B conversion-complete-interrupt fixture: `Adc::start_supply_half` +
//! `enable_conversion_interrupt` on the `ADC12` vector, sampling **while the
//! CPU sleeps in LPM0**.
//!
//! The polled ADC fixtures busy-wait `ADC12BUSY`; this one exercises the
//! non-blocking path instead: arm a conversion, drop the CPU into LPM0, and
//! let the converter — self-clocked by MODOSC, which it requests on demand —
//! finish alone and wake the CPU via the `ADC12` interrupt. The ISR collects
//! the result with `adc::read_result()`, whose MEM0 read also clears
//! `ADC12IFG0` in hardware: one bus access, result collected, interrupt
//! acknowledged. Reports a framed pass/fail verdict over the UART backchannel
//! (eUSCI_A0, 9600 8N1 on `/dev/cu.usbmodem11203`), driven by the host-side
//! `adc_irq_tests` runner. **No wiring** — the source is the internal
//! (AVCC–AVSS)/2 supply monitor (channel A31).
//!
//! ```text
//! cargo +nightly build --bin adc_irq_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/adc_irq_test_runner
//! ```
//!
//! # What it checks
//!
//! Eight cycles of `start_supply_half(); enter_lpm0();`:
//!
//! 1. **Every sleep returns (`ADC IRQ WAKE`).** A conversion whose interrupt
//!    never fired would leave the part in LPM0 forever and no burst would ever
//!    appear — reaching the verdict at all proves all eight wakes (the
//!    deep-sleep fixture's "doesn't hang" logic).
//!
//! 2. **One interrupt per conversion (`ADC IRQ COUNT`).** The ISR tally must
//!    be exactly 8 — a re-firing flag (e.g. a result collected without
//!    clearing) or a spurious source falls outside.
//!
//! 3. **The results are real (`ADC IRQ VALUE`).** Ratiometric AVCC/2 against
//!    AVCC reads ~half scale by construction; every collected result must be
//!    within 2048 ± 200 at 12-bit — the same plausibility band the polled
//!    `adc_internal` fixture uses.
//!
//! All verdicts are computed **once** at startup; the loop re-emits the fixed
//! verdict burst once per second, GREEN toggling as a heartbeat, steady RED on
//! failure.
//!
//! # Framed output for the host runner
//!
//! ```text
//! adc irq n=8 min=2035 max=2059   (human-readable info, skipped by host)
//! ADC_IRQ_TEST_BEGIN
//! ADC IRQ WAKE OK                 (always OK if reached — see above)
//! ADC IRQ COUNT OK                (or `ADC IRQ COUNT FAIL`)
//! ADC IRQ VALUE OK                (or `ADC IRQ VALUE FAIL`)
//! ADC_IRQ_TEST_END
//! ```

use core::cell::Cell;

use critical_section::Mutex;
use hal::adc::{self, Adc, Config as AdcConfig, SampleTime};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Last collected MEM0 result and the ISR firing tally, shared ISR → main.
static RESULT: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static FIRED: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// ADC12 conversion-complete ISR: collect MEM0 (the read clears `ADC12IFG0` —
/// no separate acknowledge) and count the firing. `wake_cpu` lets `main`
/// resume after `enter_lpm0()`.
#[msp430_rt::interrupt(wake_cpu)]
fn ADC12() {
    let counts = adc::read_result();
    critical_section::with(|cs| {
        RESULT.borrow(cs).set(counts);
        let f = FIRED.borrow(cs);
        f.set(f.get().wrapping_add(1));
    });
}

/// Number of sleep-sample cycles to run.
const CYCLES: usize = 8;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // Performance profile: SMCLK = 8 MHz (UART BRCLK), MCLK = 1 MHz (Delay).
    // The ADC itself rides MODOSC and needs none of this during conversion.
    let clocks = hal::clocks::configure(p.cs);

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

    tx.write_all(b"\r\nMSP430FR5969 ADC12 interrupt self-check\r\n")
        .ok();

    // 12-bit, MODOSC, long sample time — the internal supply divider is
    // high-impedance and under-samples at the default 32 cycles.
    let mut adc = Adc::new(
        p.adc12,
        AdcConfig::default().sample_time(SampleTime::Cycles256),
    );
    adc.enable_conversion_interrupt();

    // --- Sleep-sample cycles --------------------------------------------------
    // Each cycle: arm A31, sleep in LPM0 (MCLK off, MODOSC self-clocks the
    // conversion), resume when the ADC12 ISR (wake_cpu) collected the result.
    // Reaching the loop's end at all proves every enter_lpm0() returned. The
    // UART is flushed first so the banner isn't truncated by MCLK stopping
    // mid-burst (TX itself rides SMCLK, which LPM0 keeps alive, but the write
    // path pushes bytes from the CPU).
    tx.flush().ok();
    let mut min = u16::MAX;
    let mut max = 0u16;
    for _ in 0..CYCLES {
        adc.start_supply_half();
        hal::power::enter_lpm0();
        let counts = critical_section::with(|cs| RESULT.borrow(cs).get());
        if counts < min {
            min = counts;
        }
        if counts > max {
            max = counts;
        }
    }
    let fired = critical_section::with(|cs| FIRED.borrow(cs).get());

    let count_ok = fired == CYCLES as u16;
    // Ratiometric AVCC/2 against AVCC: ~half scale regardless of the actual
    // supply voltage. 2048 ± 200 at 12-bit.
    let value_ok = min >= 1848 && max <= 2248;

    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"adc irq n=").ok();
        write_dec(&mut tx, fired as u32);
        tx.write_all(b" min=").ok();
        write_dec(&mut tx, min as u32);
        tx.write_all(b" max=").ok();
        write_dec(&mut tx, max as u32);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"ADC_IRQ_TEST_BEGIN\r\n").ok();
        // Reaching here proves every enter_lpm0() returned, so WAKE always passes.
        tx.write_all(b"ADC IRQ WAKE OK\r\n").ok();
        tx.write_all(if count_ok {
            b"ADC IRQ COUNT OK\r\n" as &[u8]
        } else {
            b"ADC IRQ COUNT FAIL\r\n"
        })
        .ok();
        tx.write_all(if value_ok {
            b"ADC IRQ VALUE OK\r\n" as &[u8]
        } else {
            b"ADC IRQ VALUE FAIL\r\n"
        })
        .ok();
        tx.write_all(b"ADC_IRQ_TEST_END\r\n").ok();

        if count_ok && value_ok {
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
