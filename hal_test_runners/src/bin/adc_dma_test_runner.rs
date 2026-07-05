#![no_std]
#![no_main]

//! ADC12_B + DMA integration fixture — **no wiring at all**, driven by the
//! host-side `adc_dma_tests` runner. The converter free-runs in
//! repeat-single-channel mode while a DMA channel drains every `MEM0` result
//! into a buffer (`ADC12IFG0` is DMA trigger 26; the DMA's word read of MEM0
//! is what clears the flag), so a 32-sample burst costs the CPU nothing but
//! the setup and the wait.
//!
//! ```text
//! cargo +nightly build --bin adc_dma_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/adc_dma_test_runner
//! ```
//!
//! # What it checks
//!
//! Both internal sources, so the fixture is hands-free like its `adc_internal`
//! and `ref_temp` siblings:
//!
//! - **SUPPLY** — 32 DMA-drained conversions of the (AVCC–AVSS)/2 monitor
//!   against AVCC (`read_supply_half_repeated_dma`). Ratiometric, so every
//!   sample must sit near half full-scale (2048 at 12-bit) regardless of the
//!   actual rail; the window (±160 counts) is wide enough for the divider's
//!   noise, tight enough that a mis-collected buffer (zeros, truncated bytes,
//!   stale repeats of one conversion) fails.
//! - **TEMP** — 32 DMA-drained conversions of the temperature sensor against
//!   the 2.0 V REF_A reference (`read_temperature_repeated_dma`), each sample
//!   individually interpolated through the factory 30/85 °C TLV points and
//!   required to land in the same 5–60 °C plausibility window `ref_temp`
//!   uses. A buffer of anything but genuine back-to-back conversions cannot
//!   put 32 samples inside it.
//! - **SINGLE** — after both DMA runs, one plain polled `read_supply_half`
//!   must still read ~half scale: proves the driver restored its
//!   single-conversion contract (`CONSEQ`, `MSC`) after free-running.
//!
//! The info line carries min/max of each buffer so a human (or a failing run)
//! can see the spread, not just the verdict.
//!
//! # Framed output for the host runner
//!
//! ```text
//! adc dma supply min=2010 max=2035 temp min=1520 max=1531 t=28.1C single=2021
//! ADC_DMA_TEST_BEGIN
//! ADC_DMA CAL OK                       (or `ADC_DMA CAL MISSING`)
//! ADC_DMA SUPPLY OK                    (or `... FAIL`)
//! ADC_DMA TEMP OK
//! ADC_DMA SINGLE OK
//! ADC_DMA_TEST_END
//! ```
//!
//! **GREEN** LED while all checks pass, **RED** otherwise.

use hal::adc::{Adc, Config as AdcConfig, SampleTime};
use hal::delay::Delay;
use hal::dma::DmaExt;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::ref_a::{Ref, ReferenceVoltage};
use hal::serial::{Config as UartConfig, SerialExt};
use hal::tlv;
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Samples per DMA-drained burst.
const SAMPLES: usize = 32;

/// The supply monitor is ratiometric against AVCC, so half full-scale by
/// construction; ±160 counts (~4 %) absorbs divider noise, catches garbage.
const SUPPLY_MID: u16 = 2048;
const SUPPLY_TOL: u16 = 160;

/// Same per-sample temperature window as the `ref_temp` fixture: only a real
/// fault flips it, not a warm office.
const TEMP_MIN_DECI_C: i16 = 50; // 5.0 °C
const TEMP_MAX_DECI_C: i16 = 600; // 60.0 °C

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (SMCLK feeds the UART BRCLK below).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the UART pin mux takes effect. (The
    // internal ADC channels and the DMA need no pins.)
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz. Plain polled TX —
    // the DMA under test here is the ADC's, not the UART's.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1).
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    // One DMA channel drains the converter; the other two stay free.
    let channels = p.dma.split();
    let mut ch0 = channels.ch0;

    // REF_A at 2.0 V: powers the temperature sensor and is the reference its
    // conversions measure against (same choice as the ref_temp fixture).
    let vref = Ref::new(p.shared_reference, ReferenceVoltage::V2_0);

    // ADC: 12-bit (the TLV temperature points are 12-bit results), MODOSC,
    // long sample time — both internal sources are high-impedance.
    let mut adc = Adc::new(
        p.adc12,
        AdcConfig::default().sample_time(SampleTime::Cycles256),
    );

    let cal = tlv::adc_cal();

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"MSP430FR5969 ADC12_B + DMA: repeat-single-channel drained by DMA (no wiring)\r\n")
        .ok();

    loop {
        // SUPPLY: 32 free-running ratiometric conversions of (AVCC-AVSS)/2,
        // collected by DMA. Every sample should hug half scale.
        let mut supply = [0u16; SAMPLES];
        adc.read_supply_half_repeated_dma(&mut ch0, &mut supply);
        let supply_ok = supply
            .iter()
            .all(|&s| s.abs_diff(SUPPLY_MID) <= SUPPLY_TOL);
        let (sup_min, sup_max) = min_max(&supply);

        // TEMP: 32 free-running absolute conversions of the sensor, collected
        // by DMA, each interpolated through the factory TLV pair.
        let mut temp = [0u16; SAMPLES];
        adc.read_temperature_repeated_dma(&vref, &mut ch0, &mut temp);
        let deci_of = |raw: u16| cal.and_then(|c| c.temp_deci_celsius(vref.voltage(), raw));
        let temp_ok = temp.iter().all(|&raw| {
            deci_of(raw)
                .map(|t| (TEMP_MIN_DECI_C..=TEMP_MAX_DECI_C).contains(&t))
                .unwrap_or(false)
        });
        let (tmp_min, tmp_max) = min_max(&temp);

        // SINGLE: the driver must have restored single-conversion mode after
        // free-running — a plain polled read still behaves.
        let single = adc.read_supply_half();
        let single_ok = single.abs_diff(SUPPLY_MID) <= SUPPLY_TOL;

        let cal_ok = cal.is_some();

        // Human-readable info line (host skips everything up to BEGIN).
        tx.write_all(b"adc dma supply min=").ok();
        write_dec(&mut tx, sup_min as u32);
        tx.write_all(b" max=").ok();
        write_dec(&mut tx, sup_max as u32);
        tx.write_all(b" temp min=").ok();
        write_dec(&mut tx, tmp_min as u32);
        tx.write_all(b" max=").ok();
        write_dec(&mut tx, tmp_max as u32);
        tx.write_all(b" t=").ok();
        match deci_of(temp[0]) {
            Some(t) => write_deci(&mut tx, t),
            None => {
                tx.write_all(b"?").ok();
            }
        }
        tx.write_all(b"C single=").ok();
        write_dec(&mut tx, single as u32);
        tx.write_all(b"\r\n").ok();

        // The framed verdict burst.
        tx.write_all(b"ADC_DMA_TEST_BEGIN\r\n").ok();
        tx.write_all(if cal_ok {
            b"ADC_DMA CAL OK\r\n" as &[u8]
        } else {
            b"ADC_DMA CAL MISSING\r\n"
        })
        .ok();
        verdict(&mut tx, b"ADC_DMA SUPPLY", supply_ok);
        verdict(&mut tx, b"ADC_DMA TEMP", temp_ok);
        verdict(&mut tx, b"ADC_DMA SINGLE", single_ok);
        tx.write_all(b"ADC_DMA_TEST_END\r\n").ok();

        if cal_ok && supply_ok && temp_ok && single_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Smallest and largest sample in a buffer (buffer is never empty here).
fn min_max(buf: &[u16]) -> (u16, u16) {
    let mut min = u16::MAX;
    let mut max = 0;
    for &s in buf {
        if s < min {
            min = s;
        }
        if s > max {
            max = s;
        }
    }
    (min, max)
}

/// Write `name` + ` OK`/` FAIL` + CRLF.
fn verdict<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], ok: bool) {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" as &[u8] } else { b" FAIL\r\n" })
        .ok();
}

/// Write a deci-value (tenths) as `-X.Y` decimal ASCII, e.g. 273 → `27.3`.
fn write_deci<W: hal::embedded_io::Write>(tx: &mut W, deci: i16) {
    let mut v = deci as i32;
    if v < 0 {
        tx.write_all(b"-").ok();
        v = -v;
    }
    write_dec(tx, (v / 10) as u32);
    tx.write_all(b".").ok();
    write_dec(tx, (v % 10) as u32);
}

/// Write an unsigned value as decimal ASCII over the UART. `core::fmt` is
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
// Provide a minimal one so we don't link newlib's libc (and its syscall stubs).
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
