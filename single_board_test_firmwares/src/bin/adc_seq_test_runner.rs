#![no_std]
#![no_main]

//! ADC12_B sequence-of-channels integration fixture — **no wiring at all**,
//! driven by the host-side `adc_seq_tests` runner. One hardware trigger scans
//! several *different* inputs (`CONSEQ = 1` + MSC) into `MEM0..MEMn` — the
//! datalogger shape single-channel fixtures structurally cannot check,
//! because here the verdicts are **order-sensitive**: the three members'
//! plausibility windows are pairwise disjoint, so a swapped MCTLx→MEMx
//! mapping puts temperature counts in a supply window and fails loudly.
//!
//! ```text
//! cargo +nightly build --bin adc_seq_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/adc_seq_test_runner
//! ```
//!
//! # The scan
//!
//! Three members, all internal (hands-free), deliberately **mixing
//! references within one scan** (each `MCTLx` carries its own `VRSEL`):
//!
//! | Slot | Member                      | Window                             |
//! |------|-----------------------------|------------------------------------|
//! | 0    | temperature (vs 2.0 V VREF) | 5–60 °C through the TLV points     |
//! | 1    | supply (A31 vs 2.0 V VREF)  | AVCC = 2·mV in 2900–3700 mV        |
//! | 2    | supply-half (A31 vs AVCC)   | ratiometric ~2048 ± 160 counts     |
//!
//! At 12 bits under the 2.0 V reference those are counts of roughly
//! ~1300–1600, ~2970–3790, and 1888–2208 — pairwise disjoint, so **every**
//! slot transposition lands at least one result outside its window.
//!
//! # What it checks
//!
//! - **ORDER** — one polled scan ([`read_sequence_vref`]), each slot in its
//!   window.
//! - **REVERSED** — the same members in reverse order; the windows must
//!   follow the members, proving the slot↔member tracking is real and not a
//!   coincidence of the register file.
//! - **DMA** — the scan drained by a DMA channel ([`read_sequence_dma_vref`]):
//!   source walking `MEM0..MEM2` one completion edge at a time, the final
//!   transfer riding the *last* member's flag.
//! - **RERUN** — six more DMA scans, with a deliberate polled conversion
//!   injected midway: per the ADC12→DMA **trigger-latch erratum** (TI E2E
//!   #401588, characterized on this board 2026-07-05 in repeat-single-channel
//!   mode), that unserviced completion parks the trigger latch high. Every
//!   scan after it must still complete and stay in-window — this verdict is
//!   what establishes on hardware that the per-run
//!   `consume_stale_trigger_word` scrub works identically when the DMA
//!   trigger edges come from a multi-member sequence. Without the scrub,
//!   only the first DMA scan after reset would ever finish (the driver's
//!   bounded wait turns a missing edge into `DmaIncomplete`, so a failure is
//!   a FAIL line, not a dark board).
//! - **SINGLE** — a plain polled `read_supply_half` afterward: the driver
//!   restored its single-conversion contract (`CONSEQ = 0`, `MSC = 0`).
//!
//! # Framed output for the host runner
//!
//! ```text
//! adc seq polled t=28.1C mv=3630 half=2040 dma t=28.0C mv=3628 half=2043 single=2045
//! ADC_SEQ_TEST_BEGIN
//! ADC_SEQ CAL OK                       (or `ADC_SEQ CAL MISSING`)
//! ADC_SEQ ORDER OK                     (or `... FAIL`)
//! ADC_SEQ REVERSED OK
//! ADC_SEQ DMA OK
//! ADC_SEQ RERUN OK
//! ADC_SEQ SINGLE OK
//! ADC_SEQ_TEST_END
//! ```
//!
//! **GREEN** LED while all checks pass, **RED** otherwise.

use hal::adc::{Adc, Config as AdcConfig, SampleTime, SeqMember};
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

/// The scan: temperature, absolute supply, ratiometric supply — three
/// pairwise-disjoint windows, two references, both internal map bits.
const SCAN: [SeqMember; 3] = [
    SeqMember::temperature(),
    SeqMember::supply_vref(),
    SeqMember::supply_half(),
];

/// The same members reversed — the windows must follow them.
const SCAN_REVERSED: [SeqMember; 3] = [
    SeqMember::supply_half(),
    SeqMember::supply_vref(),
    SeqMember::temperature(),
];

/// DMA scans in the RERUN verdict; the latch-parking polled conversion is
/// injected after the first half.
const RERUNS: usize = 6;

/// Same per-sample temperature window as the `ref_temp`/`adc_dma` fixtures:
/// only a real fault flips it, not a warm office.
const TEMP_MIN_DECI_C: i16 = 50; // 5.0 °C
const TEMP_MAX_DECI_C: i16 = 600; // 60.0 °C

/// Same AVCC window as `ref_temp`: the eZ-FET LDO feeds this LaunchPad
/// ~3.6 V on USB power (HW-measured 2026-07-03).
const SUPPLY_MIN_MV: u32 = 2900;
const SUPPLY_MAX_MV: u32 = 3700;

/// The ratiometric supply monitor sits at half full-scale by construction;
/// ±160 counts (~4 %) absorbs divider noise, catches garbage.
const SUPPLY_MID: u16 = 2048;
const SUPPLY_TOL: u16 = 160;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (SMCLK feeds the UART BRCLK below).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the UART pin mux takes effect. (The
    // internal ADC channels and the DMA need no pins.)
    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz. Plain polled TX.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1).
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    // One DMA channel drains the sequence scans; the other two stay free.
    let channels = p.dma.split();
    let mut ch0 = channels.ch0;

    // REF_A at 2.0 V: powers the temperature sensor, is the reference the
    // VREF members convert against, and covers AVCC/2 (≈ 1.8 V) headroom.
    let vref = Ref::new(p.shared_reference, ReferenceVoltage::V2_0);

    // ADC: 12-bit (the TLV temperature points are 12-bit results), MODOSC,
    // long sample time — both internal sources are high-impedance, and the
    // one ADC12SHT0x setting covers every sequence member (MEM0..7).
    let mut adc = Adc::new(
        p.adc12,
        AdcConfig::default().sample_time(SampleTime::Cycles256),
    );

    let cal = tlv::adc_cal();

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"MSP430FR5969 ADC12_B sequence-of-channels: 3-member mixed-reference scan (no wiring)\r\n")
        .ok();

    loop {
        let cal_ok = cal.is_some();

        // ORDER: one polled scan; each slot must land in its member's window.
        let mut polled = [0u16; 3];
        let order_ok = adc.read_sequence_vref(&SCAN, &vref, &mut polled).is_ok()
            && scan_windows_ok(&polled, &adc, &vref, cal);

        // REVERSED: the mirrored member list — windows must follow members.
        let mut reversed = [0u16; 3];
        let reversed_ok = adc
            .read_sequence_vref(&SCAN_REVERSED, &vref, &mut reversed)
            .is_ok()
            && temp_ok(reversed[2], &vref, cal)
            && supply_mv_ok(reversed[1], &adc, &vref)
            && supply_half_ok(reversed[0]);

        // DMA: the same scan collected by the DMA channel, one completion
        // edge per member, the last transfer riding the final member's flag.
        let mut dma = [0u16; 3];
        let dma_ok = adc
            .read_sequence_dma_vref(&SCAN, &vref, &mut ch0, &mut dma)
            .is_ok()
            && scan_windows_ok(&dma, &adc, &vref, cal);

        // RERUN: repeated DMA scans with a deliberately parked trigger latch
        // midway (the polled conversion completes with ADC12IE0 clear —
        // exactly the erratum's poison). Every scan must still complete.
        let mut rerun_ok = true;
        for i in 0..RERUNS {
            if i == RERUNS / 2 {
                // Park the latch: an unserviced completion the next scan's
                // scrub must absorb.
                let _ = adc.read_supply_half();
            }
            let mut buf = [0u16; 3];
            rerun_ok &= adc
                .read_sequence_dma_vref(&SCAN, &vref, &mut ch0, &mut buf)
                .is_ok()
                && scan_windows_ok(&buf, &adc, &vref, cal);
        }

        // SINGLE: the sequence engine restored the single-conversion
        // contract — a plain polled read still behaves.
        let single = adc.read_supply_half();
        let single_ok = supply_half_ok(single);

        // Human-readable info line (host skips everything up to BEGIN).
        tx.write_all(b"adc seq polled t=").ok();
        write_temp(&mut tx, polled[0], &vref, cal);
        tx.write_all(b"C mv=").ok();
        write_dec(&mut tx, adc.to_millivolts(polled[1], &vref) * 2);
        tx.write_all(b" half=").ok();
        write_dec(&mut tx, polled[2] as u32);
        tx.write_all(b" dma t=").ok();
        write_temp(&mut tx, dma[0], &vref, cal);
        tx.write_all(b"C mv=").ok();
        write_dec(&mut tx, adc.to_millivolts(dma[1], &vref) * 2);
        tx.write_all(b" half=").ok();
        write_dec(&mut tx, dma[2] as u32);
        tx.write_all(b" single=").ok();
        write_dec(&mut tx, single as u32);
        tx.write_all(b"\r\n").ok();

        // The framed verdict burst.
        tx.write_all(b"ADC_SEQ_TEST_BEGIN\r\n").ok();
        tx.write_all(if cal_ok {
            b"ADC_SEQ CAL OK\r\n" as &[u8]
        } else {
            b"ADC_SEQ CAL MISSING\r\n"
        })
        .ok();
        verdict(&mut tx, b"ADC_SEQ ORDER", order_ok);
        verdict(&mut tx, b"ADC_SEQ REVERSED", reversed_ok);
        verdict(&mut tx, b"ADC_SEQ DMA", dma_ok);
        verdict(&mut tx, b"ADC_SEQ RERUN", rerun_ok);
        verdict(&mut tx, b"ADC_SEQ SINGLE", single_ok);
        tx.write_all(b"ADC_SEQ_TEST_END\r\n").ok();

        if cal_ok && order_ok && reversed_ok && dma_ok && rerun_ok && single_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// All three windows for a `SCAN`-ordered result buffer.
fn scan_windows_ok(results: &[u16; 3], adc: &Adc, vref: &Ref, cal: Option<tlv::AdcCal>) -> bool {
    temp_ok(results[0], vref, cal) && supply_mv_ok(results[1], adc, vref) && supply_half_ok(results[2])
}

/// Temperature slot: TLV-interpolated deci-°C inside the 5–60 °C window.
fn temp_ok(raw: u16, vref: &Ref, cal: Option<tlv::AdcCal>) -> bool {
    cal.and_then(|c| c.temp_deci_celsius(vref.voltage(), raw))
        .map(|t| (TEMP_MIN_DECI_C..=TEMP_MAX_DECI_C).contains(&t))
        .unwrap_or(false)
}

/// Absolute supply slot: 2 × the monitor's millivolt worth = AVCC, inside
/// the 2900–3700 mV window.
fn supply_mv_ok(raw: u16, adc: &Adc, vref: &Ref) -> bool {
    let mv = adc.to_millivolts(raw, vref) * 2;
    (SUPPLY_MIN_MV..=SUPPLY_MAX_MV).contains(&mv)
}

/// Ratiometric supply slot: half full-scale by construction.
fn supply_half_ok(raw: u16) -> bool {
    raw.abs_diff(SUPPLY_MID) <= SUPPLY_TOL
}

/// Write a temperature slot as `-X.Y` deci-°C (or `?` without calibration).
fn write_temp<W: hal::embedded_io::Write>(tx: &mut W, raw: u16, vref: &Ref, cal: Option<tlv::AdcCal>) {
    match cal.and_then(|c| c.temp_deci_celsius(vref.voltage(), raw)) {
        Some(t) => write_deci(tx, t),
        None => {
            tx.write_all(b"?").ok();
        }
    }
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
