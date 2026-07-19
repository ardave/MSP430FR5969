#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! ADC12_B **window comparator** fixture: thresholds compared in silicon
//! (`ADC12HI`/`ADC12LO` + `ADC12WINC`), verdicts read from the latched
//! `ADC12IFGR2` flags, crossings demuxed on `ADC12IV`, and the free-running
//! threshold watch (`Adc::start_monitor`) waking the CPU from **LPM0**.
//! Reports a framed pass/fail verdict over the UART backchannel (eUSCI_A0,
//! 9600 8N1), driven by the host-side `adc_window_tests` runner. **No
//! wiring** — the input is the internal (AVCC–AVSS)/2 supply monitor
//! (channel A31), which reads ~half scale (≈ 2048 at 12-bit) by
//! construction, so windows placed around / entirely below / entirely above
//! half scale make all three comparator outcomes reachable from software.
//!
//! ```text
//! cargo +nightly build --bin adc_window_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/adc_window_test_runner
//! ```
//!
//! # What it checks
//!
//! 1. **`ADC WIN IN`** — a window spanning half scale: verdict `Within`,
//!    `ADC12INIFG` latched **alone**, count in the 2048 ± 200 band.
//! 2. **`ADC WIN HI`** — a window entirely below the reading: verdict
//!    `Above`, `ADC12HIIFG` alone (result > `ADC12HI`).
//! 3. **`ADC WIN LO`** — a window entirely above the reading: verdict
//!    `Below`, `ADC12LOIFG` alone (result < `ADC12LO`).
//! 4. **`ADC WIN CLEAR`** — `clear_window_flags()` leaves all three low,
//!    and a plain (non-windowed) `read_supply_half` latches none: `WINC`
//!    does not leak into ordinary conversions.
//! 5. **`ADC WIN IV`** — above-only interrupt armed, one windowed
//!    conversion started with GIE off, `enter_lpm0()` delivers the latched
//!    crossing: ISR sees `ADC12IV` = **0x06** (`IV_WINDOW_ABOVE`), exactly
//!    one firing, and the IV read cleared the flag.
//! 6. **`ADC WIN MON`** — free-running monitor with the window around the
//!    reading and only *outside* crossings armed: ~20 ms of conversions
//!    fire **nothing**, while the accumulated `ADC12INIFG` proves the
//!    converter really was free-running through the comparator.
//! 7. **`ADC WIN WAKE`** — monitor with the window below the reading and
//!    `above` armed, entered with GIE off, CPU asleep in LPM0: the first
//!    conversion wakes it, and because the ISR disarms via
//!    `isr_disable_window_interrupts()`, the count stays at exactly **one**
//!    across further milliseconds of out-of-range free-running — the
//!    re-latching flag must not storm the ISR.
//! 8. **`ADC WIN RESTORE`** — after `stop_monitor()`, a plain
//!    `read_supply_half` reads half scale again and latches no window flag:
//!    `CONSEQ`/`MSC`/`WINC` all restored.
//!
//! All verdicts are computed **once** at startup; the loop re-emits the
//! fixed verdict burst once per second, GREEN toggling as a heartbeat,
//! steady RED on failure.
//!
//! # Framed output for the host runner
//!
//! ```text
//! adc win in=2052 hi=2050 lo=2047 iv=6 fired=2   (info, skipped by host)
//! ADC_WIN_TEST_BEGIN
//! ADC WIN IN OK
//! ADC WIN HI OK
//! ADC WIN LO OK
//! ADC WIN CLEAR OK
//! ADC WIN IV OK
//! ADC WIN MON OK
//! ADC WIN WAKE OK
//! ADC WIN RESTORE OK
//! ADC_WIN_TEST_END
//! ```

use core::cell::Cell;

use critical_section::Mutex;
use hal::adc::{self, Adc, Config as AdcConfig, SampleTime, Window, WindowEvents, WindowVerdict};
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

/// Last `ADC12IV` value the ISR observed, and its firing tally.
static IV_SEEN: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static FIRED: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// ADC12 window-crossing ISR: demux (the IV read clears the reported flag),
/// then **disarm** — a free-running monitor re-latches the crossing every
/// conversion (~60 µs apart), so a keep-armed handler would re-enter forever
/// (the capture module's ACLK starvation lesson, ADC edition). `wake_cpu`
/// lets `main` resume after `enter_lpm0()`.
#[msp430_rt::interrupt(wake_cpu)]
fn ADC12() {
    let iv = adc::read_iv();
    adc::isr_disable_window_interrupts();
    critical_section::with(|cs| {
        IV_SEEN.borrow(cs).set(iv);
        let f = FIRED.borrow(cs);
        f.set(f.get().wrapping_add(1));
    });
}

/// ISR firing tally so far.
fn fired() -> u16 {
    critical_section::with(|cs| FIRED.borrow(cs).get())
}

/// Last IV the ISR captured.
fn iv_seen() -> u16 {
    critical_section::with(|cs| IV_SEEN.borrow(cs).get())
}

/// The supply monitor's plausibility band at 12-bit: half scale ± 200.
fn in_band(counts: u16) -> bool {
    (1848..=2248).contains(&counts)
}

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

    tx.write_all(b"\r\nMSP430FR5969 ADC12 window comparator self-check (no wiring)\r\n")
        .ok();

    // 12-bit, MODOSC, long sample time — the internal supply divider is
    // high-impedance and under-samples at the default 32 cycles.
    let mut adc = Adc::new(
        p.adc12,
        AdcConfig::default().sample_time(SampleTime::Cycles256),
    );

    // The supply monitor reads ~2048; three windows that place that reading
    // inside, above, and below the window respectively.
    let win_around = Window::new(1848, 2248).unwrap(); // reading inside
    let win_below_reading = Window::new(16, 1024).unwrap(); // reading above it
    let win_above_reading = Window::new(3072, 4080).unwrap(); // reading below it

    // --- 1: comparator says Within, INIFG alone ----------------------------
    let (c_in, v_in) = adc.read_supply_half_windowed(&win_around);
    let f = adc::window_flags();
    let in_ok =
        v_in == WindowVerdict::Within && f.within && !f.above && !f.below && in_band(c_in);

    // --- 2: comparator says Above, HIIFG alone ------------------------------
    let (c_hi, v_hi) = adc.read_supply_half_windowed(&win_below_reading);
    let f = adc::window_flags();
    let hi_ok = v_hi == WindowVerdict::Above && f.above && !f.within && !f.below && in_band(c_hi);

    // --- 3: comparator says Below, LOIFG alone ------------------------------
    let (c_lo, v_lo) = adc.read_supply_half_windowed(&win_above_reading);
    let f = adc::window_flags();
    let lo_ok = v_lo == WindowVerdict::Below && f.below && !f.within && !f.above && in_band(c_lo);

    // --- 4: flags clear, and stay clear across a non-windowed read ----------
    adc::clear_window_flags();
    let cleared = adc::window_flags().none();
    let _ = adc.read_supply_half(); // plain read: MCTL0 rewritten without WINC
    let clear_ok = cleared && adc::window_flags().none();

    // --- 5: one-shot windowed conversion delivers IV 0x06 from LPM0 ---------
    // GIE is off (never enabled yet): the crossing latches while we set up,
    // and `enter_lpm0` (which sets GIE atomically with sleeping) delivers it
    // — the comp fixture's race-free latch-then-sleep pattern.
    let f0 = fired();
    adc.enable_window_interrupts(WindowEvents {
        above: true,
        ..WindowEvents::default()
    });
    adc.start_supply_half_windowed(&win_below_reading);
    hal::power::enter_lpm0();
    let iv_first = iv_seen();
    let iv_ok = iv_first == adc::IV_WINDOW_ABOVE
        && fired().wrapping_sub(f0) == 1
        // The IV read is the acknowledge: HIIFG must be down again.
        && adc::window_flags().none();

    // --- 6: in-window monitor fires nothing (but really runs) ---------------
    let f0 = fired();
    adc.start_supply_half_monitor(&win_around, WindowEvents::outside());
    delay.delay_ms(20); // hundreds of conversions at ~60 µs each
    let f = adc::window_flags();
    let mon_ok = fired().wrapping_sub(f0) == 0 && f.within && !f.above && !f.below;
    adc.stop_monitor();

    // --- 7: out-of-range monitor wakes LPM0 exactly once --------------------
    // GIE went live in phase 5's wake and stays set; drop it so the arm →
    // sleep window is race-free again (the first conversion may complete in
    // ~60 µs, comparable to the instructions between).
    msp430::interrupt::disable();
    let f0 = fired();
    adc.start_supply_half_monitor(&win_below_reading, WindowEvents {
        above: true,
        ..WindowEvents::default()
    });
    hal::power::enter_lpm0();
    // The monitor is still free-running out-of-range: milliseconds' worth of
    // conversions re-latch HIIFG, but the ISR disarmed — the tally must hold.
    delay.delay_ms(5);
    let wake_fired = fired().wrapping_sub(f0);
    adc.stop_monitor();
    let wake_ok = wake_fired == 1 && iv_seen() == adc::IV_WINDOW_ABOVE;

    // --- 8: single-conversion contract restored -----------------------------
    let c_after = adc.read_supply_half();
    let restore_ok = in_band(c_after) && adc::window_flags().none();

    let all_ok =
        in_ok && hi_ok && lo_ok && clear_ok && iv_ok && mon_ok && wake_ok && restore_ok;

    let mut on = false;
    loop {
        // Human-readable info line (the host skips everything up to BEGIN).
        tx.write_all(b"adc win in=").ok();
        write_dec(&mut tx, c_in as u32);
        tx.write_all(b" hi=").ok();
        write_dec(&mut tx, c_hi as u32);
        tx.write_all(b" lo=").ok();
        write_dec(&mut tx, c_lo as u32);
        tx.write_all(b" iv=").ok();
        write_dec(&mut tx, iv_first as u32);
        tx.write_all(b" fired=").ok();
        write_dec(&mut tx, fired() as u32);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"ADC_WIN_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"ADC WIN IN", in_ok);
        verdict(&mut tx, b"ADC WIN HI", hi_ok);
        verdict(&mut tx, b"ADC WIN LO", lo_ok);
        verdict(&mut tx, b"ADC WIN CLEAR", clear_ok);
        verdict(&mut tx, b"ADC WIN IV", iv_ok);
        verdict(&mut tx, b"ADC WIN MON", mon_ok);
        verdict(&mut tx, b"ADC WIN WAKE", wake_ok);
        verdict(&mut tx, b"ADC WIN RESTORE", restore_ok);
        tx.write_all(b"ADC_WIN_TEST_END\r\n").ok();

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
