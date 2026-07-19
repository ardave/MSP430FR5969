#![no_std]
#![no_main]

//! Capacitive touch sensing on a bare header pin — no external parts.
//!
//! CAPTIO0 turns pad **P3.0** into a relaxation oscillator counted by its
//! silicon-paired timer (TA2). The pad's parasitic capacitance sets the
//! untouched frequency (a few MHz); a fingertip on the header pin adds a few
//! picofarads and drops it sharply. The demo measures an untouched baseline
//! at boot, then lights the green LED (LED2) whenever the frequency falls
//! more than 20 % below it.
//!
//! Touch the metal of the P3.0 header pin (or a jumper wire plugged into
//! it — more metal, bigger signal). Don't hold the board by that pin while
//! it boots: the baseline would be a touched reading.
//!
//! ```text
//! cargo +nightly build --example touch --features rt,critical-section
//! tools/flash.sh target/msp430-none-elf/debug/examples/touch
//! ```

use msp430fr5969_hal as hal;

use hal::captio::TouchSense;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::gpio::GpioExt;
use hal::timer::{Counter, Divider};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// 2 ms measurement gate at the counter's 1 MHz tick — resolves up to
/// 32.7 MHz without wrapping the 16-bit oscillation count (bare pads run
/// 3–7 MHz).
const GATE_TICKS: u16 = 2_000;

#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in
    // that order — hal::peripherals::take fuses them so the ordering can't be gotten wrong.
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz.
    let clocks = hal::clocks::configure(p.cs);
    hal::gpio::unlock_pins(&p.pmm);

    let (port1, _port2) = p.port_1_2.split();
    let (port3, _port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output(); // LED2 = P1.0

    // The touch pad: a floating input — the CAPTIO module drives the pad
    // through its own pull-resistor control, so no GPIO driver or pull may
    // fight it.
    let pad = port3.pin0.into_floating_input();

    let mut delay = Delay::new(clocks.mclk());

    // The frequency yardstick: SMCLK/8 = 1 MHz tick.
    let counter = Counter::new_smclk(p.timer_0_a3, &clocks, Divider::Div8);

    // CAPTIO0 with its paired timer TA2; route the pad and it oscillates.
    let mut touch = TouchSense::new(p.capacitive_touch_io_0, p.timer_2_a2);
    touch.route_pin(&pad);

    // Untouched baseline: the average of 8 gated measurements. A wrapped
    // count (`None` — oscillation too fast for the gate) simply doesn't
    // contribute; a pad reading 0 would make the threshold 0 and the LED
    // stay dark, which is the honest failure mode.
    let mut sum: u32 = 0;
    let mut n: u32 = 0;
    for _ in 0..8 {
        if let Some(hz) = touch.measure_hz(&counter, GATE_TICKS) {
            sum += hz;
            n += 1;
        }
        delay.delay_ms(10);
    }
    let baseline = if n > 0 { sum / n } else { 0 };
    // Touched = more than 20 % below baseline (a fingertip typically drops
    // the frequency far more; 20 % clears gate jitter by orders of
    // magnitude).
    let threshold = baseline - baseline / 5;

    loop {
        let touched = match touch.measure_hz(&counter, GATE_TICKS) {
            Some(hz) => hz < threshold,
            None => false,
        };
        if touched {
            green_led.set_high().ok();
        } else {
            green_led.set_low().ok();
        }
        delay.delay_ms(30);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// compiler-builtins' memcpy/memcmp reference `abort` on their safety paths.
// Provide a minimal one so we don't link newlib's libc (and its syscall stubs).
#[no_mangle]
pub extern "C" fn abort() -> ! {
    loop {}
}
