#![no_std]
#![no_main]
// The #[interrupt] macro emits handlers with the unstable `extern
// "msp430-interrupt"` ABI (so they end in RETI, not RET). That ABI is gated
// behind this nightly feature in the crate that *contains* the generated ISR.
#![feature(abi_msp430_interrupt)]

use core::cell::Cell;
use critical_section::Mutex;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
// Vector-name shim: `#[msp430_rt::interrupt]` validates the handler name against
// `interrupt::TIMER0_A1`, so the module must be in scope.
use hal::interrupt;
use hal::serial::{Config, SerialExt};
use hal::timer::{Counter, Divider};
use msp430_rt::entry;

// The `msp430` crate provides the critical-section implementation for MSP430
// (acquire: read SR then DINT+NOP, release: restore GIE if it was set) and the
// global-interrupt-enable used below. Referenced directly so its symbols link.
use msp430 as _;

/// Counter-overflow tally for Timer0_A3, shared between the `TIMER0_A1` ISR (the
/// writer) and `main` (the reader). A `critical-section` `Mutex` makes the
/// cross-context access sound: the ISR receives a `CriticalSection` token from
/// the `#[interrupt]` macro, and `main` obtains one via `critical_section::with`
/// — neither can touch the cell without proving interrupts are masked.
static OVERFLOWS: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// Timer0_A3 counter-overflow ISR — the project's first real interrupt.
///
/// Fires every time the 16-bit counter wraps 0xFFFF→0x0000 (≈65.5 ms at the
/// 1 MHz tick). Its whole job: clear the hardware flag and bump the software
/// tally, which [`Counter::now64`] later combines with `TA0R` into a 32-bit
/// timestamp. `#[msp430_rt::interrupt]` emits the correct `msp430-interrupt`
/// ABI (RETI) and overrides the weak default vector by symbol name; the `cs`
/// argument is the macro-supplied critical-section token.
#[msp430_rt::interrupt]
fn TIMER0_A1(cs: CriticalSection) {
    hal::timer::clear_overflow_irq();
    let ovf = OVERFLOWS.borrow(cs);
    ovf.set(ovf.get().wrapping_add(1));
}

// Watchdog Timer Password
const WDTPW: u16 = 0x5A00;
// Watchdog Timer Hold.  Setting it stops (pauses) the watchdog timer.
const WDTHOLD: u16 = 0x0080;

/// Firmware entry point.
///
/// `#[entry]` (from msp430-rt) names the function the runtime calls after reset.
/// msp430-rt now owns everything `_start` used to do by hand: its `Reset`
/// handler loads the stack pointer from `_stack_start` (verify with `objdump -d`
/// that `Reset` opens with `mov #0x2400, r1`), zeroes `.bss` and copies `.data`,
/// then jumps here. The reset and interrupt vectors come from the PAC's `rt`
/// feature + msp430-rt's linker script, so there is no naked `_start`, no
/// `.reset_vector` static and no manual `.bss` loop in this crate anymore.
///
/// (An uninitialized stack pointer was the bug this used to guard against: it
/// masqueraded as a UART that hung on its first transmission only some of the
/// time. msp430-rt's Reset closes that hole for us now.)
#[entry]
fn main() -> ! {
    // Stop the watchdog before anything else. msp430-rt initializes RAM but does
    // not touch the WDT, and the default timeout is ~32 ms; Peripherals::take()
    // below also enters a critical section. Raw access because we don't hold the
    // peripheral singletons yet.
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    let p = hal::pac::Peripherals::take().unwrap();

    // Configure the clock tree first: this owns the CS module and returns the
    // resulting frequencies, which every clocked peripheral below reads from
    // (single source of truth). Performance profile: MCLK stays 1 MHz; SMCLK is
    // bumped to the full 8 MHz DCO for fine-resolution peripheral timing.
    // (hal::clocks::configure_low_power puts ACLK on the 32.768 kHz LFXT crystal
    // for LPM3 sleep instead — use that for the sleep-based watchdog.)
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO pins (clear LOCKLPM5 in PM5CTL0) so the UART pin mux takes
    // effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // Configure eUSCI_A0 as a 9600 8N1 UART. BRCLK = SMCLK, now 8 MHz (from the
    // clocks config above), so the baud math is derived from clocks.smclk()
    // rather than a hard-coded number. UCA0TXD = P2.0, UCA0RXD = P2.1.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(Config::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs on the MSP430FR5969 LaunchPad: P1.0 = LED2 (GREEN), P4.6 = LED1 (RED).
    // (Verified on hardware — the colours are the opposite of what's often
    // assumed; the UART labels would not match the LEDs if these were swapped.)
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    tx.write_all(b"MSP430FR5969 UART up @ 9600 8N1\r\n").ok();

    // Software cycle-counting delay, calibrated for the reset MCLK (1 MHz, the
    // same clock SMCLK derives the UART BRCLK from above). This replaces the old
    // hand-tuned black_box busy loop with the HAL's `DelayNs` impl; still a
    // software delay (approximate, biased slightly long), but now expressed in
    // real time units and shared logic. A hardware timer remains the proper fix
    // once the clock/timer HAL exists. MCLK comes from the clocks config (1 MHz).
    let mut delay = Delay::new(clocks.mclk());

    // Free-running counter on Timer0_A3, clocked from SMCLK (8 MHz here) ÷8 =
    // 1 MHz, so one tick = 1 µs and the 16-bit counter wraps every 65.5 ms.
    // This is an *independent* time reference from `delay`: the delay spends
    // MCLK cycles, the counter measures SMCLK ticks, so timing one with the
    // other is a genuine cross-check rather than a tautology.
    let counter = Counter::new_smclk(p.timer_0_a3, &clocks, Divider::Div8);

    // Step 2 left in place: overflow counting + GIE, so the counter still spans
    // long intervals and the TIMER0_A1 ISR keeps tallying wraps underneath us.
    counter.enable_overflow_interrupt();
    unsafe { msp430::interrupt::enable() };

    // Step 3: hardware capture. CCR1 in capture mode latches TA0R the instant an
    // edge arrives — here a software-toggled internal edge (no pin). The point:
    // the captured timestamp reflects the *event*, not when software reads it.
    counter.configure_capture();

    // Each loop: mark an "event" (a software read at the trigger instant, plus a
    // hardware capture at the same instant), then wait 5 ms to simulate latency
    // between the event and servicing it. Afterwards, re-read the *frozen*
    // capture and the *live* counter:
    //   - jitter  = capture - trigger-read: a few µs, how closely the hardware
    //     latch tracked the event (predict ~0).
    //   - drift   = live - frozen: ~5000 µs, how wrong you'd be reading the
    //     counter at service time instead of capturing at event time. The
    //     capture is immune to it — that's the whole reason capture mode exists.
    let mut buf = [0u8; 12];
    loop {
        red_led.set_high().ok();
        green_led.set_low().ok();

        let trigger = counter.now(); // software timestamp at ~the event instant
        let cap = counter.software_capture(); // hardware latch at ~the event instant

        delay.delay_ms(5); // latency between the event and getting around to it

        let frozen = counter.capture_value(); // capture re-read 5 ms later — unchanged
        let live = counter.now(); // live counter 5 ms later

        let jitter = cap.wrapping_sub(trigger);
        let drift = live.wrapping_sub(frozen);

        tx.write_all(b"capture jitter ").ok();
        tx.write_all(format_u32(counter.ticks_to_us(jitter as u32), &mut buf)).ok();
        tx.write_all(b" us; 5ms-late read drifted ").ok();
        tx.write_all(format_u32(counter.ticks_to_us(drift as u32), &mut buf)).ok();
        tx.write_all(b" us (capture immune)\r\n").ok();

        green_led.set_high().ok();
        red_led.set_low().ok();
        delay.delay_ms(1000); // pacing between measurements
    }
}

/// Format `n` as decimal ASCII into `buf`, returning the written slice.
///
/// Hand-rolled because `hal::serial` deliberately does not implement
/// `core::fmt::Write` — pulling in `core::fmt` would blow the FRAM budget (see
/// CLAUDE.md). Digits are generated least-significant first into the tail of the
/// buffer, then the filled tail is returned.
fn format_u32(mut n: u32, buf: &mut [u8; 12]) -> &[u8] {
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (n % 10) as u8;
        n /= 10;
        if n == 0 {
            break;
        }
    }
    &buf[i..]
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// compiler-builtins' memcpy/memcmp routines reference `abort` on their safety
// paths. Provide a minimal one so we don't have to link newlib's libc (which
// would in turn pull in unhosted syscall stubs: _exit, kill, getpid).
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
