#![no_std]
#![no_main]
// The #[interrupt] macro emits handlers with the unstable `extern
// "msp430-interrupt"` ABI (so they end in RETI, not RET). That ABI is gated
// behind this nightly feature in the crate that *contains* the generated ISR.
#![feature(abi_msp430_interrupt)]
// `#[interrupt(wake_cpu)]` additionally emits a naked-asm trampoline (it clears
// the low-power bits in the stacked SR), which needs inline asm enabled here.
#![feature(asm_experimental_arch)]

use core::cell::Cell;
use critical_section::Mutex;
use hal::clocks::AclkSource;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
// Vector-name shim: `#[msp430_rt::interrupt]` validates the handler name against
// `interrupt::TIMER0_A1`, so the module must be in scope.
use hal::interrupt;
use hal::power;
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
/// Fires every time the 16-bit counter wraps 0xFFFF→0x0000 (≈2 s with the
/// counter on the 32.768 kHz ACLK). Its whole job: clear the hardware flag and
/// bump the software tally, which [`Counter::now64`] later combines with `TA0R`
/// into a 32-bit timestamp — and it keeps tallying *during* LPM3 sleep, since
/// the ACLK overflow still fires (the CPU briefly wakes to service this plain
/// handler, then RETIs back to sleep). `#[msp430_rt::interrupt]` emits the
/// `msp430-interrupt` ABI (RETI) and overrides the weak default vector by name.
#[msp430_rt::interrupt]
fn TIMER0_A1(cs: CriticalSection) {
    hal::timer::clear_overflow_irq();
    let ovf = OVERFLOWS.borrow(cs);
    ovf.set(ovf.get().wrapping_add(1));
}

/// CCR0 compare-match ISR — wakes the CPU from LPM3 at the scheduled tick.
///
/// `#[interrupt(wake_cpu)]` is the key: it clears the low-power bits (CPUOFF/
/// SCG0/SCG1) in the *stacked* status register before RETI, so the part returns
/// to active mode at the `enter_lpm3()` call site instead of dropping back to
/// sleep. The handler only disarms the one-shot wake; the elapsed-time
/// measurement happens back in `main` once the CPU is running again.
#[msp430_rt::interrupt(wake_cpu)]
fn TIMER0_A0() {
    hal::timer::clear_wake_irq();
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

    // Configure the clock tree first (single source of truth for frequencies).
    // Step 4 uses the **low-power** profile: MCLK/SMCLK = 1 MHz, and crucially
    // ACLK on the 32.768 kHz LFXT crystal — the one clock that keeps running in
    // LPM3, so an ACLK-sourced timer can measure and wake through deep sleep.
    let clocks = hal::clocks::configure_low_power(p.cs);

    // Unlock GPIO pins (clear LOCKLPM5 in PM5CTL0) so the UART pin mux takes
    // effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // Configure eUSCI_A0 as a 9600 8N1 UART. BRCLK = SMCLK = 1 MHz under the
    // low-power profile; the baud math derives from clocks.smclk(), so it adapts
    // automatically. UCA0TXD = P2.0, UCA0RXD = P2.1.
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
    if clocks.aclk_source() == AclkSource::Lfxt {
        tx.write_all(b"ACLK = LFXT 32768 Hz (crystal)\r\n").ok();
    } else {
        tx.write_all(b"ACLK = VLO (crystal failed; timing approximate)\r\n").ok();
    }

    // Free-running counter on Timer0_A3, clocked from **ACLK** ÷1. With the
    // crystal that is 32.768 kHz → ~30.5 µs/tick and a ~2 s wrap. This clock
    // survives LPM3, which is the whole point of step 4.
    let counter = Counter::new_aclk(p.timer_0_a3, &clocks, Divider::Div1);

    // Overflow counting + GIE (from step 2): now64() spans the ~2 s wrap, and the
    // TIMER0_A1 ISR keeps tallying even during sleep. GIE also lets the CCR0
    // compare wake fire. Arm the overflow source, then unmask globally.
    counter.enable_overflow_interrupt();
    unsafe { msp430::interrupt::enable() };

    // Ticks in one second at the ACLK rate — derived from the counter so it is
    // correct whether ACLK ended up on the crystal (32768) or the VLO fallback.
    // Stays < 65536, so it fits the 16-bit CCR0 compare.
    let one_sec = counter.tick_hz() as u16;

    // Each iteration: timestamp, schedule a wake ~1 s out, drop into LPM3 (CPU,
    // MCLK, SMCLK, DCO all off — only ACLK + this timer alive), and on wake
    // measure how much time the counter logged while we slept. Predict ≈
    // 1_000_000 µs: proof the timer ran through deep sleep. The print happens
    // *after* wake so SMCLK (the UART clock) is running for it.
    let mut buf = [0u8; 12];
    loop {
        let start = critical_section::with(|cs| counter.now64(OVERFLOWS.borrow(cs).get()));
        counter.schedule_wake(counter.now().wrapping_add(one_sec));

        // Both LEDs off during sleep (true low power); red blinks on each wake.
        green_led.set_low().ok();
        red_led.set_low().ok();
        power::enter_lpm3(); // deep sleep until the CCR0 compare wakes us (~1 s)
        red_led.set_high().ok();

        let end = critical_section::with(|cs| counter.now64(OVERFLOWS.borrow(cs).get()));
        let elapsed = end.wrapping_sub(start);

        tx.write_all(b"slept in LPM3, measured ").ok();
        tx.write_all(format_u32(counter.ticks_to_us(elapsed), &mut buf)).ok();
        tx.write_all(b" us across deep sleep\r\n").ok();
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
