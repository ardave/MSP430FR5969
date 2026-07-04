#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits a handler with the `extern "msp430-interrupt"`
// ABI (RETI, not RET); the `wake_cpu` variant additionally emits inline asm to
// clear the low-power bits in the stacked SR. Both are still nightly-gated.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! UART RX-interrupt integration fixture: `Rx::enable_rx_interrupt` +
//! `serial::isr_read_byte` on the `USCI_A0` vector, with an
//! [`hal::rx_queue::RxQueue`] carrying bytes from the ISR to a main loop that
//! sleeps in **LPM0** between characters.
//!
//! Unlike every other fixture, this one is **two-directional**: the host-side
//! `serial_irq_tests` runner *sends* bytes down the backchannel (eUSCI_A0,
//! 9600 8N1 on `/dev/cu.usbmodem11203`) and verifies what comes back. The
//! board echoes every received byte **plus one** — `b'A'` comes back `b'B'` —
//! which is the point: a wire loopback or a polled echo could return the byte
//! unchanged, but only software that actually received `b` through the
//! ISR → queue → main path can transmit `b + 1`.
//!
//! ```text
//! cargo +nightly build --bin serial_irq_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/serial_irq_test_runner
//! ```
//!
//! # What the host checks
//!
//! 1. **Echo integrity.** Every byte of a known pattern comes back `+1`, in
//!    order — ISR reception, queue FIFO order, and no byte loss, in one pass.
//! 2. **No overflow.** After a `\n`-terminated line the board reports
//!    `UART_IRQ_STATS dropped=N`; the host asserts `N == 0` (the queue's
//!    drop-newest counter never fired at line rate).
//! 3. **LPM0 wake.** The main loop parks in `enter_lpm0()` and only the
//!    `USCI_A0` ISR (`wake_cpu`) resumes it — an echo arriving at all proves
//!    the RX interrupt woke the CPU. (LPM0, not LPM3: BRCLK = SMCLK, which
//!    LPM3 would stop — an ACLK-clocked UART for LPM3 listening is future
//!    work.)
//!
//! Until the first byte arrives the board emits `UART_IRQ_READY` once per
//! second so the host knows when to start transmitting. GREEN toggles on
//! every processed byte; there are no self-computed verdicts — the *host*
//! holds the expectations here.
//!
//! Human check: `screen /dev/cu.usbmodem11203 9600`, type — each key echoes
//! as the next character up (`a` → `b`), served from a CPU that was asleep.

use core::cell::Cell;

use critical_section::Mutex;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::rx_queue::RxQueue;
use hal::serial::{self, Config as UartConfig, SerialExt, UsciA0};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// ISR → main byte queue. 32 bytes ≈ 33 ms of consumer latency at 9600 baud —
/// deep headroom for a main loop that wakes on every byte. All access (ISR
/// push, main pop) inside `critical_section::with`; that discipline is what
/// makes the `Cell`-based queue sound.
static RX: Mutex<RxQueue<32>> = Mutex::new(RxQueue::new());

/// Tally of receive-path errors (framing/parity/overrun/break) seen by the
/// ISR — surfaced in the stats line rather than silently swallowed.
static RX_ERRORS: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));

/// USCI_A0 RX ISR: consume RXBUF (this is what clears `UCRXIFG`) and queue
/// the byte. `wake_cpu` resumes the LPM0-parked main loop to drain it.
#[msp430_rt::interrupt(wake_cpu)]
fn USCI_A0() {
    match serial::isr_read_byte::<UsciA0>() {
        Ok(byte) => critical_section::with(|cs| {
            RX.borrow(cs).push(byte);
        }),
        // WouldBlock = spurious (nothing latched) — nothing to do. A real
        // receive error consumed the corrupt byte; count it.
        Err(hal::embedded_hal_nb::nb::Error::WouldBlock) => {}
        Err(hal::embedded_hal_nb::nb::Error::Other(_)) => critical_section::with(|cs| {
            let e = RX_ERRORS.borrow(cs);
            e.set(e.get().saturating_add(1));
        }),
    }
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Performance profile: SMCLK = 8 MHz — BRCLK for the UART, which is why
    // the sleep below is LPM0 (SMCLK must keep running to receive).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the pin muxes take effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, mut rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let mut green_led = port1.pin0.into_output(); // LED2, per-byte blink

    let mut delay = Delay::new(clocks.mclk());

    // From here on the ISR owns RXBUF; thread-mode reads would only ever see
    // WouldBlock. GIE up front — bytes must queue even during the READY
    // announce phase below (host may transmit the moment it sees READY).
    rx.enable_rx_interrupt();
    // SAFETY: all state shared with the ISR sits in critical-section Mutexes.
    unsafe {
        msp430::interrupt::enable();
    }

    tx.write_all(b"\r\nMSP430FR5969 UART RX-interrupt echo (byte+1)\r\n")
        .ok();

    // Two phases. Until the first byte arrives, announce READY once per
    // second from an *active-mode* delay — the eZ-FET gates our TX on the
    // host's DTR, so anything sent before the host attaches is simply lost,
    // and a board parked in LPM0 has no clock to re-announce on. Once traffic
    // has started the loop flips to the real pattern: park in LPM0 and let
    // the RX ISR (wake_cpu) resume it per byte.
    let mut seen_any = false;
    let mut on = false;
    loop {
        if seen_any {
            // Sleep until the RX ISR wakes us with at least one byte queued.
            tx.flush().ok();
            hal::power::enter_lpm0();
        } else {
            tx.write_all(b"UART_IRQ_READY\r\n").ok();
            delay.delay_ms(1000); // bytes landing mid-delay queue via the ISR
        }

        // Drain everything the ISR queued and echo each byte + 1. A '\n'
        // terminator additionally triggers the stats line the host asserts on.
        loop {
            let byte = critical_section::with(|cs| RX.borrow(cs).pop());
            let Some(byte) = byte else { break };
            seen_any = true;
            on = !on;
            if on {
                green_led.set_high().ok();
            } else {
                green_led.set_low().ok();
            }

            tx.write_all(&[byte.wrapping_add(1)]).ok();

            if byte == b'\n' {
                let (dropped, errors) = critical_section::with(|cs| {
                    (RX.borrow(cs).dropped(), RX_ERRORS.borrow(cs).get())
                });
                tx.write_all(b"\r\nUART_IRQ_STATS dropped=").ok();
                write_dec(&mut tx, dropped as u32);
                tx.write_all(b" errors=").ok();
                write_dec(&mut tx, errors as u32);
                tx.write_all(b"\r\n").ok();
            }
        }
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
