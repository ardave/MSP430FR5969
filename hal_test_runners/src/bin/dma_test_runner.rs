#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits handlers with the `extern "msp430-interrupt"`
// ABI (RETI, not RET) — still nightly-gated.
#![feature(abi_msp430_interrupt)]

//! DMA integration fixture: the three-channel controller end to end, framed
//! for the host-side `dma_tests` runner. **Hands-free** — no jumpers, no
//! external parts; the only I/O is the backchannel UART (eUSCI_A0, 9600 8N1),
//! and *every byte of it is itself moved by DMA* (`serial::DmaTx` on channel
//! 0), so the report arriving intact is the UART-TX-pacing test.
//!
//! ```text
//! cargo +nightly build --bin dma_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/dma_test_runner
//! ```
//!
//! # What it checks
//!
//! On-device verdicts, computed once at boot:
//!
//! - **COPYB / COPYW** — software-triggered block copies (`copy_bytes` /
//!   `copy_words`): destination matches a 64-byte / 32-word pattern and the
//!   source survives untouched.
//! - **FILL / REV** — the fixed-source and decrementing-destination address
//!   modes (`fill_bytes`, `copy_bytes_reversed`).
//! - **IRQ** — each channel is armed software-paced (`arm_single_bytes` +
//!   `request`), fires the shared `DMA` vector, and the ISR's
//!   `dma::read_iv()` reports exactly 0x02/0x04/0x06 for channel 0/1/2,
//!   exactly once each, with the moved byte landing.
//!
//! Host-verified phases, repeating once per second:
//!
//! - The framed burst below arrives intact (DMA-paced TX of every line,
//!   including a fixed 36-byte `TXPAT` payload the host string-compares).
//! - **RX pacing**, two rounds per served loop. Round 1: the fixture arms
//!   its RX channel for a 16-byte payload **before** announcing
//!   `DMA_RX_READY n=16`, so even the first byte the host sends meets an
//!   already-armed channel — every byte lands by DMA, no CPU in the loop.
//!   (Design lesson, hardware-observed: a CPU poll loop cannot open a
//!   9600-baud stream — with `delay_ms(1)` really costing ~3.5 ms, back-to-
//!   back bytes 1.04 ms apart overrun RXBUF between polls. Arm first;
//!   announce second.) Round 2 repeats the exchange through the blocking
//!   public API `Rx::read_exact_dma` (safe to block: a host that completed
//!   round 1 is committed). Both payloads are echoed back **plus one**
//!   through the DMA transmit path — only software that actually collected
//!   the DMA'd buffers can produce the transform.
//!
//! # Framed output for the host runner
//!
//! ```text
//! DMA_TEST_BEGIN
//! DMA COPYB OK                          (or `... FAIL`)
//! DMA COPYW OK
//! DMA FILL OK
//! DMA REV OK
//! DMA IRQ OK
//! DMA TXPAT 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ
//! DMA_TEST_END
//! DMA_RX_READY n=16                     (armed; ~5 s window to send 16 bytes)
//! DMA_ECHO <16 payload bytes each +1>   (only after a served round 1)
//! DMA_RX2_READY n=16                    (read_exact_dma round, blocking)
//! DMA_ECHO2 <16 payload bytes each +1>
//! ```
//!
//! GREEN = all on-device verdicts OK; RED = something failed (the burst says
//! what). Human check: `screen /dev/cu.usbmodem11203 9600`, type any 16
//! characters within ~5 s of a READY — they come back shifted by one (then
//! 16 more for round 2).

use core::cell::Cell;

use critical_section::Mutex;
use hal::delay::Delay;
use hal::dma::{AddrMode, Channel, DmaExt, TriggerSource};
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

/// Per-source tally from the `DMA` ISR: hits for DMAIV = 0x02 / 0x04 / 0x06
/// (channels 0/1/2) and, in the last slot, anything unexpected (0 included —
/// a spurious entry would land there).
static IRQ_HITS: Mutex<Cell<[u8; 4]>> = Mutex::new(Cell::new([0; 4]));

/// Shared `DMA` vector: demux and consume via `DMAIV` (the read clears the
/// reported channel's flag in silicon) and tally which source it was.
#[msp430_rt::interrupt]
fn DMA() {
    let iv = hal::dma::read_iv();
    critical_section::with(|cs| {
        let hits = IRQ_HITS.borrow(cs);
        let mut counts = hits.get();
        let slot = match iv {
            0x02 => 0,
            0x04 => 1,
            0x06 => 2,
            _ => 3,
        };
        counts[slot] = counts[slot].saturating_add(1);
        hits.set(counts);
    });
}

/// The fixed pattern whose intact, DMA-transmitted arrival the host asserts.
const TXPAT: &[u8] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

/// RX phase geometry: payload bytes per round.
const RX_LEN: usize = 16;

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (BRCLK for the UART).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the UART pin mux takes effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0) 9600 8N1 — the report channel *and* the RX test bus.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (tx, mut rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1).
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());

    // The controller, split (this also sets DMARMWDIS module-wide).
    let channels = p.dma.split();
    let mut ch0 = channels.ch0;
    let mut ch1 = channels.ch1;
    let mut ch2 = channels.ch2;

    // ------------------------------------------------------------------
    // On-device self tests (software-triggered block moves + the IRQ path),
    // all before the UART transport is even touched.
    // ------------------------------------------------------------------

    // COPYB: 64-byte block copy, destination matches, source untouched.
    let mut src_b = [0u8; 64];
    for (i, s) in src_b.iter_mut().enumerate() {
        *s = (i as u8).wrapping_mul(7).wrapping_add(3);
    }
    let mut dst_b = [0u8; 64];
    let moved = ch0.copy_bytes(&src_b, &mut dst_b);
    let copyb_ok = moved == 64
        && dst_b == src_b
        && src_b
            .iter()
            .enumerate()
            .all(|(i, &s)| s == (i as u8).wrapping_mul(7).wrapping_add(3));

    // COPYW: 32-word block copy at the bus's native width.
    let mut src_w = [0u16; 32];
    for (i, s) in src_w.iter_mut().enumerate() {
        *s = (i as u16).wrapping_mul(0x0113).wrapping_add(0x2447);
    }
    let mut dst_w = [0u16; 32];
    let moved = ch1.copy_words(&src_w, &mut dst_w);
    let copyw_ok = moved == 32 && dst_w == src_w;

    // FILL: fixed-source address mode sweeping a constant across a buffer.
    let mut fill = [0u8; 32];
    let moved = ch2.fill_bytes(0x5A, &mut fill);
    let fill_ok = moved == 32 && fill.iter().all(|&b| b == 0x5A);

    // REV: decrementing-destination mode — src[i] lands at dst[n-1-i].
    let rev_src = *b"DMA reversed me!";
    let mut rev_dst = [0u8; 16];
    let moved = ch0.copy_bytes_reversed(&rev_src, &mut rev_dst);
    let rev_ok = moved == 16
        && rev_dst
            .iter()
            .zip(rev_src.iter().rev())
            .all(|(&got, &want)| got == want);

    // IRQ: software-paced single transfers so the *ISR* owns the completion
    // flag (the blocking APIs poll-and-clear it themselves and would race).
    // GIE up for the rest of the run; all ISR-shared state is in Mutexes.
    unsafe { msp430::interrupt::enable() };
    let irq_ok = irq_test(&mut ch0, [1, 0, 0, 0], &mut delay)
        && irq_test(&mut ch1, [1, 1, 0, 0], &mut delay)
        && irq_test(&mut ch2, [1, 1, 1, 0], &mut delay);

    let all_ok = copyb_ok && copyw_ok && fill_ok && rev_ok && irq_ok;

    // ------------------------------------------------------------------
    // Report + RX-pacing loop. From here on, channel 0 belongs to the
    // transmit path (every report byte is DMA-moved) and channel 1 serves
    // the RX rounds.
    // ------------------------------------------------------------------
    let mut dma_tx = tx.with_dma(ch0);

    loop {
        if all_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        dma_tx.write_all(b"DMA_TEST_BEGIN\r\n").ok();
        verdict(&mut dma_tx, b"DMA COPYB", copyb_ok);
        verdict(&mut dma_tx, b"DMA COPYW", copyw_ok);
        verdict(&mut dma_tx, b"DMA FILL", fill_ok);
        verdict(&mut dma_tx, b"DMA REV", rev_ok);
        verdict(&mut dma_tx, b"DMA IRQ", irq_ok);
        dma_tx.write_all(b"DMA TXPAT ").ok();
        dma_tx.write_all(TXPAT).ok();
        dma_tx.write_all(b"\r\nDMA_TEST_END\r\n").ok();

        // RX round 1: arm the channel for the WHOLE payload *before*
        // announcing readiness, so the host's first byte — and every byte
        // after it — meets an already-armed channel and lands by DMA. (An
        // earlier design had the CPU consume an opening go-byte from a poll
        // loop here; hardware said no: `delay_ms(1)` really costs ~3.5 ms
        // — the known fixed Delay overhead — while payload bytes land
        // 1.04 ms apart, so RXBUF overran before the poll ever saw the
        // go-byte. The CPU cannot poll its way into a 9600-baud stream;
        // arming first makes the timing irrelevant.)
        let mut payload = [0u8; RX_LEN];
        unsafe { rx.start_read_dma(&mut ch1, &mut payload) };
        dma_tx.write_all(b"DMA_RX_READY n=16\r\n").ok();
        // ~5 s window for the host (or a human) to send; DMA collects the
        // bytes regardless of what this loop is doing.
        let mut done = false;
        for _ in 0..1500 {
            if ch1.is_done() {
                done = true;
                break;
            }
            delay.delay_ms(1);
        }
        if done {
            ch1.clear_done();
            echo(&mut dma_tx, b"DMA_ECHO ", &mut payload);

            // RX round 2: the same geometry through the *blocking* public
            // API, `Rx::read_exact_dma`. Blocking is safe now — a host that
            // just completed round 1 is present and committed; and it arms
            // within microseconds of the announce, milliseconds before the
            // host can respond.
            let mut payload2 = [0u8; RX_LEN];
            dma_tx.write_all(b"DMA_RX2_READY n=16\r\n").ok();
            if rx.read_exact_dma(&mut ch1, &mut payload2).is_ok() {
                echo(&mut dma_tx, b"DMA_ECHO2 ", &mut payload2);
            }
        } else {
            // No host this round: abandon the arm (partial bytes discarded).
            ch1.disarm();
        }

        delay.delay_ms(1000);
    }
}

/// One channel's turn on the IRQ path: arm a 1-byte software-paced transfer,
/// enable its completion interrupt (after arming — the arm writes the whole
/// control word), pulse `request`, and check the ISR tally advanced to
/// exactly `expected` (each channel exactly once, nothing unexpected) and the
/// byte actually moved.
fn irq_test<const N: u8>(ch: &mut Channel<N>, expected: [u8; 4], delay: &mut Delay) -> bool {
    let src = [0xC3u8];
    let mut dst = [0u8];
    unsafe {
        ch.arm_single_bytes(
            TriggerSource::DmaReq,
            src.as_ptr(),
            AddrMode::Increment,
            dst.as_mut_ptr(),
            AddrMode::Increment,
            1,
        );
    }
    ch.enable_done_interrupt();
    ch.request();
    // The transfer halts the CPU for ~2 cycles and the ISR runs on the next
    // instruction boundary; 1 ms is oceans of settling time.
    delay.delay_ms(1);
    ch.disable_done_interrupt();
    let counts = critical_section::with(|cs| IRQ_HITS.borrow(cs).get());
    counts == expected && dst[0] == 0xC3
}

/// Transform a received payload `+1` in place and transmit it, tagged, via
/// the DMA transmit path — only software that actually collected the DMA'd
/// buffer can produce the transform.
fn echo<W: hal::embedded_io::Write>(tx: &mut W, tag: &[u8], payload: &mut [u8; RX_LEN]) {
    for b in payload.iter_mut() {
        *b = b.wrapping_add(1);
    }
    tx.write_all(tag).ok();
    tx.write_all(payload).ok();
    tx.write_all(b"\r\n").ok();
}

/// Write `name` + ` OK`/` FAIL` + CRLF through the DMA transmit path.
fn verdict<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], ok: bool) {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" as &[u8] } else { b" FAIL\r\n" })
        .ok();
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
