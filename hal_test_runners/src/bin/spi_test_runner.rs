#![no_std]
#![no_main]

//! SPI loopback fixture for the eUSCI SPI master driver, framed for the
//! host-side `spi_tests` runner. Covers **two instances in one flash**:
//! eUSCI_B0 and eUSCI_A1.
//!
//! This is the hardware-in-the-loop counterpart to the interactive SPI loopback
//! demo in `src/main.rs`: instead of re-printing the live `sent…/recv…` line
//! forever, it transfers a known pattern **once per bus** at startup and then
//! re-emits a fixed pass/fail verdict burst, exactly like the
//! `i2c`/`timer`/`deep_sleep` fixtures, so the host can flash it and assert one
//! framed result. Because eUSCI_B0 is *either* SPI or I2C (one register block,
//! shared P1.6/P1.7 pins), only one of those binaries can run on the board at a
//! time. eUSCI_A1 conflicts with nothing here — that pairing (A1-SPI alongside
//! B0-whatever) is exactly the capability the generic driver adds.
//!
//! eUSCI_A0 SPI is *not* testable this way: its SPI pins P2.0/P2.1 are the
//! backchannel UART (the fixture's report channel), and the eZ-FET's own TX
//! driver sits on P2.1, so a loopback jumper there would fight it. A0 gets
//! driver coverage by construction — it shares every line of code with A1.
//!
//! ```text
//! cargo +nightly build --bin spi_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/spi_test_runner
//! ```
//!
//! # Wiring
//!
//! Driven by `spi_tests`, which prints the jumper hookup and waits for the
//! operator before flashing. Each SPI master needs its own transmit line looped
//! back to its receive line:
//!
//! - **eUSCI_B0**: jumper **P1.6 (SIMO) to P1.7 (SOMI)**
//! - **eUSCI_A1**: jumper **P2.5 (SIMO) to P2.6 (SOMI)**
//!
//! With a jumper every transmitted byte is clocked straight back in; without it
//! SOMI floats and the received bytes do not match.
//!
//! # What it checks
//!
//! `transfer_in_place`s a 6-byte pattern on each bus and checks that every byte
//! round-trips. The A1 pattern is the **bitwise complement** of the B0 pattern,
//! so a miswired cross-connection (one bus's SIMO into the other's SOMI) cannot
//! pass either check. Reaching the verdicts at all also proves both drivers
//! complete their transfers rather than hanging.
//!
//! Then the same jumpers serve the **DMA engine**: B0 re-runs its loopback
//! through the [`hal::spi::SpiDma`] wrapper (the `SpiBus` impl whose per-byte
//! work is done by an RX + TX DMA channel pair), and A1 runs the split
//! `transfer_dma` path (separate write/read buffers) with the channels
//! reclaimed from B0 — three channels covering two buses, sequentially. The
//! DMA patterns are the byte-reversals of the plain ones, so a stale buffer
//! from the first phase cannot fake a pass.
//!
//! # Framed output for the host runner
//!
//! ```text
//! spi b0 sent=A53CFF0055AA recv=A53CFF0055AA   (human-readable, skipped by host)
//! spi a1 sent=5AC300FFAA55 recv=5AC300FFAA55
//! SPI_TEST_BEGIN
//! SPI B0 LOOPBACK OK                           (or `SPI B0 LOOPBACK FAIL`)
//! SPI A1 LOOPBACK OK                           (or `SPI A1 LOOPBACK FAIL`)
//! SPI B0 DMA OK                                (SpiDma / SpiBus round-trip)
//! SPI A1 DMA OK                                (transfer_dma round-trip)
//! SPI_TEST_END
//! ```

use hal::delay::Delay;
use hal::dma::DmaExt;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_hal::spi::SpiBus as _;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use hal::spi::{Config as SpiConfig, SpiExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// A pattern with a mix of bit positions, so a stuck/floating SOMI line is
/// obvious in the received bytes. Sent on eUSCI_B0.
const PATTERN_B0: [u8; 6] = [0xA5, 0x3C, 0xFF, 0x00, 0x55, 0xAA];

/// Bitwise complement of [`PATTERN_B0`], sent on eUSCI_A1 — distinct patterns
/// mean a jumper landed on the wrong bus cannot make either check pass.
const PATTERN_A1: [u8; 6] = [0x5A, 0xC3, 0x00, 0xFF, 0xAA, 0x55];

/// DMA-phase patterns: the byte-reversals of the polled-phase ones (distinct
/// from those *and* from each other, closing the same cross-wiring hole).
const PATTERN_B0_DMA: [u8; 6] = [0xAA, 0x55, 0x00, 0xFF, 0x3C, 0xA5];
const PATTERN_A1_DMA: [u8; 6] = [0x55, 0xAA, 0xFF, 0x00, 0xC3, 0x5A];

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds the UART BRCLK and both SPI
    // bit-rate generators below.
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0) for printing results: 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1). P1.6/P1.7 are now SPI pins.
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());

    // Two independent 3-pin SPI masters, both 1 MHz (SMCLK 8 MHz / 8), Mode 0,
    // MSB first:
    //   eUSCI_B0: SIMO = P1.6, SOMI = P1.7, CLK = P2.2
    //   eUSCI_A1: SIMO = P2.5, SOMI = P2.6, CLK = P2.4
    let mut spi_b0 = p
        .usci_b0_spi_mode
        .into_spi(SpiConfig::new(clocks.smclk()).bit_rate(1_000_000));
    let mut spi_a1 = p
        .usci_a1_spi_mode
        .into_spi(SpiConfig::new(clocks.smclk()).bit_rate(1_000_000));

    // --- Transfer once per bus at startup; the loop below re-emits a stable
    // verdict. transfer_in_place sends each byte and overwrites it with the
    // byte received in its place. With the loopback jumper, received == sent.
    let mut b0_buf = PATTERN_B0;
    spi_b0.transfer_in_place(&mut b0_buf).ok();
    let b0_ok = b0_buf == PATTERN_B0;

    let mut a1_buf = PATTERN_A1;
    spi_a1.transfer_in_place(&mut a1_buf).ok();
    let a1_ok = a1_buf == PATTERN_A1;

    // --- DMA phase: the same loopbacks, per-byte work moved to DMA channels.
    // Three channels serve two buses sequentially: B0 borrows ch0 (RX) + ch1
    // (TX) inside the SpiDma wrapper, releases them, and A1 uses the same
    // pair through the inherent transfer_dma. Reversed patterns, so a buffer
    // left over from the polled phase can't fake a round-trip.
    let channels = p.dma.split();

    // B0 through the SpiBus impl of SpiDma.
    let mut spi_b0_dma = spi_b0.with_dma(channels.ch0, channels.ch1);
    let mut b0_dma_buf = PATTERN_B0_DMA;
    spi_b0_dma.transfer_in_place(&mut b0_dma_buf).ok();
    let b0_dma_ok = b0_dma_buf == PATTERN_B0_DMA;
    let (_spi_b0, mut rx_ch, mut tx_ch) = spi_b0_dma.release();

    // A1 through the split-buffer engine: what goes out must come back in
    // the separate read buffer.
    let mut a1_dma_recv = [0u8; 6];
    spi_a1.transfer_dma(&mut rx_ch, &mut tx_ch, &mut a1_dma_recv, &PATTERN_A1_DMA);
    let a1_dma_ok = a1_dma_recv == PATTERN_A1_DMA;

    loop {
        // Human-readable info lines (the host skips everything up to BEGIN).
        tx.write_all(b"spi b0 sent=").ok();
        write_hex(&mut tx, &PATTERN_B0);
        tx.write_all(b" recv=").ok();
        write_hex(&mut tx, &b0_buf);
        tx.write_all(b"\r\nspi a1 sent=").ok();
        write_hex(&mut tx, &PATTERN_A1);
        tx.write_all(b" recv=").ok();
        write_hex(&mut tx, &a1_buf);
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"SPI_TEST_BEGIN\r\n").ok();
        tx.write_all(if b0_ok {
            b"SPI B0 LOOPBACK OK\r\n" as &[u8]
        } else {
            b"SPI B0 LOOPBACK FAIL\r\n"
        })
        .ok();
        tx.write_all(if a1_ok {
            b"SPI A1 LOOPBACK OK\r\n" as &[u8]
        } else {
            b"SPI A1 LOOPBACK FAIL\r\n"
        })
        .ok();
        tx.write_all(if b0_dma_ok {
            b"SPI B0 DMA OK\r\n" as &[u8]
        } else {
            b"SPI B0 DMA FAIL\r\n"
        })
        .ok();
        tx.write_all(if a1_dma_ok {
            b"SPI A1 DMA OK\r\n" as &[u8]
        } else {
            b"SPI A1 DMA FAIL\r\n"
        })
        .ok();
        tx.write_all(b"SPI_TEST_END\r\n").ok();

        // Green = both loopbacks verified, red = a mismatch somewhere (missing
        // jumper / floating SOMI — the info lines say which bus).
        if b0_ok && a1_ok && b0_dma_ok && a1_dma_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write a byte slice as uppercase hex over the UART.
fn write_hex<W: hal::embedded_io::Write>(tx: &mut W, bytes: &[u8]) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for &b in bytes {
        let pair = [HEX[(b >> 4) as usize], HEX[(b & 0x0F) as usize]];
        tx.write_all(&pair).ok();
    }
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
