#![no_std]
#![no_main]

use hal::delay::Delay;
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

// Watchdog Timer Password / Hold.
const WDTPW: u16 = 0x5A00;
const WDTHOLD: u16 = 0x0080;

/// Firmware entry point.
///
/// SPI loopback demo for the eUSCI_B0 driver. Jumper **P1.6 (SIMO) to P1.7
/// (SOMI)** and every byte transmitted comes straight back; the demo transfers a
/// known pattern in place and checks it round-trips. Without the jumper SOMI
/// floats and the check FAILs — connect it live and watch FAIL turn to PASS.
#[entry]
fn main() -> ! {
    // Stop the watchdog before anything else (default timeout ~32 ms, and
    // Peripherals::take() enters a critical section). Raw write — we don't hold
    // the peripheral singletons yet.
    unsafe {
        (0x015C as *mut u16).write_volatile(WDTPW | WDTHOLD);
    }

    let p = hal::pac::Peripherals::take().unwrap();

    // Performance clock profile: MCLK 1 MHz, SMCLK 8 MHz. SMCLK feeds both the
    // UART BRCLK and the SPI bit-rate generator below.
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO pins (clear LOCKLPM5) so the UART and SPI pin muxes take effect.
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

    // UART (eUSCI_A0) for printing results: 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1). P1.6/P1.7 are now SPI pins,
    // so they are unavailable as GPIO — these two are clear of the SPI mux.
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());

    // eUSCI_B0 as a 3-pin SPI master: 1 MHz SPI clock (SMCLK 8 MHz / 8), Mode 0,
    // MSB first. SIMO=P1.6, SOMI=P1.7, CLK=P2.2.
    let mut spi = p
        .usci_b0_spi_mode
        .into_spi(SpiConfig::new(clocks.smclk()).bit_rate(1_000_000));

    tx.write_all(b"MSP430FR5969 SPI loopback demo\r\n").ok();
    tx.write_all(b"Jumper P1.6 (SIMO) <-> P1.7 (SOMI). FAIL until connected.\r\n")
        .ok();

    // A pattern with a mix of bit positions, so a stuck/floating SOMI line is
    // obvious in the received bytes.
    const PATTERN: [u8; 6] = [0xA5, 0x3C, 0xFF, 0x00, 0x55, 0xAA];

    loop {
        // transfer_in_place sends each byte and overwrites it with the byte
        // received in its place. With the loopback jumper, received == sent.
        let mut buf = PATTERN;
        spi.transfer_in_place(&mut buf).ok();
        let pass = buf == PATTERN;

        tx.write_all(b"sent ").ok();
        write_hex(&mut tx, &PATTERN);
        tx.write_all(b" recv ").ok();
        write_hex(&mut tx, &buf);
        tx.write_all(if pass { b" PASS\r\n" } else { b" FAIL\r\n" }).ok();

        // Green = loopback verified, red = not (no jumper / wrong data).
        if pass {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write a byte slice as space-separated uppercase hex over the UART.
fn write_hex<W: hal::embedded_io::Write>(tx: &mut W, bytes: &[u8]) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for &b in bytes {
        let pair = [HEX[(b >> 4) as usize], HEX[(b & 0x0F) as usize], b' '];
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
