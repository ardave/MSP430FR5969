#![no_std]
#![no_main]

//! eUSCI_B0 **I2C slave** fixture: a 16-register register-file device at
//! address **0x48**, the classic pointer-plus-autoincrement protocol every
//! I2C sensor speaks.
//!
//! **Hardware verification pending.** The driver (`hal::i2c::I2cSlave`) is
//! code-complete against SLAU367P but has not yet been exercised on silicon;
//! this fixture exists so a hardware-in-the-loop strategy can be built around
//! it later. Any I2C master can drive it — a Raspberry Pi, a second
//! LaunchPad, a USB-I2C dongle, or a bit-banged GPIO master:
//!
//! - **Wiring:** SDA = P1.6, SCL = P1.7 (remove the SPI loopback jumper!),
//!   ~4.7 kΩ pull-ups to 3V3, common ground with the master.
//! - **Protocol:** master write = `[reg_ptr, data...]` — first byte sets the
//!   register pointer (wrapped to 0..=15), each further byte stores and
//!   autoincrements. Master read = stream from the pointer, autoincrementing.
//!   The write-then-read (repeated START) register idiom therefore works as
//!   on any real sensor: `write_read(0x48, &[0x00], &mut id)`.
//! - **Register 0 is a fixed ID byte, `0x69`** (read-only; writes to it are
//!   dropped) — the BME280-chip-ID-style correctness probe: a master that
//!   reads `0x69` back through a repeated START has exercised address match,
//!   RX, the turnaround, and TX in one transaction.
//!
//! Observability over the usual UART backchannel (eUSCI_A0, 9600 8N1):
//! `I2C_SLAVE_READY addr=0x48` once per idle second until the first
//! transaction, then one line per completed transaction
//! (`i2c wr ptr=P n=N` / `i2c rd n=N [gc]`), with the GREEN LED toggling per
//! transaction. `n` counts data bytes for writes and *requested* bytes for
//! reads — the eUSCI asks one byte ahead, so a read of K bytes usually
//! reports `n=K+1` with the surplus byte flushed at STOP (that flush is
//! exactly the driver behavior a hardware fixture needs to confirm).

use hal::embedded_hal::digital::StatefulOutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::i2c::{I2cSlaveExt, SlaveConfig, SlaveEvent};
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// Own address. 0x48 is the TMP102/ADS1115 neighborhood — a typical sensor
/// address, and comfortably inside the scanner's 0x08..=0x77 probe range.
const ADDRESS: u8 = 0x48;

/// Register-file size (pointer wraps at 16).
const REGS: usize = 16;

/// Fixed contents of register 0 — the device-ID probe target.
const ID_BYTE: u8 = 0x69;

/// What the current transaction has been doing (for the report line).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Phase {
    Idle,
    /// Master write: `got_ptr` once the first (pointer) byte has landed.
    Write { got_ptr: bool },
    Read,
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz (the UART's BRCLK; the I2C slave itself is
    // clocked by the master's SCL and needs nothing from the clock tree).
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    let (port1, _port2) = p.port_1_2.split();
    let mut green_led = port1.pin0.into_output(); // toggles per transaction

    // The slave: valid-by-construction address, so unwrap is fine here.
    let mut slave = p
        .usci_b0_i2c_mode
        .into_i2c_slave(SlaveConfig::new(ADDRESS))
        .unwrap();

    tx.write_all(b"MSP430FR5969 I2C slave register file (SDA=P1.6, SCL=P1.7, pull-ups!)\r\n")
        .ok();

    let mut file = [0u8; REGS];
    file[0] = ID_BYTE;

    let mut ptr: usize = 0;
    let mut phase = Phase::Idle;
    let mut byte_count: u32 = 0;
    let mut general_call = false;
    let mut transactions: u32 = 0;

    // Idle-second pacing for the READY handshake line, counted in poll-loop
    // iterations (~a second at MCLK 1 MHz) — no timer spent, and the loop
    // never blocks, so bus events are picked up at full polling speed.
    let mut idle_spins: u32 = 0;
    const SPINS_PER_READY: u32 = 40_000;

    loop {
        let Some(event) = slave.poll() else {
            // Handshake until the first transaction proves the host's master
            // is alive (the house pattern: announce until poked).
            if transactions == 0 {
                idle_spins += 1;
                if idle_spins >= SPINS_PER_READY {
                    idle_spins = 0;
                    tx.write_all(b"I2C_SLAVE_READY addr=0x48\r\n").ok();
                }
            }
            continue;
        };

        match event {
            SlaveEvent::Start { read } => {
                phase = if read {
                    Phase::Read
                } else {
                    Phase::Write { got_ptr: false }
                };
                byte_count = 0;
                general_call = slave.addressed_as_general_call();
            }

            SlaveEvent::Received(b) => match phase {
                Phase::Write { got_ptr: false } => {
                    ptr = (b as usize) % REGS;
                    phase = Phase::Write { got_ptr: true };
                }
                Phase::Write { got_ptr: true } => {
                    if ptr != 0 {
                        file[ptr] = b; // register 0 (the ID) is read-only
                    }
                    ptr = (ptr + 1) % REGS;
                    byte_count += 1;
                }
                // A byte outside a write phase (e.g. before any START was
                // seen after boot): count it so it is visible, change nothing.
                _ => byte_count += 1,
            },

            SlaveEvent::TxRequest => {
                slave.write_byte(file[ptr]);
                ptr = (ptr + 1) % REGS;
                byte_count += 1;
            }

            SlaveEvent::Stop => {
                transactions += 1;
                green_led.toggle().ok();
                match phase {
                    Phase::Read => {
                        tx.write_all(b"i2c rd n=").ok();
                        write_dec(&mut tx, byte_count);
                    }
                    _ => {
                        tx.write_all(b"i2c wr ptr=").ok();
                        write_dec(&mut tx, ptr as u32);
                        tx.write_all(b" n=").ok();
                        write_dec(&mut tx, byte_count);
                    }
                }
                if general_call {
                    tx.write_all(b" gc").ok();
                }
                tx.write_all(b"\r\n").ok();
                phase = Phase::Idle;
            }
        }
    }
}

/// Write an unsigned value as decimal ASCII. `core::fmt` is avoided
/// project-wide (FRAM budget), so format by hand into a small stack buffer.
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
