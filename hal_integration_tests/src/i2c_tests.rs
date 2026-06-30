use std::error::Error;
use std::io::{self, Write as _};
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// Default macOS device node for the eUSCI_A0 UART backchannel. Override with
/// the `MSP430_UART_PORT` env var if the board enumerates differently.
const DEFAULT_PORT: &str = "/dev/cu.usbmodem11203";

/// The `i2c_test_runner` fixture reports over the backchannel at the project's
/// baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting I2C Tests...");

    test_bus_scan_finds_device()?;

    println!("I2C Tests Completed Successfully");
    Ok(())
}

/// Unlike the on-chip fixtures (timer, deep-sleep, ADC-internal), the I2C scan
/// needs a real device on the bus, so it cannot be fully self-contained. Prompt
/// the operator to wire a BME280 breakout and confirm before we flash and scan.
fn test_bus_scan_finds_device() -> Result<(), Box<dyn Error>> {
    prompt_for_bme280_wiring()?;
    deployment::build_and_flash("i2c_test_runner")?;
    verify_scan_burst()
}

/// Print the BME280 hookup and block until the operator presses Enter (or aborts
/// with Ctrl-C). The fixture probes the open-drain bus, which only works once the
/// device and its pull-ups are present, so we must not flash until it is wired.
fn prompt_for_bme280_wiring() -> Result<(), Box<dyn Error>> {
    println!();
    println!("  ┌─ I2C bus scan: connect a BME280 breakout to eUSCI_B0 ──────────────┐");
    println!("  │                                                                    │");
    println!("  │    BME280            MSP430FR5969 LaunchPad                         │");
    println!("  │    ------            -------------------------                      │");
    println!("  │    VCC / VIN  ─────  3V3                                            │");
    println!("  │    GND        ─────  GND                                            │");
    println!("  │    SDA / SDI  ─────  P1.6                                           │");
    println!("  │    SCL / SCK  ─────  P1.7                                           │");
    println!("  │                                                                    │");
    println!("  │    • REMOVE the SPI loopback jumper between P1.6 and P1.7 first —   │");
    println!("  │      it shorts SDA to SCL and no transfer can work.                │");
    println!("  │    • I2C is open-drain: ~4.7 kΩ pull-ups from SDA and SCL to 3V3    │");
    println!("  │      are required (most BME280 breakouts populate their own).       │");
    println!("  │                                                                    │");
    println!("  │    The board scans 0x08..=0x77; the BME280 answers at 0x76 or      │");
    println!("  │    0x77, so a correct hookup makes the scan PASS.                   │");
    println!("  │                                                                    │");
    println!("  └────────────────────────────────────────────────────────────────────┘");
    print!("  Wire it up, then press Enter to flash and scan (Ctrl-C to abort)... ");
    io::stdout().flush()?;

    let mut _line = String::new();
    let n = io::stdin().read_line(&mut _line)?;
    if n == 0 {
        // EOF (e.g. stdin closed / non-interactive) — no operator to confirm.
        return Err("aborted: no confirmation on stdin (EOF)".into());
    }
    println!();
    Ok(())
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `i2c_test_runner` fixture transmits once per second.
///
/// The fixture scans once at startup and always emits the complete burst, so a
/// body mismatch after BEGIN is a real failure: `I2C SCAN FAIL` means the bus
/// came up empty (no device, missing pull-ups, or an unremoved SPI jumper).
fn verify_scan_burst() -> Result<(), Box<dyn Error>> {
    let port_path =
        std::env::var("MSP430_UART_PORT").unwrap_or_else(|_| DEFAULT_PORT.to_string());

    println!("  opening {port_path} @ {BAUD} 8N1...");
    let mut port = serialport::new(&port_path, BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        // Per-read timeout; the deadline loop below bounds the overall wait.
        .timeout(Duration::from_millis(1000))
        .open()?;

    // The eZ-FET gates the board's TX on DTR; assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    const BEGIN: &str = "I2C_TEST_BEGIN";
    const END: &str = "I2C_TEST_END";
    let expected_body = ["I2C SCAN OK"];

    // Bound the whole search generously: the fixture scans (~microseconds) at
    // startup, then repeats the burst every ~1 s.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `i2c found=` line),
    // then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            // Surface the fixture's `i2c found=…` diagnostics while scanning for BEGIN.
            if !line.is_empty() {
                println!("  [board] {line}");
            }
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "i2c mismatch after BEGIN: expected {expected:?}, got {got:?} \
                     (empty bus? check the device, pull-ups, and SPI jumper)"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (device ACKed on the bus)");
        return Ok(());
    }
}
