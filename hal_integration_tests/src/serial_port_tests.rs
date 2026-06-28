use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// Default macOS device node for the eUSCI_A0 UART backchannel. Override with
/// the `MSP430_UART_PORT` env var if the board enumerates differently.
const DEFAULT_PORT: &str = "/dev/cu.usbmodem11203";

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Serial Port Tests...");

    // Build the on-device fixture and flash it before any assertions, so the
    // board is guaranteed to be running the firmware these tests expect.
    deployment::build_and_flash("serial_uart")?;

    test_9600_8_n_1_comms()?;

    println!("Serial Port Tests Completed Successfully");
    Ok(())
}

/// Open the board's UART at 9600 8N1 and verify it emits the fixed confirmation
/// burst that `serial_uart.rs` transmits once per second.
fn test_9600_8_n_1_comms() -> Result<(), Box<dyn Error>> {
    let port_path =
        std::env::var("MSP430_UART_PORT").unwrap_or_else(|_| DEFAULT_PORT.to_string());

    println!("  opening {port_path} @ 9600 8N1...");
    let mut port = serialport::new(&port_path, 9600)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        // Per-read timeout; the deadline loop below bounds the overall wait.
        .timeout(Duration::from_millis(1000))
        .open()?;

    // The eZ-FET gates the board's TX on DTR (the same reason `screen` works but
    // a bare reader sees nothing). Assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    // The exact lines serial_uart.rs emits between its frame markers.
    const BEGIN: &str = "SERIAL_UART_TEST_BEGIN";
    const END: &str = "SERIAL_UART_TEST_END";
    let expected_body = ["UART 9600 8N1 OK", "hello from msp430fr5969"];

    // Bound the whole search; the fixture repeats every ~1 s, so a full burst
    // is captured well within this even if we attach mid-gap.
    let deadline = Instant::now() + Duration::from_secs(10);

    // Scan for a BEGIN marker, then assert the following lines match the
    // expected body and terminate with END. Mismatches after a BEGIN are real
    // failures (the fixture always emits the complete burst).
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            continue; // skip the boot banner / partial first burst until BEGIN
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "serial mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst at 9600 8N1");
        return Ok(());
    }
}
