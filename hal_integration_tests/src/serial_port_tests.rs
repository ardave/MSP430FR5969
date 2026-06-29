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

    test_9600_8_n_1_comms()?;
    test_115200_8_n_1_comms()?;

    println!("Serial Port Tests Completed Successfully");
    Ok(())
}

/// Flash the 9600-baud fixture and verify its confirmation burst — the baseline
/// rate the board has been validated at.
fn test_9600_8_n_1_comms() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("serial_uart_test_runner")?;
    verify_confirmation_burst(9600)
}

/// Flash the 115200-baud fixture and verify its confirmation burst. 115200 is
/// the fastest standard rate the eZ-FET USB backchannel carries dependably; the
/// generator hits it from the 8 MHz SMCLK with < 1% bit-timing error. (The MCU's
/// own clean ceiling at 8 MHz is 500 kbaud — an exact /16 — but the backchannel
/// is the practical limiter, so 115200 is the reliable high-rate test.)
fn test_115200_8_n_1_comms() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash_with_features("serial_uart_test_runner", &["baud_115200"])?;
    verify_confirmation_burst(115200)
}

/// Open the board's UART at `baud` (8N1) and verify it emits the fixed
/// confirmation burst that the `serial_uart_test_runner*` fixtures transmit once per second.
///
/// The fixtures are identical apart from their line rate and the baud they name
/// in the `UART <baud> 8N1 OK` line, so a single verifier parameterized on
/// `baud` covers every rate.
fn verify_confirmation_burst(baud: u32) -> Result<(), Box<dyn Error>> {
    let port_path =
        std::env::var("MSP430_UART_PORT").unwrap_or_else(|_| DEFAULT_PORT.to_string());

    println!("  opening {port_path} @ {baud} 8N1...");
    let mut port = serialport::new(&port_path, baud)
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

    // The exact lines the fixture emits between its frame markers. The middle
    // confirmation line names the active baud, so it doubles as a check that the
    // board really came up at the rate we opened the port with.
    const BEGIN: &str = "SERIAL_UART_TEST_BEGIN";
    const END: &str = "SERIAL_UART_TEST_END";
    let uart_line = format!("UART {baud} 8N1 OK");
    let expected_body = [uart_line.as_str(), "hello from msp430fr5969"];

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

        println!("  verified full {BEGIN}..{END} burst at {baud} 8N1");
        return Ok(());
    }
}
