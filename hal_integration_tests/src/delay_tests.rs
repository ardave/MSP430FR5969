use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// Default macOS device node for the eUSCI_A0 UART backchannel. Override with
/// the `MSP430_UART_PORT` env var if the board enumerates differently.
const DEFAULT_PORT: &str = "/dev/cu.usbmodem11203";

/// The `delay_test_runner` fixture reports over the backchannel at the project's
/// baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Delay Tests...");

    test_delay_self_check()?;

    println!("Delay Tests Completed Successfully");
    Ok(())
}

/// Flash the Delay fixture and verify its self-check burst. No external wiring is
/// involved: the software busy-loop is graded against an on-chip ACLK counter on
/// the populated 32.768 kHz crystal, validating the ms and µs delay paths.
fn test_delay_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("delay_test_runner")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `delay_test_runner` fixture transmits once per second.
///
/// The fixture grades `delay_ms`/`delay_us` against the independent crystal counter
/// and always emits the complete burst, so a body mismatch after BEGIN is a real
/// failure. A missing crystal yields a single `DELAY CLOCK FAIL` line, which
/// likewise surfaces as a clean mismatch.
fn verify_self_check_burst() -> Result<(), Box<dyn Error>> {
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

    const BEGIN: &str = "DELAY_TEST_BEGIN";
    const END: &str = "DELAY_TEST_END";
    let expected_body = ["DELAY MS OK", "DELAY US OK"];

    // Bound the whole search generously: the fixture spends ~1.75 s measuring the
    // 250/500/1000 ms + 2 ms delays at startup, then repeats every ~1 s.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `delay ...` line),
    // then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            // Surface the fixture's `delay ms250=… ms500=… ms1000=… us50k=…`
            // diagnostics while scanning for BEGIN.
            if !line.is_empty() {
                println!("  [board] {line}");
            }
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "delay mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (ms + µs delays vs crystal)");
        return Ok(());
    }
}
