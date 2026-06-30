use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// Default macOS device node for the eUSCI_A0 UART backchannel. Override with
/// the `MSP430_UART_PORT` env var if the board enumerates differently.
const DEFAULT_PORT: &str = "/dev/cu.usbmodem11203";

/// The `timer_test_runner` fixture reports over the backchannel at the project's
/// baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Timer Tests...");

    test_counter_self_check()?;

    println!("Timer Tests Completed Successfully");
    Ok(())
}

/// Flash the Timer0_A3 fixture and verify its self-check burst. No external wiring
/// is involved: Timer0_A3 is on-chip and runs from the DCO performance profile, so
/// this validates the free-running counter, software capture, and the overflow +
/// `now32` path end-to-end.
fn test_counter_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("timer_test_runner")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `timer_test_runner` fixture transmits once per second.
///
/// The fixture computes all three verdicts at startup and always emits the
/// complete burst, so a body mismatch after BEGIN is a real failure (a failed
/// check flips the line to `FAIL`).
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

    const BEGIN: &str = "TIMER_TEST_BEGIN";
    const END: &str = "TIMER_TEST_END";
    let expected_body = ["TIMER RUN OK", "TIMER CAPTURE OK", "TIMER OVERFLOW OK"];

    // Bound the whole search generously: the fixture spends ~150 ms measuring the
    // overflow window at startup, then repeats every ~1 s.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `timer us=` line),
    // then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            // Surface the fixture's human-readable diagnostics (e.g. the
            // `timer run=… cap_lag=… ovf_us=…` line) while scanning for BEGIN.
            if !line.is_empty() {
                println!("  [board] {line}");
            }
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "timer mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (run + capture + overflow/now32)");
        return Ok(());
    }
}
