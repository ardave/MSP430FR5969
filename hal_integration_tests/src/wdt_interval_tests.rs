use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `wdt_interval_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting WDT Interval Tests...");

    test_interval_self_check()?;

    println!("WDT Interval Tests Completed Successfully");
    Ok(())
}

/// Flash the interval-timer fixture and verify its self-check burst. No wiring:
/// WDT_A is on-chip. The fixture proves the inverted personality — expiry fires
/// the `WDT` vector at the configured ~1.05 s cadence and never resets the chip.
fn test_interval_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("wdt_interval_test_runner")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `wdt_interval_test_runner` fixture transmits once per second.
fn verify_self_check_burst() -> Result<(), Box<dyn Error>> {
    let port_path = crate::serial::resolve_port()?;

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

    const BEGIN: &str = "WDT_INTERVAL_TEST_BEGIN";
    const END: &str = "WDT_INTERVAL_TEST_END";
    let expected_body = ["WDT INTERVAL OK", "WDT NORESET OK"];

    // The fixture measures a ~3.2 s tick window at startup before the first
    // burst; bound the whole search generously.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `wdt ticks=…`
    // info line), then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            if !line.is_empty() {
                println!("  [board] {line}");
            }
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "wdt interval mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (tick cadence + no reset)");
        return Ok(());
    }
}
