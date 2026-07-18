use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `fram_test_runner` fixture reports over the backchannel at the project's
/// baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting FRAM Tests...");

    test_fram_round_trips()?;

    println!("FRAM Tests Completed Successfully");
    Ok(())
}

/// Flash the FRAM fixture and verify its self-check burst. No external wiring is
/// involved: both FRAM regions are on-chip, so this validates the Info-FRAM
/// (16-bit) and upper-FRAM (20-bit) read/write paths end-to-end.
fn test_fram_round_trips() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("fram_test_runner")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the `fram_test_runner`
/// fixture transmits once per second.
///
/// The fixture round-trips both FRAM regions on-device and emits an `OK` verdict
/// line for each, framed by BEGIN/END markers. A failed round-trip flips a verdict
/// to `FAIL`, so a body mismatch after BEGIN is a real failure (the fixture always
/// emits the complete burst).
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

    // The eZ-FET gates the board's TX on DTR (the same reason `screen` works but
    // a bare reader sees nothing). Assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    // The exact verdict lines the fixture emits between its frame markers. Both
    // are the pass-state strings: the Info-FRAM boot counter reads back the value
    // just written, and the upper-FRAM pattern round-trips intact.
    const BEGIN: &str = "FRAM_TEST_BEGIN";
    const END: &str = "FRAM_TEST_END";
    let expected_body = ["INFO FRAM OK", "HIGH FRAM OK"];

    // Bound the whole search; the fixture repeats every ~1 s, so a full burst
    // is captured well within this even if we attach mid-gap.
    let deadline = Instant::now() + Duration::from_secs(10);

    // Scan for a BEGIN marker (skipping the boot banner and the human-readable
    // boot-count line), then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "fram mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (info-fram counter + upper-fram round-trip)");
        return Ok(());
    }
}
