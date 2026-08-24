use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `deep_sleep_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Deep Sleep Tests...");

    test_lpm3_wake_self_check()?;

    println!("Deep Sleep Tests Completed Successfully");
    Ok(())
}

/// Flash the LPM3 wake fixture and verify its self-check burst. No external wiring
/// is involved: the part sleeps in LPM3 and wakes on an ACLK CCR0 compare driven
/// by the populated 32.768 kHz crystal, validating `schedule_wake_in` +
/// `enter_lpm3` end-to-end.
fn test_lpm3_wake_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("deep_sleep_test_firmware")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `deep_sleep_test_firmware` fixture transmits once per second.
///
/// The fixture runs four LPM3 sleep/wake cycles at startup, then always emits the
/// complete burst, so a body mismatch after BEGIN is a real failure. A missing
/// crystal yields a single `SLEEP CLOCK FAIL` line, which likewise surfaces as a
/// clean mismatch.
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

    const BEGIN: &str = "SLEEP_TEST_BEGIN";
    const END: &str = "SLEEP_TEST_END";
    let expected_body = ["SLEEP WAKE OK", "SLEEP TIMING OK"];

    // Bound the whole search generously: the fixture spends ~2 s (4 × 0.5 s sleeps)
    // before its first burst, then repeats every ~1 s.
    let deadline = Instant::now() + Duration::from_secs(20);

    // Scan for a BEGIN marker (skipping the boot banner and the `sleep ...` line),
    // then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            // Surface the fixture's `sleep active=… cycles=…` diagnostics while
            // scanning for BEGIN.
            if !line.is_empty() {
                println!("  [board] {line}");
            }
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "deep-sleep mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (4× LPM3 wake + timing)");
        return Ok(());
    }
}
