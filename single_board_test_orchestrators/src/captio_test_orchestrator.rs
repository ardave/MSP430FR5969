use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `captio_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Capacitive Touch I/O Tests...");

    test_captio_self_check()?;

    println!("Capacitive Touch I/O Tests Completed Successfully");
    Ok(())
}

/// Flash the CAPTIO fixture and verify its self-check burst. No wiring —
/// the stimulus is each bare header pad's own parasitic capacitance turned
/// into a relaxation oscillation: plausible frequencies on five pads across
/// both instances (CAPTIO0/TA2, CAPTIO1/TA3), the live state bit, frozen
/// counts while disabled, the uncrossed instance↔timer pairing, typed-scan
/// vs raw-route agreement, an LPM0 wake from the self-clocked count's
/// overflow landing exactly once, and clean disarm.
fn test_captio_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("captio_test_firmware")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `captio_test_firmware` fixture transmits once per second.
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

    const BEGIN: &str = "CAPTIO_TEST_BEGIN";
    const END: &str = "CAPTIO_TEST_END";
    let expected_body = [
        "CAPTIO OSC OK",
        "CAPTIO STATE OK",
        "CAPTIO OFF OK",
        "CAPTIO PAIR OK",
        "CAPTIO OSC1 OK",
        "CAPTIO SCAN OK",
        "CAPTIO WAKE OK",
        "CAPTIO STOP OK",
    ];

    // The fixture's phases finish in well under a second; the burst repeats
    // every ~1 s. Bound the whole search generously anyway.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `captio …`
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
                    "captio mismatch after BEGIN: expected {expected:?}, got {got:?} \
                     (the fixture's `captio …` info line carries the per-pad oscillation \
                     frequencies and the wake tally)"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!(
            "  verified full {BEGIN}..{END} burst (pad oscillation on five pads across \
             both instances + live state bit + frozen-while-disabled + uncrossed \
             instance/timer pairing + typed-vs-raw route agreement + LPM0 wake \
             exactly-once + clean disarm)"
        );
        return Ok(());
    }
}
