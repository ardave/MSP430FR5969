use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `rtc_tick_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting RTC Prescaler Tick Tests...");

    test_prescaler_tick_self_check()?;

    println!("RTC Prescaler Tick Tests Completed Successfully");
    Ok(())
}

/// Flash the prescaler-tick fixture and verify its self-check burst. No
/// wiring (the LaunchPad's 32.768 kHz crystal is the tick source): 128 Hz
/// wall-clocked against the DCO, the RTCIV 0x08/0x0A demux for the two
/// prescaler banks, concurrent banks, a 32 Hz LPM3 wake landing exactly
/// once, and clean disarm.
fn test_prescaler_tick_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("rtc_tick_test_firmware")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `rtc_tick_test_firmware` fixture transmits once per second.
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

    const BEGIN: &str = "RTC_TICK_TEST_BEGIN";
    const END: &str = "RTC_TICK_TEST_END";
    let expected_body = [
        "RTC TICK RATE OK",
        "RTC TICK IV0 OK",
        "RTC TICK IV1 OK",
        "RTC TICK BOTH OK",
        "RTC TICK WAKE OK",
        "RTC TICK STOP OK",
    ];

    // The fixture's phases finish in under a second; the burst repeats every
    // ~1 s. Bound the whole search generously anyway.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `rtc tick …`
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
                    "rtc tick mismatch after BEGIN: expected {expected:?}, got {got:?} \
                     (the fixture's `rtc tick …` info line carries the per-source ISR \
                     tallies and the measured 128 Hz window)"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!(
            "  verified full {BEGIN}..{END} burst (128 Hz rate vs DCO + IV demux for \
             both banks + concurrent banks + LPM3 wake exactly-once + clean disarm)"
        );
        return Ok(());
    }
}
