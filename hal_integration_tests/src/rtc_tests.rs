use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `rtc_test_runner` fixture reports over the backchannel at the project's
/// baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting RTC Tests...");

    test_calendar_self_check()?;

    println!("RTC Tests Completed Successfully");
    Ok(())
}

/// Flash the RTC fixture and verify its self-check burst. No external wiring is
/// involved: RTC_B is on-chip and clocked by the populated 32.768 kHz crystal, so
/// this validates the load-and-read-back and the 1 Hz advance end-to-end.
fn test_calendar_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("rtc_test_runner")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the `rtc_test_runner`
/// fixture transmits once per second.
///
/// The fixture loads the calendar to a known instant, reads it back, and checks
/// (against the independent DCO-timed delay) that it advances at 1 Hz, emitting an
/// `OK` verdict line for each, framed by BEGIN/END markers. A failed check flips a
/// verdict to `FAIL`, so a body mismatch after BEGIN is a real failure (the fixture
/// always emits the complete burst). A missing crystal yields a single
/// `RTC CLOCK FAIL` line, which likewise surfaces as a clean mismatch.
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
    // are the pass-state strings: the calendar reads back the loaded instant, and
    // it advances ~3 s over the fixture's startup measurement window.
    const BEGIN: &str = "RTC_TEST_BEGIN";
    const END: &str = "RTC_TEST_END";
    let expected_body = ["RTC SET OK", "RTC TICK OK"];

    // Bound the whole search generously: the fixture spends ~3 s measuring the
    // calendar advance at startup before its first burst, then repeats every ~1 s.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the human-readable
    // `now:` line), then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "rtc mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (load read-back + 1 Hz advance)");
        return Ok(());
    }
}
