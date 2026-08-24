use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `lpm_barrier_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting LPM Barrier Tests...");

    test_sleep_barrier_self_check()?;

    println!("LPM Barrier Tests Completed Successfully");
    Ok(())
}

/// Flash the sleep compiler-barrier fixture and verify its self-check burst.
/// No external wiring: the fixture sleeps on the WDT interval metronome —
/// LPM0 woken from SMCLK, then LPM3 woken from ACLK/VLO — and checks that a
/// flag written by the wake ISR is visible to the code after each wake, i.e.
/// that the HAL's single shared sleep asm site (`power::sleep_bis`) is the
/// compiler barrier the "returns once an interrupt has woken the CPU"
/// contract requires. A build whose optimizer was licensed (e.g. by
/// `options(nomem)` on the sleep asm) to reuse pre-sleep loads reports
/// `BARRIER LPMx LOOP FAIL` / `BARRIER LPMx RELOAD FAIL`.
fn test_sleep_barrier_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("lpm_barrier_test_firmware")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `lpm_barrier_test_firmware` fixture transmits once per second.
///
/// The fixture runs both probes within ~10 ms of boot, then always emits the
/// complete burst, so a body mismatch after BEGIN is a real failure.
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

    const BEGIN: &str = "BARRIER_TEST_BEGIN";
    const END: &str = "BARRIER_TEST_END";
    let expected_body = [
        "BARRIER LPM0 LOOP OK",
        "BARRIER LPM0 RELOAD OK",
        "BARRIER LPM3 LOOP OK",
        "BARRIER LPM3 RELOAD OK",
        "BARRIER ISR OK",
    ];

    // The LPM0 probes take ~3 ms and the LPM3 probes ~110 ms (two ~54 ms VLO
    // metronome wakes); the burst repeats every ~1 s. Bound generously.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `barrier ...`
    // info line), then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            // Surface the fixture's `barrier w0=… w3=… fires=…` diagnostics
            // while scanning for BEGIN.
            if !line.is_empty() {
                println!("  [board] {line}");
            }
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "lpm-barrier mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (sleep is a compiler barrier)");
        return Ok(());
    }
}
