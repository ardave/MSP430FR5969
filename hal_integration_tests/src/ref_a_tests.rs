use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `ref_temp_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting REF_A Tests...");

    test_calibrated_temperature_and_supply()?;

    println!("REF_A Tests Completed Successfully");
    Ok(())
}

/// Flash the REF_A fixture and verify its self-check burst. No external wiring
/// is involved: the fixture brings the reference up at 2.0 V and measures the
/// two on-chip sources that are only meaningful against it — the temperature
/// sensor (TLV-calibrated to °C) and the supply monitor (calibrated to mV) —
/// self-checking both against wide plausibility windows on-device.
fn test_calibrated_temperature_and_supply() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("ref_temp_test_runner")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `ref_temp_test_runner` fixture transmits once per second.
///
/// The fixture emits an `OK` verdict line per check, framed by BEGIN/END
/// markers. A missing TLV table, an implausible temperature (outside 5–60 °C)
/// or supply (outside 2900–3700 mV — this LaunchPad's eZ-FET rail is ~3.6 V,
/// not 3.3 V) flips the corresponding line to `MISSING`/`FAIL`, so a body
/// mismatch after BEGIN is a real failure (the fixture always emits the
/// complete burst).
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

    // The exact verdict lines the fixture emits between its frame markers. All
    // three are the pass-state strings: the factory TLV calibration was found,
    // and the calibrated temperature and supply landed in their windows.
    const BEGIN: &str = "REF_TEMP_TEST_BEGIN";
    const END: &str = "REF_TEMP_TEST_END";
    let expected_body = ["TLV CAL OK", "TEMP PLAUSIBLE OK", "SUPPLY PLAUSIBLE OK"];

    // Bound the whole search; the fixture repeats every ~1 s, so a full burst
    // is captured well within this even if we attach mid-gap.
    let deadline = Instant::now() + Duration::from_secs(10);

    // Scan for a BEGIN marker (skipping the boot banner and the human-readable
    // info line), then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "ref_a mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (TLV cal + calibrated temp & supply)");
        return Ok(());
    }
}
