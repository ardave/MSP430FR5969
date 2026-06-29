use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// Default macOS device node for the eUSCI_A0 UART backchannel. Override with
/// the `MSP430_UART_PORT` env var if the board enumerates differently.
const DEFAULT_PORT: &str = "/dev/cu.usbmodem11203";

/// The `adc_internal` fixture reports over the backchannel at the project's
/// baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting ADC Tests...");

    test_internal_channels()?;

    println!("ADC Tests Completed Successfully");
    Ok(())
}

/// Flash the internal-channel fixture and verify its self-check burst. No
/// external wiring is involved: the converter reads its own on-chip supply
/// divider and temperature sensor, so this validates the ADC end-to-end against
/// known-good internal sources.
fn test_internal_channels() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("adc_internal")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `adc_internal` fixture transmits once per second.
///
/// The fixture self-checks two internal channels on-device and emits a `OK`/`OFF`
/// verdict line for each, framed by BEGIN/END markers. A bad ADC reading flips a
/// verdict to `FAIL`/`ON`, so a body mismatch after BEGIN is a real failure (the
/// fixture always emits the complete burst).
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

    // The eZ-FET gates the board's TX on DTR (the same reason `screen` works but
    // a bare reader sees nothing). Assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    // The exact verdict lines the fixture emits between its frame markers. Both
    // are the pass-state strings: the supply monitor sits within ±10% of
    // half-scale, and the temperature sensor reads near zero (REF_A unpowered).
    const BEGIN: &str = "ADC_INTERNAL_TEST_BEGIN";
    const END: &str = "ADC_INTERNAL_TEST_END";
    let expected_body = ["AVCC/2 SELF-CHECK OK", "TEMP SENSOR OFF"];

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
                    "adc mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (AVCC/2 self-check + temp sensor off)");
        return Ok(());
    }
}
