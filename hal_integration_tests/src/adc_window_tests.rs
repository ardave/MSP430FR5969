use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `adc_window_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting ADC Window Comparator Tests...");

    test_window_comparator_self_check()?;

    println!("ADC Window Comparator Tests Completed Successfully");
    Ok(())
}

/// Flash the window-comparator fixture and verify its self-check burst. No
/// wiring: the input is the internal (AVCC–AVSS)/2 monitor (~half scale by
/// construction), so windows around / below / above that reading exercise
/// all three comparator outcomes, the ADC12IV window slots, and the
/// free-running monitor's LPM0 wake.
fn test_window_comparator_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("adc_window_test_runner")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `adc_window_test_runner` fixture transmits once per second.
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

    const BEGIN: &str = "ADC_WIN_TEST_BEGIN";
    const END: &str = "ADC_WIN_TEST_END";
    let expected_body = [
        "ADC WIN IN OK",
        "ADC WIN HI OK",
        "ADC WIN LO OK",
        "ADC WIN CLEAR OK",
        "ADC WIN IV OK",
        "ADC WIN MON OK",
        "ADC WIN WAKE OK",
        "ADC WIN RESTORE OK",
    ];

    // The fixture's phases finish in well under a second; the burst repeats
    // every ~1 s. Bound the whole search generously anyway.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `adc win …`
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
                    "adc window mismatch after BEGIN: expected {expected:?}, got {got:?} \
                     (the fixture's `adc win …` info line carries the counts, the ISR's \
                     ADC12IV value, and the firing tally)"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!(
            "  verified full {BEGIN}..{END} burst (3 comparator outcomes + flag clear + \
             IV demux + monitor + LPM0 wake + restore)"
        );
        return Ok(());
    }
}
