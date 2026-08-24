use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `adc_irq_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting ADC Interrupt Tests...");

    test_sleep_sample_self_check()?;

    println!("ADC Interrupt Tests Completed Successfully");
    Ok(())
}

/// Flash the ADC12-interrupt fixture and verify its self-check burst. No
/// wiring: the source is the internal (AVCC–AVSS)/2 monitor. The fixture arms
/// eight conversions and sleeps in LPM0 through each — MODOSC finishes the
/// conversion alone and the ADC12 interrupt wakes the CPU with the result.
fn test_sleep_sample_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("adc_irq_test_firmware")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `adc_irq_test_firmware` fixture transmits once per second.
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

    const BEGIN: &str = "ADC_IRQ_TEST_BEGIN";
    const END: &str = "ADC_IRQ_TEST_END";
    let expected_body = ["ADC IRQ WAKE OK", "ADC IRQ COUNT OK", "ADC IRQ VALUE OK"];

    // Eight ~60 µs conversions finish essentially instantly; the burst repeats
    // every ~1 s. Bound the whole search generously anyway.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `adc irq n=…`
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
                    "adc irq mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (LPM0 wake + count + value)");
        return Ok(());
    }
}
