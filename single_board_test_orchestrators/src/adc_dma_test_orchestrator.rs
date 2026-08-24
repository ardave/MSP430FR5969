use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `adc_dma_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// The fixture's self-delimited verdict burst, in order. All verdicts are
/// computed on-device: 32-sample DMA-drained bursts of the supply monitor
/// (ratiometric, every sample near half scale) and the temperature sensor
/// (every sample within the 5–60 °C TLV-calibrated window), plus a polled
/// single conversion afterward proving the driver restored single-conversion
/// mode after free-running.
const EXPECTED_BURST: [&str; 4] = [
    "ADC_DMA CAL OK",
    "ADC_DMA SUPPLY OK",
    "ADC_DMA TEMP OK",
    "ADC_DMA SINGLE OK",
];

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting ADC-DMA Tests...");

    test_adc_dma_fixture()?;

    println!("ADC-DMA Tests Completed Successfully");
    Ok(())
}

/// Flash the ADC-DMA fixture (hands-free — internal channels only) and assert
/// one complete verdict burst.
fn test_adc_dma_fixture() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("adc_dma_test_firmware")?;

    let port_path = crate::serial::resolve_port()?;

    println!("  opening {port_path} @ {BAUD} 8N1...");
    let mut port = serialport::new(&port_path, BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .timeout(Duration::from_millis(1000))
        .open()?;

    // The eZ-FET gates the board's TX on DTR; assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    // Scan to a BEGIN (the burst repeats once per second), then assert the
    // body and the closing END.
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "ADC_DMA_TEST_BEGIN" {
            break;
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    }
    for expected in EXPECTED_BURST {
        let got = read_line(port.as_mut(), deadline)?;
        if got != expected {
            return Err(format!(
                "adc-dma verdict mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (the fixture's info line preceding the burst has the sample min/max)"
            )
            .into());
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "ADC_DMA_TEST_END" {
        return Err(format!("expected \"ADC_DMA_TEST_END\" to close the burst, got {got:?}").into());
    }

    println!(
        "  verified {}-line verdict burst (DMA-drained supply + temperature sample runs, \
         single-conversion restore)",
        EXPECTED_BURST.len()
    );
    Ok(())
}
