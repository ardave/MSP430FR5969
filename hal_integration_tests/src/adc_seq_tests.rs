use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `adc_seq_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// The fixture's self-delimited verdict burst, in order. All verdicts are
/// computed on-device from one 3-member hardware-sequenced scan
/// (temperature + absolute supply + ratiometric supply — pairwise-disjoint
/// windows, so they are order-sensitive: a swapped MCTLx→MEMx mapping fails
/// loudly): the polled scan, the member list reversed (windows must follow
/// the members), the DMA-drained scan, repeated DMA scans across a
/// deliberately parked ADC12→DMA trigger latch (the erratum probe — the
/// per-run scrub must absorb it), and a polled single conversion afterward
/// proving the driver restored single-conversion mode.
const EXPECTED_BURST: [&str; 6] = [
    "ADC_SEQ CAL OK",
    "ADC_SEQ ORDER OK",
    "ADC_SEQ REVERSED OK",
    "ADC_SEQ DMA OK",
    "ADC_SEQ RERUN OK",
    "ADC_SEQ SINGLE OK",
];

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting ADC-Sequence Tests...");

    test_adc_seq_fixture()?;

    println!("ADC-Sequence Tests Completed Successfully");
    Ok(())
}

/// Flash the sequence-of-channels fixture (hands-free — internal channels
/// only) and assert one complete verdict burst.
fn test_adc_seq_fixture() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("adc_seq_test_runner")?;

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
        if line == "ADC_SEQ_TEST_BEGIN" {
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
                "adc-seq verdict mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (the fixture's info line preceding the burst has the per-slot readings)"
            )
            .into());
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "ADC_SEQ_TEST_END" {
        return Err(format!("expected \"ADC_SEQ_TEST_END\" to close the burst, got {got:?}").into());
    }

    println!(
        "  verified {}-line verdict burst (order-sensitive mixed-reference scan, reversed scan, \
         DMA drain, latch-park reruns, single-conversion restore)",
        EXPECTED_BURST.len()
    );
    Ok(())
}
