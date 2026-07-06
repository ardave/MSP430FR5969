use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `comp_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// The fixture's self-delimited verdict burst, in order. All verdicts are
/// computed on-device with no wiring — REFOUT (the shared reference buffered
/// onto P1.1 = C1) is the analog stimulus, and stepping the VCC ladder taps
/// across it makes real comparator edges from software: far-tap rail
/// comparisons, the CEEX swap+invert-cancel invariant at both output states,
/// the CEIV edge demux (rising 0x02 / falling 0x04, exactly once each,
/// auto-cleared), a comparator-edge wake from LPM0, and two full 32-tap
/// ladder sweeps at 2.0 V and 1.2 V whose flip taps must match the
/// prediction from the ADC-measured AVCC within ±2 taps. (Keep button S2
/// released — it shorts P1.1/REFOUT to ground.)
const EXPECTED_BURST: [&str; 7] = [
    "COMP RAILS OK",
    "COMP EXCHANGE OK",
    "COMP IRQ IV OK",
    "COMP LPM0 WAKE OK",
    "COMP SWEEP MONO OK",
    "COMP SWEEP 2V0 OK",
    "COMP SWEEP 1V2 OK",
];

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Comp_E (analog comparator) Tests...");

    test_comp_fixture()?;

    println!("Comp_E Tests Completed Successfully");
    Ok(())
}

/// Flash the comparator fixture (hands-free) and assert one complete verdict
/// burst.
fn test_comp_fixture() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("comp_test_runner")?;

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
    // body and the closing END. The pre-burst info line the scan echoes
    // carries the measured AVCC, both sweep masks, and the flip taps — the
    // diagnostics for any sweep-window miss.
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "COMP_TEST_BEGIN" {
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
                "comp verdict mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (the fixture's info line preceding the burst has the ADC-measured AVCC, \
                 both 32-tap sweep masks, and the flip taps)"
            )
            .into());
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "COMP_TEST_END" {
        return Err(format!("expected \"COMP_TEST_END\" to close the burst, got {got:?}").into());
    }

    println!(
        "  verified {}-line verdict burst (rails, CEEX, CEIV demux, LPM0 wake, \
         REFOUT ladder sweeps at 2.0/1.2 V)",
        EXPECTED_BURST.len()
    );
    Ok(())
}
