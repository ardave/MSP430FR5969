use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `ta0_probe_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// The fixture's self-delimited burst, in order: four on-device verdicts
/// (channel-2 positive control, per-channel probe consistency, no aliasing
/// onto CCR0–CCR2), then the two **findings** pinned to what silicon
/// established. SLAS704G's TA0 register table lists TA0CCTL3/4 + TA0CCR3/4
/// (a five-channel TA0), but its own §6.10.10 prose, its Table 6-13, TI's
/// msp430fr5969.h (`Timer0_A3`), and the SVD all say three channels. The
/// probe (register readback + functional software-CCIS capture + TA0IV
/// demux, all of which must agree) decides; the pinned lines below record
/// the answer, so a regression — or a different die revision — fails here.
const EXPECTED_BURST: [&str; 6] = [
    "TA0 CH2 CONTROL OK",
    "TA0 CH3 CONSISTENT OK",
    "TA0 CH4 CONSISTENT OK",
    "TA0 NO ALIAS OK",
    "TA0 CH3 ABSENT",
    "TA0 CH4 ABSENT",
];

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting TA0 Channel-Count Probe Tests...");

    test_ta0_probe_fixture()?;

    println!("TA0 Channel-Count Probe Tests Completed Successfully");
    Ok(())
}

/// Flash the probe fixture (hands-free — the software CCIS fire needs no
/// pins) and assert one complete verdict-and-findings burst.
fn test_ta0_probe_fixture() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("ta0_probe_test_runner")?;

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
        if line == "TA0_PROBE_BEGIN" {
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
                "ta0 probe mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (a CH3/CH4 PRESENT finding would mean the silicon really has the extra \
                 channels and the SVD/PAC/HAL should grow them; MIXED means the probes \
                 disagreed — see the fixture's info line preceding the burst for the raw \
                 rbccr/rbctl/cap/brk/iv bits per channel)"
            )
            .into());
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "TA0_PROBE_END" {
        return Err(format!("expected \"TA0_PROBE_END\" to close the burst, got {got:?}").into());
    }

    println!(
        "  verified {}-line burst (ch2 control, ch3/ch4 probe consistency, no aliasing, \
         and the pinned findings: TA0 has no CCR3/CCR4 — SLAS704G's register table is \
         the erratum, the three-channel prose/header/SVD are right)",
        EXPECTED_BURST.len()
    );
    Ok(())
}
