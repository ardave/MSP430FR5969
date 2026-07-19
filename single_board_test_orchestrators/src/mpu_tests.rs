use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `mpu_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// The fixture's self-delimited verdict burst, in order. All verdicts are
/// computed on-device across two lives (the fixture deliberately takes an
/// MPU-violation PUC partway through — see its state machine): write-through
/// before/after protection, a suppressed write to the fenced HighFram bank,
/// `MPUCTL1` flag latch + clear, `SYSNMI` delivery with exact `SYSSNIV`
/// demux (seg3 and info sources), the info-segment fence, the
/// PUC-on-violation reset cause (`SYSRSTIV` = MPU seg 3), and
/// lock-until-BOR.
const EXPECTED_BURST: [&str; 8] = [
    "MPU WRITE PRE OK",
    "MPU WRITE BLOCKED OK",
    "MPU FLAG LATCH OK",
    "MPU NMI DEMUX OK",
    "MPU INFO BLOCKED OK",
    "MPU WRITE POST OK",
    "MPU RESET ON VIOLATION OK",
    "MPU LOCK OK",
];

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting FRAM MPU Tests...");

    test_mpu_fixture()?;

    println!("FRAM MPU Tests Completed Successfully");
    Ok(())
}

/// Flash the MPU fixture (hands-free — protected memory, violations, and all
/// three consequence paths are software-only) and assert one complete verdict
/// burst. The burst only starts after the fixture's deliberate
/// PUC-on-violation reboot, a few seconds after flashing.
fn test_mpu_fixture() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("mpu_test_runner")?;

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

    // Give the freshly-flashed board a moment to reset, run the cold phase,
    // and reboot through its deliberate PUC.
    thread::sleep(Duration::from_millis(500));

    // Scan to a BEGIN (the burst repeats once per second once the state
    // machine settles), then assert the body and the closing END.
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "MPU_TEST_BEGIN" {
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
                "mpu verdict mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (the fixture's info line preceding the burst carries the state byte, \
                 cold-phase flags bitmask, border readback, and lock-probe evidence)"
            )
            .into());
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "MPU_TEST_END" {
        return Err(format!("expected \"MPU_TEST_END\" to close the burst, got {got:?}").into());
    }

    println!(
        "  verified {}-line verdict burst (fenced writes, flag latch, SYSNMI demux, \
         info segment, PUC-on-violation, lock-until-BOR)",
        EXPECTED_BURST.len()
    );
    Ok(())
}
