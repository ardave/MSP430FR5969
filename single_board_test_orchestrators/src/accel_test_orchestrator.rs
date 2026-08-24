use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `accel_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// The fixture's self-delimited verdict burst, in order. All verdicts are
/// computed on-device: the CRC16 silicon against the software reference model
/// (both bit-order register pairs, three patterns, two seeds, the word-write
/// fast path) and against the published catalog check values for
/// CCITT-FALSE/XMODEM/KERMIT/X-25; the AES accelerator against the FIPS-197
/// appendix-C known-answer vectors in both directions, the SP800-38A
/// CBC-AES128 two-block vector, and a 48-byte multi-block ECB round trip.
const EXPECTED_BURST: [&str; 8] = [
    "ACCEL CRC MODEL OK",
    "ACCEL CRC CATALOG OK",
    "ACCEL AES128 KAT OK",
    "ACCEL AES192 KAT OK",
    "ACCEL AES256 KAT OK",
    "ACCEL AES DECRYPT OK",
    "ACCEL AES CBC OK",
    "ACCEL AES ROUNDTRIP OK",
];

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Accelerator (CRC16 + AES256) Tests...");

    test_accel_fixture()?;

    println!("Accelerator Tests Completed Successfully");
    Ok(())
}

/// Flash the accelerator fixture (hands-free — both modules are pure bus
/// peripherals) and assert one complete verdict burst.
fn test_accel_fixture() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("accel_test_firmware")?;

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
        if line == "ACCEL_TEST_BEGIN" {
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
                "accel verdict mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (the fixture's info line preceding the burst has the raw CRC catalog values \
                 and the first AES-128 ciphertext bytes)"
            )
            .into());
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "ACCEL_TEST_END" {
        return Err(format!("expected \"ACCEL_TEST_END\" to close the burst, got {got:?}").into());
    }

    println!(
        "  verified {}-line verdict burst (CRC16 model + catalog, AES KATs both directions, \
         CBC vector, multi-block round trip)",
        EXPECTED_BURST.len()
    );
    Ok(())
}
