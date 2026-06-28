use std::error::Error;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

/// Absolute path to the eZ-FET emulator's flashing tool.
const DSLITE: &str =
    "/Applications/ti/ccs2051/ccs/ccs_base/DebugServer/bin/DSLite";

/// Default macOS device node for the eUSCI_A0 UART backchannel. Override with
/// the `MSP430_UART_PORT` env var if the board enumerates differently.
const DEFAULT_PORT: &str = "/dev/cu.usbmodem11203";

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Serial Port Tests...");

    // Build the on-device fixture and flash it before any assertions, so the
    // board is guaranteed to be running the firmware these tests expect.
    build_and_flash_serial_uart()?;

    test_9600_8_n_1_comms()?;

    println!("Serial Port Tests Completed Successfully");
    Ok(())
}

/// Cross-compile `hal_consumer`'s `serial_uart` binary for the MSP430 and flash
/// it to the attached board via DSLite.
fn build_and_flash_serial_uart() -> Result<(), Box<dyn Error>> {
    let repo_root = repo_root();

    println!("  building serial_uart (msp430-none-elf)...");
    let status = Command::new("cargo")
        .args(["+nightly", "build", "--bin", "serial_uart"])
        .current_dir(&repo_root)
        // This runner is itself launched by cargo, which exports env that would
        // otherwise leak into the child build: RUSTUP_TOOLCHAIN would override
        // our `+nightly`, and the target/target-dir vars would point the build
        // at the host triple / wrong output dir instead of the repo's msp430
        // config. Strip them so the child build uses the repo-root .cargo config.
        .env_remove("RUSTUP_TOOLCHAIN")
        .env_remove("CARGO_BUILD_TARGET")
        .env_remove("CARGO_TARGET_DIR")
        .status()?;
    if !status.success() {
        return Err("cargo build of serial_uart failed".into());
    }

    let elf = repo_root.join("target/msp430-none-elf/debug/serial_uart");
    let ccxml = repo_root.join("MSP430FR5969.ccxml");

    println!("  flashing serial_uart to board...");
    let status = Command::new(DSLITE)
        .arg("load")
        .arg("-c")
        .arg(&ccxml)
        .arg("-f")
        .arg(&elf)
        .status()?;
    if !status.success() {
        return Err("DSLite flash of serial_uart failed".into());
    }

    Ok(())
}

/// Open the board's UART at 9600 8N1 and verify it emits the fixed confirmation
/// burst that `serial_uart.rs` transmits once per second.
fn test_9600_8_n_1_comms() -> Result<(), Box<dyn Error>> {
    let port_path =
        std::env::var("MSP430_UART_PORT").unwrap_or_else(|_| DEFAULT_PORT.to_string());

    println!("  opening {port_path} @ 9600 8N1...");
    let mut port = serialport::new(&port_path, 9600)
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

    // The exact lines serial_uart.rs emits between its frame markers.
    const BEGIN: &str = "SERIAL_UART_TEST_BEGIN";
    const END: &str = "SERIAL_UART_TEST_END";
    let expected_body = ["UART 9600 8N1 OK", "hello from msp430fr5969"];

    // Bound the whole search; the fixture repeats every ~1 s, so a full burst
    // is captured well within this even if we attach mid-gap.
    let deadline = Instant::now() + Duration::from_secs(10);

    // Scan for a BEGIN marker, then assert the following lines match the
    // expected body and terminate with END. Mismatches after a BEGIN are real
    // failures (the fixture always emits the complete burst).
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            continue; // skip the boot banner / partial first burst until BEGIN
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "serial mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst at 9600 8N1");
        return Ok(());
    }
}

/// Read one CRLF/LF-terminated line from the port, byte at a time, giving up
/// once `deadline` passes. Per-read timeouts are retried until then.
fn read_line(
    port: &mut dyn serialport::SerialPort,
    deadline: Instant,
) -> Result<String, Box<dyn Error>> {
    let mut line: Vec<u8> = Vec::new();
    loop {
        if Instant::now() >= deadline {
            return Err(format!(
                "timed out waiting for a serial line (partial: {:?})",
                String::from_utf8_lossy(&line)
            )
            .into());
        }

        let mut byte = [0u8; 1];
        match port.read(&mut byte) {
            Ok(0) => continue,
            Ok(_) => {
                if byte[0] == b'\n' {
                    if line.last() == Some(&b'\r') {
                        line.pop();
                    }
                    return Ok(String::from_utf8_lossy(&line).into_owned());
                }
                line.push(byte[0]);
            }
            // A per-read timeout with no data yet — keep waiting until deadline.
            Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e.into()),
        }
    }
}

/// Repo root = parent of this crate's manifest directory.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("hal_integration_tests should have a parent directory")
        .to_path_buf()
}
