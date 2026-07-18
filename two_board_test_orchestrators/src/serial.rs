//! Serial plumbing for talking to BOTH boards' eUSCI_A0 backchannels.
//!
//! Same line-framing/timeout discipline as hal_integration_tests' serial
//! module, extended for a two-board world: instead of demanding exactly one
//! `/dev/cu.usbmodem*3` node, discovery returns them all and the caller
//! identifies which board is which by *asking* (the fixture's `i` command) —
//! never by device path, which moves with USB topology.

use std::error::Error;
use std::io;
use std::time::{Duration, Instant};

/// Env vars that pin the two backchannels explicitly, skipping /dev scanning
/// (identity is still verified over the wire afterwards).
pub const PARENT_PORT_ENV: &str = "TWO_BOARD_PARENT_PORT";
pub const CHILD_PORT_ENV: &str = "TWO_BOARD_CHILD_PORT";

/// All candidate eZ-FET backchannel UART nodes. The eZ-FET enumerates two CDC
/// interfaces per board: the debug interface ends in `1`, the
/// application-UART backchannel ends in `3` — so a two-board rig shows two
/// `cu.usbmodem*3` nodes.
pub fn candidate_ports() -> Result<Vec<String>, Box<dyn Error>> {
    if let (Ok(parent), Ok(child)) = (
        std::env::var(PARENT_PORT_ENV),
        std::env::var(CHILD_PORT_ENV),
    ) {
        return Ok(vec![parent, child]);
    }

    let mut modems: Vec<String> = std::fs::read_dir("/dev")?
        .filter_map(|entry| entry.ok()?.file_name().into_string().ok())
        .filter(|name| name.starts_with("cu.usbmodem") && name.ends_with('3'))
        .map(|name| format!("/dev/{name}"))
        .collect();
    modems.sort();
    Ok(modems)
}

/// Open a backchannel at 9600 8N1 with DTR asserted (the eZ-FET gates the
/// board's TX on DTR) and a 1 s per-read timeout (overall bounds come from
/// caller deadlines).
pub fn open(path: &str) -> Result<Box<dyn serialport::SerialPort>, Box<dyn Error>> {
    let mut port = serialport::new(path, 9600)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .timeout(Duration::from_millis(1000))
        .open()?;
    port.write_data_terminal_ready(true)?;
    Ok(port)
}

/// Read one CRLF/LF-terminated line, byte at a time, giving up once
/// `deadline` passes. Per-read timeouts are retried until then, so a silent
/// board fails with a clear message instead of blocking forever.
pub fn read_line(
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

/// Pull `key=<number>` out of a fixture report line.
pub fn field(line: &str, key: &str) -> Option<u32> {
    let idx = line.find(&format!("{key}="))? + key.len() + 1;
    let rest = &line[idx..];
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}
