//! Shared serial-port helpers for talking to flashed fixtures over the eUSCI_A0
//! UART backchannel.
//!
//! Peripheral-agnostic plumbing that any `*_tests.rs` module reporting over the
//! UART can reuse, so the line-framing and timeout logic lives in one place.

use std::error::Error;
use std::io;
use std::time::Instant;

/// Env var that overrides backchannel auto-discovery.
const PORT_ENV: &str = "MSP430_UART_PORT";

/// Find the eZ-FET backchannel UART's device node.
///
/// Resolution order: `MSP430_UART_PORT` if set, else scan `/dev`. The scan
/// exists because macOS names the nodes `cu.usbmodem<location><iface>`, where
/// `<location>` encodes the physical USB port — plug the LaunchPad into a
/// different port and the node moves (observed: `11201`/`11203` on one port,
/// `11401`/`11403` on another), which is exactly how a hardcoded default
/// rots. The eZ-FET enumerates two CDC interfaces: the debug interface ends
/// in `1`, the application-UART backchannel ends in `3` — so pick the sole
/// `cu.usbmodem*3`. If that pattern doesn't identify exactly one node (no
/// board, several boards, an unrelated modem), fail with the candidate list
/// rather than guess.
pub fn resolve_port() -> Result<String, Box<dyn Error>> {
    if let Ok(port) = std::env::var(PORT_ENV) {
        return Ok(port);
    }

    let mut modems: Vec<String> = std::fs::read_dir("/dev")?
        .filter_map(|entry| entry.ok()?.file_name().into_string().ok())
        .filter(|name| name.starts_with("cu.usbmodem"))
        .map(|name| format!("/dev/{name}"))
        .collect();
    modems.sort();

    let backchannels: Vec<&String> = modems.iter().filter(|n| n.ends_with('3')).collect();
    match backchannels.as_slice() {
        [port] => Ok((*port).clone()),
        _ => Err(format!(
            "could not identify the eZ-FET backchannel UART (want exactly one \
             /dev/cu.usbmodem*3; found {modems:?}); set {PORT_ENV} to the right node"
        )
        .into()),
    }
}

/// Read one CRLF/LF-terminated line from the port, byte at a time, giving up
/// once `deadline` passes. Per-read timeouts are retried until then, so a silent
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
