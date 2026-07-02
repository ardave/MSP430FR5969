//! Shared serial-port helpers for talking to flashed fixtures over the eUSCI_A0
//! UART backchannel.
//!
//! Peripheral-agnostic plumbing that any `*_tests.rs` module reporting over the
//! UART can reuse, so the line-framing and timeout logic lives in one place.

use std::error::Error;
use std::io;
use std::time::Instant;

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
