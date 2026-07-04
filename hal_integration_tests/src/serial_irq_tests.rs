use std::error::Error;
use std::io::{self, Write as _};
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `serial_irq_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// What the host sends once the board reports READY: 23 payload bytes plus
/// the `\n` that triggers the stats line — 24 in flight, comfortably under
/// the fixture's 32-byte queue even if the board sat in its announce delay
/// for the whole burst.
const PATTERN: &[u8] = b"abcdefghijklmnopqrstuvw\n";

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Serial RX-Interrupt Tests...");

    test_interrupt_echo()?;

    println!("Serial RX-Interrupt Tests Completed Successfully");
    Ok(())
}

/// Flash the RX-interrupt echo fixture and drive it. This is the suite's first
/// **two-directional** test: the host transmits a known pattern and the board
/// echoes every byte **plus one** — a value only software that received the
/// byte through the ISR → RxQueue → main path (waking from LPM0 each time)
/// could produce. The trailing stats line then proves the queue never dropped
/// and the ISR saw no receive errors.
fn test_interrupt_echo() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("serial_irq_test_runner")?;

    let port_path = crate::serial::resolve_port()?;

    println!("  opening {port_path} @ {BAUD} 8N1...");
    let mut port = serialport::new(&port_path, BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        // Per-read timeout; the deadline loops below bound the overall wait.
        .timeout(Duration::from_millis(1000))
        .open()?;

    // The eZ-FET gates the board's TX on DTR; assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    // 1. Wait for the READY announce (repeats once per second until traffic).
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "UART_IRQ_READY" {
            break;
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    }

    // 2. Send the pattern and collect exactly one echo byte per sent byte.
    println!("  sending {}-byte pattern...", PATTERN.len());
    port.write_all(PATTERN)?;
    port.flush()?;

    let deadline = Instant::now() + Duration::from_secs(10);
    let echoed = read_exact(port.as_mut(), PATTERN.len(), deadline)?;
    for (i, (&sent, &got)) in PATTERN.iter().zip(echoed.iter()).enumerate() {
        let expected = sent.wrapping_add(1);
        if got != expected {
            return Err(format!(
                "echo mismatch at byte {i}: sent 0x{sent:02X}, expected 0x{expected:02X} \
                 back, got 0x{got:02X} (full echo: {echoed:02X?})"
            )
            .into());
        }
    }

    // 3. The `\n` terminator triggers the stats line; the CRLF that precedes
    // it reads as one empty line first.
    let stats = loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line.starts_with("UART_IRQ_STATS") {
            break line;
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    };
    if stats != "UART_IRQ_STATS dropped=0 errors=0" {
        return Err(format!("expected a clean stats line, got {stats:?}").into());
    }

    println!(
        "  verified {} bytes echoed +1 through ISR/RxQueue/LPM0-wake, no drops, no errors",
        PATTERN.len()
    );
    Ok(())
}

/// Read exactly `n` raw bytes (the echo stream is *not* line-framed — a `+1`
/// echo of `\n` is 0x0B, not a terminator), retrying per-read timeouts until
/// `deadline`.
fn read_exact(
    port: &mut dyn serialport::SerialPort,
    n: usize,
    deadline: Instant,
) -> Result<Vec<u8>, Box<dyn Error>> {
    let mut bytes = Vec::with_capacity(n);
    while bytes.len() < n {
        if Instant::now() >= deadline {
            return Err(format!(
                "timed out after {} of {n} echo bytes (got: {bytes:02X?})",
                bytes.len()
            )
            .into());
        }
        let mut byte = [0u8; 1];
        match port.read(&mut byte) {
            Ok(0) => continue,
            Ok(_) => bytes.push(byte[0]),
            Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
            Err(e) => return Err(e.into()),
        }
    }
    Ok(bytes)
}
