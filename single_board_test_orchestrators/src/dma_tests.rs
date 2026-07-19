use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `dma_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// The RX-pacing rounds: 16-byte payloads the fixture receives entirely by
/// DMA (its channel is armed *before* each READY announce) and echoes back
/// `+1`. Round 2 goes through the blocking `Rx::read_exact_dma` API, on a
/// distinct payload so a replayed round-1 echo can't pass.
const PAYLOAD: &[u8] = b"0123456789ABCDEF";
const PAYLOAD2: &[u8] = b"PQRSTUVWXYZabcde";

/// What the fixture's verdict burst must contain, in order. The `TXPAT` line
/// is a byte-for-byte content check on a 36-byte DMA-paced transmit; the
/// others are on-device self-verdicts (block copies, address modes, and the
/// DMAIV interrupt demux across all three channels).
const EXPECTED_BURST: [&str; 6] = [
    "DMA COPYB OK",
    "DMA COPYW OK",
    "DMA FILL OK",
    "DMA REV OK",
    "DMA IRQ OK",
    "DMA TXPAT 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ",
];

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting DMA Tests...");

    test_dma_fixture()?;

    println!("DMA Tests Completed Successfully");
    Ok(())
}

/// Flash the DMA fixture and drive it: assert the framed verdict burst
/// (which, arriving intact at all, is the DMA-paced UART TX test — every
/// byte the fixture transmits is moved by DMA channel 0), then run the
/// two-directional RX round (host sends go-byte + payload, fixture receives
/// the payload via `Rx::read_exact_dma` on channel 1 and echoes it `+1`
/// through the DMA transmit path).
fn test_dma_fixture() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("dma_test_runner")?;

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

    // 1. The verdict burst (repeats once per second; we may join mid-burst,
    // so scan for BEGIN first).
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "DMA_TEST_BEGIN" {
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
                "dma verdict mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (on-device detail precedes the burst)"
            )
            .into());
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "DMA_TEST_END" {
        return Err(format!("expected \"DMA_TEST_END\" to close the burst, got {got:?}").into());
    }
    println!(
        "  verified {}-line verdict burst (block copies, fill/reverse address modes, \
         DMAIV demux, DMA-paced TX)",
        EXPECTED_BURST.len()
    );

    // 2. The RX rounds. The fixture's receive window is anchored to *board*
    // time: its channel is armed just before READY goes out and abandoned a
    // few seconds later. But everything read so far may have been sitting in
    // the OS serial buffer (the port buffers while we sleep after flashing
    // and while we verify the burst), so a READY already in the backlog can
    // be seconds stale — answer it and the payload arrives after that
    // round's arm was abandoned. Drop the backlog first: with the buffer
    // empty, every read blocks until the board actually transmits, so the
    // READY we act on is at most one line-time (~20 ms) old. (Clearing may
    // split a line mid-flight; the partial line is skipped like any other.)
    port.clear(serialport::ClearBuffer::Input)?;
    let deadline = Instant::now() + Duration::from_secs(20);

    // Round 1: the pre-armed window (every payload byte lands by DMA).
    rx_round(port.as_mut(), deadline, "DMA_RX_READY", PAYLOAD, "DMA_ECHO ")?;
    println!(
        "  verified {}-byte payload received via pre-armed DMA RX and echoed +1 via DMA TX",
        PAYLOAD.len()
    );

    // Round 2: the blocking `read_exact_dma` public API, distinct payload.
    rx_round(port.as_mut(), deadline, "DMA_RX2_READY", PAYLOAD2, "DMA_ECHO2 ")?;
    println!(
        "  verified {}-byte payload received via read_exact_dma and echoed +1 via DMA TX",
        PAYLOAD2.len()
    );
    Ok(())
}

/// One announce → send → echo-check exchange: wait for a line starting with
/// `ready`, transmit `payload` in a single write, and assert the `echo_tag`
/// line carries every byte `+1`.
fn rx_round(
    port: &mut dyn serialport::SerialPort,
    deadline: Instant,
    ready: &str,
    payload: &[u8],
    echo_tag: &str,
) -> Result<(), Box<dyn Error>> {
    loop {
        let line = read_line(port, deadline)?;
        if line.starts_with(ready) {
            break;
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    }
    port.write_all(payload)?;
    port.flush()?;

    let echo = loop {
        let line = read_line(port, deadline)?;
        if let Some(rest) = line.strip_prefix(echo_tag) {
            break rest.to_string();
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    };
    let expected: String = payload.iter().map(|&b| (b + 1) as char).collect();
    if echo != expected {
        return Err(format!(
            "DMA RX echo mismatch after {ready}: sent {:?}, expected {expected:?} back, got {echo:?}",
            String::from_utf8_lossy(payload)
        )
        .into());
    }
    Ok(())
}
