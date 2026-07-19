use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `lpmx5_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting LPMx.5 Tests...");

    test_lpmx5_wake_chain()?;

    println!("LPMx.5 Tests Completed Successfully");
    Ok(())
}

/// Flash the LPMx.5 fixture and walk it through its three lives. No wiring:
/// the host itself is the LPM4.5 wake source — the UART TX line lands on the
/// board's P2.1, and a start bit is a falling edge on a pin armed as a GPIO
/// wake. The byte that carries the edge is sacrificed to unpowered silicon
/// (the UART doesn't exist while the core domain is off); a follow-up
/// exchange after the reboot proves the link still works — which is itself
/// the "you will lose the first byte" LPMx.5 lesson, asserted.
fn test_lpmx5_wake_chain() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("lpmx5_test_runner")?;

    let port_path = crate::serial::resolve_port()?;
    println!("  opening {port_path} @ {BAUD} 8N1...");
    let mut port = serialport::new(&port_path, BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        // Per-read timeout; deadline loops below bound the overall waits.
        .timeout(Duration::from_millis(1000))
        .open()?;

    // The eZ-FET gates the board's TX on DTR; assert it so its bytes reach us.
    port.write_data_terminal_ready(true)?;

    // --- Life 1: handshake, then let it power down -------------------------
    // The fixture repeats LPMX5_READY until poked, so nothing is lost if the
    // board booted before the port opened.
    wait_for_line(port.as_mut(), "LPMX5_READY", Duration::from_secs(10))?;
    port.write_all(b"g")?; // any byte: "host is listening, go to sleep"
    port.flush()?;
    wait_for_line(port.as_mut(), "LPMX5_SLEEPING mode=4.5", Duration::from_secs(5))?;
    println!("  board is entering LPM4.5; waiting, then sending the wake byte...");

    // Let it actually reach LPM4.5 (entry is microseconds; this margin is for
    // the UART FIFO to drain and for us to be conservative).
    thread::sleep(Duration::from_millis(1500));

    // --- Wake it: one byte = one falling edge on P2.1 ----------------------
    port.write_all(b"W")?;
    port.flush()?;

    // Life 2 reboots through the BOR path (~ms), verifies the pin evidence,
    // stages the RTC at 00:00:55, and goes back down in LPM3.5.
    wait_for_line(port.as_mut(), "LPMX5_SLEEPING mode=3.5", Duration::from_secs(10))?;
    println!("  woke from LPM4.5 via UART-pin edge; board now in LPM3.5 awaiting RTC...");

    // --- Life 3: the RTC minute event fires ~5 s later ---------------------
    // Generous bound: crystal settle + 5 s to the minute rollover + reboot.
    wait_for_line(port.as_mut(), "LPMX5_TEST_BEGIN", Duration::from_secs(30))?;
    let deadline = Instant::now() + Duration::from_secs(5);
    for expected in ["LPMX5 PIN OK", "LPMX5 RTC OK", "LPMX5 TIME OK", "LPMX5_TEST_END"] {
        let got = read_line(port.as_mut(), deadline)?;
        if got != expected {
            return Err(format!("lpmx5 mismatch: expected {expected:?}, got {got:?}").into());
        }
    }

    println!("  verified full LPM4.5 pin-wake + LPM3.5 RTC-wake chain (3 lives)");
    Ok(())
}

/// Read lines (echoing info lines) until `wanted` appears, or fail at the
/// timeout.
fn wait_for_line(
    port: &mut dyn serialport::SerialPort,
    wanted: &str,
    timeout: Duration,
) -> Result<(), Box<dyn Error>> {
    let deadline = Instant::now() + timeout;
    loop {
        let line = read_line(port, deadline)?;
        if line == wanted {
            return Ok(());
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    }
}
