use std::error::Error;
use std::io::{self, Write as _};
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `spi_test_runner` fixture reports over the backchannel at the project's
/// baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting SPI Tests...");

    test_loopback_round_trips()?;

    println!("SPI Tests Completed Successfully");
    Ok(())
}

/// The eUSCI_B0 SPI master has no on-chip way to feed its own transmit line back
/// to its receive line, so a loopback test needs an external jumper. Prompt the
/// operator to install it and confirm before we flash and check the round-trip.
fn test_loopback_round_trips() -> Result<(), Box<dyn Error>> {
    prompt_for_loopback_jumper()?;
    deployment::build_and_flash("spi_test_runner")?;
    verify_loopback_burst()
}

/// Print the jumper hookup and block until the operator presses Enter (or aborts
/// with Ctrl-C). The fixture only round-trips its pattern once SIMO is wired back
/// to SOMI, so we must not flash until the jumper is in place.
fn prompt_for_loopback_jumper() -> Result<(), Box<dyn Error>> {
    println!();
    println!("  ┌─ SPI loopback: jumper eUSCI_B0's SIMO back to its SOMI ────────────┐");
    println!("  │                                                                    │");
    println!("  │    MSP430FR5969 LaunchPad                                           │");
    println!("  │    ----------------------                                          │");
    println!("  │    P1.6 (SIMO) ───┐                                                 │");
    println!("  │                   │  one jumper wire                                │");
    println!("  │    P1.7 (SOMI) ───┘                                                 │");
    println!("  │                                                                    │");
    println!("  │    • Install a single jumper between P1.6 and P1.7 so every byte   │");
    println!("  │      the SPI master transmits is clocked straight back in.          │");
    println!("  │    • If you previously ran the I2C scan, no pull-ups are needed     │");
    println!("  │      here — but make sure any BME280 breakout is disconnected so    │");
    println!("  │      it doesn't drive SOMI.                                         │");
    println!("  │                                                                    │");
    println!("  │    The fixture transfers A5 3C FF 00 55 AA in place; with the       │");
    println!("  │    jumper every byte round-trips, so the loopback PASSes.           │");
    println!("  │                                                                    │");
    println!("  └────────────────────────────────────────────────────────────────────┘");
    print!("  Install the jumper, then press Enter to flash and test (Ctrl-C to abort)... ");
    io::stdout().flush()?;

    let mut _line = String::new();
    let n = io::stdin().read_line(&mut _line)?;
    if n == 0 {
        // EOF (e.g. stdin closed / non-interactive) — no operator to confirm.
        return Err("aborted: no confirmation on stdin (EOF)".into());
    }
    println!();
    Ok(())
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `spi_test_runner` fixture transmits once per second.
///
/// The fixture transfers once at startup and always emits the complete burst, so
/// a body mismatch after BEGIN is a real failure: `SPI LOOPBACK FAIL` means the
/// received pattern did not match the sent one (missing jumper or floating SOMI).
fn verify_loopback_burst() -> Result<(), Box<dyn Error>> {
    let port_path = crate::serial::resolve_port()?;

    println!("  opening {port_path} @ {BAUD} 8N1...");
    let mut port = serialport::new(&port_path, BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        // Per-read timeout; the deadline loop below bounds the overall wait.
        .timeout(Duration::from_millis(1000))
        .open()?;

    // The eZ-FET gates the board's TX on DTR; assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    const BEGIN: &str = "SPI_TEST_BEGIN";
    const END: &str = "SPI_TEST_END";
    let expected_body = ["SPI LOOPBACK OK"];

    // Bound the whole search generously: the fixture transfers (~microseconds) at
    // startup, then repeats the burst every ~1 s.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `spi sent=…` line),
    // then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            // Surface the fixture's `spi sent=…/recv=…` diagnostics while scanning for BEGIN.
            if !line.is_empty() {
                println!("  [board] {line}");
            }
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "spi mismatch after BEGIN: expected {expected:?}, got {got:?} \
                     (missing jumper? check the P1.6<->P1.7 loopback wire)"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (pattern round-tripped through loopback)");
        return Ok(());
    }
}
