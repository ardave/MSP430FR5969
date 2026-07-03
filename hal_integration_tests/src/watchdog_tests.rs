use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `watchdog_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Watchdog Tests...");

    test_watchdog_reset_chain()?;

    println!("Watchdog Tests Completed Successfully");
    Ok(())
}

/// Flash the watchdog fixture and verify its final verdict burst. No external
/// wiring is involved: WDT_A, the SYS reset-vector generator, and the Info
/// FRAM that carries state across the fixture's self-inflicted reboots are
/// all on-chip.
fn test_watchdog_reset_chain() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("watchdog_test_runner")?;
    verify_verdict_burst()
}

/// Open the board's UART (8N1) and verify the framed verdict burst the
/// fixture settles into once its reboot chain completes.
///
/// The fixture reboots itself twice on the way here — a genuine watchdog
/// timeout, then a deliberate `force_reset` password violation — persisting
/// each boot's verdict in Info FRAM and classifying each reboot by draining
/// `SYSRSTIV` (`hal::sys::ResetReasons`). The three verdict lines therefore
/// certify, in order: feeding held a ~1 s watchdog off for 3 s; the starved
/// watchdog reset the chip and the next boot decoded `WDT timeout` (0x16);
/// and `force_reset` rebooted with `WDT password` (0x18) decoded. Any broken
/// link flips its line to FAIL — the fixture always completes the chain and
/// always emits the full burst, so a mismatch here is a real failure, not a
/// hung board.
fn verify_verdict_burst() -> Result<(), Box<dyn Error>> {
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

    // The eZ-FET gates the board's TX on DTR (the same reason `screen` works but
    // a bare reader sees nothing). Assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    // The exact verdict lines between the frame markers, all in their
    // pass state.
    const BEGIN: &str = "WDT_TEST_BEGIN";
    const END: &str = "WDT_TEST_END";
    let expected_body = ["WDT FEED OK", "WDT TIMEOUT RESET OK", "WDT KEY RESET OK"];

    // Bound the whole search generously: the reboot chain takes ~5 s from
    // flash (3 s of feeding, ~1 s until the bite, two fast reboots), then the
    // burst repeats every ~1 s.
    let deadline = Instant::now() + Duration::from_secs(25);

    // Scan for a BEGIN marker (skipping boot banners, the `reasons:` info
    // lines, and the intermediate-phase narration), then assert the verdict
    // body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "watchdog mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!(
            "  verified full {BEGIN}..{END} burst (feed survival + timeout reset 0x16 + key reset 0x18)"
        );
        return Ok(());
    }
}
