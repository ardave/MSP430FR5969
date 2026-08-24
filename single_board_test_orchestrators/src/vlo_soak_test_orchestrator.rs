use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `vlo_soak_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1 — but only after its 200-reboot soak.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting VLO/ACLK boot-race soak (200 self-reboots; instrument, not a regression gate)...");

    run_soak()?;

    println!("VLO Soak Completed");
    Ok(())
}

/// Flash the soak fixture, wait out the reboot storm (the board is silent
/// while sampling — each boot measures ACLK, records to Info FRAM, and
/// watchdog-resets), then collect the tally.
fn run_soak() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("vlo_soak_test_firmware")?;

    let port_path = crate::serial::resolve_port()?;

    println!("  opening {port_path} @ {BAUD} 8N1 (board is rebooting itself ~200x; report follows)...");
    let mut port = serialport::new(&port_path, BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .timeout(Duration::from_millis(1000))
        .open()?;

    // The eZ-FET gates the board's TX on DTR; assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Generous deadline: 200 boots at tens of ms each is seconds, but leave
    // slack for a chip that spends its full 0.1 s retry budget every boot.
    let deadline = Instant::now() + Duration::from_secs(120);
    let mut info_line = String::new();
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "VLO_SOAK_BEGIN" {
            break;
        }
        if !line.is_empty() {
            println!("  [board] {line}");
            if line.starts_with("vlosoak ") {
                info_line = line;
            }
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "SOAK COMPLETE OK" {
        return Err(format!(
            "soak did not complete: expected \"SOAK COMPLETE OK\", got {got:?} \
             (the vlosoak info line has the partial tally)"
        )
        .into());
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "VLO_SOAK_END" {
        return Err(format!("expected \"VLO_SOAK_END\" to close the burst, got {got:?}").into());
    }

    println!("  soak tally: {info_line}");
    println!(
        "  (flaky = ACLK recovered after >1 attempt; dead = no edge within the \
         per-boot 0.1 s retry budget; maxtries = worst recovery. These are PUC-class \
         boots — the field observations were reset-pin boots, so zero flakes here \
         localizes the race to reset-pin/debugger-release boots.)"
    );
    Ok(())
}
