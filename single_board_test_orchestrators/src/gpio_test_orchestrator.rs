use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `gpio_irq_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting GPIO Interrupt Tests...");

    test_port_irq_self_check()?;

    println!("GPIO Interrupt Tests Completed Successfully");
    Ok(())
}

/// Flash the port-interrupt fixture and verify its self-check burst. No hands
/// and no wiring involved: the fixture "presses" the LaunchPad buttons from
/// software (PxIFG is software-settable and fires the vector like a real
/// edge), so this validates enable → vector → PxIV demux → auto-clear on both
/// PORT1 and PORT4 end-to-end.
fn test_port_irq_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("gpio_irq_test_firmware")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `gpio_irq_test_firmware` fixture transmits once per second.
fn verify_self_check_burst() -> Result<(), Box<dyn Error>> {
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

    const BEGIN: &str = "GPIO_TEST_BEGIN";
    const END: &str = "GPIO_TEST_END";
    let expected_body = ["GPIO IV OK", "GPIO CLEAR OK", "GPIO P4 OK"];

    // The fixture's self-checks finish within ~50 ms of boot and the burst
    // repeats every ~1 s; bound the whole search generously anyway.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `gpio p1=…`
    // info line), then assert the verdict body and the closing END.
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line != BEGIN {
            if !line.is_empty() {
                println!("  [board] {line}");
            }
            continue;
        }

        for expected in expected_body {
            let got = read_line(port.as_mut(), deadline)?;
            if got != expected {
                return Err(format!(
                    "gpio mismatch after BEGIN: expected {expected:?}, got {got:?}"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!("  verified full {BEGIN}..{END} burst (IV demux + auto-clear + PORT4)");
        return Ok(());
    }
}
