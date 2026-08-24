use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `ta_pwm_test_firmware` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Timer_A PWM Tests...");

    test_ta_pwm_self_check()?;

    println!("Timer_A PWM Tests Completed Successfully");
    Ok(())
}

/// Flash the Timer_A PWM fixture and verify its self-check burst. No wiring:
/// the fixture samples its own PWM pads through P1IN — frequency against the
/// TA0 counter, 25/75% duties on TA1's two channels, clean 0/100% rails,
/// channel independence, and the freed TA0 rebuilt as a second generator
/// (LED2 glows at half brightness as the alive indicator).
fn test_ta_pwm_self_check() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("ta_pwm_test_firmware")?;
    verify_self_check_burst()
}

/// Open the board's UART (8N1) and verify the fixed verdict burst the
/// `ta_pwm_test_firmware` fixture transmits once per second.
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

    const BEGIN: &str = "TA_PWM_TEST_BEGIN";
    const END: &str = "TA_PWM_TEST_END";
    let expected_body = [
        "TA PWM FREQ OK",
        "TA PWM DUTY25 OK",
        "TA PWM DUTY75 OK",
        "TA PWM RAILS OK",
        "TA PWM INDEP OK",
        "TA PWM TA0 OK",
    ];

    // The fixture's phases finish in well under a second; the burst repeats
    // every ~1 s. Bound the whole search generously anyway.
    let deadline = Instant::now() + Duration::from_secs(15);

    // Scan for a BEGIN marker (skipping the boot banner and the `ta pwm …`
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
                    "ta pwm mismatch after BEGIN: expected {expected:?}, got {got:?} \
                     (the fixture's `ta pwm …` info line carries the measured period \
                     and the sampled duty permilles)"
                )
                .into());
            }
        }

        let got = read_line(port.as_mut(), deadline)?;
        if got != END {
            return Err(format!("expected {END:?} to close the burst, got {got:?}").into());
        }

        println!(
            "  verified full {BEGIN}..{END} burst (frequency vs counter + 25/75% duties + \
             clean rails + channel independence + TA0 instance)"
        );
        return Ok(());
    }
}
