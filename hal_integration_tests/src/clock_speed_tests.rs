use std::error::Error;
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `clock_speed_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1 — from a **16 MHz BRCLK**, which is itself
/// under test here.
const BAUD: u32 = 9600;

/// The fixture's verdict burst, in order (default build = the
/// `configure_max_speed` profile, MCLK = SMCLK = 16 MHz with one FRAM wait
/// state): the `FRCTL0.NWAITS` readback, the MCLK-vs-SMCLK ratio via a
/// timer-measured Delay, the DCO-independent VLO frequency through the
/// capture module, and an Info-FRAM round-trip under the new wait-state
/// setting.
const EXPECTED_BURST: [&str; 4] = [
    "CLKSPD FRCTL OK",
    "CLKSPD DELAY TIMER OK",
    "CLKSPD VLO OK",
    "CLKSPD FRAM RW OK",
];

/// Wall-clock gate on the BEGIN→BEGIN period. The loop is a 1 s Delay plus
/// ~0.2 s of burst at 9600 baud, so ~1.25 s nominal; a DCO stuck in the
/// low range would double the Delay to ~2.25 s. (It would also garble the
/// UART, since BRCLK rides the same SMCLK — this gate is the belt to that
/// suspenders.)
const PERIOD_MIN: Duration = Duration::from_millis(900);
const PERIOD_MAX: Duration = Duration::from_millis(1_800);

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting high-speed clock profile Tests...");

    test_clock_speed_fixture()?;

    println!("High-speed clock profile Tests Completed Successfully");
    Ok(())
}

/// Flash the clock-speed fixture (hands-free) and assert one complete verdict
/// burst plus the wall-clock repeat period.
fn test_clock_speed_fixture() -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("clock_speed_test_runner")?;

    let port_path = crate::serial::resolve_port()?;

    println!("  opening {port_path} @ {BAUD} 8N1...");
    let mut port = serialport::new(&port_path, BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .timeout(Duration::from_millis(1000))
        .open()?;

    // The eZ-FET gates the board's TX on DTR; assert it so the board's bytes reach us.
    port.write_data_terminal_ready(true)?;

    // Give the freshly-flashed board a moment to reset and start transmitting.
    thread::sleep(Duration::from_millis(500));

    // Scan to a BEGIN, assert the body and END, then time the gap to the
    // NEXT BEGIN — the absolute (host-wall-clock) check that the DCO really
    // moved to the high range. The info line the scan echoes carries the
    // profile's mclk/smclk, the NWAITS readback, the measured Delay and VLO
    // numbers, and the reset reason — the diagnostics for any miss.
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "CLKSPD_TEST_BEGIN" {
            break;
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    }
    let first_begin = Instant::now();
    for expected in EXPECTED_BURST {
        let got = read_line(port.as_mut(), deadline)?;
        if got != expected {
            return Err(format!(
                "clock-speed verdict mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (the fixture's info line preceding the burst has the NWAITS readback and \
                 the measured Delay/VLO numbers)"
            )
            .into());
        }
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "CLKSPD_TEST_END" {
        return Err(format!("expected \"CLKSPD_TEST_END\" to close the burst, got {got:?}").into());
    }

    // Wall-clock the repeat period (skip the info line, catch the next BEGIN).
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "CLKSPD_TEST_BEGIN" {
            break;
        }
    }
    let period = first_begin.elapsed();
    if !(PERIOD_MIN..=PERIOD_MAX).contains(&period) {
        return Err(format!(
            "burst period {period:?} outside {PERIOD_MIN:?}..{PERIOD_MAX:?} — the 1 s Delay is \
             running at the wrong rate, i.e. MCLK is not what the profile reports"
        )
        .into());
    }

    println!(
        "  verified {}-line verdict burst (NWAITS readback, Delay-vs-timer ratio, VLO via \
         capture, FRAM round-trip) + burst period {period:?} within wall-clock gate",
        EXPECTED_BURST.len()
    );
    Ok(())
}
