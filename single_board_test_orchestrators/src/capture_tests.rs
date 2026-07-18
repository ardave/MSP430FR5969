use std::error::Error;
use std::io::{self, Write as _};
use std::thread;
use std::time::{Duration, Instant};

use crate::deployment;
use crate::serial::read_line;

/// The `capture_test_runner` fixture reports over the backchannel at the
/// project's baseline 9600 8N1.
const BAUD: u32 = 9600;

/// The fixture's hands-free verdicts, in burst order. All six come from
/// stimuli TA1 can reach with no wiring: a software-fired capture (the `CCIS`
/// GND→VCC flip), ACLK on the internal `CCI2B` input (the crystal timestamped
/// by DCO ticks — the span ratio *is* the DCO's frequency error, gated at
/// ±5%), Comp_E's `COUT` on the internal `CCI1B` input (edges made by
/// stepping the ladder taps across REFOUT — keep button S2 released), the
/// `TA1IV` demux with its read-auto-clear, and an ACLK-capture wake from
/// LPM0.
const HANDS_FREE_BURST: [&str; 6] = [
    "CAPT SOFT FIRE OK",
    "CAPT SOFT COV OK",
    "CAPT ACLK SPAN OK",
    "CAPT COUT EDGES OK",
    "CAPT IV DEMUX OK",
    "CAPT LPM0 WAKE OK",
];

/// The two jumper-dependent verdicts that follow: the fixture's own ~1 kHz
/// TB0.1 PWM measured back through the P1.4→P1.2 jumper. The fixture detects
/// the jumper itself (GPIO drive + pull-down read) and emits `SKIP` instead
/// of a verdict when it is absent.
const PWM_LINES: [&str; 2] = ["CAPT PWM FREQ", "CAPT PWM DUTY"];

/// Default (hands-free) entry: flash the fixture and assert the six
/// wiring-free verdicts; the PWM lines may be `OK` **or** `SKIP` (no jumper
/// required in the default suite), but never `FAIL`.
pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting Timer_A capture Tests (hands-free; PWM jumper optional)...");

    verify_burst(false)?;

    println!("Timer_A capture Tests Completed Successfully");
    Ok(())
}

/// Interactive entry (run by name: `cargo +nightly run -- capture_jumper`):
/// prompt for the P1.4→P1.2 jumper first, then require the PWM verdicts to
/// really pass — `SKIP` here means the jumper was not detected and fails.
pub fn run_with_jumper() -> Result<(), Box<dyn Error>> {
    println!("Starting Timer_A capture Tests (with PWM loopback jumper)...");

    prompt_for_jumper()?;
    verify_burst(true)?;

    println!("Timer_A capture Tests Completed Successfully");
    Ok(())
}

/// Print the jumper hookup and block until the operator presses Enter (or
/// aborts with Ctrl-C). The fixture only measures its own PWM once P1.4 is
/// wired to P1.2, so we must not flash until the jumper is in place.
fn prompt_for_jumper() -> Result<(), Box<dyn Error>> {
    println!();
    println!("  ┌─ PWM capture loopback: jumper the PWM output to the capture input ─┐");
    println!("  │                                                                    │");
    println!("  │    MSP430FR5969 LaunchPad                                          │");
    println!("  │    ----------------------                                          │");
    println!("  │    P1.4 (TB0.1 PWM out) ───┐                                       │");
    println!("  │                            │  jumper wire                          │");
    println!("  │    P1.2 (TA1.CCI1A in)  ───┘                                       │");
    println!("  │                                                                    │");
    println!("  │    • The fixture generates ~1 kHz PWM at 25% and 75% duty on P1.4  │");
    println!("  │      and measures it back through TA1 capture on P1.2 — frequency  │");
    println!("  │      must match the PWM driver's own report within 1%, both duty   │");
    println!("  │      points within ±2% (asymmetric duties, so an inverted or       │");
    println!("  │      stuck line cannot pass).                                      │");
    println!("  │    • The fixture detects the jumper itself (P1.4 driven as GPIO,   │");
    println!("  │      P1.2 read with a pull-down) — SKIP in the output means it     │");
    println!("  │      saw no jumper.                                                │");
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

/// Flash the capture fixture and assert one complete verdict burst.
/// `require_pwm` decides whether the two jumper-dependent lines must be `OK`
/// or may be `SKIP`.
fn verify_burst(require_pwm: bool) -> Result<(), Box<dyn Error>> {
    deployment::build_and_flash("capture_test_runner")?;

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

    // Scan to a BEGIN (the burst repeats once per second), then assert the
    // body and the closing END. The pre-burst info line the scan echoes
    // carries the ACLK span/ratio, the measured PWM numbers, the jumper
    // detection, and the TA1IV tallies — the diagnostics for any miss.
    let deadline = Instant::now() + Duration::from_secs(20);
    loop {
        let line = read_line(port.as_mut(), deadline)?;
        if line == "CAPT_TEST_BEGIN" {
            break;
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    }
    for expected in HANDS_FREE_BURST {
        let got = read_line(port.as_mut(), deadline)?;
        if got != expected {
            return Err(format!(
                "capture verdict mismatch after BEGIN: expected {expected:?}, got {got:?} \
                 (the fixture's info line preceding the burst has the ACLK span, DCO/crystal \
                 ratio in permille, PWM measurements, jumper detection, and TA1IV tallies)"
            )
            .into());
        }
    }
    for name in PWM_LINES {
        let got = read_line(port.as_mut(), deadline)?;
        let ok = format!("{name} OK");
        let skip = format!("{name} SKIP");
        if got == ok {
            continue;
        }
        if got == skip && !require_pwm {
            println!("  [board] {got} (no P1.4→P1.2 jumper — fine in the hands-free suite)");
            continue;
        }
        return Err(if got == skip {
            format!(
                "{name}: fixture reports SKIP — it did not detect the P1.4→P1.2 jumper \
                 (check the wire; the fixture probes it with P1.4 driven as GPIO against \
                 a pull-down on P1.2)"
            )
            .into()
        } else {
            format!("capture verdict mismatch: expected {ok:?} (or SKIP), got {got:?}").into()
        });
    }
    let got = read_line(port.as_mut(), deadline)?;
    if got != "CAPT_TEST_END" {
        return Err(format!("expected \"CAPT_TEST_END\" to close the burst, got {got:?}").into());
    }

    println!(
        "  verified verdict burst (software capture + COV, ACLK/DCO span ratio, \
         COUT edge timestamps, TA1IV demux, LPM0 wake{})",
        if require_pwm {
            ", PWM frequency + duty through the jumper"
        } else {
            "; PWM lines OK-or-SKIP"
        }
    );
    Ok(())
}
