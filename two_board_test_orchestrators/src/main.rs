//! Host-side runner for the permanent two-LaunchPad integration rig.
//!
//! Two MSP-EXP430FR5969 LaunchPads, wired together once (the exact hookup is
//! printed at the start of every run — see [`wiring`]), each on its own USB
//! cable. One shared fixture binary (`two_board_test_firmwares`, bin
//! `two_board_fixture`) is flashed to both; which board is "parent" and which
//! is "child" lives in each board's Info FRAM, so the host re-discovers the
//! mapping every run by asking, and nothing breaks when USB paths move.
//!
//! ```text
//! cd two_board_test_orchestrators
//! cargo +nightly run -- wiring               # print the hookup table only
//! cargo +nightly run -- provision parent     # ONE board attached: brand it
//! cargo +nightly run -- provision child      # the other board: brand it
//! cargo +nightly run                          # flash both + all suites
//! cargo +nightly run -- --no-flash i2c_bridge # just one suite, no reflash
//! cargo +nightly run -- flash                # fallback: flash the sole
//!                                             # attached board (repeat per board)
//! ```
//!
//! Default suites: `identity`, `i2c_bridge`, `uart_link`, `gpio_edge`,
//! `lpm4_wake`, `pwm_cross`. Name-only (like the single-board runner's `spi`
//! and `capture_jumper`): `adc_dac`, which needs the W10/W11 RC capacitors
//! fitted first — see the wiring banner's future-addition note.

use std::error::Error;
use std::time::Duration;

mod boards;
mod deployment;
mod serial;
mod suites;
mod wiring;

use boards::Rig;

type Suite = fn(&mut Rig) -> Result<(), Box<dyn Error>>;

const SUITES: [(&str, Suite); 6] = [
    ("identity", suites::identity),
    ("i2c_bridge", suites::i2c_bridge),
    ("uart_link", suites::uart_link),
    ("gpio_edge", suites::gpio_edge),
    ("lpm4_wake", suites::lpm4_wake),
    ("pwm_cross", suites::pwm_cross),
];

/// Suites that run ONLY when named explicitly: `adc_dac` requires the
/// optional 10 µF caps on W10/W11 (a rig built to the base parts list has
/// none, and without them the ADC sees a raw square wave — a guaranteed,
/// meaningless FAIL).
const NAMED_ONLY_SUITES: [(&str, Suite); 1] = [("adc_dac", suites::adc_dac)];

fn main() -> Result<(), Box<dyn Error>> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();

    // The hookup specification leads every invocation, so any log of a test
    // run doubles as the instructions for building the rig.
    wiring::print_banner();

    let no_flash = args.iter().any(|a| a == "--no-flash");
    args.retain(|a| a != "--no-flash");

    match args.first().map(String::as_str) {
        Some("wiring") => Ok(()),
        Some("provision") => provision(args.get(1).map(String::as_str)),
        Some("flash") => {
            deployment::build()?;
            deployment::flash_sole_board()
        }
        Some("identify") => {
            for path in serial::candidate_ports()? {
                match boards::identify(&path, Duration::from_secs(15)) {
                    Ok((Some(role), info)) => println!("  {path}: {} ({info})", role.as_str()),
                    Ok((None, info)) => println!("  {path}: unprovisioned ({info})"),
                    Err(e) => println!("  {path}: {e}"),
                }
            }
            Ok(())
        }
        _ => run_suites(&args, no_flash),
    }
}

/// Brand the SOLE attached board as parent or child (Info FRAM, offset 0xA0).
/// One board at a time keeps the physical identification unambiguous: the
/// board on the desk in front of you is the one being named.
fn provision(role: Option<&str>) -> Result<(), Box<dyn Error>> {
    let (cmd, want) = match role {
        Some("parent") => (b'P', "2B_PROVISIONED role=parent"),
        Some("child") => (b'C', "2B_PROVISIONED role=child"),
        _ => return Err("usage: provision parent|child (with exactly one board attached)".into()),
    };

    let candidates = serial::candidate_ports()?;
    let [path] = candidates.as_slice() else {
        return Err(format!(
            "provisioning needs exactly ONE board attached (found {candidates:?}) — \
             that is how you and the tooling agree on which physical board gets the name"
        )
        .into());
    };
    let path = path.clone();

    deployment::build()?;
    deployment::flash_sole_board()?;

    // Wait out the post-flash reboot, then brand it.
    let (_, info) = boards::identify(&path, Duration::from_secs(15))?;
    println!("  {path}: fixture up ({info}), provisioning...");
    let mut port = serial::open(&path)?;
    port.clear(serialport::ClearBuffer::Input)?;
    port.write_all(&[cmd])?;
    port.flush()?;
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        let line = serial::read_line(port.as_mut(), deadline)?;
        if line.starts_with(want) {
            println!("  [board] {line}");
            println!("Provisioned. Repeat with the other board, then attach both and run the suites.");
            return Ok(());
        }
        if !line.is_empty() {
            println!("  [board] {line}");
        }
    }
}

/// The default path: build once, flash both boards (each via a ccxml pinned
/// to its own probe), identify who is who, run the requested suites.
fn run_suites(only: &[String], no_flash: bool) -> Result<(), Box<dyn Error>> {
    let wanted =
        |name: &str| only.is_empty() || only.iter().any(|o| o == name);
    for name in only {
        if !SUITES.iter().any(|(n, _)| n == name)
            && !NAMED_ONLY_SUITES.iter().any(|(n, _)| n == name)
        {
            return Err(format!(
                "unknown suite {name:?}; available: {:?}, name-only: {:?}",
                SUITES.map(|(n, _)| n),
                NAMED_ONLY_SUITES.map(|(n, _)| n)
            )
            .into());
        }
    }

    let ports = serial::candidate_ports()?;
    if !no_flash {
        if ports.len() != 2 {
            return Err(format!(
                "need exactly two boards attached to flash the rig, found {ports:?} \
                 (or flash one at a time with `flash`, then rerun with --no-flash)"
            )
            .into());
        }
        deployment::build()?;
        // Flash USB FET #1 and #2 (TI's enumeration-index addressing). Which
        // index is which physical board doesn't matter — identical binary —
        // and the identity suite cross-checks firmware revisions afterwards.
        for fet_index in 1..=ports.len() {
            deployment::flash_to(fet_index)?;
        }
    }

    println!("== discovering boards ==");
    let mut rig = boards::discover(Duration::from_secs(20))?;
    println!(
        "  parent = {}, child = {}\n",
        rig.parent.path, rig.child.path
    );

    let mut ran = 0;
    for (name, run) in SUITES {
        if wanted(name) {
            run(&mut rig).map_err(|e| format!("suite {name} failed: {e}"))?;
            ran += 1;
        }
    }
    // Name-only suites never run implicitly — `wanted` is true for
    // everything when no suites were named, so gate on an explicit mention.
    for (name, run) in NAMED_ONLY_SUITES {
        if !only.is_empty() && wanted(name) {
            run(&mut rig).map_err(|e| format!("suite {name} failed: {e}"))?;
            ran += 1;
        }
    }
    println!("\nAll {ran} suite(s) passed.");
    Ok(())
}
