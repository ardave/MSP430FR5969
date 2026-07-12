//! Building the two-board fixture and flashing it to a *specific* board.
//!
//! Flashing one board out of two attached probes is the one thing the
//! single-board tooling can't do: `DSLite load` has no probe-selection flag,
//! so selection has to live in the ccxml. TI's MSP430-USB connection carries
//! a `portAddr1` property (the value mspdebug's `tilib -d` also feeds to
//! MSP430.DLL), which accepts the probe's CDC device node — the
//! `/dev/cu.usbmodem*1` sibling of the board's `*3` backchannel. We generate
//! one ccxml per board under `target/two_board/` with that property pinned
//! and hand it to `tools/flash.sh` (which grew an optional ccxml argument for
//! exactly this).
//!
//! If your DSLite build ignores `portAddr1` (it would flash whichever probe
//! enumerates first — same binary either way, but only one board would get
//! reflashed), fall back to flashing with a single board attached at a time:
//! `cargo +nightly run -- flash` with one USB cable in, then the other.

use std::error::Error;
use std::path::{Path, PathBuf};

use xshell::{cmd, Shell};

/// The one fixture binary, flashed identically to both boards (role lives in
/// each board's Info FRAM, not in the image).
pub const FIXTURE_BIN: &str = "two_board_fixture";

const FLASH_SH: &str = "tools/flash.sh";
const TARGET_DIR: &str = "target/msp430-none-elf/debug";

/// Cross-compile the fixture for msp430-none-elf.
pub fn build() -> Result<(), Box<dyn Error>> {
    println!("  building {FIXTURE_BIN} (msp430-none-elf)...");
    let sh = Shell::new()?;
    sh.change_dir(repo_root());
    // This runner is itself launched by cargo, which exports env that would
    // otherwise leak into the child build (see hal_integration_tests).
    cmd!(sh, "cargo +nightly build --bin {FIXTURE_BIN}")
        .env_remove("RUSTUP_TOOLCHAIN")
        .env_remove("CARGO_BUILD_TARGET")
        .env_remove("CARGO_TARGET_DIR")
        .run()?;
    Ok(())
}

/// Flash the already-built fixture to the probe whose eZ-FET debug interface
/// is at `debug_port` (e.g. `/dev/cu.usbmodem11201`).
pub fn flash_to(debug_port: &str) -> Result<(), Box<dyn Error>> {
    let root = repo_root();
    let elf = root.join(TARGET_DIR).join(FIXTURE_BIN);
    let ccxml = write_ccxml(debug_port)?;
    let flash_sh = root.join(FLASH_SH);
    println!("  flashing {FIXTURE_BIN} via probe {debug_port}...");
    let sh = Shell::new()?;
    cmd!(sh, "{flash_sh} {elf} {ccxml}").run()?;
    Ok(())
}

/// Flash via the repo-default ccxml (no probe pinned) — for `provision` and
/// the single-board-attached fallback, where only one probe exists.
pub fn flash_sole_board() -> Result<(), Box<dyn Error>> {
    let root = repo_root();
    let elf = root.join(TARGET_DIR).join(FIXTURE_BIN);
    let flash_sh = root.join(FLASH_SH);
    println!("  flashing {FIXTURE_BIN} to the attached board...");
    let sh = Shell::new()?;
    cmd!(sh, "{flash_sh} {elf}").run()?;
    Ok(())
}

/// Emit a ccxml pinned to one probe: the repo's MSP430FR5969.ccxml with the
/// MSP430-USB connection's `portAddr1` property overridden to the probe's
/// CDC device node.
fn write_ccxml(debug_port: &str) -> Result<PathBuf, Box<dyn Error>> {
    let dir = repo_root().join("target/two_board");
    std::fs::create_dir_all(&dir)?;
    let name = debug_port.rsplit('/').next().unwrap_or("probe");
    let path = dir.join(format!("{name}.ccxml"));
    let contents = format!(
        r#"<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<configurations XML_version="1.2" id="configurations_0">
<configuration XML_version="1.2" id="TI MSP430 USB1_0">
        <instance XML_version="1.2" desc="TI MSP430 USB1_0" href="connections/TIMSP430-USB.xml" id="TI MSP430 USB1_0" xml="TIMSP430-USB.xml" xmlpath="connections"/>
        <connection XML_version="1.2" id="TI MSP430 USB1_0">
            <instance XML_version="1.2" href="drivers/msp430_emu.xml" id="drivers" xml="msp430_emu.xml" xmlpath="drivers"/>
            <property Type="hiddenfield" Value="{debug_port}" id="portAddr1"/>
            <platform XML_version="1.2" id="platform_0">
                <instance XML_version="1.2" desc="MSP430FR5969_0" href="devices/MSP430FR5969.xml" id="MSP430FR5969_0" xml="MSP430FR5969.xml" xmlpath="devices"/>
            </platform>
        </connection>
    </configuration>
</configurations>
"#
    );
    std::fs::write(&path, contents)?;
    Ok(path)
}

/// Repo root = parent of this crate's manifest directory.
pub fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("two_board_integration_tests should have a parent directory")
        .to_path_buf()
}
