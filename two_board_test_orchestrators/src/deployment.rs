//! Building the two-board fixture and flashing it to a *specific* board.
//!
//! Flashing one board out of two attached probes is the one thing the
//! single-board tooling can't do: `DSLite load` has no probe-selection flag,
//! so selection has to live in the ccxml. TI's MSP430-USB connection carries
//! a `portAddr1` property encoding **which USB FET by enumeration index**:
//! `100 + N`, 1-based — TI ships one connection file per slot
//! (`TIMSP430-USB.xml` = 101, `TIMSP430-USB2.xml` = 102, `TIMSP430-USB3.xml`
//! = 103), and libmsp430_emu's own error text ("Tried to initialize USB FET
//! number %u, but only found %d USB FETs") confirms the semantics. It is NOT
//! a device path — feeding it one parses as a garbage index and produces
//! exactly that error. We generate one ccxml per FET slot under
//! `target/two_board/` and hand it to `tools/flash.sh` (which grew an
//! optional ccxml argument for exactly this).
//!
//! Which enumeration index is which physical board doesn't matter: both
//! boards get the identical binary (role lives in Info FRAM), and the
//! `identity` suite verifies both ends answered with the same firmware
//! revision afterwards. Fallback if index selection ever misbehaves: flash
//! with a single board attached at a time (`cargo +nightly run -- flash`).

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
    // otherwise leak into the child build (see single_board_test_orchestrators).
    cmd!(sh, "cargo +nightly build --bin {FIXTURE_BIN}")
        .env_remove("RUSTUP_TOOLCHAIN")
        .env_remove("CARGO_BUILD_TARGET")
        .env_remove("CARGO_TARGET_DIR")
        .run()?;
    Ok(())
}

/// Flash the already-built fixture to the `fet_index`-th (1-based, in USB
/// enumeration order) of the attached eZ-FET probes.
pub fn flash_to(fet_index: usize) -> Result<(), Box<dyn Error>> {
    let root = repo_root();
    let elf = root.join(TARGET_DIR).join(FIXTURE_BIN);
    let ccxml = write_ccxml(fet_index)?;
    let flash_sh = root.join(FLASH_SH);
    println!("  flashing {FIXTURE_BIN} via USB FET #{fet_index}...");
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

/// Emit a ccxml pinned to one USB FET slot: the repo's MSP430FR5969.ccxml
/// reshaped around TI's per-slot connection variant (`TIMSP430-USBn.xml`)
/// with the matching `portAddr1 = 100 + n` property. TI only ships slots
/// 1–3, which bounds how many probes this can address.
fn write_ccxml(fet_index: usize) -> Result<PathBuf, Box<dyn Error>> {
    if !(1..=3).contains(&fet_index) {
        return Err(format!(
            "USB FET index {fet_index} out of range: TI's MSP430 connection \
             variants address FETs 1-3 only"
        )
        .into());
    }
    let dir = repo_root().join("target/two_board");
    std::fs::create_dir_all(&dir)?;
    let path = dir.join(format!("usb-fet-{fet_index}.ccxml"));
    // Slot 1's connection file has no digit suffix (TIMSP430-USB.xml).
    let suffix = if fet_index == 1 {
        String::new()
    } else {
        fet_index.to_string()
    };
    let port_addr = 100 + fet_index;
    let contents = format!(
        r#"<?xml version="1.0" encoding="UTF-8" standalone="no"?>
<configurations XML_version="1.2" id="configurations_0">
<configuration XML_version="1.2" id="TI MSP430 USB{fet_index}_0">
        <instance XML_version="1.2" desc="TI MSP430 USB{fet_index}_0" href="connections/TIMSP430-USB{suffix}.xml" id="TI MSP430 USB{fet_index}_0" xml="TIMSP430-USB{suffix}.xml" xmlpath="connections"/>
        <connection XML_version="1.2" id="TI MSP430 USB{fet_index}_0">
            <instance XML_version="1.2" href="drivers/msp430_emu.xml" id="drivers" xml="msp430_emu.xml" xmlpath="drivers"/>
            <property Type="hiddenfield" Value="{port_addr}" id="portAddr1"/>
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
        .expect("two_board_test_orchestrators should have a parent directory")
        .to_path_buf()
}
