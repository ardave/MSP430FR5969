//! Building and flashing of `hal_consumer` MSP430 fixture binaries.
//!
//! Each peripheral's integration tests are driven by a dedicated on-device
//! binary under `hal_consumer/src/bin/`. The helpers here cross-compile and
//! flash any of them by name, so every `*_tests.rs` module can stand its
//! fixture up with one call (typically `build_and_flash("<bin>")`) instead of
//! duplicating the cargo/DSLite plumbing.

use std::error::Error;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Absolute path to the eZ-FET emulator's flashing tool.
const DSLITE: &str = "/Applications/ti/ccs2051/ccs/ccs_base/DebugServer/bin/DSLite";

/// DSLite target-configuration file, relative to the repo root.
const CCXML: &str = "MSP430FR5969.ccxml";

/// Where `cargo +nightly build` places the (debug) MSP430 ELFs, relative to the
/// repo root.
const TARGET_DIR: &str = "target/msp430-none-elf/debug";

/// Cross-compile the named `hal_consumer` binary for the MSP430 and flash it to
/// the attached board. The common entry point for a test module's `run()`.
pub fn build_and_flash(bin: &str) -> Result<(), Box<dyn Error>> {
    build(bin)?;
    flash(bin)?;
    Ok(())
}

/// Cross-compile the named `hal_consumer` binary for `msp430-none-elf`.
pub fn build(bin: &str) -> Result<(), Box<dyn Error>> {
    println!("  building {bin} (msp430-none-elf)...");
    let status = Command::new("cargo")
        .args(["+nightly", "build", "--bin", bin])
        .current_dir(repo_root())
        // This runner is itself launched by cargo, which exports env that would
        // otherwise leak into the child build: RUSTUP_TOOLCHAIN would override
        // our `+nightly`, and the target/target-dir vars would point the build
        // at the host triple / wrong output dir instead of the repo's msp430
        // config. Strip them so the child build uses the repo-root .cargo config.
        .env_remove("RUSTUP_TOOLCHAIN")
        .env_remove("CARGO_BUILD_TARGET")
        .env_remove("CARGO_TARGET_DIR")
        .status()?;
    if !status.success() {
        return Err(format!("cargo build of {bin} failed").into());
    }
    Ok(())
}

/// Flash an already-built `hal_consumer` binary to the board via DSLite.
pub fn flash(bin: &str) -> Result<(), Box<dyn Error>> {
    let root = repo_root();
    let elf = root.join(TARGET_DIR).join(bin);
    let ccxml = root.join(CCXML);

    println!("  flashing {bin} to board...");
    let status = Command::new(DSLITE)
        .arg("load")
        .arg("-c")
        .arg(&ccxml)
        .arg("-f")
        .arg(&elf)
        .status()?;
    if !status.success() {
        return Err(format!("DSLite flash of {bin} failed").into());
    }
    Ok(())
}

/// Repo root = parent of this crate's manifest directory.
pub fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("hal_integration_tests should have a parent directory")
        .to_path_buf()
}
