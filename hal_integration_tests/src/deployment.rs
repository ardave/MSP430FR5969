//! Building and flashing of `hal_test_runners` MSP430 fixture binaries.
//!
//! Each peripheral's integration tests are driven by a dedicated on-device
//! binary under `hal_test_runners/src/bin/`. The helpers here cross-compile and
//! flash any of them by name, so every `*_tests.rs` module can stand its
//! fixture up with one call (typically `build_and_flash("<bin>")`) instead of
//! duplicating the cargo/DSLite plumbing.

use std::error::Error;
use std::path::{Path, PathBuf};

use xshell::{cmd, Shell};

/// Absolute path to the eZ-FET emulator's flashing tool.
const DSLITE: &str = "/Applications/ti/ccs2051/ccs/ccs_base/DebugServer/bin/DSLite";

/// DSLite target-configuration file, relative to the repo root.
const CCXML: &str = "MSP430FR5969.ccxml";

/// Where `cargo +nightly build` places the (debug) MSP430 ELFs, relative to the
/// repo root.
const TARGET_DIR: &str = "target/msp430-none-elf/debug";

/// Cross-compile the named `hal_test_runners` binary for the MSP430 and flash it to
/// the attached board. The common entry point for a test module's `run()`.
pub fn build_and_flash(bin: &str) -> Result<(), Box<dyn Error>> {
    build_and_flash_with_features(bin, &[])
}

/// Like [`build_and_flash`], but enables the given cargo `--features` on the
/// build. Lets one fixture source cover multiple compile-time configurations
/// (e.g. `serial_uart_test_runner` built at 9600 vs `baud_115200`) without a second binary.
pub fn build_and_flash_with_features(bin: &str, features: &[&str]) -> Result<(), Box<dyn Error>> {
    build(bin, features)?;
    flash(bin)?;
    Ok(())
}

/// Cross-compile the named `hal_test_runners` binary for `msp430-none-elf`, enabling
/// any requested cargo features.
pub fn build(bin: &str, features: &[&str]) -> Result<(), Box<dyn Error>> {
    println!("  building {bin} (msp430-none-elf){}...", feature_note(features));
    let sh = Shell::new()?;
    sh.change_dir(repo_root());
    let feature_args = feature_args(features);
    // This runner is itself launched by cargo, which exports env that would
    // otherwise leak into the child build: RUSTUP_TOOLCHAIN would override our
    // `+nightly`, and the target/target-dir vars would point the build at the
    // host triple / wrong output dir instead of the repo's msp430 config. Strip
    // them so the child build uses the repo-root .cargo config.
    cmd!(sh, "cargo +nightly build --bin {bin} {feature_args...}")
        .env_remove("RUSTUP_TOOLCHAIN")
        .env_remove("CARGO_BUILD_TARGET")
        .env_remove("CARGO_TARGET_DIR")
        .run()?; // non-zero exit -> Err automatically
    Ok(())
}

/// The `--features a,b` arguments for a build, or empty when none are requested.
fn feature_args(features: &[&str]) -> Vec<String> {
    if features.is_empty() {
        Vec::new()
    } else {
        vec!["--features".to_string(), features.join(",")]
    }
}

/// Human-readable " (features: a, b)" suffix for the build log line.
fn feature_note(features: &[&str]) -> String {
    if features.is_empty() {
        String::new()
    } else {
        format!(" (features: {})", features.join(", "))
    }
}

/// Flash an already-built `hal_test_runners` binary to the board via DSLite.
pub fn flash(bin: &str) -> Result<(), Box<dyn Error>> {
    let root = repo_root();
    let elf = root.join(TARGET_DIR).join(bin);
    let ccxml = root.join(CCXML);

    println!("  flashing {bin} to board...");
    let sh = Shell::new()?;
    cmd!(sh, "{DSLITE} load -c {ccxml} -f {elf}").run()?;
    Ok(())
}

/// Repo root = parent of this crate's manifest directory.
pub fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("hal_integration_tests should have a parent directory")
        .to_path_buf()
}
