//! Building and flashing of `single_board_test_firmwares` MSP430 fixture binaries.
//!
//! Each peripheral's integration tests are driven by a dedicated on-device
//! binary under `single_board_test_firmwares/src/bin/`. The helpers here cross-compile and
//! flash any of them by name, so every `*_tests.rs` module can stand its
//! fixture up with one call (typically `build_and_flash("<bin>")`) instead of
//! duplicating the cargo/DSLite plumbing.

use std::error::Error;
use std::path::{Path, PathBuf};

use xshell::{cmd, Shell};

/// Repo-root-relative path to the DSLite flashing wrapper. The script is the
/// one place that knows where DSLite lives (`$MSP430_DSLITE`, PATH, or known
/// CCS install locations) and where the ccxml target configuration is; cargo's
/// `runner` (.cargo/config.toml) goes through the same script.
const FLASH_SH: &str = "tools/flash.sh";

/// Where `cargo +nightly build` places the (debug) MSP430 ELFs, relative to the
/// repo root.
const TARGET_DIR: &str = "target/msp430-none-elf/debug";

/// Cross-compile the named `single_board_test_firmwares` binary for the MSP430 and flash it to
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

/// Cross-compile the named `single_board_test_firmwares` binary for `msp430-none-elf`, enabling
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

/// Flash an already-built `single_board_test_firmwares` binary to the board via DSLite.
pub fn flash(bin: &str) -> Result<(), Box<dyn Error>> {
    let root = repo_root();
    let elf = root.join(TARGET_DIR).join(bin);
    let flash_sh = root.join(FLASH_SH);

    let deployed = deployed_size(&elf)?;
    println!(
        "  flashing {bin} to board ({} deployed to FRAM, {deployed} bytes)...",
        human_size(deployed)
    );
    let sh = Shell::new()?;
    cmd!(sh, "{flash_sh} {elf}").run()?;
    Ok(())
}

/// The number of bytes DSLite will actually program into the chip: the sum of
/// the ELF's allocated, content-bearing section sizes (`SHF_ALLOC` set, type
/// not `SHT_NOBITS`) — `.text` + `.rodata` + `.data` + `.vector_table`.
///
/// The ELF's on-disk size is the wrong number to report — a debug build is
/// dominated by DWARF sections that never leave the host. Program headers are
/// wrong too, subtly: this linker emits the `.bss` segment with a nonzero
/// `p_filesz` even though `.bss` is `SHT_NOBITS`, so summing `PT_LOAD` file
/// sizes over-counts (observed: +148 B vs what DSLite reports programming).
/// Sections carry the honest answer, and it matches both `msp430-elf-size`'s
/// text+data and DSLite's "Flash/FRAM usage" line. The ELF32 header layout is
/// stable enough to read the three needed fields directly rather than pull in
/// an ELF-parsing dependency.
fn deployed_size(elf: &Path) -> Result<u64, Box<dyn Error>> {
    let bytes = std::fs::read(elf)?;
    let name = elf.display();

    // ELF ident: magic, then class/endianness. msp430-none-elf is ELF32 LE;
    // anything else here means we are looking at the wrong file.
    if bytes.len() < 52 || bytes[..4] != [0x7F, b'E', b'L', b'F'] {
        return Err(format!("{name} is not an ELF file").into());
    }
    if bytes[4] != 1 || bytes[5] != 1 {
        return Err(format!("{name} is not a 32-bit little-endian ELF").into());
    }

    let u16_at = |off: usize| u16::from_le_bytes([bytes[off], bytes[off + 1]]) as usize;
    let u32_at = |off: usize| {
        u32::from_le_bytes([bytes[off], bytes[off + 1], bytes[off + 2], bytes[off + 3]])
    };

    // Section-header table location, from the fixed ELF32 header offsets.
    let sh_off = u32_at(0x20) as usize;
    let sh_entsize = u16_at(0x2E);
    let sh_num = u16_at(0x30);
    if sh_entsize < 40 {
        return Err(format!("{name} has malformed section headers").into());
    }

    const SHT_NOBITS: u32 = 8; // occupies memory but no file/flash content (.bss)
    const SHF_ALLOC: u32 = 0x2; // occupies target memory at run time

    let mut total: u64 = 0;
    for i in 0..sh_num {
        let entry = sh_off + i * sh_entsize;
        if entry + 40 > bytes.len() {
            return Err(format!("{name} has a truncated section-header table").into());
        }
        let sh_type = u32_at(entry + 4);
        let sh_flags = u32_at(entry + 8);
        if sh_flags & SHF_ALLOC != 0 && sh_type != SHT_NOBITS {
            total += u32_at(entry + 20) as u64; // sh_size
        }
    }
    Ok(total)
}

/// Render a byte count for humans: `999 B`, `18.2 KB`, `1.4 MB` (1 KB = 1024 B,
/// the embedded convention — matches how the 48 KB FRAM region is described).
fn human_size(bytes: u64) -> String {
    const KB: f64 = 1024.0;
    let b = bytes as f64;
    if b < KB {
        format!("{bytes} B")
    } else if b < KB * KB {
        format!("{:.1} KB", b / KB)
    } else {
        format!("{:.1} MB", b / (KB * KB))
    }
}

/// Repo root = parent of this crate's manifest directory.
pub fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("single_board_test_orchestrators should have a parent directory")
        .to_path_buf()
}
