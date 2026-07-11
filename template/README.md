# MSP430FR5969 starter project

A minimal, self-contained firmware project for the TI MSP430FR5969 LaunchPad
(MSP-EXP430FR5969), using [`msp430fr5969-hal`](https://crates.io/crates/msp430fr5969-hal)
from crates.io. It blinks the green LED. Copy this directory anywhere, rename
the package in `Cargo.toml`, and build from it.

> **Copy it out first.** Building in place inside the HAL repository does not
> work: cargo merges `.cargo/config.toml` files from all parent directories,
> so the repository's link flags get concatenated with this project's and the
> link fails with `linker script file 'link.x' appears multiple times`.

## Prerequisites

1. **Rust nightly + rust-src** — installed automatically by rustup on first
   build, via `rust-toolchain.toml`. (Nightly is needed for `-Z build-std`:
   `msp430-none-elf` is a tier-3 target with no prebuilt `core`.)

2. **TI MSP430-GCC** — the linker plus libgcc and the hardware-multiply
   runtime library. Free download, no login:
   <https://www.ti.com/tool/MSP430-GCC-OPENSOURCE>

   Unpack it and either add its `bin/` directory to `PATH`, or set

   ```bash
   export CARGO_TARGET_MSP430_NONE_ELF_LINKER=/path/to/msp430-gcc-9.3.1.11/bin/msp430-elf-gcc
   ```

3. **A flashing tool** (any one of):
   - **DSLite** — ships with [Code Composer Studio](https://www.ti.com/tool/CCSTUDIO)
     (also inside CCS's standalone "UniFlash"). Works with the LaunchPad's
     on-board eZ-FET probe.
   - **mspdebug** with TI's `libmsp430` (`mspdebug tilib "prog firmware.elf"`).
   - **CCS / UniFlash GUI** — point it at the built ELF.

## Build

```bash
cargo build            # debug — size-tuned to fit the 48 KB FRAM
cargo build --release
```

The ELF lands at `target/msp430-none-elf/debug/msp430fr5969-app`.

## Flash

With DSLite (adjust paths; the `.ccxml` target-configuration file is created
by CCS, or copy `MSP430FR5969.ccxml` from the HAL repository):

```bash
DSLite load -c MSP430FR5969.ccxml -f target/msp430-none-elf/debug/msp430fr5969-app
```

## Where to go next

- Examples for UART, ADC/temperature, PWM, and low-power button wake:
  <https://github.com/ardave/MSP430FR5969/tree/main/hal/examples>
- HAL API docs: <https://docs.rs/msp430fr5969-hal>
- The LaunchPad's UART backchannel is eUSCI_A0 at 9600 8N1 (e.g.
  `screen /dev/cu.usbmodem<N> 9600` on macOS).

## Notes worth keeping

- The dev profile disables overflow checks and unwinding and optimizes for
  size — an unoptimized build drags the ~30 KB `core::fmt` engine into the
  48 KB FRAM. For the same reason, format numbers by hand (see the HAL
  examples) instead of using `core::fmt`.
- Adding interrupt handlers needs two nightly features in `main.rs`
  (`#![feature(abi_msp430_interrupt)]`, plus `asm_experimental_arch` for
  `wake_cpu` handlers) — see the `button_wake` example in the HAL repo.
