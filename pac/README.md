# msp430fr5969

Peripheral access crate (PAC) for the Texas Instruments
[MSP430FR5969](https://www.ti.com/product/MSP430FR5969) microcontroller,
generated from its SVD file with [svd2rust](https://crates.io/crates/svd2rust).

## Features

- `rt` — pulls in [msp430-rt](https://crates.io/crates/msp430-rt) (reset
  handler, RAM init, vector table, `#[entry]`/`#[interrupt]` macros) and
  provides `memory.x` (MSP430FR5969 memory map) and `device.x` (interrupt
  vector defaults) to the linker.
- `critical-section` — gates `Peripherals::take()` on a
  [critical-section](https://crates.io/crates/critical-section)
  implementation, e.g. the `critical-section-single-core` feature of the
  [msp430](https://crates.io/crates/msp430) crate.

## Deviations from generated code

`src/lib.rs` is `svd2rust --target msp430` output (v0.37.1), run through
`rustfmt`, with one hand-applied patch: the unstable `abi_msp430_interrupt`
feature and the vector-table `Vector` union are gated on the `rt` feature,
so the non-`rt` build does not require it. (The msp430 target flavor already
emits the 16-bit vector slots and the `extern "msp430-interrupt"` handler
ABI correctly at the source.)

`memory.x`, `device.x`, and `build.rs` are hand-written, not generated.

## Usage

Build for the `msp430-none-elf` target, which requires the nightly toolchain
and `-Z build-std=core`. See the
[repository](https://github.com/ardave/MSP430FR5969) for a working firmware
setup (HAL, linker configuration, and hardware test fixtures) built on this
crate.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.
