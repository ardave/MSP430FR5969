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

`src/lib.rs` is svd2rust output with two hand-applied patches (both would be
lost on regeneration; the msp430-flavored svd2rust fork produces both
correctly at the source):

1. **Vector-table union width.** svd2rust emitted the vector-table `Vector`
   union with a `_reserved: u32` variant — 4 bytes, wrong for a 16-bit
   target. The doubled table width overran the `VECTORS` memory region once
   msp430-rt `KEEP`s and places the table. Patched to `u16`.
2. **Interrupt handler ABI.** The interrupt handler declarations (the
   `extern` block and `Vector::_handler`) were emitted `extern "C"`; they are
   patched to `extern "msp430-interrupt"` (handlers return with `RETI`, not
   `RET`), gated on the `rt` feature so the non-`rt` build does not need the
   unstable `abi_msp430_interrupt` feature.

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
