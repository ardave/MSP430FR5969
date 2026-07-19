# msp430fr5969-hal

Hardware abstraction layer for the Texas Instruments
[MSP430FR5969](https://www.ti.com/product/MSP430FR5969) microcontroller,
built on the [msp430fr5969](https://crates.io/crates/msp430fr5969) peripheral
access crate. Implements the [embedded-hal](https://crates.io/crates/embedded-hal)
1.0 traits (plus `embedded-hal-nb`, `embedded-io`, and `embedded-storage`)
where they exist, with native APIs where they don't (RTC, watchdog, DMA,
comparator, MPU).

Every driver has been exercised on real hardware (the MSP-EXP430FR5969
LaunchPad), including several silicon behaviors that documentation does not
cover — e.g. the ADC12→DMA trigger latch erratum and the edge-sensitive DMA
arming order — which are documented in the relevant module docs.

## Peripheral coverage

| Module | Peripheral |
|---|---|
| `gpio` | Typed pins, digital I/O traits, port edge interrupts, LPMx.5 pin-state unlock |
| `serial` | eUSCI_A UART (polling, RX interrupt, DMA-paced TX/RX) |
| `spi` | eUSCI_A0/A1/B0 SPI master (CPU-paced and DMA-paced `SpiBus`) |
| `i2c` | eUSCI_B0 I2C master + slave (slave: code-complete, hardware verification pending) |
| `adc` | ADC12_B: ratiometric + VREF-referenced reads, interrupts, DMA-drained repeat mode |
| `dma` | 3-channel DMA: block copies, peripheral pacing primitives, `DMAIV` demux |
| `aes` | AES256 accelerator (ECB primitive, software CBC chaining) |
| `crc` | CRC16 accelerator (both bit conventions + catalog one-shots) |
| `comp_e` | Comp_E analog comparator with reference-ladder hysteresis |
| `ref_a` / `tlv` | Shared voltage reference; factory calibration constants |
| `pwm` | Timer_B0 PWM (`SetDutyCycle`) |
| `timer` / `delay` | Timer_A free-running counter, tick↔time math, blocking delays |
| `rtc` | RTC_B calendar, incl. reattachment after LPM3.5 wake |
| `clocks` / `power` | Clock tree; LPM0/3/4 entry and the regulator-off LPM3.5/4.5 modes |
| `fram` | FRAM as `embedded-storage` |
| `mpu` | FRAM memory protection unit (segment borders, NMI/PUC violation policy) |
| `watchdog` / `sys` | WDT_A (watchdog + interval timer), reset-reason and NMI demux |

## Getting started

The boot front door is `hal::peripherals::take`, which stops (or arms) the watchdog and
takes the PAC peripherals in a guaranteed-safe order:

```rust,ignore
#![no_std]
#![no_main]

use msp430_rt::entry;
use msp430fr5969_hal as hal;

#[entry]
fn main() -> ! {
    let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();
    // configure clocks, split GPIO ports, bring up drivers...
    loop {}
}
```

## Features

- `rt` — passes through to the PAC's `rt` feature: msp430-rt runtime, vector
  table, `#[entry]`/`#[interrupt]` macros, and the linker scripts.
- `critical-section` — interrupt-enable registers shared between drivers are
  RMW'd inside `critical_section::with`; pair it with an implementation such
  as the `msp430` crate's `critical-section-single-core` feature.

## Building

Targets `msp430-none-elf` (tier 3), which requires the nightly toolchain and
`-Z build-std=core`; linking a flashable binary additionally needs the TI
MSP430-GCC toolchain. See the
[repository](https://github.com/ardave/MSP430FR5969) for a complete working
setup, hardware-in-the-loop test fixtures, and example binaries for every
driver.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.
