# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Bare-metal Rust firmware for the **Texas Instruments MSP430FR5969** microcontroller dev board. No RTOS, no `std` — this is `#![no_std]` / `#![no_main]` embedded development.

## Build Commands

```bash
# Build (requires nightly toolchain for build-std)
cargo +nightly build

# Flash via DSLite (TI debug server, runs under Rosetta on M-series Macs)
/Applications/ti/ccs2051/ccs/ccs_base/DebugServer/bin/DSLite load \
  -c /Users/davidfalkner/git/MSP430FR5969/MSP430FR5969.ccxml \
  -f /Users/davidfalkner/git/MSP430FR5969/target/msp430-none-elf/debug/pac_consumer

# Flash and run via mspdebug (not currently working — mspdebug is arm64, TI's libmsp430 is x86_64)
# cargo +nightly run

# Check without building
cargo +nightly check

# Run the host-side math tests (NOT on the msp430 target)
cd unit_tests && cargo +nightly test
```

Requires the **nightly** Rust toolchain (for `-Z build-std=core` on the `msp430-none-elf` target). The TI MSP430 GCC toolchain must be installed — the linker path is hardcoded in `.cargo/config.toml`.

## Testing

Most firmware can only be validated on hardware, but pure logic is tested on the host. The pattern: keep the pure arithmetic in a dependency-free file (no PAC/HAL types, `//` not `//!` comments so it can be `include!`d mid-crate), and have the **detached `unit_tests/` crate** (its own `[workspace]` table + `.cargo/config.toml` overriding the target back to the host triple `aarch64-apple-darwin`, with `build-std = ["std"]` to neutralize the inherited `build-std = ["core"]` — and, being outside the workspace, escaping the `panic = "abort"` dev profile that would break libtest) `include!` that real source, one module per subject. Because the tests include the actual shipping source, a bad edit to the math fails them. Current modules:

- `baud` includes `hal/src/baud.rs` and checks `compute_baud`/`ucbrs_lookup` against SLAU367P Table 30-5. The pass/fail criterion is the resulting average bit-timing error (< 2%), since the driver follows the datasheet *procedure* while Table 30-5 lists values from a separate lowest-error search — the two legitimately differ in some rows (mode choice near N≈16, and alternate `UCBRSx` bytes).
- `ticks` includes `hal/src/ticks.rs` (the tick↔time math `Counter` delegates to: `ticks_to_us`/`ticks_to_ns`/`assemble_now32`) and checks exact conversions, the `u64`-widening overflow guard, and the `now32` bit-packing.
- `fram_addr` includes `hal/src/fram_addr.rs` and pins the FRAM region geometry (Info 0x1800/512 B, upper 0x10000/16 KB) plus the overflow-safe `check_bounds`.
- `adc_cal` includes `hal/src/adc_cal.rs` (the math `tlv::AdcCal`/`tlv::RefCal`/`Adc::to_millivolts` delegate to) and checks the SLAU367 temperature interpolation (30/85 °C TLV points, deci-°C, half-away-from-zero rounding), the 1.15 fixed-point gain/offset/REF corrections (unity identity, clamping), and count→mV rounding.

Run with `cd unit_tests && cargo +nightly test` (must be run from inside the directory so its `.cargo/config.toml` takes precedence). The host triple there is `aarch64-apple-darwin`; change it for other hosts. New pure-math files get a new module here, not a new crate.

## Hardware test setups (consumer demos)

The `hal_test_runners` demos that exercise eUSCI_B0 each need specific board wiring. **eUSCI_B0 is *either* SPI or I2C** (one register block at 0x0640, shared P1.6/P1.7 pins), so the SPI and I2C demos are **separate binaries** — only one can be flashed at a time. Observe all of them over the UART backchannel: eUSCI_A0 on `/dev/cu.usbmodem11203` @ **9600 8N1** (e.g. `screen /dev/cu.usbmodem11203 9600`; the eZ-FET gates TX on DTR, which `screen` asserts).

- **SPI loopback (`--bin hal_test_runners`, `src/main.rs`):** install a **jumper wire from P1.6 (SIMO) to P1.7 (SOMI)**. The demo `transfer_in_place`s a 6-byte pattern; with the jumper every byte round-trips → solid **GREEN** LED + UART `PASS`. Without it SOMI floats → solid **RED** + `FAIL` (the transfer still completes, proving it doesn't hang). This is the self-contained SPI test — no external device needed.

- **I2C bus scan (`--bin i2c_scan`, `src/bin/i2c_scan.rs`):** P1.6 = **SDA**, P1.7 = **SCL**. **Remove the SPI loopback jumper first** — on P1.6/P1.7 it shorts SDA directly to SCL and no I2C transfer can work. I2C is open-drain, so add **~4.7 kΩ pull-ups** from SDA and SCL to 3V3 (many breakout boards include their own; a BME280 board typically does). The scanner probes 0x08..=0x77 with zero-length writes and reports ACKing addresses; GREEN if any device answers, RED if the bus is empty. **Without pull-ups SCL can't be released high and the (current) driver will spin forever** — no output and no LED change means suspect pull-ups/wiring before code.

- **BME280 I2C validation:** wire VCC→3V3, GND→GND, SDA→P1.6, SCL→P1.7 (jumper removed, pull-ups present). The BME280 answers at **0x76** (SDO low) or **0x77** (SDO high) — the scanner catches either. Beyond the address probe, the chip-ID register is the natural `write_read` correctness check: `i2c.write_read(addr, &[0xD0], &mut id)` should return **0x60** (BME280's fixed ID), exercising the write→repeated-START→read path that the scanner alone does not.

The remaining demos use **different peripherals than eUSCI_B0**, so they need no bus wiring and do not conflict with the SPI/I2C demos (they can be flashed independently):

- **Timer_B0 PWM (`--bin pwm_fade`, `src/bin/pwm_fade.rs`):** drives ~1 kHz PWM on **TB0.1 = P1.4** and ramps duty 0→100→0 %. Put an **LED + ~330 Ω from P1.4 to GND** (or a scope on P1.4) to see it breathe; the UART prints `duty: N%` and the on-board LEDs show ramp direction (GREEN up, RED down). The driver picks `TB0.n` outputs with **`SEL0=1, SEL1=0`** (secondary function) via `gpio::Pin::into_timer_b_output`; only P1.4(TB0.1)/P1.5(TB0.2)/P1.6(TB0.3)/P1.7(TB0.4) implement `pwm::PwmPin`. **TB0.0 is the period (`TB0CCR0`), not a usable output**; P1.6/P1.7 collide with eUSCI_B0, so the typestate makes PWM-vs-SPI/I2C on those pins an exclusive choice. Clean rails: 0 % parks the pin low and 100 % high via `OUTMOD=0` (no glitch), everything between uses `OUTMOD=7` Reset/Set.

- **RTC_B calendar (`--bin rtc_clock`, `src/bin/rtc_clock.rs`):** prints `YYYY-MM-DD HH:MM:SS` once per second and blinks GREEN. **Requires the 32.768 kHz LFXT crystal** — the RTC "is clocked by XT1" (datasheet), so the demo uses `clocks::configure_low_power` to start LFXT, and `rtc::Rtc::new` **returns `Error::ClockNot32768` unless `clocks.aclk() == 32768`** (lights RED + prints a refusal). The LaunchPad populates the crystal, so it should run crystal-accurate (check for drift against a watch over a minute). Reads go through `now()`, which gates on **`RTCRDY`** to avoid a torn read across the 1 Hz update. Binary mode (`RTCBCD=0`) so `DateTime` fields are plain integers; the single `RTC` interrupt vector (sources demuxed via `RTCIV`, see `rtc::read_iv`) is available via `enable_event_interrupt`/`enable_second_interrupt` but the demo polls.

- **GPIO port interrupts (`--bin gpio_irq_test_runner` / `--bin button_wake`):** **no wiring** — the LaunchPad buttons are on-board (S2 = P1.1, S1 = P4.5, both short to GND with no external resistor → internal pull-up, press = falling edge). The fixture is hands-free: `PxIFG` is software-settable and a software-set flag traverses the same latch → vector → `PxIV` path as a real edge, so it "presses" both buttons from software and checks the IV demux value (P1.1 → 0x04, P4.5 → 0x0C), the exactly-once firing, and the PxIV auto-clear on PORT1 *and* PORT4; the buttons stay armed, so real presses bump the counts in the `gpio p1=… p4=…` info line. `button_wake` is the human demo: main parks in **LPM4** (`power::enter_lpm4`) and S2/S1 presses — the only thing that can wake a part with every clock stopped — toggle GREEN/RED and print a line per press; it `flush()`es the UART before sleeping because LPM4 stops SMCLK mid-character otherwise.

- **REF_A calibrated temperature & supply (`--bin ref_temp_test_runner`):** **no wiring at all.** Brings the shared reference up at **2.0 V** (one setting serves both measurements: the temp sensor fits under any reference, but AVCC/2 ≈ 1.65 V would clip against 1.2 V) and prints the die temperature in °C — interpolated between the factory 30/85 °C TLV points — plus AVCC in mV via the (AVCC–AVSS)/2 monitor through the full gain→offset→REF-factor calibration chain. Expect a few °C above ambient (die self-heating) and **≈3630 mV** on USB power — the eZ-FET LDO feeds this LaunchPad ~3.6 V, *not* 3.3 V (HW-measured 2026-07-03, confirmed by the fixture's two-reference cross-check: supply re-measured against the 2.5 V reference agrees within 0.3%); GREEN while the on-device plausibility windows (5–60 °C, 2900–3700 mV) and the TLV lookup pass, RED otherwise. Complements `--bin adc_internal_test_runner`, which deliberately leaves REF_A **off** and asserts the temp sensor reads ~0 (the sensor is biased by REF_A, not the ADC).

## Architecture

**Workspace layout:**
- `pac/` — Peripheral Access Crate auto-generated by `svd2rust` from `msp430fr5969.svd`. Provides typed register access for all MSP430FR5969 peripherals. This is a ~71K-line generated file — do not manually edit `pac/src/lib.rs`. Owns `memory.x` (MSP430FR5969 memory map) and `device.x` (interrupt vector defaults); both are copied to the linker search path by `build.rs` when the `rt` feature is enabled.
- `hal/` — Hardware Abstraction Layer crate built on top of the PAC. Re-exports `pac`, `embedded_hal`, `embedded_hal_nb`, `embedded_io`, and `embedded_storage`. Passes through `rt` and `critical-section` features to the PAC. Modules: `gpio` (typed pins, `embedded-hal` digital traits; port edge interrupts on input pins — `enable_interrupt(Edge)` etc., ISR-side `gpio::read_iv::<Px>()` whose PxIV read atomically clears the served flag), `serial` (eUSCI_A UART, `embedded-hal-nb` + `embedded-io`), `spi`/`i2c` (eUSCI_B0, `embedded-hal` SPI/I2C), `adc` (ADC12_B; ratiometric AVCC-referenced reads plus absolute VREF-referenced reads that take `&ref_a::Ref`), `ref_a` (REF_A shared 1.2/2.0/2.5 V reference; also powers the on-die temperature sensor), `tlv` (factory ADC/REF calibration constants from the 0x1A00 device-descriptor table), `clocks`, `delay`, `timer` (Timer_A free-running counter), `power` (LPMx entry: `enter_lpm3` for clocked sleep, `enter_lpm4` for wait-for-pin-edge; both set GIE atomically with sleeping and need `#[interrupt(wake_cpu)]` handlers to resume), `fram` (`embedded-storage`), `pwm` (Timer_B0 PWM, `embedded_hal::pwm::SetDutyCycle`), `rtc` (RTC_B calendar; native API — embedded-hal has no RTC trait), and `watchdog` (WDT_A: pre-`take()` `disable()` free function, `Watchdog` start/feed/stop, `force_reset()` software reset, plus interval-timer mode — `start_interval` turns expiry into a periodic `WDT`-vector interrupt instead of a PUC, `enable_interval_interrupt` RMWs the chip-global `SFRIE1.WDTIE` under the shared-SFRIE1 rule; native API — embedded-hal 1.0 dropped the watchdog traits). The crate-root `hal::init(watchdog::WdtMode)` is the boot front door: watchdog policy (`Hold` / `LeaveRunning` / `Arm { source, interval }` to run boot under a chosen timeout) + `Peripherals::take()` fused in guaranteed order.
- `pac_consumer/` — Test/experimentation binary that exercises the PAC directly. Entry point is `_start()` in `src/main.rs`.
- `hal_test_runners/` — Test/experimentation binary that exercises the HAL. Minimal `_start()` entry point.
- `unit_tests/` — Host-target unit tests for all the HAL's pure math (baud, ticks, FRAM addressing, ADC calibration — see Testing below). Not part of the workspace.

**Key configuration:**
- `.cargo/config.toml` — Sets `msp430-none-elf` target, enables `build-std = ["core"]`, configures TI GCC linker and `mspdebug tilib` as the runner.
- `pac/memory.x` — Defines **only** the MEMORY regions, named `RAM`/`ROM`/`VECTORS` exactly as msp430-rt's `link.x` requires (no SECTIONS — `link.x` `INCLUDE`s this file and supplies them). RAM at 0x1C00 (2KB), ROM/FRAM at 0x4400 (48KB, ends 0xFF7F), VECTORS at **0xFF90** (the 56-word table — 55 PAC interrupt slots + reset word — must end at 0x10000; msp430-rt `ASSERT`s this). Provided to downstream crates via `pac/build.rs` when the `rt` feature is enabled.

**Runtime model:** Startup is provided by **`msp430-rt`** (pulled in through the PAC's `rt` feature). Its `Reset` handler is the reset-vector entry point: it loads the stack pointer, zeroes `.bss`, copies `.data`, then calls the `#[entry] fn main() -> !`. The watchdog must still be stopped as the first thing in `main` — `let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap()` stops it and then takes the peripherals, in that order by construction (msp430-rt does not touch the watchdog, and `Peripherals::take()` enters a critical section). A custom `#[panic_handler]` loops forever. There is no hand-rolled `_start` / `.reset_vector` static / manual `.bss` loop anymore.

**Stack-pointer init (now handled by msp430-rt):** The MSP430 does *not* set the stack pointer (R1) at reset. This project used to hand-roll a `#[naked]` `_start` whose first instruction was `mov #__stack_top, r1`; msp430-rt's `Reset` now does exactly that (`mov #0x2400, r1`, where 0x2400 = `ORIGIN(RAM) + LENGTH(RAM)`). The failure mode if SP is ever left uninitialized is worth remembering: code runs on whatever garbage R1 holds, *appears* to work intermittently, and corrupts memory differently with every build/reset — it once masqueraded as a UART that hung on its first transmit only sometimes. Verify after any runtime change with `msp430-elf-objdump -d` that **`Reset`** begins with `mov #0x2400, r1` and that the last word of `.vector_table` (0xFFFE) points at `Reset`.

**Linking against the TI runtime libraries:** The link uses msp430-rt's `link.x` (`-C link-arg=-T -C link-arg=link.x`), which msp430-rt's `build.rs` writes to the link search path; `link.x` in turn `INCLUDE`s the PAC-provided `memory.x` (regions) and `device.x` (weak interrupt `PROVIDE`s → `DefaultHandler`). `.cargo/config.toml` also links `libgcc` and the device hardware-multiply library (`libmul_f5`) for the MSP430 EABI runtime routines (`__mspabi_mpyi/mpyl`, `__mspabi_divu/remu`) that `core` references once any multiply/divide/format code is present. Key flags: `-nostartfiles` (msp430-rt, not gcc's crt0, owns `Reset`); `-mcpu=msp430` forces gcc to pick the baseline-ISA `430/` multilib that matches rustc's output (`-mmcu` alone selects the MSP430**X** variant and the link fails with an ISA mismatch); `-Wl,--gc-sections` drops dead code. `abort` (referenced by compiler-builtins' `memcpy`) is provided by each binary so newlib's `libc` (with its syscall stubs) need not be linked.

**FRAM budget:** With only 48 KB of FRAM, an *unoptimized* debug build that does arithmetic pulls in overflow-check panics, which drag in the ~30 KB `core::fmt` engine and overflow FRAM. The workspace `[profile.dev]` therefore sets `opt-level = "s"`, `overflow-checks = false`, and `panic = "abort"` so `cargo +nightly build` (debug) still fits. For the same reason `hal::serial` does not implement `core::fmt::Write` (it would pull `core::fmt::write`); format into a buffer and use `embedded_io::Write::write_all` instead.

## Critical-section support

The `msp430` crate (v0.4.1) with feature `critical-section-single-core` provides the `critical_section::set_impl!` implementation. Acquire saves SR then executes DINT+NOP; release restores GIE only if it was previously set. The `pac_consumer` crate force-links `msp430` (`use msp430 as _`) so these symbols resolve. `Peripherals::take()` works.

Note: the watchdog must be stopped *before* `Peripherals::take()`, since `take()` enters a critical section and the default watchdog timeout is ~32ms. HAL consumers call `hal::init(WdtMode)`, which fuses the watchdog stop and `Peripherals::take()` so the ordering can't be gotten wrong (`hal::watchdog::disable()` remains the underlying free function, usable pre-`take()`); only `pac_consumer` still does the raw `0x015C` write, deliberately, since it exercises the PAC without the HAL. **Never write `WDTCTL` through the PAC field API** (`write`/`modify` on `wdtctl`): the SVD does not model the `WDTPW` password byte, so those produce a wrong-password write and an instant PUC — `hal::watchdog` composes the `0x5A00` key into whole-register writes instead.

## Interrupt conventions

Real ISRs are in use and hardware-verified. The recipe, end to end:

- The binary declares `#![feature(abi_msp430_interrupt)]` (plus `#![feature(asm_experimental_arch)]` if any handler uses `wake_cpu`) and defines handlers with `#[msp430_rt::interrupt] fn NAME()`, where `NAME` is a `pac::Interrupt` variant. The macro validates the name against an `interrupt::NAME` path, which `use hal::interrupt;` (the shim in `hal/src/lib.rs`) satisfies. Handlers get the `msp430-interrupt` ABI (RETI) from the macro; the linker overrides the matching `device.x` weak PROVIDE by symbol name.
- ISR↔main shared state is `static X: critical_section::Mutex<Cell<T>>`, accessed on both sides inside `critical_section::with`. Not `RefCell` — no borrow-panic machinery, and a `Cell` is enough when each side only gets/sets.
- Driver structs own their peripheral, so ISRs use module-level **free functions** backed by `pac::X::steal()` that touch only ISR-facing bits: `timer::clear_overflow_irq()`, `timer::clear_wake_irq()`, `rtc::read_iv()`, `gpio::read_iv::<Px>()`. Prefer `read_iv`-style consumption where the peripheral has an IV register — its read-and-clear is atomic in silicon, immune to the lost-flag RMW hazard.
- Interrupt-enable registers shared between bits/owners (`PxIE`, `PxIES`, `PxIFG`) are only ever RMW'd inside `critical_section::with`; the HAL's `enable_*_interrupt` methods do this internally (which is why the HAL's `critical-section` feature now pulls in the `critical-section` crate proper). GPIO arming order is fixed by SLAU367: program `PxIES`, *then* clear `PxIFG` (an IES write can spuriously latch it), *then* set `PxIE`.
- Nothing fires until GIE is set: either `unsafe { msp430::interrupt::enable() }` for stay-awake code, or the `power::enter_lpm*` entries, which set GIE and the LPM bits in one `bis` (atomic — no wake-lost-in-the-gap). Any ISR that must let `main` resume after `enter_lpm*` is `#[interrupt(wake_cpu)]`; port interrupts latch asynchronously and wake even LPM4.
- **Shared-SFRIE1 rule**: `SFRIE1`/`SFRIFG1` are chip-global (WDTIE next to NMIIE/VMAIE…). No driver owns `pac::Sfr` — touch those registers only via `pac::Sfr::steal()`, only with bit-field `modify`, only inside `critical_section::with`, from thread mode only (`hal::watchdog`'s interval-interrupt methods are the reference implementation). Note the dedicated `WDT` vector auto-resets `WDTIFG` on service, so a `fn WDT()` handler needs no flag work.

## Shortcomings in the PAC crate

- The PAC's `rt` feature now depends on `msp430-rt` 0.4.1 (with its `device` feature); consumers use `#[entry]` from `msp430_rt`. **Two intentional hand-patches to generated code** in `pac/src/lib.rs`, both lost on regeneration (regenerate with the msp430 flavor of svd2rust to get both right at the source):
  1. `svd2rust` emitted the vector-table `Vector` union with `_reserved: u32` (4 bytes — wrong for a 16-bit target; correct msp430 PACs use `u16`). It was dormant while `__INTERRUPTS` got garbage-collected, but msp430-rt KEEPs and places the table, so the doubled width overran the VECTORS region. Patched to `u16`.
  2. The interrupt handler declarations (the `extern` block and `Vector::_handler`) were emitted `extern "C"`; they are patched to `extern "msp430-interrupt"` (with `#![cfg_attr(feature = "rt", feature(abi_msp430_interrupt))]` at the crate root, and the `Vector` union gated on `rt` so the non-`rt` build doesn't need the feature). msp430-rt's `#[interrupt]` macro defines handlers with that ABI (they return with `RETI`, not `RET`) and the linker patches the table by symbol name, so the old mismatch was harmless — but the declared type was a lie; verified byte-identical linker output before/after the patch. Real ISRs exist and are hardware-verified (`timer_test_runner` `TIMER0_A1`, `deep_sleep_test_runner` `TIMER0_A0` with `wake_cpu`).