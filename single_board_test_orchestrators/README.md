# Single-board HiL test orchestrators

Host-side runner for the hardware-in-the-loop (HiL) integration tests of the
repo's MSP430FR5969 HAL. Each suite builds one of the on-board fixtures from
`single_board_test_firmwares`, flashes it to the LaunchPad (via DSLite, see
`src/deployment.rs`), and then drives/observes it over the eUSCI_A0 UART
backchannel that rides the eZ-FET's USB CDC port (9600 8N1). The fixtures
emit framed report/verdict lines; the modules here (`src/*_tests.rs`) parse
them and turn hardware behavior into pass/fail assertions. This is the
project's regression suite for everything that can only be validated on real
silicon — pure math is covered separately by the host-side `unit_tests/`
crate.

## Why "single board"?

These tests are deliberately scoped so that **the only hardware required is
the MSP-EXP430FR5969 LaunchPad itself**, plugged in over USB. That makes
this crate the easy on-ramp for contributors: if you have the LaunchPad —
and nothing else, no second board, no logic analyzer, no breakout modules —
you can run the full default suite and verify the HAL end to end. The
fixtures lean on the board's own resources as stimulus and instrument
(REFOUT as an analog source, internal capture inputs, the supply monitor,
the on-board buttons and crystal), so almost everything is hands-free.

A few suites optionally use a jumper wire or two (SPI loopback, PWM-to-
capture), and those are excluded from the default run or report `SKIP` for
the jumper-dependent verdicts — see below. Tests that genuinely need a
second, independent piece of hardware (a second clock domain, an external
bus master for the I2C slave, real cross-board wires) live in the separate
`two_board_test_orchestrators` rig instead.

## Running

```sh
cd single_board_test_orchestrators
cargo +nightly run              # full default suite (hands-free)
cargo +nightly run -- lpmx5 rtc # only the named suites
```

(Running from inside this directory matters: the crate is detached from the
main workspace — its own `[workspace]` table and `.cargo/config.toml` — so
it builds for the *host* instead of inheriting the repo's `msp430-none-elf`
target. It runs on your workstation; nothing here is cross-compiled to the
MSP430.)

Each suite reflashes the board with its own fixture binary, so a full run
takes a while and the board reboots once per suite. The suite names and
their order are the table in `src/main.rs`.

Three suites run **only when named explicitly**, because they are
interactive or destructive of a normal run:

- `spi` — prompts you to install loopback jumpers (P1.6→P1.7 and P2.5→P2.6).
- `capture_jumper` — prompts for a PWM loopback jumper (P1.4→P1.2); the
  hands-free `capture` variant in the default set reports those verdicts as
  `SKIP` instead.
- `vlo_soak` — an instrument, not a regression gate: reboots the chip 200
  times to characterize an ACLK boot race.

## Expectations

The full default suite passes and is re-run frequently. Treat a suite
failure after a driver change as a regression in the driver, not as fixture
flakiness. Per-fixture wiring notes, verdict meanings, and the hardware
lessons each suite encodes are documented in the "Hardware test setups"
section of the repo's `CLAUDE.md`.
