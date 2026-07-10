//! Host-side unit tests for the HAL's pure math, one module per subject.
//!
//! Each module `include!`s the REAL driver source from `hal/src/` — the same
//! bytes that ship in firmware — so a bad edit to any of the math fails these
//! tests without a board on the desk. The included files are dependency-free
//! (`//` comments, no PAC/HAL types) by design, exactly so they can land
//! inside a module here.
//!
//! Run with `cd unit_tests && cargo +nightly test` (the crate is detached from
//! the workspace and carries its own `.cargo/config.toml`, so it must be built
//! from inside this directory to pick up the host-target override).

// The included sources are exercised only by the test harness; without this
// the non-test build of the library would warn on every included item.
#![allow(dead_code)]

mod adc_cal;
mod adc_seq;
mod baud;
mod capture_math;
mod comp_ladder;
mod crc_soft;
mod fram_addr;
mod mpu_seg;
mod rtc_alarm;
mod rtc_tick;
mod rx_queue;
mod ticks;
