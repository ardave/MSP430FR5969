#![no_std]
// Inline asm for msp430 is gated behind this nightly feature (used by
// `delay` for its cycle-accurate busy-loop).
#![feature(asm_experimental_arch)]

pub mod adc;
mod adc_cal;
mod adc_seq;
pub mod aes;
mod baud;
pub mod capture;
mod capture_math;
pub mod captio;
mod captio_ctl;
pub mod clocks;
pub mod comp_e;
mod comp_ladder;
pub mod crc;
mod crc_soft;
pub mod delay;
pub mod dma;
pub mod fram;
mod fram_addr;
pub mod gpio;
pub mod i2c;
mod i2c_slave;
pub mod mpu;
mod mpu_seg;
pub mod peripherals;
pub mod power;
pub mod pwm;
pub mod ref_a;
pub mod rtc;
mod rtc_alarm;
mod rtc_tick;
pub mod rx_queue;
pub mod serial;
pub mod spi;
pub mod sys;
mod ticks;
pub mod timer;
pub mod tlv;
pub mod watchdog;

pub use embedded_hal;
pub use embedded_hal_nb;
pub use embedded_io;
pub use embedded_storage;
pub use pac;

/// Interrupt vector names for use with msp430-rt's `#[interrupt]` attribute.
///
/// msp430-rt's `#[interrupt]` macro validates the handler's name against a
/// `interrupt::<NAME>` path (e.g. `interrupt::TIMER0_A1`). The PAC exposes these
/// as variants of `pac::Interrupt` rather than as a module, so this shim
/// re-exports them under the `interrupt` path the macro expects. Bring it into
/// scope (`use hal::interrupt;`) alongside the `#[msp430_rt::interrupt]`
/// attribute when defining an ISR.
pub mod interrupt {
    pub use crate::pac::Interrupt::*;
}
