#![no_std]
// Inline asm for msp430 is gated behind this nightly feature (used by
// `delay` for its cycle-accurate busy-loop).
#![feature(asm_experimental_arch)]

mod baud;
pub mod clocks;
pub mod delay;
pub mod gpio;
pub mod power;
pub mod serial;
mod ticks;
pub mod timer;

pub use embedded_hal;
pub use embedded_hal_nb;
pub use embedded_io;
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
