#![no_std]
// Inline asm for msp430 is gated behind this nightly feature (used by
// `delay` for its cycle-accurate busy-loop).
#![feature(asm_experimental_arch)]

mod baud;
pub mod clocks;
pub mod delay;
pub mod gpio;
pub mod serial;
pub mod timer;

pub use embedded_hal;
pub use embedded_hal_nb;
pub use embedded_io;
pub use pac;
