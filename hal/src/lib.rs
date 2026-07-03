#![no_std]
// Inline asm for msp430 is gated behind this nightly feature (used by
// `delay` for its cycle-accurate busy-loop).
#![feature(asm_experimental_arch)]

pub mod adc;
mod baud;
pub mod clocks;
pub mod delay;
pub mod fram;
mod fram_addr;
pub mod gpio;
pub mod i2c;
pub mod power;
pub mod pwm;
pub mod rtc;
pub mod serial;
pub mod spi;
mod ticks;
pub mod timer;
pub mod watchdog;

/// Boot front door: optionally stop the watchdog, then take the PAC
/// peripherals — **in that order, guaranteed by construction**.
///
/// The two steps are order-sensitive: the watchdog powers up running as a
/// ~32 ms fuse, and `Peripherals::take()` enters a critical section, so the
/// stop must come first (see [`watchdog`]). Written as two statements in every
/// binary, nothing stops a future `main` from swapping them; fused here, the
/// ordering cannot be gotten wrong at a call site. Make this the first
/// statement in `main`:
///
/// ```ignore
/// #[entry]
/// fn main() -> ! {
///     let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();
///     // ...
/// }
/// ```
///
/// The three policies (see [`watchdog::WdtMode`]): `Hold` stops the watchdog,
/// `Arm { source, interval }` installs a fresh timeout so boot itself runs
/// guarded, and `LeaveRunning` performs no `WDTCTL` write at all.
///
/// Returns what `Peripherals::take()` returns: `Some` on the first call,
/// `None` after (the watchdog policy is still applied either way).
pub fn init(wdt: watchdog::WdtMode) -> Option<pac::Peripherals> {
    match wdt {
        watchdog::WdtMode::Hold => watchdog::disable(),
        watchdog::WdtMode::LeaveRunning => {}
        watchdog::WdtMode::Arm { source, interval } => watchdog::arm(source, interval),
    }
    pac::Peripherals::take()
}

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
