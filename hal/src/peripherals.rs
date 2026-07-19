//! Boot front door: take the PAC peripherals with the watchdog policy
//! applied first, in guaranteed order — see [`take`].

#[cfg(feature = "critical-section")]
use crate::{pac, watchdog};

/// Optionally stop the watchdog, then take the PAC peripherals — **in that
/// order, guaranteed by construction**.
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
///     let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();
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
///
/// Gated on the `critical-section` feature for the same reason the PAC gates
/// `Peripherals::take()` on it: `take()` only exists when a critical-section
/// implementation is available (see the `msp430` crate's
/// `critical-section-single-core`). Without the feature there is no safe
/// PAC `take` to fuse with, so there is no `peripherals::take` either — use
/// [`watchdog::disable`] and `Peripherals::steal()` manually in that world.
#[cfg(feature = "critical-section")]
pub fn take(wdt: watchdog::WdtMode) -> Option<pac::Peripherals> {
    match wdt {
        watchdog::WdtMode::Hold => watchdog::disable(),
        watchdog::WdtMode::LeaveRunning => {}
        watchdog::WdtMode::Arm { source, interval } => watchdog::arm(source, interval),
    }
    pac::Peripherals::take()
}
