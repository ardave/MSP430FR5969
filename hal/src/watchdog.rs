//! WDT_A watchdog timer — the countdown that resets the chip unless software
//! keeps proving it is alive.
//!
//! # Why every binary must deal with this first
//!
//! The watchdog powers up **running**: after any reset, `WDTCTL` = `0x6904`
//! (SMCLK source, /32768 divider). At the ~1 MHz default SMCLK that is a
//! **~32 ms** fuse — burn through it without either feeding the counter or
//! stopping it, and the WDT issues a PUC and the chip reboots, forever, in a
//! loop. 32 ms is not much: `msp430-rt`'s startup (`.data` copy, `.bss` zero)
//! plus `Peripherals::take()` (which enters a critical section) can flirt with
//! it. The front door is [`crate::init`], which fuses the two so the ordering
//! is guaranteed by construction — make it the first statement in `main`:
//!
//! ```ignore
//! #[entry]
//! fn main() -> ! {
//!     let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();
//!     // ...
//! }
//! ```
//!
//! ([`disable`] remains the underlying primitive: a free function callable
//! before `Peripherals::take()`, for code that doesn't go through `init`.)
//!
//! # The password mechanism (and why the PAC field API is a trap)
//!
//! `WDTCTL` is key-protected so runaway code can't silently reconfigure it.
//! The upper byte is asymmetric:
//!
//! - every **write** must carry `0x5A` in bits 15:8, or the violation itself
//!   triggers an immediate PUC (this is also the idiomatic MSP430 *software
//!   reset* — see [`force_reset`]);
//! - every **read** returns `0x69` there, so a read-modify-write that echoes
//!   the read-back high byte *also* resets the chip.
//!
//! The SVD (and therefore the PAC) models only the low-byte fields — there is
//! no `WDTPW` field, and the PAC's declared reset value is 0. So the obvious
//! PAC idioms are all landmines: `wdtctl().write(|w| w.wdthold().set_bit())`
//! writes password `0x00` → PUC; `wdtctl().modify(...)` writes back the `0x69`
//! it read → PUC. This module therefore does only **whole-register writes**
//! with the password composed in (`w.bits(0x5A00 | cfg)`), and masks the high
//! byte off anything it reads. That's the esoteric knowledge, captured once.
//!
//! # Re-arming it
//!
//! A held watchdog costs nothing, but production firmware usually wants it
//! back on: [`Watchdog::start`] picks a clock source and divider and starts
//! the countdown, and the main loop must then call [`Watchdog::feed`] more
//! often than the timeout or the chip resets. Timeout = 2^N cycles of the
//! chosen clock:
//!
//! | [`Interval`]  | ACLK = 32 768 Hz | SMCLK = 8 MHz |
//! |---------------|------------------|---------------|
//! | `Cycles2G`    | 18.2 h           | 268 s         |
//! | `Cycles128M`  | 68 min           | 16.8 s        |
//! | `Cycles8192K` | 256 s            | 1.05 s        |
//! | `Cycles512K`  | 16 s             | 65.5 ms       |
//! | `Cycles32K`   | **1 s**          | 4.1 ms        |
//! | `Cycles8192`  | 250 ms           | 1.02 ms       |
//! | `Cycles512`   | 15.6 ms          | 64 µs         |
//! | `Cycles64`    | ~2 ms            | 8 µs          |
//!
//! Feeding sources: SMCLK stops in LPM3+, which *holds* (not resets) the WDT
//! count — a watchdog meant to guard a sleepy application should run from
//! ACLK or VLOCLK so it keeps counting through sleep.
//!
//! To guard **boot itself** (everything between reset and the main loop),
//! skip the disable and arm a sane timeout up front with
//! `hal::init(WdtMode::Arm { source, interval })` — see [`WdtMode::Arm`] for
//! the boot-clock caveat.
//!
//! ```ignore
//! let mut wdt = Watchdog::new(p.watchdog_timer);
//! wdt.start(ClockSource::Aclk, Interval::Cycles32K); // 1 s @ 32.768 kHz
//! loop {
//!     do_work();
//!     wdt.feed(); // must land within every 1 s window
//! }
//! ```
//!
//! (WDT_A also has an *interval timer* mode — `WDTTMSEL = 1` turns the reset
//! into a plain periodic interrupt. This driver doesn't expose it: the project
//! already has better periodic-tick hardware in [`crate::timer`], and wiring
//! the WDT interrupt means touching the shared `SFRIE1` register. Revisit if a
//! use appears.)

use crate::pac;

/// The write key for `WDTCTL` bits 15:8. Reads return `0x6900` instead; any
/// write without this key causes a PUC.
const PASSWORD: u16 = 0x5A00;

/// `WDTHOLD` (bit 7): 1 = watchdog stopped.
const HOLD: u16 = 0x0080;
/// `WDTCNTCL` (bit 3): writing 1 zeroes the count (self-clearing).
const CNTCL: u16 = 0x0008;

/// Boot-time watchdog policy for [`crate::init`].
///
/// `WDTCTL` cannot be partially updated — every access is a whole-register,
/// password-keyed write — so there are exactly three things `init` can do to
/// the watchdog: write hold, write a fresh configuration, or not write at
/// all. One variant per possibility.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WdtMode {
    /// Stop the watchdog before taking the peripherals (the usual choice).
    Hold,
    /// Don't write `WDTCTL` at all — whatever state the watchdog is in flows
    /// through untouched. Out of reset that means the ~32 ms power-up fuse
    /// keeps burning and `Peripherals::take()` runs under it; the caller must
    /// feed or re-arm promptly after.
    LeaveRunning,
    /// Re-arm before taking the peripherals: one write installs `source` +
    /// `interval` and zeroes the count, so boot itself runs guarded by a
    /// watchdog of the caller's choosing rather than the 32 ms default. Feed
    /// it afterwards through `Watchdog::new(p.watchdog_timer)` —
    /// [`Watchdog::feed`] reads the installed configuration back from the
    /// hardware, so no state needs handing over.
    ///
    /// **Clock caveat:** at `init` time the clock tree is still at reset
    /// defaults (SMCLK = 1 MHz; ACLK = VLO until LFXT is started), so the
    /// timeout is denominated in *boot* clocks. `clocks::configure` moving
    /// SMCLK to 8 MHz shrinks an SMCLK-sourced timeout 8×; the crystal taking
    /// over ACLK stretches a VLO-fallback timeout ~3.5×.
    /// [`ClockSource::Vloclk`] never changes out from under you, making it
    /// the honest choice here — pick a generous [`Interval`] and re-run
    /// [`Watchdog::start`] after clock configuration if a precise production
    /// timeout is wanted.
    Arm {
        /// Clock feeding the countdown.
        source: ClockSource,
        /// Timeout, 2^N cycles of `source`.
        interval: Interval,
    },
}

/// Clock feeding the watchdog counter (`WDTSSEL`).
///
/// The discriminants are the raw 2-bit field values (bits 6:5 of `WDTCTL`);
/// `0b11` is reserved on this part and deliberately unrepresentable here.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// SMCLK. Fast, fine-grained timeouts — but SMCLK stops in LPM3 and
    /// below, freezing the countdown while asleep.
    Smclk = 0b00,
    /// ACLK. With the LFXT crystal (32 768 Hz) this gives crystal-accurate,
    /// sleep-surviving timeouts — the usual production choice.
    Aclk = 0b01,
    /// VLOCLK, the ~9.4 kHz always-on low-power oscillator. Imprecise
    /// (datasheet allows wide drift) but needs no crystal and never stops.
    Vloclk = 0b10,
}

/// Countdown length in clock cycles, 2^N (`WDTIS`).
///
/// The discriminants are the raw 3-bit field values (bits 2:0 of `WDTCTL`).
/// See the module docs for the resulting timeouts at common clock rates.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Interval {
    /// 2^31 cycles.
    Cycles2G = 0,
    /// 2^27 cycles.
    Cycles128M = 1,
    /// 2^23 cycles.
    Cycles8192K = 2,
    /// 2^19 cycles.
    Cycles512K = 3,
    /// 2^15 cycles (the power-up default).
    Cycles32K = 4,
    /// 2^13 cycles.
    Cycles8192 = 5,
    /// 2^9 cycles.
    Cycles512 = 6,
    /// 2^6 cycles.
    Cycles64 = 7,
}

/// Whole-register `WDTCTL` write with the password composed in — the only
/// safe way to write it (see module docs). `low` is the configuration byte;
/// anything above bit 7 is discarded so a smuggled bad key is impossible.
///
/// Uses `steal()` rather than an owned peripheral: sound because a single
/// whole-register volatile store has no read-modify-write window, and every
/// caller either runs before `Peripherals::take()` ([`disable`]/[`arm`] via
/// [`crate::init`]) or holds the peripheral exclusively ([`Watchdog`]).
fn write_ctl(low: u16) {
    let wdt = unsafe { pac::WatchdogTimer::steal() };
    wdt.wdtctl().write(|w| unsafe { w.bits(PASSWORD | (low & 0x00FF)) });
}

/// The low-byte watchdog-mode configuration for `source`/`interval`
/// (`WDTSSEL` bits 6:5, `WDTIS` bits 2:0).
fn cfg_bits(source: ClockSource, interval: Interval) -> u16 {
    ((source as u16) << 5) | interval as u16
}

/// Stop the watchdog. **Call this first in `main`, before
/// `Peripherals::take()`** — or let [`crate::init`] do it for you.
///
/// This is a free function rather than a method on [`Watchdog`] precisely so
/// it needs no `Peripherals` — the whole point is to run before `take()`'s
/// critical section, while the ~32 ms power-up fuse is still burning. It
/// compiles to a single 16-bit store (no read-modify-write), so there is
/// nothing for an interrupt to race with, and at this point in boot nothing
/// else can be touching the register anyway.
#[inline(always)]
pub fn disable() {
    write_ctl(HOLD);
}

/// Arm the watchdog with a fresh `source`/`interval` and a zeroed count, in
/// one write. Backs [`crate::init`]'s [`WdtMode::Arm`]; the owned-peripheral
/// path is [`Watchdog::start`], which does the identical write.
pub(crate) fn arm(source: ClockSource, interval: Interval) {
    write_ctl(cfg_bits(source, interval) | CNTCL);
}

/// Reboot the chip *now* via a deliberate watchdog password violation.
///
/// Writing `WDTCTL` without the `0x5A` key triggers an immediate PUC — the
/// idiomatic MSP430 software reset (there is no ARM-style `SYSRESETREQ`).
/// After the reboot, `SYSRSTIV` reads `0x18` (WDT password violation) so
/// startup code can tell this reset apart from a power-on or a genuine
/// watchdog timeout (`0x16`) — see [`crate::sys::ResetReasons`] for the
/// decoded view.
pub fn force_reset() -> ! {
    let wdt = unsafe { pac::WatchdogTimer::steal() };
    // Key byte 0x00 ≠ 0x5A → PUC on this very write.
    wdt.wdtctl().write(|w| unsafe { w.bits(0) });
    // The PUC preempts the next instruction; this loop only pacifies the type
    // system (and any simulator that doesn't model the violation).
    loop {}
}

/// The watchdog in its guard role: owns the PAC peripheral so exactly one
/// place in the program can arm, feed, or stop it.
///
/// Construction does not touch the hardware — after the boot-time
/// [`disable`], the watchdog stays held until [`start`](Watchdog::start).
pub struct Watchdog {
    wdt: pac::WatchdogTimer,
}

impl Watchdog {
    /// Take ownership of the WDT_A peripheral. No register access.
    pub fn new(wdt: pac::WatchdogTimer) -> Self {
        Watchdog { wdt }
    }

    /// Current configuration byte. The high byte reads as the `0x69` echo,
    /// never as stored state, so it is masked off here — writing it back
    /// would reset the chip.
    fn read_cfg(&self) -> u16 {
        self.wdt.wdtctl().read().bits() & 0x00FF
    }

    /// Arm the watchdog: select `source` and `interval`, zero the count, and
    /// release the hold — all in one write, so there is no window where the
    /// countdown runs with a stale configuration.
    ///
    /// From this call on, [`feed`](Watchdog::feed) must be called more often
    /// than the timeout or the chip resets.
    pub fn start(&mut self, source: ClockSource, interval: Interval) {
        write_ctl(cfg_bits(source, interval) | CNTCL);
    }

    /// Pet the dog: zero the count, keeping the current source/interval.
    /// Call from the main loop, not from a timer ISR — a feed that runs on
    /// interrupt keeps the watchdog happy even when the main loop is wedged,
    /// which defeats the entire mechanism.
    pub fn feed(&mut self) {
        let cfg = self.read_cfg();
        write_ctl(cfg | CNTCL);
    }

    /// Hold the watchdog, preserving its configuration. A later
    /// [`start`](Watchdog::start) re-arms it (and restarts the count from
    /// zero).
    pub fn stop(&mut self) {
        let cfg = self.read_cfg();
        write_ctl(cfg | HOLD);
    }

    /// Release the peripheral, holding the watchdog first so it cannot bite
    /// once nothing owns the feeding duty.
    pub fn free(mut self) -> pac::WatchdogTimer {
        self.stop();
        self.wdt
    }
}
