//! FRAM Memory Protection Unit — hardware read/write/execute fences inside
//! the FRAM, because on this part "flash-like" memory is writable by any
//! stray `MOV`.
//!
//! # Why FRAM needs an MPU at all
//!
//! On a flash MCU, firmware is protected from itself by physics: overwriting
//! code takes an erase/program sequence behind an unlock dance. FRAM removed
//! all of that friction — a write is a plain store ([`crate::fram`]) — which
//! means a single wild pointer can rewrite the running program as easily as
//! it scribbles on a buffer. The MPU is the missing friction, reintroduced as
//! a bus-level checker: it carves main FRAM into **three contiguous
//! segments** with two movable borders (16-byte granularity), gives each
//! segment read/write/execute enables, and polices every access — CPU *and*
//! DMA. The 512 B Information FRAM gets a fourth, fixed segment.
//!
//! A violating access is **suppressed** (a blocked write leaves FRAM
//! untouched; a blocked fetch never executes), the segment's flag latches in
//! `MPUCTL1`, and then one of two things happens, per segment
//! ([`Violation`]): nothing more (poll the flag, or escalate to the `SYSNMI`
//! vector with [`Config::nmi`]), or a **PUC reset** — after which `SYSRSTIV`
//! names the guilty segment ([`crate::sys::ResetReason::MpuSeg1`] etc., and
//! the pre-reset flags survive into the next boot for forensics).
//!
//! The intended production posture: borders at the code/data split, code
//! segment [`Access::rx`] with [`Access::reset_on_violation`], data segment
//! writable. Firmware that corrupts itself then reboots into a recorded
//! reason instead of limping on executing garbage.
//!
//! ```ignore
//! use hal::mpu::{Access, Config, Mpu};
//! let mut mpu = Mpu::new();
//! mpu.enable(&Config {
//!     border1: 0x1_0000,          // seg1 = both lower-bank code+data,
//!     border2: 0x1_0000,          // seg2 = empty,
//!     seg1: Access::rwx(),        // seg3 = the upper 16 KB bank
//!     seg2: Access::rwx(),
//!     seg3: Access::rx(),         // HighFram log area: no stray writes
//!     info: Access::rwx(),
//!     nmi: true,                  // violations fire the SYSNMI vector
//! }).unwrap();
//! ```
//!
//! # Register access: why this module bypasses the PAC entirely
//!
//! Two independent reasons, one esoteric each:
//!
//! 1. **The PAC has no MPU.** `pac/src/lib.rs` was generated from an SVD
//!    revision that lacked the MPU block; the checked-in
//!    `msp430fr5969.svd` *does* carry it (base `0x05A0`), so a future
//!    regeneration (msp430 svd2rust flavor, see "Shortcomings in the PAC
//!    crate") will grow it. Until then: raw volatile pointers, the same
//!    pattern [`crate::clocks`] uses for `CSCTL0_H` and [`crate::power`] for
//!    `PMMCTL0`.
//! 2. **It wouldn't help.** Even the SVD's MPU model omits the `MPUPW`
//!    password field, exactly like `WDTCTL`/`PMMCTL0`/`CSCTL0` — a generated
//!    `modify()` would echo back a wrong key. This family's password modules
//!    are all landmines behind field-level PAC APIs; see [`crate::watchdog`].
//!
//! # The password discipline (`MPUPW`)
//!
//! `MPUCTL0`'s high byte is a key: writes carrying `0xA5` there **open** all
//! MPU registers for writing; writing any other value to the high byte
//! **closes** them again (reads always work, and read back `0x96` in the key
//! byte). While closed, a write to an MPU register is itself a violation that
//! triggers a PUC (`SYSRSTIV` = `0x22`, [`crate::sys::ResetReason::MpuPassword`]).
//! This driver therefore brackets every register sequence with byte-writes to
//! `MPUCTL0_H` — open `0xA5`, work, close `0x00` — the sequence TI's own
//! examples use. Byte-writing the *high* byte leaves `MPUENA`/`MPULOCK`/
//! `MPUSEGIE` in the low byte untouched, so flags can be cleared without
//! momentarily dropping protection.
//!
//! **The NMI re-lock race:** MPU violation NMIs are non-maskable — a critical
//! section cannot hold them off. If a violation NMI fires *while thread-mode
//! code is inside an open→write→close bracket* and the handler runs its own
//! bracket ([`clear_violation_flags`]), its close re-locks the registers
//! under the interrupted sequence, whose next write then PUCs. The discipline
//! that keeps this theoretical: configure the MPU *before* the protected
//! regions can be touched, and treat reconfiguration as something you don't
//! do concurrently with code that may violate. (Maskable interrupts are no
//! hazard — only this module touches MPU registers, per the singleton
//! discipline on [`Mpu`].)
//!
//! # Lock-until-BOR
//!
//! [`Mpu::lock`] sets `MPULOCK`: from then on the MPU registers cannot be
//! modified — password or not — until a **BOR-class** reset (power cycle,
//! reset pin, LPMx.5 wake). Note the asymmetry with violations' PUC: a PUC is
//! *weaker* than a BOR, so even a malicious `force_reset()` cannot unlock a
//! locked MPU. That is the point: lock is the tamper-resistance tier above
//! password protection.
//!
//! # Self-lockout (read the borders twice)
//!
//! Nothing stops a [`Config`] whose executing segment lacks
//! [`Access::execute`], or whose only writable data region excludes
//! statics the program needs. The validation here checks border geometry,
//! not intent — removing X from under the program counter hands the CPU a
//! suppressed fetch (and a PUC if so configured, else a wedge until the
//! watchdog or an NMI intervenes). Rust's linker script places all code and
//! `.rodata` in the lower bank (`0x4400..0xFF80`), so a border at
//! `0x1_0000` — protecting exactly the [`crate::fram::HighFram`] bank — is
//! the safe first move, and what the `mpu_test_runner` fixture does.

use crate::mpu_seg;

pub use crate::mpu_seg::{
    segment_containing, Access, MpuError as Error, Violation, MAIN_END, MAIN_START,
};

// --- Register map (SVD: MPU block at base 0x05A0) ---
/// `MPUCTL0` low byte: `MPUENA` / `MPULOCK` / `MPUSEGIE`.
const MPUCTL0_L: usize = 0x05A0;
/// `MPUCTL0` high byte: the password lane (write `0xA5` to open, anything
/// else to close; reads as `0x96`).
const MPUCTL0_H: usize = 0x05A1;
/// `MPUCTL1`: the five violation flags (read anytime; write while open).
const MPUCTL1: usize = 0x05A2;
/// `MPUSEGB2`: upper border, address `>> 4`.
const MPUSEGB2: usize = 0x05A4;
/// `MPUSEGB1`: lower border, address `>> 4`.
const MPUSEGB1: usize = 0x05A6;
/// `MPUSAM`: the four access nibbles (see `mpu_seg::sam_value`).
const MPUSAM: usize = 0x05A8;

/// `MPUCTL0_H` open value (`MPUPW >> 8`).
const KEY_OPEN: u8 = 0xA5;
/// Any non-key value closes; `0x00` is the conventional one.
const KEY_CLOSE: u8 = 0x00;

// MPUCTL0 low-byte bits.
const ENA: u8 = 0x01;
const LOCK: u8 = 0x02;
const SEGIE: u8 = 0x10;

// MPUCTL1 flag bits.
const SEG1IFG: u16 = 0x0001;
const SEG2IFG: u16 = 0x0002;
const SEG3IFG: u16 = 0x0004;
const SEGIIFG: u16 = 0x0008;
const SEGIPIFG: u16 = 0x0010;

#[inline]
fn rd8(addr: usize) -> u8 {
    // SAFETY: `addr` is one of the MPU register byte lanes above — always
    // present, always readable (reads are not password-gated).
    unsafe { (addr as *const u8).read_volatile() }
}

#[inline]
fn rd16(addr: usize) -> u16 {
    // SAFETY: as `rd8`, word lane.
    unsafe { (addr as *const u16).read_volatile() }
}

/// Write an MPU register. Callers must hold the bracket open (see `open`) —
/// a write while closed is a password violation and PUCs the chip.
#[inline]
fn wr16(addr: usize, value: u16) {
    // SAFETY: `addr` is an MPU register; the password discipline (open
    // bracket) is upheld by every caller in this module.
    unsafe { (addr as *mut u16).write_volatile(value) }
}

#[inline]
fn wr8(addr: usize, value: u8) {
    // SAFETY: as `wr16`, byte lane.
    unsafe { (addr as *mut u8).write_volatile(value) }
}

/// Open the MPU registers for writing: `0xA5` into the password lane only.
/// High-byte write, so `MPUENA`/`MPULOCK`/`MPUSEGIE` are untouched — the MPU
/// keeps enforcing while its registers are being edited.
#[inline]
fn open() {
    wr8(MPUCTL0_H, KEY_OPEN);
}

/// Close the MPU registers (any non-key value locks the password). Same
/// high-byte-only rule as `open`.
#[inline]
fn close() {
    wr8(MPUCTL0_H, KEY_CLOSE);
}

/// The five `MPUCTL1` violation flags, as one copyable snapshot.
///
/// Flags latch on violation and stay set until cleared by software
/// ([`clear_violation_flags`] / [`Mpu::clear_violations`]) — SLAU367 does not
/// promise the `SYSSNIV` read clears them (TI's own examples clear manually),
/// so this driver never relies on that.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Violations {
    bits: u16,
}

impl Violations {
    /// Main-memory segment 1 (`MPUSEG1IFG`).
    pub fn seg1(&self) -> bool {
        self.bits & SEG1IFG != 0
    }
    /// Main-memory segment 2 (`MPUSEG2IFG`).
    pub fn seg2(&self) -> bool {
        self.bits & SEG2IFG != 0
    }
    /// Main-memory segment 3 (`MPUSEG3IFG`).
    pub fn seg3(&self) -> bool {
        self.bits & SEG3IFG != 0
    }
    /// Information-memory segment (`MPUSEGIIFG`).
    pub fn info(&self) -> bool {
        self.bits & SEGIIFG != 0
    }
    /// Encapsulated-IP segment (`MPUSEGIPIFG`) — the FR5969 carries the flag
    /// even though this HAL does not drive IP encapsulation.
    pub fn ip(&self) -> bool {
        self.bits & SEGIPIFG != 0
    }
    /// Any violation latched?
    pub fn any(&self) -> bool {
        self.bits & (SEG1IFG | SEG2IFG | SEG3IFG | SEGIIFG | SEGIPIFG) != 0
    }
    /// The raw `MPUCTL1` value, for logging.
    pub fn bits(&self) -> u16 {
        self.bits
    }
}

/// A complete MPU configuration: two borders, four segment access settings,
/// and the NMI escalation switch. Installed atomically-enough by
/// [`Mpu::enable`] (segment registers first, the enable bit last).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    /// Border between segments 1 and 2: an absolute address in
    /// `0x4400..=0x14000`, 16-byte aligned. The border byte itself belongs to
    /// the *higher* segment.
    pub border1: u32,
    /// Border between segments 2 and 3; same rules, must be `>= border1`
    /// (equal ⇒ segment 2 is empty).
    pub border2: u32,
    /// Access for segment 1 (`MAIN_START..border1`).
    pub seg1: Access,
    /// Access for segment 2 (`border1..border2`).
    pub seg2: Access,
    /// Access for segment 3 (`border2..MAIN_END`).
    pub seg3: Access,
    /// Access for the Information FRAM segment (`0x1800..=0x19FF`, fixed).
    /// Guards [`crate::fram::InfoFram`]; the factory TLV block at `0x1A00` is
    /// separate silicon and not covered.
    pub info: Access,
    /// `MPUSEGIE`: escalate [`Violation::Flag`] violations to the `SYSNMI`
    /// vector. The handler demuxes with [`crate::sys::read_nmi_iv`] and must
    /// clear the flag via [`clear_violation_flags`]. Non-maskable — fires
    /// with GIE clear, and wakes any LPM.
    pub nmi: bool,
}

impl Config {
    /// The hardware's reset posture, spelled out: everything allowed,
    /// borders parked at the top. A starting point to restrict from.
    pub const fn allow_all() -> Config {
        Config {
            border1: MAIN_END,
            border2: MAIN_END,
            seg1: Access::rwx(),
            seg2: Access::rwx(),
            seg3: Access::rwx(),
            info: Access::rwx(),
            nmi: false,
        }
    }
}

/// The MPU driver.
///
/// A zero-sized handle (there is no PAC peripheral to own — see the module
/// docs). Like [`crate::fram::InfoFram`], holding two is not memory-unsafe
/// but is a shared-mutable-resource hazard: the password bracket assumes no
/// other code opens/closes it concurrently. Treat it as a singleton.
#[derive(Debug, Default)]
pub struct Mpu {
    _private: (),
}

impl Mpu {
    /// Create the MPU handle. No register access.
    pub const fn new() -> Self {
        Mpu { _private: () }
    }

    /// Validate `config`, program it, and enable enforcement — one password
    /// bracket. Stale violation flags are cleared on the way (so a pre-reset
    /// violation cannot masquerade as a fresh one), and the segment registers
    /// are written *before* the enable bit, so enforcement never runs on a
    /// half-installed configuration. Fails with [`Error::Locked`] if
    /// `MPULOCK` is set (nothing can be changed until a BOR).
    ///
    /// Reconfiguring an already-enabled MPU is allowed but momentarily
    /// applies new borders under old access nibbles (register writes are
    /// sequential); [`disable`](Mpu::disable) first if code touching the
    /// affected segments could run concurrently (e.g. DMA).
    pub fn enable(&mut self, config: &Config) -> Result<(), Error> {
        if self.is_locked() {
            return Err(Error::Locked);
        }
        let b1 = mpu_seg::addr_to_border(config.border1)?;
        let b2 = mpu_seg::addr_to_border(config.border2)?;
        if b1 > b2 {
            return Err(Error::BordersReversed);
        }
        let sam = mpu_seg::sam_value(config.seg1, config.seg2, config.seg3, config.info);

        open();
        wr16(MPUSEGB1, b1);
        wr16(MPUSEGB2, b2);
        wr16(MPUSAM, sam);
        wr16(MPUCTL1, 0); // discard stale flags
        wr8(MPUCTL0_L, ENA | if config.nmi { SEGIE } else { 0 });
        close();
        Ok(())
    }

    /// Stop enforcing: clear `MPUENA` (and `MPUSEGIE`). The segment registers
    /// keep their values; latched violation flags stay latched for
    /// inspection. Fails with [`Error::Locked`] if `MPULOCK` is set.
    pub fn disable(&mut self) -> Result<(), Error> {
        if self.is_locked() {
            return Err(Error::Locked);
        }
        open();
        wr8(MPUCTL0_L, 0);
        close();
        Ok(())
    }

    /// Set `MPULOCK`: freeze the entire MPU register file — current borders,
    /// access rights, and enable state — until the next **BOR-class** reset.
    /// A PUC (watchdog, `force_reset`, an MPU violation itself) does *not*
    /// unlock; that asymmetry is the tamper-resistance (see module docs).
    /// One-way by design: there is no `unlock`.
    ///
    /// What happens to a register write while locked is not spelled out in
    /// SLAU367; HW-established 2026-07-05 (`mpu_test_runner` lock probe): a
    /// password-bracketed `MPUSEGB1` write is **silently ignored** — no PUC,
    /// borders unchanged. This driver refuses in software first
    /// ([`Error::Locked`]) so callers get an error instead of silence.
    pub fn lock(&mut self) {
        let current = rd8(MPUCTL0_L) & (ENA | SEGIE);
        open();
        wr8(MPUCTL0_L, current | LOCK);
        close();
    }

    /// Is enforcement on (`MPUENA`)?
    pub fn is_enabled(&self) -> bool {
        rd8(MPUCTL0_L) & ENA != 0
    }

    /// Read back the installed borders as absolute addresses
    /// `(border1, border2)` — what the hardware is actually comparing
    /// against, decoded from `MPUSEGB1`/`MPUSEGB2`. Useful for verifying a
    /// configuration took (or, after [`lock`](Mpu::lock), that a write
    /// didn't).
    pub fn borders(&self) -> (u32, u32) {
        (
            mpu_seg::border_to_addr(rd16(MPUSEGB1)),
            mpu_seg::border_to_addr(rd16(MPUSEGB2)),
        )
    }

    /// Is the register file frozen until BOR (`MPULOCK`)?
    pub fn is_locked(&self) -> bool {
        rd8(MPUCTL0_L) & LOCK != 0
    }

    /// Snapshot the latched violation flags (plain read, no password).
    pub fn violations(&self) -> Violations {
        violation_flags()
    }

    /// Clear all latched violation flags. See [`clear_violation_flags`].
    pub fn clear_violations(&mut self) {
        clear_violation_flags();
    }
}

/// Snapshot `MPUCTL1` — the thread-mode *or* ISR-side read (reads are never
/// password-gated). Free function per the ISR convention (driver structs stay
/// in thread mode; handlers use module-level free functions).
pub fn violation_flags() -> Violations {
    Violations { bits: rd16(MPUCTL1) }
}

/// Clear all five `MPUCTL1` violation flags — one open→clear→close password
/// bracket, safe to call from the `SYSNMI` handler.
///
/// A `SYSNMI` handler for MPU violations **must** call this (after
/// [`crate::sys::read_nmi_iv`] has identified the source): SLAU367 leaves the
/// flags' interaction with the `SYSSNIV` read unspecified, and a flag left
/// set keeps the source pending. Mind the re-lock race in the module docs if
/// thread code also brackets concurrently.
pub fn clear_violation_flags() {
    open();
    wr16(MPUCTL1, 0);
    close();
}
