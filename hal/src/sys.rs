//! SYS-module services — chip-level odds and ends, starting with *why did we
//! just reboot?*
//!
//! The SYS peripheral is the FR5969's own miscellaneous drawer: the reset and
//! NMI vector generators, the JTAG mailbox, BSL configuration. That makes this
//! module the natural home for HAL services that belong to the chip as a
//! whole rather than to any bus, timer, or pin. The first resident is decoding
//! the **reset reason** out of `SYSRSTIV`; the second is decoding the
//! **system-NMI source** out of `SYSSNIV` for the `SYSNMI` vector
//! ([`read_nmi_iv`]).
//!
//! # How `SYSRSTIV` works
//!
//! Every reset source on the chip latches its own flag, and those flags
//! **survive the reset they cause** — that is the whole point. `SYSRSTIV`
//! presents them as a *priority-encoded, consume-on-read vector*: a read
//! returns the highest-priority pending cause as an even offset
//! (`0x02..=0x30`), and that same read **clears the flag it just reported**,
//! promoting the next-highest cause into the register. Reading until it
//! returns zero therefore yields every latched cause in priority order and
//! leaves the register empty. (The even-offset encoding exists so an
//! `ADD @PC` jump table can dispatch on it — the same IV convention as
//! `RTCIV`, see [`crate::rtc::read_iv`].)
//!
//! Two consequences of consume-on-read:
//!
//! - **The first reader wins.** Drain once, early in boot, and keep the
//!   result — [`ResetReasons::drain`] returns an owned, copyable snapshot for
//!   exactly this reason.
//! - **The set is "everything since the last drain", not "the latest
//!   reset".** A boot that never reads the register leaves its flags latched
//!   for the next boot to find. [`ResetReasons::primary`] gives the
//!   highest-priority cause; [`ResetReasons::contains`] answers the usual
//!   question ("did the watchdog bite?") directly.
//!
//! # The reset hierarchy: BOR ⊃ POR ⊃ PUC
//!
//! MSP430 resets come in three nested severities — a **BOR** (brownout-class,
//! the deepest) implies a POR implies a PUC; a **PUC** (power-up clear, the
//! shallowest) restarts the CPU but leaves RAM and most module registers
//! intact. Each `SYSRSTIV` cause belongs to one class: power-on, the RST pin,
//! LPMx.5 wakeup and software-BOR requests are BOR-class; SVS events and
//! software-POR requests are POR-class; watchdog timeouts and the various
//! password violations are PUC-class. This is why a watchdog reset does not
//! wipe FRAM-adjacent state the way a power cycle does, and why
//! [`crate::watchdog::force_reset`] (a deliberate WDT password violation,
//! `0x18`) is a *reboot* rather than a full power-on: it is "only" a PUC.
//!
//! # Example
//!
//! ```ignore
//! let p = hal::peripherals::take(hal::watchdog::WdtMode::Hold).unwrap();
//! let reasons = hal::sys::ResetReasons::drain(&p.sys);
//! if reasons.contains(hal::sys::ResetReason::WatchdogTimeout) {
//!     // last life ended with an unfed dog — maybe come up in safe mode
//! }
//! ```

use crate::pac;

/// A single decoded `SYSRSTIV` cause.
///
/// Discriminants follow the FR5969 vector values (device datasheet /
/// SLAU367 "SYSRSTIV Reset Interrupt Vector Values"; TI header names in
/// parentheses). Reserved or unforeseen values decode to [`Unknown`]
/// (`ResetReason::Unknown`) rather than being dropped, so nothing the
/// hardware reports is ever silently lost.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ResetReason {
    /// `0x02` — brownout (BOR). Set on every full power-on, so this is what
    /// a plain power cycle reports.
    Brownout,
    /// `0x04` — RST/NMI pin (`RSTNMI`): the reset button, or the debug
    /// probe's reset line. BOR-class.
    ResetPin,
    /// `0x06` — software-requested BOR via PMM (`DOBOR`).
    SoftwareBor,
    /// `0x08` — wakeup from LPMx.5 (`LPM5WU`). Not a failure: LPMx.5 exit
    /// *is* architecturally a reset.
    Lpm5WakeUp,
    /// `0x0A` — security violation (`SECYV`), e.g. an invalid JTAG key.
    SecurityViolation,
    /// `0x0E` — high-side supply supervisor event (`SVSHIFG`). POR-class.
    SvsHigh,
    /// `0x14` — software-requested POR via PMM (`DOPOR`).
    SoftwarePor,
    /// `0x16` — watchdog timeout (`WDTTO`): the count expired unfed. The
    /// genuine "firmware hung and the guard fired" cause.
    WatchdogTimeout,
    /// `0x18` — `WDTCTL` write without the `0x5A` key (`WDTKEY`). Either
    /// runaway code hit the register, or firmware rebooted on purpose —
    /// [`crate::watchdog::force_reset`] reports as this.
    WatchdogPassword,
    /// `0x1A` — `FRCTL0` write without the `0xA5` key (`FRCTLPW`).
    FramPassword,
    /// `0x1C` — uncorrectable FRAM bit error (`UBDIFG`).
    FramBitError,
    /// `0x1E` — instruction fetch from a peripheral/config address (`PERF`):
    /// the PC ended up somewhere that is not memory.
    PeripheralFetch,
    /// `0x20` — PMM register write without its key (`PMMPW`).
    PmmPassword,
    /// `0x22` — MPU register write without its key (`MPUPW`).
    MpuPassword,
    /// `0x24` — CS register write without its key (`CSPW`).
    CsPassword,
    /// `0x26` — MPU encapsulated-IP segment violation (`MPUSEGPIFG`).
    MpuSegIp,
    /// `0x28` — MPU information-memory segment violation (`MPUSEGIIFG`).
    MpuSegInfo,
    /// `0x2A` — MPU main-memory segment 1 violation (`MPUSEG1IFG`).
    MpuSeg1,
    /// `0x2C` — MPU main-memory segment 2 violation (`MPUSEG2IFG`).
    MpuSeg2,
    /// `0x2E` — MPU main-memory segment 3 violation (`MPUSEG3IFG`).
    MpuSeg3,
    /// `0x30` — FRAM access-time error (`ACCTEIFG`): MCLK too fast for the
    /// configured `NWAITS`.
    FramAccessTime,
    /// A reserved or unrecognized `SYSRSTIV` value, preserved raw.
    Unknown(u16),
}

impl ResetReason {
    /// Decode one raw `SYSRSTIV` value. `None` only for `0` ("no cause
    /// pending" — the drain-loop terminator); any other value decodes, with
    /// reserved values landing in [`ResetReason::Unknown`].
    pub fn from_iv(iv: u16) -> Option<ResetReason> {
        Some(match iv {
            0x00 => return None,
            0x02 => ResetReason::Brownout,
            0x04 => ResetReason::ResetPin,
            0x06 => ResetReason::SoftwareBor,
            0x08 => ResetReason::Lpm5WakeUp,
            0x0A => ResetReason::SecurityViolation,
            0x0E => ResetReason::SvsHigh,
            0x14 => ResetReason::SoftwarePor,
            0x16 => ResetReason::WatchdogTimeout,
            0x18 => ResetReason::WatchdogPassword,
            0x1A => ResetReason::FramPassword,
            0x1C => ResetReason::FramBitError,
            0x1E => ResetReason::PeripheralFetch,
            0x20 => ResetReason::PmmPassword,
            0x22 => ResetReason::MpuPassword,
            0x24 => ResetReason::CsPassword,
            0x26 => ResetReason::MpuSegIp,
            0x28 => ResetReason::MpuSegInfo,
            0x2A => ResetReason::MpuSeg1,
            0x2C => ResetReason::MpuSeg2,
            0x2E => ResetReason::MpuSeg3,
            0x30 => ResetReason::FramAccessTime,
            other => ResetReason::Unknown(other),
        })
    }

    /// Short human-readable name, for UART reporting without `core::fmt`.
    pub fn as_str(&self) -> &'static str {
        match self {
            ResetReason::Brownout => "brownout",
            ResetReason::ResetPin => "reset pin",
            ResetReason::SoftwareBor => "software BOR",
            ResetReason::Lpm5WakeUp => "LPMx.5 wakeup",
            ResetReason::SecurityViolation => "security violation",
            ResetReason::SvsHigh => "SVSH event",
            ResetReason::SoftwarePor => "software POR",
            ResetReason::WatchdogTimeout => "WDT timeout",
            ResetReason::WatchdogPassword => "WDT password",
            ResetReason::FramPassword => "FRAM password",
            ResetReason::FramBitError => "FRAM bit error",
            ResetReason::PeripheralFetch => "peripheral fetch",
            ResetReason::PmmPassword => "PMM password",
            ResetReason::MpuPassword => "MPU password",
            ResetReason::CsPassword => "CS password",
            ResetReason::MpuSegIp => "MPU seg IP",
            ResetReason::MpuSegInfo => "MPU seg info",
            ResetReason::MpuSeg1 => "MPU seg 1",
            ResetReason::MpuSeg2 => "MPU seg 2",
            ResetReason::MpuSeg3 => "MPU seg 3",
            ResetReason::FramAccessTime => "FRAM access time",
            ResetReason::Unknown(_) => "unknown",
        }
    }
}

/// How many causes a drained snapshot can hold. Real boots latch one to
/// three; the hardware tops out at the ~23 defined sources, but anything past
/// the first eight (in priority order) is pathological enough that keeping
/// only the most important eight loses nothing anyone will act on.
const CAPACITY: usize = 8;

/// An owned snapshot of every reset cause that was latched in `SYSRSTIV`,
/// in hardware priority order (most severe first).
///
/// Produced by [`ResetReasons::drain`], which empties the hardware register —
/// hold on to the returned value; a second drain in the same boot comes back
/// empty. `Copy`, 18 bytes, no heap: cheap to stash in a static or pass around.
#[derive(Clone, Copy, Debug)]
pub struct ResetReasons {
    /// Raw IV values as read, `raw[..len]` valid, never zero.
    raw: [u16; CAPACITY],
    len: u8,
}

impl ResetReasons {
    /// Read `SYSRSTIV` until it reports no cause pending, collecting every
    /// latched reset cause in priority order. **Destructive**: this clears
    /// the register, so call it once, early in boot, and keep the result.
    ///
    /// Takes `&pac::Sys` rather than ownership — the SYS block hosts several
    /// unrelated services, and a one-shot destructive read has no business
    /// consuming the whole peripheral.
    pub fn drain(sys: &pac::Sys) -> ResetReasons {
        let mut reasons = ResetReasons {
            raw: [0; CAPACITY],
            len: 0,
        };
        // Bounded well above the ~23 defined sources so a misbehaving
        // register cannot hang boot; reads past CAPACITY still happen (to
        // finish clearing the hardware), they just aren't recorded.
        for _ in 0..32 {
            let iv = sys.sysrstiv().read().bits();
            if iv == 0 {
                break;
            }
            if (reasons.len as usize) < CAPACITY {
                reasons.raw[reasons.len as usize] = iv;
                reasons.len += 1;
            }
        }
        reasons
    }

    /// The highest-priority cause, or `None` if nothing was latched (only
    /// plausible if something drained the register earlier in this boot).
    pub fn primary(&self) -> Option<ResetReason> {
        self.iter().next()
    }

    /// Decoded causes, most severe first.
    pub fn iter(&self) -> impl Iterator<Item = ResetReason> + '_ {
        self.raw[..self.len as usize]
            .iter()
            .map(|&iv| ResetReason::from_iv(iv).unwrap_or(ResetReason::Unknown(iv)))
    }

    /// Was `reason` among the latched causes? The workhorse query:
    /// `reasons.contains(ResetReason::WatchdogTimeout)`.
    pub fn contains(&self, reason: ResetReason) -> bool {
        self.iter().any(|r| r == reason)
    }

    /// Number of recorded causes.
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// True if no cause was latched.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The raw IV values as read from the hardware, for logging.
    pub fn raw(&self) -> &[u16] {
        &self.raw[..self.len as usize]
    }
}

/// A single decoded `SYSSNIV` system-NMI source.
///
/// System NMIs are the chip-level fault reports that must not be maskable:
/// GIE does not gate them, and they fire (and wake) from any LPM. Each source
/// has its own *enable* though — MPU segment violations, the main clients
/// here, only reach the vector when `MPUSEGIE` is set
/// ([`crate::mpu::Config::nmi`]).
///
/// Discriminant values follow the FR5969 `SYSSNIV` table (TI header names in
/// parentheses); like [`ResetReason`], reserved values land in
/// [`Unknown`](NmiSource::Unknown) rather than being dropped.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum NmiSource {
    /// `0x04` — uncorrectable FRAM bit error (`UBDIFG`).
    FramUncorrectableBit,
    /// `0x08` — MPU encapsulated-IP segment violation (`MPUSEGPIFG`).
    MpuSegIp,
    /// `0x0A` — MPU information-memory segment violation (`MPUSEGIIFG`).
    MpuSegInfo,
    /// `0x0C` — MPU main-memory segment 1 violation (`MPUSEG1IFG`).
    MpuSeg1,
    /// `0x0E` — MPU main-memory segment 2 violation (`MPUSEG2IFG`).
    MpuSeg2,
    /// `0x10` — MPU main-memory segment 3 violation (`MPUSEG3IFG`).
    MpuSeg3,
    /// `0x12` — vacant memory access (`VMAIFG`).
    VacantMemory,
    /// `0x14` — JTAG mailbox input (`JMBINIFG`).
    JtagMailboxIn,
    /// `0x16` — JTAG mailbox output (`JMBOUTIFG`).
    JtagMailboxOut,
    /// `0x18` — correctable FRAM bit error (`CBDIFG`).
    FramCorrectableBit,
    /// A reserved or unrecognized `SYSSNIV` value, preserved raw.
    Unknown(u16),
}

impl NmiSource {
    /// Decode one raw `SYSSNIV` value. `None` only for `0` ("no source
    /// pending").
    pub fn from_iv(iv: u16) -> Option<NmiSource> {
        Some(match iv {
            0x00 => return None,
            0x04 => NmiSource::FramUncorrectableBit,
            0x08 => NmiSource::MpuSegIp,
            0x0A => NmiSource::MpuSegInfo,
            0x0C => NmiSource::MpuSeg1,
            0x0E => NmiSource::MpuSeg2,
            0x10 => NmiSource::MpuSeg3,
            0x12 => NmiSource::VacantMemory,
            0x14 => NmiSource::JtagMailboxIn,
            0x16 => NmiSource::JtagMailboxOut,
            0x18 => NmiSource::FramCorrectableBit,
            other => NmiSource::Unknown(other),
        })
    }

    /// Short human-readable name, for UART reporting without `core::fmt`.
    pub fn as_str(&self) -> &'static str {
        match self {
            NmiSource::FramUncorrectableBit => "FRAM uncorrectable bit",
            NmiSource::MpuSegIp => "MPU seg IP",
            NmiSource::MpuSegInfo => "MPU seg info",
            NmiSource::MpuSeg1 => "MPU seg 1",
            NmiSource::MpuSeg2 => "MPU seg 2",
            NmiSource::MpuSeg3 => "MPU seg 3",
            NmiSource::VacantMemory => "vacant memory",
            NmiSource::JtagMailboxIn => "JTAG mailbox in",
            NmiSource::JtagMailboxOut => "JTAG mailbox out",
            NmiSource::FramCorrectableBit => "FRAM correctable bit",
            NmiSource::Unknown(_) => "unknown",
        }
    }
}

/// ISR-side: read-and-consume the highest-priority pending system-NMI source
/// from `SYSSNIV`, for the `SYSNMI` vector handler.
///
/// Same IV convention as `SYSRSTIV`/`RTCIV`: priority-encoded,
/// consume-on-read, `0` when nothing is pending — loop until `None` to drain
/// multiple sources. **For MPU sources the consume applies to the vector
/// slot, not necessarily the `MPUCTL1` flag** — SLAU367 leaves that
/// interaction unspecified, so the handler must also call
/// [`crate::mpu::clear_violation_flags`] or the source stays pending. Free
/// function backed by `steal()`, per the ISR convention: driver structs own
/// their peripherals, handlers use module-level accessors.
pub fn read_nmi_iv() -> Option<NmiSource> {
    let sys = unsafe { pac::Sys::steal() };
    NmiSource::from_iv(sys.syssniv().read().bits())
}
