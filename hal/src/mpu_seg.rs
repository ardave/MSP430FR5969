// Pure FRAM-MPU segmentation math for the `mpu` module.
//
// Like `fram_addr.rs` and `comp_ladder.rs`, this file is intentionally
// dependency-free (pure `core` integer arithmetic, no PAC/HAL types) so the
// exact same source can be `include!`d by the host-side test crate in
// `unit_tests/`. The `mpu::Mpu` driver is a thin wrapper over these
// conversions — a regression here (a shifted border, a transposed MPUSAM
// nibble) fails the host tests without a board on the desk. Do not add
// external `use`s. (Regular `//` comments, not `//!`, so the file can be
// `include!`d mid-crate.)
//
// # The hardware's segmentation model (SLAU367P chapter 10)
//
// The MPU carves the *main* FRAM (`0x4400..=0x13FFF` on the FR5969 — both
// banks, straddling the 16-bit boundary) into three contiguous segments with
// two movable borders, plus one fixed segment for the Information FRAM:
//
// ```text
//   0x4400 ──── MMSEG1 ──── B1 ──── MMSEG2 ──── B2 ──── MMSEG3 ──── 0x13FFF
//   0x1800 ─────────────── info segment ─────────────────────────── 0x19FF
// ```
//
// A border register holds the border address right-shifted by 4 (the
// registers are 16 bits wide but must name 20-bit addresses; 16-byte
// granularity is what falls out). Each segment gets a 4-bit nibble in
// `MPUSAM`: read / write / execute enables plus a "violation select" that
// chooses between latching a flag (optionally escalated to a system NMI) and
// resetting the chip with a PUC.

/// First byte of main FRAM — where segment 1 begins.
pub const MAIN_START: u32 = 0x4400;

/// One past the last byte of main FRAM (`0x13FFF`) — where segment 3 ends.
/// A border placed here makes segment 3 empty, just as a border at
/// [`MAIN_START`] makes segment 1 empty.
pub const MAIN_END: u32 = 0x1_4000;

/// Errors from MPU segmentation setup.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MpuError {
    /// A border address is not 16-byte aligned. `MPUSEGBx` stores the address
    /// `>> 4`, so the low four bits simply cannot be represented — rejecting
    /// them is honest where silently truncating would protect (or expose!) up
    /// to 15 bytes the caller didn't ask about.
    Misaligned,
    /// A border address lies outside main FRAM (`0x4400..=0x14000`). The
    /// hardware only ever compares main-memory accesses against the borders,
    /// so a border outside that range is a confused caller, not a
    /// configuration.
    OutOfRange,
    /// `border1 > border2`, which would give segment 2 negative size. (Equal
    /// borders — an empty segment 2 — are fine.)
    BordersReversed,
    /// `MPUCTL0.MPULOCK` is set: the MPU registers are frozen until the next
    /// BOR-class reset. Produced by the driver, never by the pure math.
    Locked,
}

/// What a segment violation does, beyond being latched in `MPUCTL1`
/// (the `MPUSEGxVS` bit of the segment's `MPUSAM` nibble).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Violation {
    /// Latch the segment's `MPUCTL1` flag; if NMI escalation is enabled
    /// (`MPUSEGIE`, [`Config::nmi`] in the driver), also fire the `SYSNMI`
    /// vector. The violating access itself is still suppressed either way.
    #[default]
    Flag,
    /// Reset the chip with a PUC. After the reboot `SYSRSTIV` reports which
    /// segment fired (`ResetReason::MpuSeg1/2/3/Info` in `sys`). The
    /// production posture for "firmware must never do this" regions.
    Puc,
}

/// Access rights for one MPU segment — one `MPUSAM` nibble.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Access {
    /// Reads allowed (`MPUSEGxRE`). A blocked read returns bus garbage, not
    /// data — but it is *suppressed*, latched, and reported like any other
    /// violation.
    pub read: bool,
    /// Writes allowed (`MPUSEGxWE`). A blocked write is silently ignored by
    /// the memory (FRAM keeps its contents) and latched as a violation.
    pub write: bool,
    /// Instruction fetches allowed (`MPUSEGxXE`). Removing execute from the
    /// segment the CPU is running from is a self-lockout, not an error the
    /// math can catch — see the driver docs.
    pub execute: bool,
    /// Flag-and-optionally-NMI, or PUC.
    pub violation: Violation,
}

impl Access {
    /// Full access — the hardware's reset posture for every segment.
    pub const fn rwx() -> Access {
        Access { read: true, write: true, execute: true, violation: Violation::Flag }
    }

    /// Read + execute, no write: the classic "protect the firmware image"
    /// setting for code segments.
    pub const fn rx() -> Access {
        Access { read: true, write: false, execute: true, violation: Violation::Flag }
    }

    /// Read only: for constant data that should never be executed or
    /// overwritten.
    pub const fn read_only() -> Access {
        Access { read: true, write: false, execute: false, violation: Violation::Flag }
    }

    /// No access at all.
    pub const fn none() -> Access {
        Access { read: false, write: false, execute: false, violation: Violation::Flag }
    }

    /// Same rights, but a violation resets the chip (PUC) instead of latching
    /// a flag. Builder-style: `Access::rx().reset_on_violation()`.
    pub const fn reset_on_violation(mut self) -> Access {
        self.violation = Violation::Puc;
        self
    }

    // The segment's 4-bit MPUSAM nibble: RE bit 0, WE bit 1, XE bit 2, VS
    // bit 3 (SLAU367P table "MPUSAM Register Description").
    pub(crate) const fn nibble(&self) -> u16 {
        (self.read as u16)
            | (self.write as u16) << 1
            | (self.execute as u16) << 2
            | (matches!(self.violation, Violation::Puc) as u16) << 3
    }
}

/// Convert a border address to its `MPUSEGBx` register value (`addr >> 4`).
///
/// The address must be 16-byte aligned (the register cannot represent finer)
/// and within main FRAM `0x4400..=0x14000` — both ends inclusive, because a
/// border *at* either end is the legitimate way to make the outer segment
/// empty.
pub(crate) fn addr_to_border(addr: u32) -> Result<u16, MpuError> {
    if addr & 0xF != 0 {
        return Err(MpuError::Misaligned);
    }
    if addr < MAIN_START || addr > MAIN_END {
        return Err(MpuError::OutOfRange);
    }
    Ok((addr >> 4) as u16)
}

/// The border address a raw `MPUSEGBx` register value names (`value << 4`).
pub(crate) fn border_to_addr(border: u16) -> u32 {
    (border as u32) << 4
}

/// Compose the full 16-bit `MPUSAM` value from the four segment nibbles.
/// Segment 1 sits in bits 3:0, segment 2 in 7:4, segment 3 in 11:8, and the
/// info segment in 15:12. Sanity anchor: four `Access::rwx()` gives `0x7777`,
/// the register's documented reset value.
pub(crate) fn sam_value(seg1: Access, seg2: Access, seg3: Access, info: Access) -> u16 {
    seg1.nibble() | seg2.nibble() << 4 | seg3.nibble() << 8 | info.nibble() << 12
}

/// Which main-memory segment (1, 2, or 3) contains `addr`, given the two
/// border addresses. Borders belong to the higher segment: segment 1 is
/// `[MAIN_START, b1)`, segment 2 `[b1, b2)`, segment 3 `[b2, MAIN_END)` —
/// SLAU367P: "the border address is the first address of the next segment".
/// Useful for predicting which violation flag an access will raise; `addr`
/// is assumed to be within main FRAM.
pub fn segment_containing(border1: u32, border2: u32, addr: u32) -> u8 {
    if addr < border1 {
        1
    } else if addr < border2 {
        2
    } else {
        3
    }
}
