//! Non-volatile storage in the MSP430FR5969's FRAM.
//!
//! FRAM (ferroelectric RAM) is the defining feature of this part: non-volatile
//! like flash, but **written like RAM**. A write is a single `MOV` to memory —
//! there is no erase step, no unlock/password sequence, no program-then-verify,
//! and no busy flag to poll. Write time equals read time, and endurance is
//! ~10^15 cycles (datasheet SLAS704G Table 5-34). ECC and the small read cache
//! are handled transparently by the FRAM controller. That makes FRAM a near
//! perfect fit for the byte-granular [`embedded_storage`] traits, which this
//! module implements.
//!
//! # Two regions, two backends
//!
//! The FR5969's 63 KB of main FRAM is one contiguous block, `0x4400..=0x13FFF`,
//! that **straddles the 16-bit address boundary** (datasheet Table 6-6). That
//! split is the whole story of this module:
//!
//! - [`InfoFram`] — the **Information FRAM** at `0x1800..=0x19FF` (512 B). This
//!   is ordinary low memory, reached with normal 16-bit pointers and volatile
//!   loads/stores. It is TI's intended home for persistent configuration and
//!   calibration, and (like the upper region) sits outside every linker
//!   `MEMORY` region, so using it can never clobber code or data.
//!
//! - [`HighFram`] — the **upper 16 KB** at `0x10000..=0x13FFF`: the storage
//!   "beyond the first 48 K." This address is **above the 16-bit address
//!   space**, so no Rust pointer (`usize`/`*mut T` are 16-bit on
//!   `msp430-none-elf`) can name it. See below.
//!
//! Both backends implement [`embedded_storage::ReadStorage`] and
//! [`embedded_storage::Storage`]; `offset` is relative to the start of each
//! region.
//!
//! # Reaching the upper 16 KB (the 20-bit problem)
//!
//! The MSP430**X** CPU in this chip has a 20-bit address space, but two things
//! conspire to hide the top of FRAM from Rust:
//!
//! 1. rustc's `msp430-none-elf` target emits **baseline MSP430** code, where
//!    pointers and `usize` are 16 bits. `0x10000` does not fit in a pointer.
//! 2. LLVM's MSP430 backend defines **no MSP430X instructions**, so even inline
//!    `asm!` cannot use the `MOVX`/`MOVA` mnemonics that perform 20-bit access —
//!    the integrated assembler rejects them.
//!
//! TI's C toolchain solves the identical problem with the `__data20_read_char` /
//! `__data20_write_char` runtime intrinsics (the "large data model"), which are
//! just hand-written MSP430X instruction sequences. We do the same: the byte
//! primitives below emit the 20-bit instructions as raw `.word` opcodes inside
//! an `asm!` block, pinned to explicit registers. The opcodes were verified
//! against the TI assembler (`msp430-elf-as -mcpu=msp430x`) — see the comments
//! on [`read_high_byte`].
//!
//! Wait states (`FRCTL0.NWAITS`) stay at their reset default of 0 because this
//! project runs MCLK at 1 MHz (they are only needed above 8 MHz), and the MPU is
//! disabled out of reset, so all of FRAM is writable. This module therefore
//! never touches the FRAM controller — which is just as well, since the PAC's
//! generated `frctl0().write()` would zero the mandatory `0xA5` password byte
//! and trigger a PUC.
//!
//! # Example: a reset-surviving boot counter in Info FRAM
//!
//! ```ignore
//! use hal::fram::InfoFram;
//! use hal::embedded_storage::{ReadStorage, Storage};
//!
//! let mut info = InfoFram::new();
//! let mut buf = [0u8; 4];
//! info.read(0, &mut buf).unwrap();
//! let mut count = u32::from_le_bytes(buf);
//! count = count.wrapping_add(1);
//! info.write(0, &count.to_le_bytes()).unwrap(); // persists across power-cycles
//! ```

use crate::fram_addr::{check_bounds, FramError, HIGH_BASE, HIGH_CAPACITY, INFO_BASE, INFO_CAPACITY};
use embedded_storage::{ReadStorage, Storage};

pub use crate::fram_addr::FramError as Error;

/// Byte-addressable storage backed by the 512 B of Information FRAM
/// (`0x1800..=0x19FF`), reached with ordinary 16-bit pointers.
///
/// A zero-sized handle. Construct one with [`InfoFram::new`]. Holding more than
/// one at a time is not unsafe (accesses are volatile byte loads/stores) but is
/// logically a shared-mutable-resource hazard — treat it as a singleton.
#[derive(Debug, Default)]
pub struct InfoFram {
    _private: (),
}

impl InfoFram {
    /// Create a handle to the Information FRAM region.
    pub const fn new() -> Self {
        InfoFram { _private: () }
    }
}

impl ReadStorage for InfoFram {
    type Error = FramError;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_bounds(offset, bytes.len(), INFO_CAPACITY)?;
        let base = INFO_BASE + offset as u16; // offset < 512 after the bounds check
        for (i, b) in bytes.iter_mut().enumerate() {
            let addr = (base + i as u16) as *const u8;
            // SAFETY: `addr` is within Info FRAM (bounds-checked above), which is
            // valid, byte-readable memory outside any linker region. Volatile so
            // the read is not elided or reordered around a prior write.
            *b = unsafe { core::ptr::read_volatile(addr) };
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        INFO_CAPACITY
    }
}

impl Storage for InfoFram {
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_bounds(offset, bytes.len(), INFO_CAPACITY)?;
        let base = INFO_BASE + offset as u16;
        for (i, &b) in bytes.iter().enumerate() {
            let addr = (base + i as u16) as *mut u8;
            // SAFETY: `addr` is within Info FRAM (bounds-checked), writable, and
            // not used by the linker for code/data. A FRAM store is a plain MOV;
            // no erase/unlock is required. Volatile so it is actually performed.
            unsafe { core::ptr::write_volatile(addr, b) };
        }
        Ok(())
    }
}

/// Byte-addressable storage backed by the upper 16 KB of FRAM
/// (`0x10000..=0x13FFF`) — the region "beyond the first 48 K," reached with
/// hand-emitted MSP430X 20-bit instructions.
///
/// A zero-sized handle; see [`HighFram::new`]. Access is one byte per `asm!`
/// block, so this is slower than [`InfoFram`]; it is meant for bulk persistent
/// data rather than hot loops.
#[derive(Debug, Default)]
pub struct HighFram {
    _private: (),
}

impl HighFram {
    /// Create a handle to the upper-FRAM region.
    pub const fn new() -> Self {
        HighFram { _private: () }
    }
}

impl ReadStorage for HighFram {
    type Error = FramError;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_bounds(offset, bytes.len(), HIGH_CAPACITY)?;
        let base = offset as u16; // offset < 16384 after the bounds check
        for (i, b) in bytes.iter_mut().enumerate() {
            // SAFETY: region-relative offset is in 0..16384 (bounds-checked), so
            // the absolute 20-bit address 0x10000 + off is within upper FRAM.
            *b = unsafe { read_high_byte(base + i as u16) };
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        HIGH_CAPACITY
    }
}

impl Storage for HighFram {
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_bounds(offset, bytes.len(), HIGH_CAPACITY)?;
        let base = offset as u16;
        for (i, &b) in bytes.iter().enumerate() {
            // SAFETY: as in `read`; absolute address is within upper FRAM.
            unsafe { write_high_byte(base + i as u16, b) };
        }
        Ok(())
    }
}

// The 20-bit base baked into the `MOVA #0x10000` immediate below must match the
// region constant; assert it at compile time so the two cannot drift apart.
const _: () = assert!(HIGH_BASE == 0x1_0000);

/// Read one byte from upper FRAM at `offset` (0-based within the 16 KB region).
///
/// The address arithmetic and access are done entirely in MSP430X 20-bit
/// instructions, emitted as raw opcodes because LLVM's assembler has no
/// `MOVX`/`MOVA` mnemonics. Register allocation is pinned: R12 is the 20-bit
/// address scratch, R13 carries `offset` in and the result byte out.
///
/// Opcodes (confirmed with `msp430-elf-as -mcpu=msp430x`):
/// ```text
/// 018C 0000   MOVA   #0x10000, R12   ; R12 = 0x10000 (20-bit base)
/// 0DEC        ADDA   R13, R12        ; R12 += offset  -> absolute address
/// 1840 4C6D   MOVX.B @R12, R13       ; R13 = [R12] (byte; upper bits cleared)
/// ```
///
/// # Safety
/// `offset` must be `< 16384` so the absolute address stays within upper FRAM.
#[inline]
unsafe fn read_high_byte(offset: u16) -> u8 {
    let out: u16;
    core::arch::asm!(
        ".word 0x018C", // MOVA #0x10000, R12
        ".word 0x0000", //   (immediate low word)
        ".word 0x0DEC", // ADDA R13, R12
        ".word 0x1840", // MOVX.B extension word (A/L+B/W = byte)
        ".word 0x4C6D", // MOVX.B @R12, R13
        inout("r13") offset => out,
        out("r12") _,
        options(nostack),
    );
    out as u8
}

/// Write one byte `val` to upper FRAM at `offset` (0-based within the region).
///
/// Same scheme as [`read_high_byte`]: R12 is the address scratch, R13 carries
/// `offset` in, R14 carries the value. A FRAM store is a plain (extended) `MOV`
/// — no erase or unlock.
///
/// Opcodes (confirmed with `msp430-elf-as -mcpu=msp430x`):
/// ```text
/// 018C 0000   MOVA   #0x10000, R12     ; R12 = 0x10000
/// 0DEC        ADDA   R13, R12          ; R12 += offset
/// 1840 4ECC 0000  MOVX.B R14, 0(R12)   ; [R12] = R14 (low byte), index 0
/// ```
///
/// # Safety
/// `offset` must be `< 16384` so the absolute address stays within upper FRAM.
#[inline]
unsafe fn write_high_byte(offset: u16, val: u8) {
    core::arch::asm!(
        ".word 0x018C", // MOVA #0x10000, R12
        ".word 0x0000", //   (immediate low word)
        ".word 0x0DEC", // ADDA R13, R12
        ".word 0x1840", // MOVX.B extension word (A/L+B/W = byte)
        ".word 0x4ECC", // MOVX.B R14, 0(R12)
        ".word 0x0000", //   (index for 0(R12))
        in("r13") offset,
        in("r14") val as u16,
        out("r12") _,
        options(nostack),
    );
}
