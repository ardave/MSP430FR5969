// Pure FRAM addressing / bounds arithmetic for the `fram` module.
//
// Like `baud.rs` and `ticks.rs`, this file is intentionally dependency-free
// (pure `core` integer arithmetic, no PAC/HAL types) so the exact same source
// can be `include!`d by the host-side test crate in `unit_tests/`. The `InfoFram`
// and `HighFram` storage backends are thin wrappers over these constants and
// `check_bounds`, so a regression here (e.g. dropping the overflow widening, or
// fat-fingering a region size) fails the host tests. Do not add external `use`s.
// (Regular `//` comments, not `//!`, so the file can be `include!`d mid-crate.)

/// Error type for the FRAM storage backends. The only failure a plain FRAM
/// access can have is an out-of-range offset/length — the write itself never
/// fails (FRAM has no erase step, no program/verify cycle, no busy flag).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FramError {
    /// The requested `offset + len` would run past the end of the region.
    OutOfBounds,
}

/// Base address of Information FRAM (Info D..A occupy 0x1800..=0x19FF).
///
/// This 512-byte block sits *below* every `MEMORY` region in `pac/memory.x`
/// (RAM starts at 0x1C00), so the linker never places code or data here — it is
/// free for use and writing to it cannot corrupt the running program. TLV /
/// factory calibration lives separately at 0x1A00 and is never touched.
pub(crate) const INFO_BASE: u16 = 0x1800;

/// Size of the Information FRAM block in bytes (4 segments x 128 B).
pub(crate) const INFO_CAPACITY: usize = 512;

/// Base address of the upper FRAM region — the "beyond 48K" memory at
/// 0x10000..=0x13FFF. It lives above the 16-bit address space, so it is
/// unreachable by an ordinary 16-bit Rust pointer; `fram::HighFram` reaches it
/// with hand-emitted MSP430X 20-bit instructions whose immediate must equal
/// this constant.
pub(crate) const HIGH_BASE: u32 = 0x1_0000;

/// Size of the upper FRAM region in bytes (16 KB).
pub(crate) const HIGH_CAPACITY: usize = 0x4000;

/// Reject an access whose `offset + len` runs past `capacity`.
///
/// The sum is computed in `u64` so it cannot wrap: `offset` is the trait's
/// `u32` and `len` a `usize`, and on a 16-bit target a near-`u32::MAX` offset
/// plus a length would overflow a `u32` intermediate and *falsely pass* the
/// bound. Widening first makes the check honest (mirrors the `u64` guard in
/// `ticks.rs`). An empty access (`len == 0`) at `offset == capacity` is allowed.
pub(crate) fn check_bounds(offset: u32, len: usize, capacity: usize) -> Result<(), FramError> {
    let end = offset as u64 + len as u64;
    if end > capacity as u64 {
        Err(FramError::OutOfBounds)
    } else {
        Ok(())
    }
}
