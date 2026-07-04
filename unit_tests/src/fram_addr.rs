//! Host-side tests for the FRAM addressing / bounds math.
//!
//! This module `include!`s the REAL source (`hal/src/fram_addr.rs`) so the tests
//! exercise the exact code that ships in firmware — a bad edit to `check_bounds`
//! (e.g. dropping the `u64` widening), or to a region base/size constant, will
//! fail these tests. The hardware access paths in `fram.rs` cannot run on the
//! host; the region geometry they depend on is pinned here instead.

// Pull in the actual addressing math (pure, dependency-free core arithmetic).
include!("../../hal/src/fram_addr.rs");

#[cfg(test)]
mod tests {
    use super::*;

    /// The documented region geometry (datasheet Table 6-6). These constants are
    /// what `fram.rs` and the hand-written 20-bit `asm!` rely on, so pin them.
    #[test]
    fn region_geometry_matches_datasheet() {
        // Info FRAM: Info D..A, 0x1800..=0x19FF, 512 bytes.
        assert_eq!(INFO_BASE, 0x1800);
        assert_eq!(INFO_CAPACITY, 512);
        assert_eq!(INFO_BASE as usize + INFO_CAPACITY - 1, 0x19FF);

        // Upper FRAM ("beyond 48K"): 0x10000..=0x13FFF, 16 KB.
        assert_eq!(HIGH_BASE, 0x1_0000);
        assert_eq!(HIGH_CAPACITY, 16_384);
        assert_eq!(HIGH_BASE as usize + HIGH_CAPACITY - 1, 0x1_3FFF);

        // The upper region genuinely lives above the 16-bit address space — that
        // is the entire reason it needs MSP430X 20-bit access.
        assert!(HIGH_BASE > 0xFFFF);
    }

    /// `check_bounds` accepts in-range accesses and the empty access at the end.
    #[test]
    fn check_bounds_accepts_in_range() {
        assert_eq!(check_bounds(0, 0, INFO_CAPACITY), Ok(()));
        assert_eq!(check_bounds(0, INFO_CAPACITY, INFO_CAPACITY), Ok(())); // whole region
        assert_eq!(check_bounds(511, 1, INFO_CAPACITY), Ok(())); // last byte
        assert_eq!(check_bounds(512, 0, INFO_CAPACITY), Ok(())); // empty at the end
        assert_eq!(check_bounds(0, HIGH_CAPACITY, HIGH_CAPACITY), Ok(()));
    }

    /// `check_bounds` rejects accesses that spill past the end by even one byte.
    #[test]
    fn check_bounds_rejects_overrun() {
        assert_eq!(check_bounds(0, 513, INFO_CAPACITY), Err(FramError::OutOfBounds));
        assert_eq!(check_bounds(511, 2, INFO_CAPACITY), Err(FramError::OutOfBounds));
        assert_eq!(check_bounds(512, 1, INFO_CAPACITY), Err(FramError::OutOfBounds));
        assert_eq!(
            check_bounds(0, HIGH_CAPACITY + 1, HIGH_CAPACITY),
            Err(FramError::OutOfBounds)
        );
    }

    /// Regression guard for the `u64` widening: a near-`u32::MAX` offset plus a
    /// length must not wrap a 32-bit intermediate into a false pass. With honest
    /// widening these are firmly out of bounds.
    #[test]
    fn check_bounds_does_not_overflow() {
        assert_eq!(check_bounds(u32::MAX, 1, INFO_CAPACITY), Err(FramError::OutOfBounds));
        // offset + len = 0x1_0000_0000, which is 0 in u32 — the classic wrap that
        // would falsely report "in bounds" without the u64 widening.
        assert_eq!(
            check_bounds(u32::MAX, 1, HIGH_CAPACITY),
            Err(FramError::OutOfBounds)
        );
        assert_eq!(check_bounds(u32::MAX - 10, 100, HIGH_CAPACITY), Err(FramError::OutOfBounds));
    }

    /// The absolute upper-FRAM address an `offset` maps to (what the `asm!`
    /// computes as `MOVA #0x10000` + `ADDA offset`) must land inside the region.
    #[test]
    fn high_offset_maps_into_region() {
        assert_eq!(HIGH_BASE + 0, 0x1_0000); // first byte
        assert_eq!(HIGH_BASE + (HIGH_CAPACITY as u32 - 1), 0x1_3FFF); // last byte
    }
}
