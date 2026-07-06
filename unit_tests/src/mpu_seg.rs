//! Host-side tests for `hal/src/mpu_seg.rs` — the FRAM-MPU segmentation math
//! (border address ↔ `MPUSEGBx` register conversion, `MPUSAM` nibble
//! composition, segment lookup) that `hal::mpu` delegates to.
//!
//! Anchors: the border encoding is pinned to TI's own example values
//! (`msp430fr59xx_mpu_01.c` writes `MPUSEGB1 = 0x0480` for a border at
//! `0x4800`), and the all-permissions `MPUSAM` composition is pinned to
//! `0x7777`, the register's documented reset value (SLAU367P) — so a
//! transposed nibble or a shifted border can't survive these tests.

include!("../../hal/src/mpu_seg.rs");

// ---------------------------------------------------------------- borders

#[test]
fn border_encoding_matches_ti_example() {
    // msp430fr59xx_mpu_01.c: "MPUSEGB1 = 0x0480; // B1 = 0x4800"
    assert_eq!(addr_to_border(0x4800), Ok(0x0480));
    assert_eq!(addr_to_border(0x4C00), Ok(0x04C0));
}

#[test]
fn border_encoding_key_addresses() {
    // Bottom of main FRAM (segment 1 empty).
    assert_eq!(addr_to_border(MAIN_START), Ok(0x0440));
    // The bank split — the natural "protect HighFram" border.
    assert_eq!(addr_to_border(0x1_0000), Ok(0x1000));
    // Top of main FRAM (segment 3 empty).
    assert_eq!(addr_to_border(MAIN_END), Ok(0x1400));
}

#[test]
fn border_round_trips() {
    for addr in [0x4400, 0x4800, 0x8000, 0xFF80, 0x1_0000, 0x1_3FF0, 0x1_4000] {
        let b = addr_to_border(addr).unwrap();
        assert_eq!(border_to_addr(b), addr, "round trip failed for {addr:#x}");
    }
}

#[test]
fn border_rejects_misalignment() {
    // The register stores addr >> 4; any of the low four bits set is
    // unrepresentable and must be rejected, not truncated.
    assert_eq!(addr_to_border(0x4401), Err(MpuError::Misaligned));
    assert_eq!(addr_to_border(0x480F), Err(MpuError::Misaligned));
    assert_eq!(addr_to_border(0x1_0008), Err(MpuError::Misaligned));
}

#[test]
fn border_rejects_out_of_range() {
    // Below main FRAM (RAM, peripherals, info memory)...
    assert_eq!(addr_to_border(0x43F0), Err(MpuError::OutOfRange));
    assert_eq!(addr_to_border(0x1800), Err(MpuError::OutOfRange));
    assert_eq!(addr_to_border(0), Err(MpuError::OutOfRange));
    // ...and past the top (both ends of MAIN range are themselves allowed).
    assert_eq!(addr_to_border(0x1_4010), Err(MpuError::OutOfRange));
}

#[test]
fn alignment_checked_before_range() {
    // An address failing both checks reports Misaligned (the & 0xF test runs
    // first); pin the precedence so error messages stay stable.
    assert_eq!(addr_to_border(0x0001), Err(MpuError::Misaligned));
}

// ---------------------------------------------------------------- MPUSAM

#[test]
fn sam_all_rwx_is_reset_value() {
    // SLAU367P: MPUSAM resets to 0x7777 (everything readable, writable,
    // executable, violations flag-only).
    let rwx = Access::rwx();
    assert_eq!(sam_value(rwx, rwx, rwx, rwx), 0x7777);
}

#[test]
fn sam_nibble_bit_positions() {
    // One bit at a time, in segment-1 position: RE=1, WE=2, XE=4, VS=8.
    let none = Access::none();
    let r = Access { read: true, ..none };
    let w = Access { write: true, ..none };
    let x = Access { execute: true, ..none };
    let v = none.reset_on_violation();
    assert_eq!(sam_value(r, none, none, none), 0x0001);
    assert_eq!(sam_value(w, none, none, none), 0x0002);
    assert_eq!(sam_value(x, none, none, none), 0x0004);
    assert_eq!(sam_value(v, none, none, none), 0x0008);
}

#[test]
fn sam_segment_nibble_positions() {
    // The same access setting walks the four nibbles: seg1 bits 3:0,
    // seg2 7:4, seg3 11:8, info 15:12.
    let rwx = Access::rwx();
    let none = Access::none();
    assert_eq!(sam_value(rwx, none, none, none), 0x0007);
    assert_eq!(sam_value(none, rwx, none, none), 0x0070);
    assert_eq!(sam_value(none, none, rwx, none), 0x0700);
    assert_eq!(sam_value(none, none, none, rwx), 0x7000);
}

#[test]
fn sam_write_protect_seg3() {
    // The fixture's cold-phase config: everything default except segment 3
    // loses write. 0x7777 with the seg3 nibble dropping WE (2): 0x7577.
    assert_eq!(
        sam_value(Access::rwx(), Access::rwx(), Access::rx(), Access::rwx()),
        0x7577
    );
    // Same but violations reset the chip (VS=8 joins the seg3 nibble): 0x7D77.
    assert_eq!(
        sam_value(
            Access::rwx(),
            Access::rwx(),
            Access::rx().reset_on_violation(),
            Access::rwx()
        ),
        0x7D77
    );
}

#[test]
fn access_constructors() {
    assert_eq!(Access::rwx().nibble(), 0x7);
    assert_eq!(Access::rx().nibble(), 0x5);
    assert_eq!(Access::read_only().nibble(), 0x1);
    assert_eq!(Access::none().nibble(), 0x0);
    assert_eq!(Access::rx().reset_on_violation().nibble(), 0xD);
}

// ---------------------------------------------------------------- lookup

#[test]
fn segment_lookup_edges() {
    // A border byte belongs to the higher segment.
    let (b1, b2) = (0x8000, 0x1_0000);
    assert_eq!(segment_containing(b1, b2, MAIN_START), 1);
    assert_eq!(segment_containing(b1, b2, 0x7FFF), 1);
    assert_eq!(segment_containing(b1, b2, 0x8000), 2);
    assert_eq!(segment_containing(b1, b2, 0xFFFF), 2);
    assert_eq!(segment_containing(b1, b2, 0x1_0000), 3);
    assert_eq!(segment_containing(b1, b2, 0x1_3FFF), 3);
}

#[test]
fn segment_lookup_empty_segments() {
    // Equal borders: segment 2 vanishes, everything is 1 or 3.
    assert_eq!(segment_containing(0x1_0000, 0x1_0000, 0xFFFF), 1);
    assert_eq!(segment_containing(0x1_0000, 0x1_0000, 0x1_0000), 3);
    // Both borders at the bottom: everything is segment 3.
    assert_eq!(segment_containing(MAIN_START, MAIN_START, MAIN_START), 3);
    // Both at the top: everything is segment 1.
    assert_eq!(segment_containing(MAIN_END, MAIN_END, 0x1_3FFF), 1);
}
