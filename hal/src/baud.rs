// eUSCI_A baud-rate generator math (SLAU367P §30.3.10).
//
// This module is intentionally dependency-free (pure `core` integer
// arithmetic, no PAC/HAL types) so that the exact same source can be
// `include!`d by the host-side test crate in `baud-test/` and checked against
// the datasheet's recommended-settings table. Do not add external `use`s here.
// (Regular `//` comments, not `//!`, so the file can be `include!`d mid-crate.)

/// Computed baud-rate generator register values.
pub(crate) struct BaudRegs {
    pub(crate) ucbr: u16,
    pub(crate) ucbrf: u8,
    pub(crate) ucbrs: u8,
    pub(crate) oversampling: bool,
}

/// `UCBRSx` second-stage modulation lookup (SLAU367P Table 30-4).
///
/// Each entry is `(fractional_part_x10000, UCBRSx)`. The correct setting is the
/// value of the last entry whose threshold is `<=` the fractional part of N.
pub(crate) const UCBRS_TABLE: [(u32, u8); 36] = [
    (0, 0x00),
    (529, 0x01),
    (715, 0x02),
    (835, 0x04),
    (1001, 0x08),
    (1252, 0x10),
    (1430, 0x20),
    (1670, 0x11),
    (2147, 0x21),
    (2224, 0x22),
    (2503, 0x44),
    (3000, 0x25),
    (3335, 0x49),
    (3575, 0x4A),
    (3753, 0x52),
    (4003, 0x92),
    (4286, 0x53),
    (4378, 0x55),
    (5002, 0xAA),
    (5715, 0x6B),
    (6003, 0xAD),
    (6254, 0xB5),
    (6432, 0xB6),
    (6667, 0xD6),
    (7001, 0xB7),
    (7147, 0xBB),
    (7503, 0xDD),
    (7861, 0xED),
    (8004, 0xEE),
    (8333, 0xBF),
    (8464, 0xDF),
    (8572, 0xEF),
    (8751, 0xF7),
    (9004, 0xFB),
    (9170, 0xFD),
    (9288, 0xFE),
];

/// Look up the `UCBRSx` modulation byte for a fractional part scaled by 10000.
pub(crate) fn ucbrs_lookup(frac_x10000: u32) -> u8 {
    let mut result = 0u8;
    let mut i = 0;
    while i < UCBRS_TABLE.len() {
        if UCBRS_TABLE[i].0 <= frac_x10000 {
            result = UCBRS_TABLE[i].1;
        } else {
            break;
        }
        i += 1;
    }
    result
}

/// Compute the baud-rate generator settings (SLAU367P §30.3.10).
///
/// `N = f_BRCLK / baud`. If `N >= 16`, oversampling mode is used; otherwise
/// low-frequency mode. The fractional part of `N` selects `UCBRSx` from
/// [`UCBRS_TABLE`].
pub(crate) fn compute_baud(clock_freq: u32, baud: u32) -> BaudRegs {
    let n_int = clock_freq / baud; // INT(N)
    // Fractional part of N, scaled by 10000: (clock_freq - n_int*baud)/baud.
    let frac_x10000 = (((clock_freq - n_int * baud) as u64) * 10_000 / baud as u64) as u32;
    let ucbrs = ucbrs_lookup(frac_x10000);

    if n_int >= 16 {
        // Oversampling: UCBRx = INT(N/16), UCBRFx = INT(frac(N/16) * 16).
        let ucbr = clock_freq / (16 * baud);
        let ucbrf = (n_int - 16 * ucbr) as u8; // = INT(N) - 16*UCBRx, in 0..15
        BaudRegs {
            ucbr: ucbr as u16,
            ucbrf,
            ucbrs,
            oversampling: true,
        }
    } else {
        // Low-frequency: UCBRx = INT(N), modulation handled entirely by UCBRSx.
        BaudRegs {
            ucbr: n_int as u16,
            ucbrf: 0,
            ucbrs,
            oversampling: false,
        }
    }
}
