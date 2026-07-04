//! TLV — the factory device-descriptor table, and the ADC/REF calibration
//! constants stored in it.
//!
//! Every FR59xx die is measured on the production tester and the results are
//! burned into a reserved 256-byte FRAM window at **0x1A00–0x1AFF** as a
//! **T**ag-**L**ength-**V**alue table (SLAU367 §1.13): after an 8-byte info
//! block (CRC, device ID, revisions), entries of `[tag: u8, length: u8,
//! data: length bytes]` follow from **0x1A08**, terminated by tag `0xFF` or
//! the end of the window. Walking by tag — rather than hardcoding addresses —
//! is the documented procedure, and degrades to `None` instead of garbage if
//! a future die revision shuffles the layout.
//!
//! Two entries matter to the ADC path:
//!
//! - **Tag `0x11`, ADC12 calibration** (8 words): the gain factor and offset
//!   of this specific converter, then the **measured temperature-sensor
//!   readings at 30 °C and 85 °C** for each reference voltage (1.2/2.0/2.5 V
//!   pairs, in that order). Two known (temperature → counts) points define the
//!   sensor's line; interpolation between them is the entire thermometer.
//! - **Tag `0x12`, REF calibration** (3 words): the measured-vs-nominal ratio
//!   of each REF_A output as a **1.15 fixed-point** factor (`2^15` = exactly
//!   nominal). A "1.2 V" reference that actually produces 1.194 V stores
//!   ≈ `32604`; multiplying a result by `factor / 2^15` cancels the deviation.
//!
//! Because the constants were measured *through this chip's own ADC*, the
//! temperature pairs already embed its gain/offset error — the temperature
//! interpolation therefore uses **raw** readings, while absolute-voltage
//! corrections chain gain → offset → REF factor explicitly (the methods here
//! delegate to the pure math in `adc_cal.rs`, host-tested in `unit_tests/`).
//!
//! Reads are plain loads: the table is ordinary (write-protected) FRAM. The
//! info-block CRC is not verified here.
//!
//! # Example
//!
//! ```ignore
//! let cal = tlv::adc_cal().unwrap();          // factory table — present on real silicon
//! let raw = adc.read_temperature(&vref);
//! let deci_c = cal.temp_deci_celsius(vref.voltage(), raw).unwrap(); // 273 = 27.3 °C
//! ```

use crate::ref_a::ReferenceVoltage;

/// First tag byte (0x1A00–0x1A07 is the info block: CRC, device ID, revisions).
const TLV_START: u16 = 0x1A08;
/// Last byte of the descriptor window.
const TLV_END: u16 = 0x1AFF;
/// End-of-table marker tag.
const TAG_END: u8 = 0xFF;
/// ADC12_B calibration entry.
const TAG_ADC12_CAL: u8 = 0x11;
/// REF_A calibration entry.
const TAG_REF_CAL: u8 = 0x12;

/// Factory ADC12_B calibration: gain/offset of this die's converter and the
/// temperature-sensor characterization points, from TLV tag `0x11`.
#[derive(Clone, Copy, Debug)]
pub struct AdcCal {
    /// Gain correction, 1.15 fixed point (`2^15` = unity).
    pub gain_factor: u16,
    /// Additive count correction, applied after the gain.
    pub offset: i16,
    /// Raw temperature-sensor conversion results at 30 °C, indexed by
    /// reference voltage (1.2 V, 2.0 V, 2.5 V).
    pub temp_30: [u16; 3],
    /// The same three readings at 85 °C.
    pub temp_85: [u16; 3],
}

impl AdcCal {
    /// The (30 °C, 85 °C) characterization pair for `voltage`.
    pub fn temp_pair(&self, voltage: ReferenceVoltage) -> (u16, u16) {
        let i = voltage.index();
        (self.temp_30[i], self.temp_85[i])
    }

    /// Interpolate a **raw** temperature-sensor reading (taken against
    /// `voltage`, e.g. via [`crate::adc::Adc::read_temperature`]) to deci-°C
    /// (273 = 27.3 °C). Raw on purpose: the calibration points already embed
    /// this ADC's gain/offset error, so correcting first would apply it twice.
    /// `None` if the stored pair is not an ascending span (corrupt/blank TLV).
    pub fn temp_deci_celsius(&self, voltage: ReferenceVoltage, raw: u16) -> Option<i16> {
        let (t30, t85) = self.temp_pair(voltage);
        crate::adc_cal::temp_deci_celsius(raw, t30, t85)
    }

    /// Apply this die's gain factor and offset to a conversion result (the
    /// first step of an absolute-voltage correction; follow with
    /// [`RefCal::correct`]).
    pub fn correct_gain_offset(&self, raw: u16) -> u16 {
        crate::adc_cal::apply_gain_offset(raw, self.gain_factor, self.offset)
    }
}

/// Factory REF_A calibration: the measured-vs-nominal factor of each
/// reference output, from TLV tag `0x12`.
#[derive(Clone, Copy, Debug)]
pub struct RefCal {
    /// 1.15 fixed-point factors, indexed by reference voltage
    /// (1.2 V, 2.0 V, 2.5 V).
    pub factor: [u16; 3],
}

impl RefCal {
    /// Rescale a (gain/offset-corrected) result as if the `voltage` reference
    /// had been exactly nominal.
    pub fn correct(&self, voltage: ReferenceVoltage, counts: u16) -> u16 {
        crate::adc_cal::apply_ref_factor(counts, self.factor[voltage.index()])
    }
}

/// This die's ADC12_B calibration, or `None` if the table has no `0x11` entry
/// of the expected size (real silicon always has one; `None` means a blank or
/// corrupted descriptor table).
pub fn adc_cal() -> Option<AdcCal> {
    let data = find(TAG_ADC12_CAL, 16)?;
    Some(AdcCal {
        gain_factor: read_u16(data),
        offset: read_u16(data + 2) as i16,
        temp_30: [read_u16(data + 4), read_u16(data + 8), read_u16(data + 12)],
        temp_85: [read_u16(data + 6), read_u16(data + 10), read_u16(data + 14)],
    })
}

/// This die's REF_A calibration, or `None` if the table has no `0x12` entry
/// of the expected size.
pub fn ref_cal() -> Option<RefCal> {
    let data = find(TAG_REF_CAL, 6)?;
    Some(RefCal {
        factor: [read_u16(data), read_u16(data + 2), read_u16(data + 4)],
    })
}

/// Walk the table for `tag` and return its data address, requiring at least
/// `min_len` bytes so the field reads that follow stay inside the entry.
fn find(tag: u8, min_len: usize) -> Option<u16> {
    let mut addr = TLV_START;
    // Each iteration needs a tag byte and a length byte inside the window.
    while addr < TLV_END {
        let t = read_u8(addr);
        if t == TAG_END {
            return None;
        }
        let len = read_u8(addr + 1) as usize;
        if t == tag {
            return (len >= min_len).then_some(addr + 2);
        }
        // Entries observed on this part are word-sized and even-length, but
        // guard the arithmetic anyway: a bogus length must not wrap the walk.
        addr = addr.checked_add(2 + len as u16)?;
    }
    None
}

/// One byte of the descriptor table.
fn read_u8(addr: u16) -> u8 {
    unsafe { core::ptr::read_volatile(addr as *const u8) }
}

/// One little-endian word, assembled from bytes so an odd `addr` (possible
/// only with a corrupt length chain) cannot fault on alignment.
fn read_u16(addr: u16) -> u16 {
    read_u8(addr) as u16 | (read_u8(addr + 1) as u16) << 8
}
