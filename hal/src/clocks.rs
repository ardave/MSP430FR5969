//! Clock System (CS) configuration and the resulting clock frequencies.
//!
//! This module owns the device clock tree and is the **single source of truth**
//! for clock frequencies: it programs the CS registers and hands back a
//! [`Clocks`] value carrying the resulting MCLK/SMCLK/ACLK rates in Hz. Other
//! drivers that need a frequency (e.g. [`crate::serial`] for its baud-rate math,
//! [`crate::delay::Delay`] for its cycle math) should read it from here instead
//! of hard-coding a number — so there is exactly one place that "knows" how fast
//! the chip runs.
//!
//! # The configuration this provides
//!
//! [`configure`] sets up a **fine-resolution, full-power** tree (no low-power
//! mode):
//!
//! | Clock | Source         | Divider | Frequency        |
//! |-------|----------------|---------|------------------|
//! | DCO   | internal       | —       | 8 MHz            |
//! | MCLK  | DCOCLK         | /8      | 1 MHz            |
//! | SMCLK | DCOCLK         | /1      | **8 MHz**        |
//! | ACLK  | VLOCLK         | /1      | ~9.4 kHz (rough) |
//!
//! SMCLK runs at the full 8 MHz DCO so peripherals clocked from it (the planned
//! hardware timer, the UART) get **125 ns resolution**. MCLK is left at 1 MHz:
//! the CPU clock is deliberately *not* sped up, which keeps FRAM at zero wait
//! states (the part needs wait states only above 8 MHz MCLK) and leaves the
//! software [`crate::delay::Delay`] calibration unchanged.
//!
//! # Why this is still DCO-based (and what that costs)
//!
//! The DCO is an internal oscillator with no crystal, so these frequencies carry
//! its tolerance (a few %). That is the trade for "fine resolution without low
//! power": a 32 kHz crystal/REFO on ACLK would be more accurate and sleep-
//! friendly but far coarser. ACLK here is sourced from the VLO purely so it has
//! a defined value; it is **not** used by the fine-resolution path and its ~9.4
//! kHz figure is approximate (VLO spreads ±lots) — don't time anything off it.
//!
//! # Register protection
//!
//! The CS registers are password-protected: writes are ignored until `0xA5`
//! (`CSKEY`) is written to the **high byte** of `CSCTL0`. We write only that
//! byte (raw, like [`crate::serial`]) because the low byte of `CSCTL0` holds
//! factory DCO/MOD trim that must not be overwritten — the PAC models `CSCTL0`
//! as one 16-bit register, so a normal word write would clobber the trim.

use crate::pac;

// High byte of CSCTL0 (CS base 0x0160). Byte-addressed so the password write
// leaves the factory DCO/MOD trim in the low byte untouched.
const CSCTL0_H: usize = 0x0161;
const CSKEY_H: u8 = 0xA5; // unlock value; any other value re-locks

// CSCTL1 — DCO range/frequency. DCOFSEL=6 with DCORSEL=0 (low range) = 8 MHz,
// which is also the reset value, so the DCO frequency is unchanged (no settling
// needed); we write it explicitly to own the configuration.
const DCOFSEL_6: u16 = 0x000C;

// CSCTL2 — clock source selects.
const SELM_DCOCLK: u16 = 0x0003; // MCLK  <- DCOCLK
const SELS_DCOCLK: u16 = 0x0030; // SMCLK <- DCOCLK
const SELA_VLOCLK: u16 = 0x0100; // ACLK  <- VLOCLK (defined, unused)

// CSCTL3 — source dividers.
const DIVM_8: u16 = 0x0003; // MCLK  = DCO / 8 = 1 MHz
const DIVS_1: u16 = 0x0000; // SMCLK = DCO / 1 = 8 MHz
const DIVA_1: u16 = 0x0000; // ACLK  = VLO / 1

const DCO_HZ: u32 = 8_000_000;
const VLO_HZ: u32 = 9_400; // VLO typical; approximate only

/// The configured clock frequencies, in Hz. Cheap to copy; pass it to any driver
/// that needs to know how fast a clock runs.
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    mclk: u32,
    smclk: u32,
    aclk: u32,
}

impl Clocks {
    /// Master clock (CPU / FRAM), in Hz.
    pub const fn mclk(&self) -> u32 {
        self.mclk
    }

    /// Subsystem master clock (peripherals: timers, eUSCI), in Hz.
    pub const fn smclk(&self) -> u32 {
        self.smclk
    }

    /// Auxiliary clock, in Hz. Approximate (VLO-sourced) and not used by the
    /// fine-resolution path — do not derive precise timing from it.
    pub const fn aclk(&self) -> u32 {
        self.aclk
    }
}

/// Configure the clock system as documented in the module header and return the
/// resulting [`Clocks`].
///
/// Consumes the CS peripheral so the type system guarantees the clock tree is
/// configured exactly once and nothing else pokes the CS registers afterward.
pub fn configure(cs: pac::Cs) -> Clocks {
    unsafe {
        // Unlock: password into the CSCTL0 high byte only (preserve trim).
        (CSCTL0_H as *mut u8).write_volatile(CSKEY_H);

        cs.csctl1().write(|w| w.bits(DCOFSEL_6));
        cs.csctl2()
            .write(|w| w.bits(SELM_DCOCLK | SELS_DCOCLK | SELA_VLOCLK));
        cs.csctl3().write(|w| w.bits(DIVM_8 | DIVS_1 | DIVA_1));

        // Re-lock (any non-password value locks).
        (CSCTL0_H as *mut u8).write_volatile(0x00);
    }

    Clocks {
        mclk: DCO_HZ / 8,
        smclk: DCO_HZ / 1,
        aclk: VLO_HZ,
    }
}
