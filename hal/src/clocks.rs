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
//! # Four profiles
//!
//! Pick **one** at startup (each consumes the CS peripheral):
//!
//! - [`configure`] — **fine resolution, full power.** SMCLK at the full 8 MHz
//!   DCO (125 ns peripheral ticks), MCLK = 1 MHz; ACLK parked on the VLO. Not
//!   sleep-oriented.
//! - [`configure_low_power`] — **sleep-friendly.** ACLK sourced from the
//!   32.768 kHz **LFXT crystal**, which keeps running in LPM3 while the DCO,
//!   MCLK, and SMCLK are all off — so a timer clocked on ACLK can wake the part
//!   from deep sleep, accurately and at microamp power. Falls back to the VLO if
//!   the crystal does not start.
//! - [`configure_high_speed`] — **high-range DCO, zero wait states.** DCO at
//!   16 MHz: SMCLK = 16 MHz (62.5 ns peripheral ticks), MCLK = 8 MHz — the
//!   fastest CPU that still reads FRAM with no wait states. ACLK on the VLO.
//! - [`configure_max_speed`] — **everything at 16 MHz.** MCLK crosses the 8 MHz
//!   FRAM ceiling, so this profile programs **one FRAM wait state** before
//!   touching the clock (see below). ACLK on the VLO.
//!
//! # FRAM wait states are not a public API on purpose
//!
//! FRAM cannot be read faster than 8 MHz; above that, `FRCTL0.NWAITS` must
//! insert CPU wait states or instruction fetches return garbage (a crash that
//! looks like random memory corruption). The safe ordering is fixed: **raise
//! wait states, then raise MCLK** (and the reverse to slow down). Rather than
//! trust every caller to remember that, the wait-state write lives *inside*
//! [`configure_max_speed`], sequenced before the DCO switch — the same
//! fused-ordering pattern as [`crate::peripherals::take`] (watchdog before `take()`) and
//! the GPIO interrupt arming order. At 8 MHz and below the reset value
//! (`NWAITS = 0`) is correct and nothing needs configuring.
//!
//! # Why LFXT is the "low power" clock
//!
//! Deep sleep (LPM3) gates off the DCO and everything derived from it; only ACLK
//! survives. To wake periodically you therefore need ACLK on an oscillator that
//! runs in LPM3. Two qualify: the **VLO** (~9.4 kHz, always available, lowest
//! power, but imprecise ±lots) and the **LFXT** 32.768 kHz crystal (a few µA,
//! crystal-accurate). [`configure_low_power`] prefers LFXT for accuracy and
//! drops to VLO only if the crystal is absent/dead.
//!
//! # Register protection
//!
//! The CS registers are password-protected: writes are ignored until `0xA5`
//! (`CSKEY`) is written to the **high byte** of `CSCTL0`. We write only that
//! byte (raw, like [`crate::serial`]) because the low byte of `CSCTL0` holds
//! factory DCO/MOD trim that must not be overwritten — the PAC models `CSCTL0`
//! as one 16-bit register, so a normal word write would clobber the trim.

use crate::pac;

// --- CSCTL0 password ---
// High byte of CSCTL0 (CS base 0x0160). Byte-addressed so the password write
// leaves the factory DCO/MOD trim in the low byte untouched.
const CSCTL0_H: usize = 0x0161;
const CSKEY_H: u8 = 0xA5; // unlock value; any other value re-locks

// --- CSCTL1: DCO range/frequency ---
// DCOFSEL=6 with DCORSEL=0 (low range) = 8 MHz, which is also the reset value,
// so the DCO frequency is unchanged (no settling needed); written explicitly to
// own the configuration.
const DCOFSEL_6: u16 = 0x000C;
// DCOFSEL=4 with DCORSEL=1 (high range) = 16 MHz, factory-trimmed ±3.5% like
// the low-range taps (datasheet SLAS704G fDCO16).
const DCOFSEL_4: u16 = 0x0008;
const DCORSEL: u16 = 0x0040;

// --- CSCTL2: clock source selects ---
const SELM_DCOCLK: u16 = 0x0003; // MCLK  <- DCOCLK
const SELS_DCOCLK: u16 = 0x0030; // SMCLK <- DCOCLK
const SELA_VLOCLK: u16 = 0x0100; // ACLK  <- VLOCLK
const SELA_LFXTCLK: u16 = 0x0000; // ACLK <- LFXTCLK (32.768 kHz crystal)

// --- CSCTL3: source dividers ---
const DIVM_1: u16 = 0x0000; // MCLK  = DCO / 1
const DIVM_2: u16 = 0x0001; // MCLK  = DCO / 2
const DIVM_8: u16 = 0x0003; // MCLK  = DCO / 8 = 1 MHz
const DIVS_1: u16 = 0x0000; // SMCLK = DCO / 1 = 8 MHz
const DIVS_8: u16 = 0x0030; // SMCLK = DCO / 8 = 1 MHz
const DIVA_1: u16 = 0x0000; // ACLK  = source / 1

// --- FRCTL0: FRAM controller wait states ---
// Password-protected exactly like CSCTL0/WDTCTL/PMMCTL0: 0xA5 into the HIGH
// byte opens the register file, any other value closes it, and a write while
// closed is a PUC (`sys::ResetReason` 0x1A decodes it). Byte-lane access for
// the same reason as the CS password. NWAITS is bits 6:4 of the low byte;
// each wait state buys 8 MHz of MCLK (NWAITS=0 → ≤8 MHz, NWAITS=1 → ≤16 MHz).
const FRCTL0_L: usize = 0x0140;
const FRCTL0_H: usize = 0x0141;
const FRKEY_H: u8 = 0xA5;
const NWAITS_MASK: u8 = 0x70;
const NWAITS_1: u8 = 0x10;

// --- CSCTL4 (LFXT control) / CSCTL5 (fault flags) ---
const LFXTOFF: u16 = 0x0001; // 1 = LFXT off; clear to enable
const LFXTOFFG: u16 = 0x0001; // CSCTL5: LFXT oscillator fault flag

// --- Oscillator fault flag in the SFR block (not CS-protected) ---
const SFRIFG1: usize = 0x0102;
const OFIFG: u16 = 0x0002; // OR of all oscillator fault flags

// --- LFXT crystal pins: LFXIN = PJ.4, LFXOUT = PJ.5 ---
const PJSEL0: usize = 0x032A; // Port J base 0x0320 + 0x0A
const PJ_XT1: u8 = (1 << 4) | (1 << 5); // select the crystal function on PJ.4/.5

// --- Nominal frequencies ---
const DCO_HZ: u32 = 8_000_000;
const DCO_HIGH_HZ: u32 = 16_000_000;
const LFXT_HZ: u32 = 32_768; // watch crystal
const VLO_HZ: u32 = 9_400; // VLO typical; approximate only

// LFXT startup budget: clear the fault flags, let the crystal swing, recheck.
// Bounded so an absent/dead crystal falls back to VLO instead of hanging. At
// MCLK = 1 MHz the per-attempt spin is tens of ms, ~16 attempts ≈ a few hundred
// ms total — enough for a healthy 32 kHz crystal, brief if there is none.
const LFXT_START_ATTEMPTS: u32 = 16;
const LFXT_SETTLE_ITERS: u32 = 2000;

/// Which oscillator ACLK is sourced from.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum AclkSource {
    /// 32.768 kHz LFXT crystal (accurate, sleep-capable).
    Lfxt,
    /// Internal VLO (~9.4 kHz, imprecise, lowest power).
    Vlo,
}

/// The configured clock frequencies, in Hz. Cheap to copy; pass it to any driver
/// that needs to know how fast a clock runs.
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    mclk: u32,
    smclk: u32,
    aclk: u32,
    aclk_source: AclkSource,
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

    /// Auxiliary clock, in Hz. With [`configure_low_power`] this is the LFXT
    /// crystal (accurate) unless it failed to start, in which case it is the
    /// imprecise VLO — check [`Clocks::aclk_source`].
    pub const fn aclk(&self) -> u32 {
        self.aclk
    }

    /// Which oscillator ACLK ended up on.
    pub const fn aclk_source(&self) -> AclkSource {
        self.aclk_source
    }
}

/// **Performance profile.** DCO 8 MHz; MCLK = 1 MHz (FRAM stays at zero wait
/// states, software [`crate::delay::Delay`] calibration unchanged); SMCLK = full
/// 8 MHz for 125 ns peripheral resolution; ACLK parked on the VLO (unused).
///
/// Consumes the CS peripheral so the clock tree is configured exactly once.
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
        aclk_source: AclkSource::Vlo,
    }
}

/// **High-speed profile.** DCO in its high range at **16 MHz**; SMCLK = 16 MHz
/// (62.5 ns peripheral ticks — twice [`configure`]'s resolution), **MCLK =
/// DCO/2 = 8 MHz** (8× [`configure`]'s CPU speed, and the fastest MCLK that
/// still reads FRAM with **zero wait states**); ACLK parked on the VLO.
///
/// Write order inside: the dividers go in *before* the DCO range switch, so
/// MCLK never exceeds its current wait-state ceiling mid-sequence (with the
/// old 8 MHz DCO the ÷2 briefly yields 4 MHz, then the range switch lands it
/// on 8 MHz).
///
/// Consumes the CS peripheral so the clock tree is configured exactly once.
pub fn configure_high_speed(cs: pac::Cs) -> Clocks {
    unsafe {
        (CSCTL0_H as *mut u8).write_volatile(CSKEY_H); // unlock

        // Dividers first (see above), then the DCO range/frequency.
        cs.csctl2()
            .write(|w| w.bits(SELM_DCOCLK | SELS_DCOCLK | SELA_VLOCLK));
        cs.csctl3().write(|w| w.bits(DIVM_2 | DIVS_1 | DIVA_1));
        cs.csctl1().write(|w| w.bits(DCORSEL | DCOFSEL_4));

        (CSCTL0_H as *mut u8).write_volatile(0x00); // lock
    }

    Clocks {
        mclk: DCO_HIGH_HZ / 2,
        smclk: DCO_HIGH_HZ / 1,
        aclk: VLO_HZ,
        aclk_source: AclkSource::Vlo,
    }
}

/// **Maximum-speed profile.** MCLK *and* SMCLK at the full **16 MHz** DCO —
/// the device's rated ceiling. MCLK above 8 MHz outruns the FRAM, so this
/// profile programs **one FRAM wait state** (`FRCTL0.NWAITS = 1`) *before*
/// switching the clock; the ordering is fused here so it cannot be gotten
/// wrong at a call site (see the module docs). The CPU stalls one cycle on
/// each FRAM fetch the cache misses, so real throughput gains over
/// [`configure_high_speed`] are workload-dependent — code running from cache
/// and all 16 MHz-clocked peripherals get the full 2×.
///
/// ACLK parks on the VLO, as in [`configure`].
///
/// Consumes the CS peripheral so the clock tree is configured exactly once.
pub fn configure_max_speed(cs: pac::Cs) -> Clocks {
    // Wait states FIRST: from this point any MCLK up to 16 MHz is legal.
    // (An extra wait state at the current slow MCLK merely wastes a cycle.)
    set_fram_wait_states(NWAITS_1);

    unsafe {
        (CSCTL0_H as *mut u8).write_volatile(CSKEY_H); // unlock

        cs.csctl2()
            .write(|w| w.bits(SELM_DCOCLK | SELS_DCOCLK | SELA_VLOCLK));
        cs.csctl3().write(|w| w.bits(DIVM_1 | DIVS_1 | DIVA_1));
        cs.csctl1().write(|w| w.bits(DCORSEL | DCOFSEL_4));

        (CSCTL0_H as *mut u8).write_volatile(0x00); // lock
    }

    Clocks {
        mclk: DCO_HIGH_HZ,
        smclk: DCO_HIGH_HZ,
        aclk: VLO_HZ,
        aclk_source: AclkSource::Vlo,
    }
}

/// Program `FRCTL0.NWAITS` under its password bracket. Private on purpose:
/// wait states have no meaning apart from the MCLK frequency they license, so
/// the only callers are the profile functions that own that pairing.
///
/// Byte-lane discipline like the CS password: `0xA5` into the high byte opens
/// the FRAM controller registers, the low-byte RMW touches only the NWAITS
/// field, and writing the high byte back to zero closes the bracket (a write
/// while closed is a PUC, so the bracket is never left open).
fn set_fram_wait_states(nwaits: u8) {
    unsafe {
        (FRCTL0_H as *mut u8).write_volatile(FRKEY_H); // unlock
        let l = FRCTL0_L as *mut u8;
        l.write_volatile((l.read_volatile() & !NWAITS_MASK) | nwaits);
        (FRCTL0_H as *mut u8).write_volatile(0x00); // lock
    }
}

/// **Low-power profile.** MCLK and SMCLK = 1 MHz (modest active power); ACLK on
/// the 32.768 kHz **LFXT crystal**, which keeps ticking in LPM3 so a timer on
/// ACLK can wake the part from deep sleep. If the crystal does not start within
/// the bounded budget (e.g. not populated), ACLK falls back to the VLO and
/// [`Clocks::aclk_source`] reports [`AclkSource::Vlo`].
///
/// Consumes the CS peripheral so the clock tree is configured exactly once.
pub fn configure_low_power(cs: pac::Cs) -> Clocks {
    // Route the dedicated crystal pins (PJ.4/PJ.5) to the LFXT function. PJSEL1
    // bits 4/5 are 0 at reset, which together with PJSEL0=1 selects XT1. Raw
    // access (these are crystal pins, never GPIO here), like crate::serial.
    unsafe {
        let pjsel0 = PJSEL0 as *mut u8;
        pjsel0.write_volatile(pjsel0.read_volatile() | PJ_XT1);
    }

    let mut aclk_source = AclkSource::Vlo;
    let mut aclk_hz = VLO_HZ;
    let sfrifg1 = SFRIFG1 as *mut u16;

    unsafe {
        (CSCTL0_H as *mut u8).write_volatile(CSKEY_H); // unlock

        cs.csctl1().write(|w| w.bits(DCOFSEL_6)); // DCO 8 MHz
        cs.csctl4().modify(|r, w| w.bits(r.bits() & !LFXTOFF)); // enable LFXT
        // Select ACLK <- LFXT (this is what actually *requests* the crystal),
        // MCLK/SMCLK <- DCO. Dividers: MCLK/SMCLK = 1 MHz, ACLK /1.
        cs.csctl2()
            .write(|w| w.bits(SELM_DCOCLK | SELS_DCOCLK | SELA_LFXTCLK));
        cs.csctl3().write(|w| w.bits(DIVM_8 | DIVS_8 | DIVA_1));

        // Attempt to start LFXT: clear its fault flag (and the SFR osc-fault
        // flag), give it time to swing, then see whether the fault reasserts.
        for _ in 0..LFXT_START_ATTEMPTS {
            cs.csctl5().modify(|r, w| w.bits(r.bits() & !LFXTOFFG));
            sfrifg1.write_volatile(sfrifg1.read_volatile() & !OFIFG);
            settle();
            if cs.csctl5().read().bits() & LFXTOFFG == 0 {
                aclk_source = AclkSource::Lfxt;
                aclk_hz = LFXT_HZ;
                break;
            }
        }

        if let AclkSource::Vlo = aclk_source {
            // Crystal never settled — move ACLK to the VLO, power the LFXT pad
            // down, and clear the lingering fault now that nothing sources it.
            cs.csctl2()
                .write(|w| w.bits(SELM_DCOCLK | SELS_DCOCLK | SELA_VLOCLK));
            cs.csctl4().modify(|r, w| w.bits(r.bits() | LFXTOFF));
            cs.csctl5().modify(|r, w| w.bits(r.bits() & !LFXTOFFG));
            sfrifg1.write_volatile(sfrifg1.read_volatile() & !OFIFG);
        }

        (CSCTL0_H as *mut u8).write_volatile(0x00); // lock
    }

    Clocks {
        mclk: DCO_HZ / 8,
        smclk: DCO_HZ / 8,
        aclk: aclk_hz,
        aclk_source,
    }
}

/// Crude busy-wait giving the crystal time to oscillate between fault-flag
/// checks. `black_box` keeps the optimizer from deleting the empty loop.
#[inline(never)]
fn settle() {
    let mut i = LFXT_SETTLE_ITERS;
    while i > 0 {
        i -= 1;
        core::hint::black_box(i);
    }
}
