//! Low-power mode (LPMx) entry.
//!
//! The MSP430 saves power by switching off clocks and the CPU and waiting for an
//! interrupt to wake back up. Each mode is just a particular set of bits in the
//! **status register** (SR / R2):
//!
//! | Bit | Name   | Effect when set            |
//! |-----|--------|----------------------------|
//! | 4   | CPUOFF | CPU and MCLK off           |
//! | 5   | OSCOFF | LFXT/low-freq oscillator off |
//! | 6   | SCG0   | FLL / DCO off              |
//! | 7   | SCG1   | SMCLK off                  |
//!
//! **LPM3** sets CPUOFF + SCG0 + SCG1 but leaves **OSCOFF = 0**, so the 32.768 kHz
//! crystal (and an ACLK-clocked timer) keeps running while MCLK, SMCLK, and the
//! DCO are all gated. That is what lets a timer measure time, and wake the part,
//! through deep sleep — at microamp power. **LPM4** additionally sets OSCOFF,
//! stopping the crystal too: no clock runs at all, so no timer can wake it —
//! only *asynchronous* events can. Port-pin edge interrupts are exactly that
//! (`PxIFG` latches without a clock), which is why "sleep until a button" is
//! the canonical LPM4 pattern; see [`crate::gpio`]'s interrupt support.
//!
//! # LPMx.5: power off instead of pause
//!
//! Every mode above keeps the **core domain** powered: the PMM's internal
//! regulator keeps VCORE up, so RAM, registers, and the frozen program all
//! retain state, and wake-up resumes execution mid-line in a handful of µs.
//! **LPM3.5** and **LPM4.5** ([`enter_lpm3_5`], [`enter_lpm4_5`]) instead turn
//! the regulator *off* (`PMMREGOFF`): VCORE collapses, and the CPU, RAM, and
//! nearly every peripheral cease to exist as state — which is how the ".5"
//! modes get below LPM4's leakage floor. There is no frozen program left to
//! resume, so **wake-up is architecturally a reset**: the device comes back
//! through a BOR, `main` starts from the top, and `SYSRSTIV` reports
//! [`Lpm5WakeUp`](crate::sys::ResetReason::Lpm5WakeUp) as the cause (drain it
//! with [`crate::sys::ResetReasons::drain`] to tell wake from cold boot).
//! Both entries are therefore `-> !`.
//!
//! What survives the dark interval: **FRAM** (nonvolatile), the **RTC_B
//! domain** (LPM3.5 only — LFXT keeps the calendar counting through the
//! sleep; the wake freezes it, contents intact, until
//! [`crate::rtc::Rtc::attach`] releases it), and the **I/O pins**, which the
//! PMM latches at their last configuration (`LOCKLPM5`). On wake the pins
//! *stay* latched until software clears `LOCKLPM5` ([`crate::gpio::unlock_pins`])
//! — reconfigure the ports first (and re-arm the wake pin's `PxIE` *before*
//! unlocking so its latched `PxIFG` is delivered), then unlock, so outputs never glitch
//! through the reboot.

/// Enter **LPM0** with interrupts enabled, and return once an interrupt has
/// woken the CPU.
///
/// LPM0 stops only the CPU and MCLK — SMCLK, the DCO, and MODOSC-on-demand
/// peripherals keep running. It is the mode for "a peripheral is busy and the
/// CPU has nothing to do until it finishes": a UART receiving on SMCLK, or an
/// ADC12 conversion self-clocking on MODOSC (`start_* → enter_lpm0() →
/// read_result()`). The same atomic GIE+sleep `bis` and the same
/// `#[interrupt(wake_cpu)]` requirement as [`enter_lpm3`] apply.
#[inline]
pub fn enter_lpm0() {
    // GIE(3) | CPUOFF(4) = 0x18.
    const LPM0_GIE: u16 = (1 << 3) | (1 << 4);
    // SAFETY: writing SR to enter a low-power mode; no memory or stack effects.
    // Immediate addressing (`#`) is load-bearing — see `enter_lpm3`.
    unsafe {
        core::arch::asm!(
            "bis #{bits}, r2",
            "nop",
            bits = const LPM0_GIE,
            options(nomem, nostack),
        );
    }
}

/// Enter **LPM3** with interrupts enabled, and return once an interrupt has
/// woken the CPU.
///
/// Sets CPUOFF + SCG0 + SCG1 (LPM3) and GIE in one `bis` to the status register:
/// enabling interrupts and sleeping must be atomic, or an interrupt arriving in
/// the gap could be serviced and the part would then sleep forever waiting for
/// an event that already happened. Execution halts at this instruction until an
/// interrupt fires; for the CPU to actually resume here (rather than service the
/// ISR and sleep again) that ISR must clear the low-power bits in the stacked SR
/// — which msp430-rt's `#[interrupt(wake_cpu)]` does.
#[inline]
pub fn enter_lpm3() {
    // GIE(3) | CPUOFF(4) | SCG0(6) | SCG1(7) = 0x08 | 0x10 | 0x40 | 0x80 = 0xD8.
    const LPM3_GIE: u16 = (1 << 3) | (1 << 4) | (1 << 6) | (1 << 7);
    // SAFETY: writing SR to enter a low-power mode; no memory or stack effects.
    // Not `preserves_flags` — `bis` to SR changes the status bits by design.
    unsafe {
        // The `#` is load-bearing: it forces immediate addressing (`bis #0xD8,
        // r2`). Without it the assembler encodes symbolic mode (`bis 0xD8(PC),
        // r2`), which ORs the *word stored at* PC+0xD8 into SR instead of the
        // constant — CPUOFF stays clear, the CPU never halts, and `enter_lpm3`
        // falls straight through (the timer wake fires "immediately").
        core::arch::asm!(
            "bis #{bits}, r2",
            "nop",
            bits = const LPM3_GIE,
            options(nomem, nostack),
        );
    }
}

/// Enter **LPM4** with interrupts enabled, and return once an interrupt has
/// woken the CPU.
///
/// LPM4 is LPM3 plus OSCOFF: every clock on the part stops, including LFXT.
/// Nothing scheduled can wake it — only asynchronous events (a port-pin edge
/// latching `PxIFG`, or RST/NMI) do, so arm a [`crate::gpio`] pin interrupt
/// *before* calling this or the part sleeps until reset. The same atomic
/// GIE+sleep `bis` and the same `#[interrupt(wake_cpu)]` requirement as
/// [`enter_lpm3`] apply.
#[inline]
pub fn enter_lpm4() {
    // GIE(3) | CPUOFF(4) | OSCOFF(5) | SCG0(6) | SCG1(7) = 0xF8.
    const LPM4_GIE: u16 = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    // SAFETY: writing SR to enter a low-power mode; no memory or stack effects.
    // Immediate addressing (`#`) is load-bearing — see `enter_lpm3`.
    unsafe {
        core::arch::asm!(
            "bis #{bits}, r2",
            "nop",
            bits = const LPM4_GIE,
            options(nomem, nostack),
        );
    }
}

// --- LPMx.5: regulator-off deep sleep ---------------------------------------

// PMMCTL0 password bytes. Like WDTCTL (see crate::watchdog) and CSCTL0 (see
// crate::clocks), the PMM register file only accepts writes while 0xA5 sits in
// PMMCTL0's high byte — and the SVD does not model the password field, so a
// PAC `modify` would write back the 0x96 the high byte reads as and trigger a
// PMM-password PUC (SYSRSTIV 0x20, sys::ResetReason::PmmPassword). Byte-wise
// raw access sidesteps it: park 0xA5 in the high byte (unlocks the whole PMM
// register file), RMW the low byte, then write anything else to re-lock.
const PMMCTL0_L: *mut u8 = 0x0120 as *mut u8;
const PMMCTL0_H: *mut u8 = 0x0121 as *mut u8;
const PMMPW_H: u8 = 0xA5;
const PMMREGOFF: u8 = 1 << 4;

/// Set `PMMREGOFF` so the next LPM3/LPM4 entry powers the core regulator off
/// (turning the pause modes into the ".5" power-off modes).
///
/// Takes `&pac::Pmm` purely as proof of post-`take()` context; the actual
/// access is byte-wise raw (see the constants above for why the PAC field API
/// cannot be used here).
fn set_regulator_off(_pmm: &crate::pac::Pmm) {
    // SAFETY: byte writes to PMMCTL0 with the password composed in, exactly as
    // SLAU367 prescribes (unlock high byte, set PMMREGOFF, re-lock). Volatile,
    // password-guarded, and touching only the REGOFF bit of the low byte.
    unsafe {
        PMMCTL0_H.write_volatile(PMMPW_H);
        PMMCTL0_L.write_volatile(PMMCTL0_L.read_volatile() | PMMREGOFF);
        PMMCTL0_H.write_volatile(0x00); // any non-password value re-locks
    }
}

/// Enter **LPM3.5** — RTC-alive power-off. Never returns: wake-up is a BOR
/// reset back into `main` (see the module docs).
///
/// LPM3.5 is the LPM3 request (`CPUOFF+SCG0+SCG1`, OSCOFF *clear* so LFXT
/// stays available to the RTC domain) with `PMMREGOFF` set: the core domain
/// powers off, but RTC_B keeps its calendar counting on LFXT and can wake the
/// part via an armed RTC interrupt (e.g.
/// [`crate::rtc::Rtc::enable_event_interrupt`]) — the enable bit acts as the
/// wake enable; no ISR ever runs for an LPMx.5 wake. Armed port pins wake it
/// too. Before calling: persist anything you need to FRAM, and `flush()` the
/// UART (SMCLK dies mid-character otherwise). On the rebooted side, adopt the
/// surviving calendar with [`crate::rtc::Rtc::attach`] — the wake freezes it
/// (and clears the RTC interrupt enables) until attach releases it, and
/// [`crate::rtc::event_irq_pending`] holds the wake evidence.
///
/// An interrupt that fires *between* arming and this call is serviced as a
/// normal interrupt (the entry `bis` sets GIE, per the SLAU367 recipe) and
/// will not wake the part later — arm the wake source immediately before
/// entering. If entry is aborted by such a pending event, this function
/// retries forever.
pub fn enter_lpm3_5(pmm: &crate::pac::Pmm) -> ! {
    set_regulator_off(pmm);
    // GIE(3) | CPUOFF(4) | SCG0(6) | SCG1(7) — LPM3 bits; REGOFF makes it 3.5.
    const LPM3_GIE: u16 = (1 << 3) | (1 << 4) | (1 << 6) | (1 << 7);
    loop {
        // SAFETY: writing SR to enter a low-power mode; no memory or stack
        // effects. Immediate addressing (`#`) is load-bearing — see enter_lpm3.
        unsafe {
            core::arch::asm!(
                "bis #{bits}, r2",
                "nop",
                bits = const LPM3_GIE,
                options(nomem, nostack),
            );
        }
    }
}

/// Enter **LPM4.5** — everything-off power-off (~¼ µA class). Never returns:
/// wake-up is a BOR reset back into `main` (see the module docs).
///
/// LPM4.5 is the LPM4 request (all clocks stopped, LFXT included — an RTC
/// does *not* survive this one) with `PMMREGOFF` set. The only wake sources
/// are armed port-pin edges (`enable_interrupt` on a [`crate::gpio`] input —
/// the `PxIE` bit acts as the wake enable; no ISR ever runs) and the RST pin.
/// Before calling: persist state to FRAM and `flush()` the UART. On the
/// rebooted side, re-arm the pin *before* clearing `LOCKLPM5`
/// ([`crate::gpio::unlock_pins`]) and its latched wake `PxIFG` is delivered
/// for inspection.
///
/// The same pre-entry race note as [`enter_lpm3_5`] applies: arm the wake pin
/// immediately before entering.
pub fn enter_lpm4_5(pmm: &crate::pac::Pmm) -> ! {
    set_regulator_off(pmm);
    // GIE(3) | CPUOFF(4) | OSCOFF(5) | SCG0(6) | SCG1(7) — LPM4 bits + REGOFF.
    const LPM4_GIE: u16 = (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);
    loop {
        // SAFETY: writing SR to enter a low-power mode; no memory or stack
        // effects. Immediate addressing (`#`) is load-bearing — see enter_lpm3.
        unsafe {
            core::arch::asm!(
                "bis #{bits}, r2",
                "nop",
                bits = const LPM4_GIE,
                options(nomem, nostack),
            );
        }
    }
}
