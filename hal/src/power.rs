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
