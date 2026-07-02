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
//! through deep sleep — at microamp power. (LPM4 additionally sets OSCOFF, which
//! stops the crystal, so a timer cannot run there.)

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
