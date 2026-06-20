/* MSP430FR5969 memory map — written to msp430-rt's contract.

   msp430-rt's link.x does `INCLUDE memory.x` and supplies all the SECTIONS, so
   this file must define ONLY the MEMORY regions, and they must be named exactly
   RAM / ROM / VECTORS. msp430-rt derives `_stack_start = ORIGIN(RAM) +
   LENGTH(RAM)` itself (the Reset handler loads it into SP), so we no longer
   PROVIDE __stack_top or write our own SECTIONS/reset-vector glue. */
MEMORY
{
    RAM     (rw) : ORIGIN = 0x1C00, LENGTH = 0x0800  /* 2 KB SRAM */
    ROM     (rx) : ORIGIN = 0x4400, LENGTH = 0xBB80  /* 48 KB FRAM (0x4400-0xFF7F) */

    /* Interrupt vector table. The PAC's `__INTERRUPTS` array has 55 entries and
       msp430-rt appends the reset vector, so the table is 56 words (0x70 bytes)
       and must end at 0x10000 (msp430-rt ASSERTs this). Hence ORIGIN = 0xFF90,
       not 0xFF80 — the FR5969's lowest 8 vector slots are unused/reserved.
       Cross-check: the PAC pins SYSNMI(54) at 0xFFFC, so entry 0 sits at
       0xFFFC - 54*2 = 0xFF90. */
    VECTORS (r)  : ORIGIN = 0xFF90, LENGTH = 0x0070
}
