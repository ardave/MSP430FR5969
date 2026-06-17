/* MSP430FR5969 Memory Layout */
MEMORY
{
    RAM  (rw)  : ORIGIN = 0x1C00, LENGTH = 0x0800  /* 2KB SRAM */
    FRAM (rx)  : ORIGIN = 0x4400, LENGTH = 0xBB80  /* 48KB FRAM (0x4400-0xFF7F) */
    VECTORS (r): ORIGIN = 0xFF80, LENGTH = 0x007E  /* Interrupt vector table */
    RESETVEC(r): ORIGIN = 0xFFFE, LENGTH = 0x0002  /* Reset vector */
}

SECTIONS
{
    .text : ALIGN(2)
    {
        *(.text .text.*)
    } > FRAM

    .rodata : ALIGN(2)
    {
        *(.rodata .rodata.*)
    } > FRAM

    .data : ALIGN(2)
    {
        *(.data .data.*)
    } > RAM AT > FRAM

    .bss : ALIGN(2)
    {
        __bss_start = .;
        *(.bss .bss.*)
        __bss_end = .;
    } > RAM

    .vectors : ALIGN(2)
    {
        KEEP(*(.vector_table))
    } > VECTORS

    .resetvec :
    {
        KEEP(*(.reset_vector))
    } > RESETVEC
}
