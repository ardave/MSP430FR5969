# PAC vs Datasheet Discrepancies

Discrepancies found between the MSP430FR5969 datasheet documentation
(`/Users/davidfalkner/git/MSP430FR5969_datasheet/`) and the PAC crate (`pac/`).

Work through these one at a time. Each item includes context to help you
research what's going on before deciding how to fix it.

---

## High Impact

### [ ] 1. Timer_A0 is missing 2 capture/compare channels

The real TA0 hardware has 5 CC registers (CCR0-CCR4), but the SVD file
(`msp430fr5969.svd`) only defines 3 (CCR0-CCR2). Because the PAC is
auto-generated from the SVD, it's also missing CCR3 and CCR4.

This means you can't access TA0CCR3/TA0CCR4 or TA0CCTL3/TA0CCTL4 through the
PAC at all. The PAC calls this peripheral `Timer0_A3` (implying 3 channels)
when it should be `Timer0_A5`.

- **Datasheet ref:** `reference/13_timer_a.md` lines 11-16, also
  `reference/02_memory_map.md` lines 466-480
- **SVD location:** `msp430fr5969.svd` around the TIMER_0_A3 peripheral
- **Missing registers:** TA0CCTL3 (offset 0x08), TA0CCTL4 (offset 0x0A),
  TA0CCR3 (offset 0x18), TA0CCR4 (offset 0x1A)
- **To fix:** Patch the SVD to add the missing registers, then regenerate
  the PAC with `svd2rust`

### [ ] 2. MPU (Memory Protection Unit) defined in SVD but missing from PAC

The MPU peripheral at base address 0x05A0 exists in the SVD file but
svd2rust silently dropped it during code generation. No `mpu` module, no
type alias, and no field in the `Peripherals` struct was generated.

This means you have no typed access to memory protection features (segment
boundaries, access permissions, IP encapsulation).

- **Datasheet ref:** `reference/02_memory_map.md` lines 316, 644-654
- **SVD location:** `msp430fr5969.svd` line 13790
- **Registers:** MPUCTL0, MPUCTL1, MPUSEGB1, MPUSEGB2, MPUSAM, MPUIPC0,
  MPUIPSEGB1, MPUIPSEGB2
- **To investigate:** Figure out why svd2rust skipped this peripheral.
  Possibly a malformed `<baseAddress>` or missing required field in the SVD.
  Compare the MPU `<peripheral>` block structure against a working one like
  PMM to spot the difference.

---

## Low Impact (Cosmetic / Modeling Differences)

### [ ] 3. Capacitive Touch I/O base address is the register, not the module

The datasheet says CapTouch I/O 0 has module base 0x0430 with its single
register (CAPTIO0CTL) at offset 0x0E = address 0x043E. The PAC uses 0x043E
as the base address directly. Same pattern for CapTouch I/O 1 (0x0470 vs
0x047E).

Functionally correct since there's only one register, but the PAC's base
address doesn't match the datasheet's module base.

- **Datasheet ref:** `reference/02_memory_map.md` lines 307-309, 533-538,
  554-556
- **PAC location:** `pac/src/lib.rs` lines 31822, 33670

### [ ] 4. Hardware multiplier split into two PAC peripherals

The datasheet documents one unified MPY module at base 0x04C0 covering both
16-bit and 32-bit operations. The SVD/PAC models this as two separate
peripherals: `Mpy16` at 0x04C0 and `Mpy32` at 0x04D0.

Not wrong — just a different way to organize the same registers. The 32-bit
registers do start at offset 0x10 from the module base.

- **Datasheet ref:** `reference/02_memory_map.md` lines 311, 586-612
- **PAC location:** `pac/src/lib.rs` lines 33845, 34369

### [ ] 5. DMA channels flattened into single peripheral

The datasheet documents DMA general control at 0x0500 and separate channel
bases at 0x0510/0x0520/0x0530. The PAC combines everything into one `Dma`
peripheral at 0x0500 with all channel registers in one RegisterBlock.

Again not wrong, just a different organizational choice.

- **Datasheet ref:** `reference/02_memory_map.md` lines 312-315, 614-641
- **PAC location:** `pac/src/lib.rs` line 34875
