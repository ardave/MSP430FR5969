# PAC vs Datasheet Discrepancies

Discrepancies found between the MSP430FR5969 datasheet documentation
(`/Users/davidfalkner/git/MSP430FR5969_datasheet/`) and the PAC crate (`pac/`).

Work through these one at a time. Each item includes context to help you
research what's going on before deciding how to fix it.

---

## High Impact

### [x] 1. ~~Timer_A0 is missing 2 capture/compare channels~~ — RESOLVED: datasheet erratum, not an SVD gap (HW-established 2026-07-11)

**The SVD and PAC are correct; SLAS704G's TA0 register table is the erratum.**
TA0 on this silicon has exactly 3 CC channels (CCR0-CCR2) — there is nothing
to add to the SVD, the PAC, or the HAL.

The claim above came from the datasheet's TA0 register-offset table
(`reference/02_memory_map.md` lines 466-480), which lists TA0CCTL3/4 and
TA0CCR3/4 — but SLAS704G contradicts itself: its own §6.10.10 prose says
"TA0 and TA1 ... with **three** capture/compare registers each", and its
Table 6-13 signal connections define CCR0-CCR2 only. TI's `msp430fr5969.h`
(`Timer0_A3`, CCR0-2 only) and the SVD both agree with the prose. The
register table was most plausibly copy-pasted from a sibling part with a
five-channel TA0.

Settled on silicon by `--bin ta0_probe_test_runner` (host suite `ta0_probe`,
in the default set): raw-pointer probes at the putative addresses
(0x0348/0x034A, 0x0358/0x035A) with channel 2 as positive control. Result:
registers hold no state, a software-CCIS capture never fires CCIFG, TA0IV
never presents 0x06/0x08, and nothing aliases onto CCR0-CCR2. The suite
stays in the default set with the ABSENT findings pinned, so a different
die revision (or a probe regression) fails loudly.

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
