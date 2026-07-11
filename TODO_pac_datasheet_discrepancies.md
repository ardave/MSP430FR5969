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

Settled on silicon 2026-07-11 by a one-off probe fixture (committed and then
removed once the question was answered — see commit 8f1bc5f on the
`ta0_ccr3_probe` branch for the code and method): raw volatile-pointer probes
at the putative addresses (CCTL3/4 at 0x0348/0x034A, CCR3/4 at
0x0358/0x035A), three independent checks per channel that had to agree —
register write/read-back, a functional software-CCIS capture with the stamp
bracketed by TA0R reads, and the TA0IV demux (0x06/0x08 slots) with only the
probed channel's CCIE armed — with channel 2 run through the identical code
as the positive control (readback stuck, capture fired bracketed, IV = 0x04)
and an alias check on CCR0-CCR2. Observed for both channels: registers hold
no state, CCIFG never latches, TA0IV reads 0, and nothing aliases onto the
real channels
(`ta0 ch3 rbccr=0 rbctl=0 cap=0 brk=0 iv=00 | ch4 rbccr=0 rbctl=0 cap=0 brk=0 iv=00 | alias=1`).

### [x] 2. ~~MPU defined in SVD but missing from PAC~~ — RESOLVED: regenerated 2026-07-11; svd2rust never dropped anything

**svd2rust was innocent.** Reproducing the generation (installed svd2rust
0.37.1 — the exact version the old PAC's doc header named — against the
checked-in SVD, `--target msp430`) emitted the MPU block correctly, with the
only peripheral-level diff vs the old checked-in PAC being exactly `Mpu`.
Since the SVD has carried the MPU since the initial commit, the old PAC must
have been generated from a *different, earlier SVD file* than the one
checked in — a generation-input mismatch, not a generator bug.

The PAC has been regenerated (v0.2.0): `mpu` module at 0x05A0 with all eight
registers, `Peripherals.mpu`, plus two incidental fixes the msp430 target
flavor emits at source (the vector-table `Vector._reserved` as `u16` and
`extern "msp430-interrupt"` handler declarations — retiring the old
hand-patches; one small rt-gating patch remains, see CLAUDE.md "PAC
generation"). `hal::mpu` now owns `pac::Mpu` (consume-by-move,
`Mpu::new(p.mpu)`) with typed reads and typed writes to the plain 16-bit
registers. **`MPUCTL0` deliberately stays raw byte lanes**: the SVD does not
model the `MPUPW` password byte (reads `0x96`, word writes must carry
`0xA5`), so a PAC `modify()` on it would echo `0x96` as the key and PUC —
never touch `MPUCTL0` through the PAC field API. All 8 `mpu` HiL suite
verdicts (SYSNMI demux, violation PUC, lock-until-BOR) re-verified on
hardware 2026-07-11 through the typed driver.

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
