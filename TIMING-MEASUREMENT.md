# Timing measurement — roadmap

Tracking the build-out of duration/interval measurement on Timer_A. Step 1 is
done; the rest is captured here so the thread can be picked up cold.

## The end goal

Measure elapsed time between two points — software-observed events *or* true
external/interrupt events — accurately, across arbitrary spans, and (eventually)
through deep sleep.

## ✅ Step 1 — free-running counter, single short interval (DONE)

Landed in [`hal/src/timer.rs`](hal/src/timer.rs) + demo in
[`hal_consumer/src/main.rs`](hal_consumer/src/main.rs).

- `Counter::new_smclk` puts Timer0_A3 in **continuous mode** on SMCLK÷N.
- `now()` snapshots the 16-bit `TA0R`; `elapsed_since()` does the
  `wrapping_sub` so a single interval is correct across one rollover.
- `ticks_to_us` / `ticks_to_ns` convert using the rate from `Clocks`.
- **Limit:** only intervals **shorter than one wrap** (65.5 ms at 1 MHz tick).
  No overflow counting, no pin capture, no sleep. Reads are software-timed
  (subject to instruction/interrupt jitter).

**Hardware-verified (2026-06-21).** Flashed via DSLite; counter on SMCLK 8 MHz
÷8 = 1 µs/tick. Used it to characterize the software `Delay` (rock-stable across
every sweep):

| req ms | measured µs | excess over spin |
|--------|-------------|------------------|
| 0      | 171         | 171 (apparatus)  |
| 1      | 3670        | ~2670            |
| 5      | 7909        | ~2909            |
| 10     | 13105       | ~3105            |
| 50     | 53497       | ~3497            |

Finding: `Delay` carries a **~2.5 ms fixed per-call overhead** plus ~1.7%
proportional bias — invisible at 1 s, dominant at 1–10 ms. `objdump` traced it
to the u64 time→cycles math: `delay_ms` calls `__muldi3` (64-bit multiply) and
`__mspabi_divull` (64-bit divide); the `req=0` row is cheap because the divide
short-circuits on a zero numerator. Concrete motivation for the hardware-timer
delay on the roadmap. (UART receiving cleanly is itself proof the 1 µs tick is
right — BRCLK is SMCLK, so a wrong SMCLK would garble characters.)

## ✅ Step 2 — overflow counting (extend the range; first real ISR) (DONE)

Goal: measure intervals longer than one 16-bit period by assembling a wider
32-bit tick count. **Hardware-verified (2026-06-21).**

- [x] Enable the counter-overflow interrupt: `Counter::enable_overflow_interrupt`
      sets `TAIE` in `TA0CTL`; the overflow sets `TAIFG` and fires `TIMER0_A1`.
- [x] **First real ISR** in `hal_consumer` via `#[msp430_rt::interrupt] fn
      TIMER0_A1`. `objdump` confirmed it ends in `reti` (the `msp430-interrupt`
      ABI) and that vector slot 44 (table offset 0xFF90 + 2·44 = 0xFFE8) holds
      the handler address, overriding only its own weak default. Needed
      `#![feature(abi_msp430_interrupt)]` in the consumer and a `pub mod
      interrupt { pub use pac::Interrupt::*; }` shim in the HAL (the macro
      name-checks `interrupt::TIMER0_A1`; this PAC exposes a `pac::Interrupt`
      enum, not a module).
- [x] Share a `u16` overflow counter via `critical_section::Mutex<Cell<u16>>`
      (`OVERFLOWS` in the consumer); ISR gets its `CriticalSection` from the
      macro, `main` via `critical_section::with`. Added `critical-section = "1"`
      to the consumer.
- [x] `Counter::now32(overflows)`: assembles `(ovf << 16) | TA0R`. Read race
      handled by calling it inside a CS and folding in a *pending-but-uncounted*
      overflow — if `TAIFG` is set, add one and re-read `TA0R` so the low half
      matches the bumped high half. `clear_overflow_irq()` (raw `TA0CTL` write)
      lets the ISR clear the flag without owning the `Counter`.
- [x] Widened `ticks_to_us`/`ticks_to_ns` to take `u32` ticks.
- [x] Demo: times a full ~1 s interval. **Result:** 16-bit-only delta reads
      22206 ticks (= 1005246 mod 65536, the step-1 failure), 32-bit `now32`
      reads 1005246 ticks = 1005246 µs. The ~5.2 ms over 1000 ms = ~2.5 ms
      `Delay` math overhead + ~15 overflow-ISR services inside the window
      (the counter rightly counts them). Stable across every reading.
- [x] Enabled interrupts globally: `unsafe { msp430::interrupt::enable() }` after
      the timer + ISR + UART are configured — the first GIE-set in the project.

## ✅ Step 3 — hardware capture (software-triggered; approach A) (DONE)

Goal: latch `TAxR` in hardware at an *event*, so the timestamp is immune to when
software reads it. **Hardware-verified (2026-06-21).** Approach **A**
(software-triggered, no pin) — a true external pin was deferred because no
LaunchPad button maps to a Timer0_A3 capture input and the one CCI1A pin
(`TA0.1` = P1.0) is the green LED.

- [x] CCR1 in **capture mode**: `Counter::configure_capture` sets `CAP=1`,
      `SCS=1` (sync), `CM=rising`, `CCIS=GND` (armed).
- [x] `Counter::software_capture()` manufactures the edge with no pin by toggling
      `CCIS` GND→VCC (capture) then back to GND (re-arm), and returns `TAxCCR1`.
      `capture_value()` re-reads the latch; `capture_overflowed()`/
      `clear_capture_overflow()` expose `COV` (missed-edge detect).
- [x] Demo contrasts a hardware capture against software reads taken 5 ms later.
      **Result (stable):** `capture jitter 17 us; 5ms-late read drifted 7933 us`.
      The capture tracked the trigger to 17 µs (a fixed ~17-instruction gap),
      while a counter read taken ~5 ms late was 7933 µs off (5000 requested +
      ~2.5 ms `Delay` fixed overhead from step 1 + instructions). Jitter ≪ drift
      is the point: the latch froze the event time regardless of read latency.
- [ ] Deferred to a follow-on (approach **B**): route a real pin (`TA0.1`/CCI1A
      = P1.0) to the capture input and service it via the `TIMER0_A1` CCR1
      interrupt (`CCIE`/`CCIFG`, decode `TA0IV`==0x02), with an interval helper
      that subtracts successive captures.

## ✅ Step 4 — measure through deep sleep (ACLK source) (DONE)

Goal: keep timing while the part is in LPM3. **Hardware-verified (2026-06-21).**

- [x] `Counter::new_aclk` sources the counter from **ACLK** (`TASSEL=1`). With
      `clocks::configure_low_power` that is the 32.768 kHz LFXT crystal (~30.5 µs
      tick, ~2 s wrap), which keeps running in LPM3 — unlike the SMCLK source.
- [x] Wake mechanism = **CCR0 compare** (the dual of step 3's capture):
      `Counter::schedule_wake_in(interval)` writes `TA0CCR0` and enables `CCIE`; the
      match fires the dedicated **`TIMER0_A0`** vector. ISR is
      `#[msp430_rt::interrupt(wake_cpu)]` — `objdump` shows it opens with
      `bic.b #0xF0, 0(r1)`, clearing CPUOFF|OSCOFF|SCG0|SCG1 on the *stacked* SR
      so RETI returns to active mode. It calls `clear_wake_irq()` (clears `CCIE`;
      CCR0's `CCIFG` auto-clears on this single-source vector).
- [x] `power::enter_lpm3()` — new `hal::power` module; one `bis #0xD8, r2`
      (CPUOFF+SCG0+SCG1+GIE) atomically sleeps with interrupts on. OSCOFF stays
      0 so the crystal lives. Consumer needed BOTH `#![feature(
      abi_msp430_interrupt)]` and `#![feature(asm_experimental_arch)]` (the
      `wake_cpu` variant emits a naked-asm trampoline).
- [x] now32 + the step-2 overflow ISR keep tallying *during* sleep: an ACLK
      overflow briefly wakes the CPU to run the plain `TIMER0_A1` handler, which
      RETIs back to LPM3. So sleeps longer than one ~2 s wrap still measure right.
- [x] Demo: sleep ~1 s in LPM3, measure on wake. **Result (stable):** `slept in
      LPM3, measured 1000091 us across deep sleep` (target 32768 ticks = 1.000 s;
      ~3 ticks = wake latency + read/schedule gap). Readings quantize in ±31 µs
      steps = one 32.768 kHz tick (30.5 µs), *not* a VLO tick (~106 µs) —
      conclusive proof the crystal drove the count through deep sleep.

## ◐ Step 5 — polish

- [x] **Host-side unit test for the tick↔time math** (done 2026-06-21). The pure
      arithmetic was extracted from `Counter`'s methods into a dependency-free
      `hal/src/ticks.rs` (`ticks_to_us`, `ticks_to_ns`, `us_to_ticks`,
      `assemble_now32`), mirroring `baud.rs`. New detached crate `timer-test/`
      `include!`s that source and checks it on the host (`cd timer-test && cargo
      +nightly test` — 10 tests). Covers exact conversions at the project's real
      rates (8 MHz, 1 MHz, 32768 Hz, VLO), the `u64`-widening overflow guard,
      truncation, the `us_to_ticks` one-wrap range check, the `now32`
      bit-packing, and reproduces the exact Step 4 hardware readings
      (32771→1000091, 32772→1000122 µs). Verified it fails on a wrong-but-
      compiling edit, so it genuinely guards the shipping source.
- [ ] Consider an `embedded-hal` trait impl if one fits
      (`embedded_hal::delay` is for delays, not measurement; a count-down /
      capture trait may suit — evaluate when the API settles).
- [ ] Generalize beyond Timer0_A3 if a second timer is ever needed.

## Reference

- SLAU367P (MSP430FR5969 family UG): Timer_A chapter — `TAxCTL`, `TAxR`,
  `TAxCCTLn`, `TAxCCRn`, `TAxIV`, `TAxEX0`.
- PAC accessors: `p.timer_0_a3.ta0ctl()` / `.ta0r()` / `.ta0cctl0()` etc.
- Vectors: `TIMER0_A0` (CCR0 only) and `TIMER0_A1` (TAIFG + CCR1/2, via TA0IV).
