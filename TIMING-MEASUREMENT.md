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

## ☐ Step 2 — overflow counting (extend the range; first real ISR)

Goal: measure intervals longer than one 16-bit period by assembling a wider
(32-bit+) tick count.

- [ ] Enable the counter-overflow interrupt: set `TAIE` in `TA0CTL`; the
      overflow sets `TAIFG` and fires the **`TIMER0_A1`** vector (the shared
      vector — `TAIFG` plus CCR1/CCR2, decoded via the `TA0IV` register).
- [ ] Write the project's **first real ISR** with msp430-rt's `#[interrupt]`
      macro. This is the moment to deal with the two CLAUDE.md "Shortcomings":
      verify the `extern "msp430-interrupt"` ABI / `RETI` is emitted, and that
      the vector is patched by symbol name.
- [ ] Share a `u16` overflow counter between the ISR and `main` via a
      `critical-section`-guarded static (`Mutex<Cell<u16>>`) — CS support is
      already wired up (`use msp430 as _`).
- [ ] Compose `now64() = (overflow << 16) | TA0R`, handling the
      **read race**: TAIFG can fire between reading the high word and the low
      word. Standard fix — re-read until two consecutive high-word reads agree,
      or read TA0R, then overflow, then TA0R again and reconcile.
- [ ] Widen the `ticks_to_*` helpers to `u64` ticks.
- [ ] Demo: time one of the ~1 s blinks end-to-end and confirm it now reads
      correctly (Step 1 deliberately couldn't).
- [ ] Enable interrupts globally (`__enable_interrupt()` / set GIE) in the
      consumer — nothing in the project does this yet.

## ☐ Step 3 — hardware capture for external events (jitter-free)

Goal: timestamp real edges on a pin with zero software jitter — the right tool
for "interval between two external events/interrupts."

- [ ] Put a CCR channel in **capture mode** (`CAP=1` in `TA0CCTLn`); select edge
      (`CM`), input (`CCIS`), and synchronize (`SCS=1`). Hardware latches `TA0R`
      into `TA0CCRn` on the edge.
- [ ] Service the capture interrupt (`CCIE`/`CCIFG`); read `TA0CCRn` for the
      timestamp. Check the `COV` (capture-overflow) bit to detect a missed edge.
- [ ] Route a launchpad pin (e.g. a button on P1.1, or loop a GPIO output back)
      to the timer's CCI input for a self-contained demo.
- [ ] API sketch: `Counter::capture(channel, edge)` returning the latched ticks,
      and/or an interval helper that subtracts successive captures.

## ☐ Step 4 — measure through deep sleep (ACLK source)

Goal: keep timing while the part is in LPM3.

- [ ] Add `Counter::new_aclk` sourcing the counter from **ACLK** (the 32.768 kHz
      LFXT crystal via `clocks::configure_low_power`), which keeps running in
      LPM3 — unlike SMCLK, which Step 1 uses and which is gated off in sleep.
- [ ] Resolution becomes ~30.5 µs; combine with Step 2 overflow counting for
      long sleep intervals.
- [ ] Demo: sleep in LPM3, wake on a timer/pin event, report the slept duration.

## ☐ Step 5 — polish

- [ ] Host-side unit test for the tick↔time math, `baud-test`-style (pure
      arithmetic; guards the divider/overflow conversion).
- [ ] Consider an `embedded-hal` trait impl if one fits
      (`embedded_hal::delay` is for delays, not measurement; a count-down /
      capture trait may suit — evaluate when the API settles).
- [ ] Generalize beyond Timer0_A3 if a second timer is ever needed.

## Reference

- SLAU367P (MSP430FR5969 family UG): Timer_A chapter — `TAxCTL`, `TAxR`,
  `TAxCCTLn`, `TAxCCRn`, `TAxIV`, `TAxEX0`.
- PAC accessors: `p.timer_0_a3.ta0ctl()` / `.ta0r()` / `.ta0cctl0()` etc.
- Vectors: `TIMER0_A0` (CCR0 only) and `TIMER0_A1` (TAIFG + CCR1/2, via TA0IV).
