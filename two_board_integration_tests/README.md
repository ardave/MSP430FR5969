# Two-LaunchPad integration rig

Two MSP-EXP430FR5969 LaunchPads, permanently wired together, testing each
other. A single board — even with loopback jumpers — can only ever talk to
itself: same clock, same silicon, and peripherals like the I2C **slave**
simply have no counterpart to answer. Wiring two boards together gives every
suite a genuinely independent partner: a second clock domain, a second
calibrated analog chain, a real external interrupt source, and a live bus
master for the slave driver's first-ever silicon verification.

Two crates implement it, mirroring the single-board pattern
(`hal_integration_tests` + `hal_test_runners`):

- **`two_board_test_runners`** (workspace member) — ONE firmware fixture,
  `two_board_fixture`, flashed identically to BOTH boards. It is a command
  server on the eUSCI_A0 USB backchannel (9600 8N1): the host sends
  single-byte commands, the board answers with framed report/verdict lines.
- **`two_board_integration_tests`** (this crate, detached from the workspace
  like the other host-side crates) — builds and flashes the fixture to both
  boards, discovers which is which, and drives all the cross-board suites.

## Board identity: parent and child

The wiring is *almost* symmetric, but the I2C pull-ups hang off one board's
3V3 and the suites need a stable way to say "the board wired as X". That
identity must survive USB replugging (macOS device paths encode the physical
USB port, not the board) **and** reflashing. So it lives in each board's
**Info FRAM at offset 0xA0** (`'2' 'B' 'P'|'C' 0`) — Info FRAM isn't part of
the flashed image, so DSLite never touches it, and the eZ-FET's USB
enumeration is irrelevant: every run *asks* each board who it is (`i` →
`2B_ID role=parent fw=1`) and pairs the serial ports from the answers.

Provision once, one board at a time (that's how you and the tooling agree on
which physical board gets which name):

```sh
cd two_board_integration_tests
# only the board that will be PARENT attached:
cargo +nightly run -- provision parent
# swap cables: only the board that will be CHILD attached:
cargo +nightly run -- provision child
```

Then label the boards with a marker. The parent is the board whose 3V3
(J4.1, the top-left header pin) sources the two I2C pull-up resistors.

## Wiring

Run `cargo +nightly run -- wiring` for the authoritative table — the same
banner is printed at the start of **every** run, so any captured test log
doubles as build instructions for the rig. Summary:

| # | Parent | | Child | Series | Purpose |
|---|--------|-|-------|--------|---------|
| W1 | GND (J5.20) | ↔ | GND (J5.20) | wire | Common ground — connect FIRST |
| W2 | P1.6 SDA (J5.15) | ↔ | P1.6 SDA (J5.15) | 1.0 kΩ | I2C, eUSCI_B0 master↔slave |
| W3 | P1.7 SCL (J5.14) | ↔ | P1.7 SCL (J5.14) | 1.0 kΩ | I2C, eUSCI_B0 master↔slave |
| R1/R2 | 3V3 (J4.1) | | — | 10 kΩ ×2 | Pull-ups to the parent's P1.6 SDA (J5.15) and P1.7 SCL (J5.14) nodes |
| W4 | P2.5 TXD (J4.4) | → | P2.6 RXD (J4.3) | 2.2 kΩ | UART cross-link (eUSCI_A1) |
| W5 | P2.6 RXD (J4.3) | ← | P2.5 TXD (J4.4) | 2.2 kΩ | UART cross-link (eUSCI_A1) |
| W6 | P3.4 (J4.8) | → | P3.5 (J4.9) | 2.2 kΩ | GPIO edge interrupts, LPM4 wake |
| W7 | P3.5 (J4.9) | ← | P3.4 (J4.8) | 2.2 kΩ | GPIO edge interrupts, LPM4 wake |
| W8 | P1.4 TB0.1 (J5.12) | → | P1.2 TA1.CCI1A (J5.19) | 2.2 kΩ | PWM → timer capture |
| W9 | P1.2 (J5.19) | ← | P1.4 (J5.12) | 2.2 kΩ | PWM → timer capture |
| W10 | P1.5 TB0.2 (J5.13) | → | P2.4 A7 (J4.6) | 2.2 kΩ | *Reserved*: future PWM-RC DAC → ADC (see below) |
| W11 | P2.4 A7 (J4.6) | ← | P1.5 (J5.13) | 2.2 kΩ | *Reserved*: future PWM-RC DAC → ADC (see below) |
| W12 | P2.2 (J4.7) | ↔ | P2.2 (J4.7) | 2.2 kΩ | *Reserved*: future eUSCI_B0 SPI CLK (master↔slave SPI would reuse W2/W3 as SIMO/SOMI; needs an SPI-slave driver in the HAL first) |

R1/R2 are the only row that isn't a board-to-board wire: they are the I2C
pull-ups, two separate 10 kΩ resistors on the breadboard — R1 from the
parent's 3V3 (J4.1) to the SDA node, i.e. the net of the **parent's P1.6
(J5.15)** pin, on the *parent* side of W2's 1 kΩ series resistor; R2 from
the same 3V3 pin to the SCL node, the net of the **parent's P1.7 (J5.14)**
pin, on the parent side of W3's. Nothing about
them touches the child board; sourcing them from exactly one board's rail
is what keeps the supplies separate — and is the asymmetry that *defines*
which board is the parent.

### Finding the pins

"J4" is the **left** BoosterPack header and "J5" the **right** one (USB
connector at the top). J4/J5 are the board schematic's connector
designators, and the pin numbers are the 20-pin BoosterPack standard's
positions — the same combination TI itself uses ("J4 Pin 1 (Vcc)",
"J5 Pin 20 (GND)", SLAU535B §2.4.4). Positions 1–10 run *down* the left
header and 20–11 run *down* the right header, so **pin 20 (GND) is the
top-right pin**, directly across from pin 1 (3V3) at the top left; the
silkscreen prints "1" and "20" at the header tops. Full map, top→bottom
(Rev 2.0 boards, pinout per SLAU535B Fig. 15):

- **J4 (left):** 1 3V3, 2 P4.2, 3 P2.6, 4 P2.5, 5 P4.3, 6 P2.4, 7 P2.2, 8 P3.4, 9 P3.5, 10 P3.6
- **J5 (right):** 20 GND, 19 P1.2, 18 P3.0, 17 NC, 16 RST, 15 P1.6, 14 P1.7, 13 P1.5, 12 P1.4, 11 P1.3

Two traps to avoid. First, match pins by their silkscreened **port names**
(P1.6, GND, RST…), not the BoosterPack *function* labels also printed
there: P1.6/P1.7 are silkscreened MOSI/MISO, and the left header's
P3.5/P3.6 are silkscreened SCL/SDA — this rig's I2C does **not** go there
(the hardware eUSCI_B0 bus is P1.6/P1.7). Second, the board's silkscreen
also has labels named "J1" (power hooks, bottom right) and "J2" (super-cap
Bypass/Use jumper, mid-board) — those are **different connectors**, not
the BoosterPack headers; wire nothing to either. (Neither the user's guide
nor the silkscreen prints "J4.x"-style pin labels — Fig. 15 identifies
pins by signal name only, which is why the port names are the ground
truth.)

**No capacitors are required.** W10/W11 are wired now so the rig never needs
rewiring, but their suite (`adc_dac`) is a **future addition**: it needs a
~10 µF cap (4.7–47 µF, any type, ≥6.3 V, + toward the pin) from each
*receiving* board's A7 pin (J4.6) to that board's GND — with the 2.2 kΩ
series R, the RC that turns the peer's PWM into a DC level. Fitting the caps
is a ten-second breadboard retrofit at the pin; until then `adc_dac` simply
never runs by default.

### Why this cannot damage anything

- **One driver per wire, by construction.** The wires come in crossed pairs
  (out→in each way), so each pin has a single fixed direction that both
  boards' firmware always configures identically — no command sequence puts
  two push-pull drivers on one wire. I2C is open-drain on both ends by
  protocol.
- **Every signal passes through a series resistor** sized against the
  MSP430FR5969's ±2 mA absolute-maximum pin clamp current (SLAS704G §5.1):
  2.2 kΩ bounds *any* fault — miswiring during setup, both-drive contention,
  a powered board driving an unpowered one — to <1.7 mA. The I2C pair uses
  1.0 kΩ (for solid logic-low margin against the 10 kΩ pull-ups: low reads
  ~0.33 V vs the 0.75 V worst-case threshold floor); its unpowered-board
  exposure is ≤0.4 mA because nothing ever drives the bus high.
- **The supply rails are never tied together.** Each board keeps its own
  eZ-FET LDO (~3.6 V — measured, not the guide's nominal 3.3 V); connecting
  them would back-drive one regulator from the other (back-powering through
  the rail is exactly the hazard class SLAU535B §2.4.4 flags when anything
  external drives V<sub>cc</sub> — TI's mitigation there is pulling the J9
  current-measurement jumper). Only the parent's 3V3 sources the two 10 kΩ
  pull-ups — a ≤0.36 mA soft path, harmless in every power state.
- **Nothing connects to 5 V, RST, or the emulator section.** USB VBUS (5 V)
  exists only on the eZ-FET half (J7 and the J13 isolation block) — never
  wire to J3/J7/J13. RST is J5.16; J5.17 (NC on the pinout, a TEST stub on
  the schematic) stays empty; and the silkscreened J1 power hooks and J2
  super-cap jumper get no wires.
- Keep both boards USB-powered whenever either is (same workstation or hub;
  W1 is still mandatory — USB ground is not a signal return).

## Running

```sh
cd two_board_integration_tests
cargo +nightly run                       # build + flash both + all suites
cargo +nightly run -- --no-flash pwm_cross   # one suite, skip reflash
cargo +nightly run -- wiring             # print the hookup table only
cargo +nightly run -- identify           # who is on which /dev node?
```

`TWO_BOARD_PARENT_PORT` / `TWO_BOARD_CHILD_PORT` pin the backchannel device
nodes explicitly if `/dev` scanning finds the wrong candidates (identity is
still verified over the wire — the env names are just search hints).

### Flashing two attached probes

`DSLite load` has no probe-selection flag, so the runner generates one ccxml
per USB FET under `target/two_board/`. TI's MSP430-USB connection selects a
probe by **enumeration index** via the `portAddr1` property, encoded
`100 + N` (TI ships `TIMSP430-USB.xml` = 101, `-USB2.xml` = 102,
`-USB3.xml` = 103; feed it anything non-numeric and libmsp430_emu fails with
"Tried to initialize USB FET number %u, but only found %d USB FETs"). The
runner flashes FET #1 then FET #2; which index is which physical board is
irrelevant because both get the identical binary, and the `identity` suite
verifies both ends report the same firmware revision afterwards. Fallback if
index selection ever misbehaves — flash one board at a time:

```sh
cargo +nightly run -- flash    # with only one board's USB attached
# swap cables, repeat, then:
cargo +nightly run -- --no-flash
```

(The board-to-board wiring never has to change for any of this; only USB
cables move.)

## Suites

| Suite | What only two boards can prove |
|---|---|
| `identity` | Both boards answer with their FRAM role and the same firmware revision (catches a half-flashed rig before anything can mislead). |
| `i2c_bridge` | **First silicon verification of the HAL's eUSCI_B0 I2C slave.** Child serves a 16-register file at 0x48; parent (HAL master) runs probe / empty-address NACK / ID read / write-read with pointer autoincrement / read-only-register enforcement / pointer wrap via a standalone read. Back-to-back reads double as the speculative-TXBUF-flush check. The child's event tally is asserted exactly (8 transactions: 4 write-phase, 4 read-phase). |
| `uart_link` | eUSCI_A1 ↔ eUSCI_A1 at 9600, echo+1 both directions of initiation: framing across two *independent* DCOs — a real baud-tolerance test a loopback jumper (same clock both ways) cannot perform. |
| `gpio_edge` | Ten genuine wire edges (not software-set `PxIFG`) counted through the PORT3 ISR and `PxIV` demux, each direction, zero stray IV slots. |
| `lpm4_wake` | One board parks in LPM4 (every clock stopped); the *other board* wakes it with a single edge — a truly external wake, hands-free. Exactly one edge tallied. |
| `pwm_cross` | 1 kHz Timer_B0 PWM measured by the peer's Timer_A1 capture: frequency gate ±5 % (the two DCOs measured against each other), 25 %/75 % duty points (asymmetric, so inversion/transposition fails), both directions. |
| `adc_dac` *(name-only; future addition — needs the W10/W11 caps)* | The generator's PWM through the rig's RC becomes DC; the measurer reports it in millivolts. Expected = duty × the generator's own ADC-measured rail — one assertion through **both** chips' calibrated analog chains, at two duty points, both directions. Run with `cargo +nightly run -- adc_dac` once the caps are fitted. |

All suites are hands-free once the rig is built; all but `adc_dac` run by
default (`adc_dac` is name-only until its caps exist). **Status:
code-complete, compiles for both targets; on-hardware verification pending**
(the rig's first physical build). Timing constants that may need on-silicon
tuning: the ±5 % cross-DCO frequency gate, the ADC tolerance (±5 % + 30 mV),
and the LPM4 500 ms settle.

## Firmware command protocol

See the module docs in
`two_board_test_runners/src/bin/two_board_fixture.rs` for the full
command/response table (`i`, `P`/`C`, `s`/`m`, `e`/`t`, `g`/`p`/`1`/`w`,
`f`/`F`/`d`/`D`/`x`/`c`/`a`, `q`). Design points worth knowing:

- The fixture never blocks against the host: sub-modes (I2C slave, UART
  echo, edge counting) poll the backchannel for `q` on every loop.
- Cross-board driver pins are parked (0 % PWM = pin low via `OUTMOD=0`,
  pulse line low, eUSCI_A1/B0 built lazily on first use) so an idle board
  presents defined, current-free levels to the wires.
- `w` (LPM4) is the one command without a timeout: if the peer never pulses,
  the board sleeps until reset — the host orders the sequence so that can't
  happen unattended.

## Future work on the same wiring

- **Fit the W10/W11 caps and promote `adc_dac`** into the default set: two
  ~10 µF caps (salvage-grade is fine) are the entire bill of materials for a
  cross-board absolute-analog test through both chips' calibrated ADC/REF
  chains — a nice addition when capacitors are on hand.
- **Cross-board SPI** over W2/W3/W12 (eUSCI_B0 SIMO/SOMI/CLK, straight-through
  is exactly what master↔slave SPI wants) once the HAL grows an SPI-slave
  driver.
- **115200 cross-link** variant of `uart_link` to probe where two-DCO baud
  tolerance actually runs out.
- I2C slave interrupt-driven mode (`SlaveInterrupts` + `USCI_B0` vector)
  under the same bridge suite.
