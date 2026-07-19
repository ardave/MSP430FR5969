//! The permanent two-board wiring table, printed at the start of every run so
//! anyone can reproduce the rig from the test output alone.
//!
//! Header naming: "J4.x" (left header, positions 1-10) / "J5.x" (right
//! header, positions 20-11). J4/J5 are the board schematic's connector
//! designators; the position numbers are the 20-pin BoosterPack standard's,
//! and TI's user's guide combines them the same way (SLAU535B sec. 2.4.4:
//! "J4 Pin 1 (Vcc)", "J5 Pin 20 (GND)"). SLAU535B Figure 15 shows the same
//! pinout by signal name only — neither the guide nor the silkscreen prints
//! "J4.x"-style labels, so locate pins by their silkscreened port names.
//! CAUTION: the silkscreen labels named "J1" (power hooks, bottom right) and
//! "J2" (super-cap Bypass/Use jumper) are unrelated connectors, NOT the
//! BoosterPack headers. Pin functions verified against SLAU535B Rev 2.0
//! (Fig. 15 + schematics) and the MSP430FR5969 datasheet (SLAS704G).

/// Print the full hookup specification.
pub fn print_banner() {
    println!(
        r#"
================================================================================
  MSP430FR5969 two-LaunchPad integration rig — permanent wiring (Rev 2.0 boards)
================================================================================

  Both boards are plugged into the SAME workstation/hub via their own USB
  cables. One board is provisioned "parent", the other "child" (identity is
  stored in each board's Info FRAM by `provision`, so it survives replugging
  and reflashing). Wire once; never change.

  Parts: 9x 2.2 kOhm, 2x 1.0 kOhm, 2x 10 kOhm resistors, jumper wires, and a
  small breadboard between the boards. (No capacitors required -- see the
  future-addition note under W10/W11.)
  EVERY signal wire passes through its series resistor on the breadboard.

  FINDING PINS (hold the board with the USB connector at the top):
  J4 = LEFT BoosterPack header, J5 = RIGHT (the schematic's designators; TI
  numbers their pins by BoosterPack position, e.g. "J4 Pin 1 (Vcc)" and
  "J5 Pin 20 (GND)" in SLAU535B sec. 2.4.4). Positions 1-10 run DOWN the
  left header and 20-11 run DOWN the right header, so pin 20 (GND) is the
  TOP-RIGHT pin, directly across from pin 1 (3V3) at the top left; the
  silkscreen prints "1" and "20" at the header tops.

    J4 (left),  top->bottom:  1 3V3  | 2 P4.2 | 3 P2.6 | 4 P2.5 | 5 P4.3
                              6 P2.4 | 7 P2.2 | 8 P3.4 | 9 P3.5 | 10 P3.6
    J5 (right), top->bottom: 20 GND  | 19 P1.2 | 18 P3.0 | 17 NC | 16 RST
                             15 P1.6 | 14 P1.7 | 13 P1.5 | 12 P1.4 | 11 P1.3

  Match pins by their silkscreened PORT names (P1.6, GND, RST, ...), not by
  the BoosterPack function labels also printed there: P1.6/P1.7 read
  MOSI/MISO, and the left header's P3.5/P3.6 read SCL/SDA -- this rig's I2C
  does NOT go there; it runs on P1.6/P1.7 (hardware eUSCI_B0).
  The silkscreen labels "J1" (power hooks, bottom right) and "J2" (super-cap
  Bypass/Use jumper, mid-board) are DIFFERENT connectors -- wire nothing to
  either.

  W#   PARENT pin              series        CHILD pin               tests
  ---  ----------------------  ------------  ----------------------  -----------------------
  W1   GND        J5.20        plain wire    GND        J5.20        common ground -- FIRST
  W2   P1.6 SDA   J5.15        1.0 kOhm      P1.6 SDA   J5.15        I2C master<->slave (eUSCI_B0)
  W3   P1.7 SCL   J5.14        1.0 kOhm      P1.7 SCL   J5.14        I2C master<->slave (eUSCI_B0)
  R1   3V3        J4.1   --- 10 kOhm pull-up to the PARENT's own P1.6 SDA
                              pin (J5.15): the parent side of W2's 1 kOhm
  R2   3V3        J4.1   --- 10 kOhm pull-up to the PARENT's own P1.7 SCL
                              pin (J5.14): the parent side of W3's 1 kOhm
  W4   P2.5 TXD   J4.4   -->   2.2 kOhm      P2.6 RXD   J4.3         UART cross-link (eUSCI_A1)
  W5   P2.6 RXD   J4.3   <--   2.2 kOhm      P2.5 TXD   J4.4         UART cross-link (eUSCI_A1)
  W6   P3.4 out   J4.8   -->   2.2 kOhm      P3.5 in    J4.9         GPIO edge irq + LPM4 wake
  W7   P3.5 in    J4.9   <--   2.2 kOhm      P3.4 out   J4.8         GPIO edge irq + LPM4 wake
  W8   P1.4 TB0.1 J5.12  -->   2.2 kOhm      P1.2 CCI1A J5.19        PWM -> timer capture
  W9   P1.2 CCI1A J5.19  <--   2.2 kOhm      P1.4 TB0.1 J5.12        PWM -> timer capture
  W10  P1.5 TB0.2 J5.13  -->   2.2 kOhm      P2.4 A7    J4.6         (reserved: future PWM-RC
  W11  P2.4 A7    J4.6   <--   2.2 kOhm      P1.5 TB0.2 J5.13         DAC -> ADC, see note)
  W12  P2.2 CLK   J4.7         2.2 kOhm      P2.2 CLK   J4.7         (reserved: future B0 SPI
                                                                      master<->slave over W2/W3)

  W10/W11 future addition: adding a ~10 uF cap (4.7-47 uF, any type, >=6.3 V,
  + toward the pin) from each RECEIVING board's A7 pin (J4.6) to that board's
  GND turns the peer's PWM into a DC level (RC with the 2.2 kOhm series R)
  and enables the name-only `adc_dac` suite -- a cross-board absolute-analog
  check through both chips' calibrated ADC/REF chains. Wire W10/W11 now
  regardless: the caps are a ten-second retrofit at the pin, no rewiring.

  Direction key: "-->" = left side drives, right side listens; wires come in
  crossed pairs so each pin has ONE fixed direction on BOTH boards -- no
  command sequence can ever put two push-pull drivers on one wire.

  SAFETY -- why this cannot damage anything (MSP430FR5969 abs max: +/-2 mA
  pin clamp current, SLAS704G 5.1):
   * 2.2 kOhm in series bounds ANY fault -- both-drive contention, a powered
     board driving an unpowered one, rail mismatch -- to < 1.7 mA per pin.
   * I2C (W2/W3) is open-drain by protocol: nothing ever drives it high, so
     1.0 kOhm + 10 kOhm pull-ups keep logic-low at ~0.33 V (spec floor for
     reading low is 0.75 V) while the unpowered-board case sees < 0.4 mA.
   * The two boards' rails are NEVER tied together: each eZ-FET LDO feeds its
     own board (~3.6 V); only the parent's 3V3 sources the two I2C pull-ups.

  NEVER connect between the boards: 3V3<->3V3, anything carrying 5 V (USB
  VBUS lives on the eZ-FET section: J7 and the J13 jumper block), RST
  (J5.16), J5.17 (NC on the pinout, a TEST stub on the schematic -- leave
  empty), any pin of the J3/J13 emulator blocks, or the silkscreened J1
  power hooks / J2 super-cap jumper. Power BOTH boards whenever either is
  powered (idle firmware parks every cross-line low/high-Z, so a
  half-powered rig is still within spec -- but don't make a habit of it).
================================================================================
"#
    );
}
