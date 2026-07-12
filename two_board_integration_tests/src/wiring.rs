//! The permanent two-board wiring table, printed at the start of every run so
//! anyone can reproduce the rig from the test output alone.
//!
//! Header naming: "J1.x" (left, 10 pins) / "J2.x" (right, 10 pins) are the
//! BoosterPack-standard positions from SLAU535B Figure 15 (the board's
//! *schematic* calls the same connectors J4/J5). Pin functions verified
//! against SLAU535B Rev 2.0 and the MSP430FR5969 datasheet (SLAS704G).

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

  W#   PARENT pin              series        CHILD pin               tests
  ---  ----------------------  ------------  ----------------------  -----------------------
  W1   GND        J2.20        plain wire    GND        J2.20        common ground -- FIRST
  W2   P1.6 SDA   J2.15        1.0 kOhm      P1.6 SDA   J2.15        I2C master<->slave (eUSCI_B0)
  W3   P1.7 SCL   J2.14        1.0 kOhm      P1.7 SCL   J2.14        I2C master<->slave (eUSCI_B0)
  R1   3V3        J1.1   --- 10 kOhm pull-up to W2's PARENT-side node (SDA)
  R2   3V3        J1.1   --- 10 kOhm pull-up to W3's PARENT-side node (SCL)
  W4   P2.5 TXD   J1.4   -->   2.2 kOhm      P2.6 RXD   J1.3         UART cross-link (eUSCI_A1)
  W5   P2.6 RXD   J1.3   <--   2.2 kOhm      P2.5 TXD   J1.4         UART cross-link (eUSCI_A1)
  W6   P3.4 out   J1.8   -->   2.2 kOhm      P3.5 in    J1.9         GPIO edge irq + LPM4 wake
  W7   P3.5 in    J1.9   <--   2.2 kOhm      P3.4 out   J1.8         GPIO edge irq + LPM4 wake
  W8   P1.4 TB0.1 J2.12  -->   2.2 kOhm      P1.2 CCI1A J2.19        PWM -> timer capture
  W9   P1.2 CCI1A J2.19  <--   2.2 kOhm      P1.4 TB0.1 J2.12        PWM -> timer capture
  W10  P1.5 TB0.2 J2.13  -->   2.2 kOhm      P2.4 A7    J1.6         (reserved: future PWM-RC
  W11  P2.4 A7    J1.6   <--   2.2 kOhm      P1.5 TB0.2 J2.13         DAC -> ADC, see note)
  W12  P2.2 CLK   J1.7         2.2 kOhm      P2.2 CLK   J1.7         (reserved: future B0 SPI
                                                                      master<->slave over W2/W3)

  W10/W11 future addition: adding a ~10 uF cap (4.7-47 uF, any type, >=6.3 V,
  + toward the pin) from each RECEIVING board's A7 pin (J1.6) to that board's
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

  NEVER connect between the boards: 3V3<->3V3, anything to 5 V (J13.17/J7),
  RST (J2.16), J2.17 (unconnected/TEST stub -- leave empty), or any pin of
  the J13 emulator blocks. Power BOTH boards whenever either is powered
  (idle firmware parks every cross-line low/high-Z, so a half-powered rig is
  still within spec -- but don't make a habit of it).
================================================================================
"#
    );
}
