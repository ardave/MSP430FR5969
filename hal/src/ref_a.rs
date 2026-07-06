//! REF_A — the shared voltage-reference module.
//!
//! Every measurement the [`crate::adc`] makes is a *ratio*: the SAR reports
//! what fraction of VR+ the input sits at. With the reset-default `VRSEL = 0`
//! that VR+ is AVCC itself, so results are ratiometric — fine for a
//! potentiometer, useless for an absolute voltage, because the "ruler" is
//! whatever the supply happens to be. REF_A is the on-chip precision ruler: a
//! **bandgap** (a circuit whose output is flat over temperature and supply,
//! ~1.2 V by physics) plus a programmable buffer that scales it to **1.2 V,
//! 2.0 V, or 2.5 V**. Point the ADC at it (`VRSEL = 1`) and a count is worth a
//! fixed number of microvolts regardless of the battery sagging.
//!
//! # One register, and a handshake instead of a delay
//!
//! The whole module is a single register, `REFCTL0`:
//!
//! - **`REFVSEL`** picks 1.2/2.0/2.5 V; **`REFON`** turns the generator on.
//! - **`REFGENBUSY`** is a *write gate*: while the generator is actively
//!   servicing a conversion, *all writes to `REFCTL0` are silently ignored* —
//!   the hardware protects an in-flight conversion from its reference being
//!   yanked. So the driver must check it **before** writing, not after.
//! - **`REFGENRDY`** is the *settled* flag. The bandgap and buffer need tens
//!   of microseconds to charge their internals after `REFON`; converting
//!   before that samples a still-rising reference. Polling `REFGENRDY`
//!   replaces the open-loop `__delay_cycles(75)` seen in TI examples with the
//!   hardware's own word that the output is stable. Both waits are internal
//!   (no pin, no external clock), so neither poll can hang the way an I2C
//!   bus wait can.
//! - **`REFTCOFF`** — the **temperature sensor lives in this module**, not in
//!   the ADC: it is a diode stack biased by the reference generator, and it is
//!   simply unpowered until `REFON` is set (with `REFTCOFF = 0`, the reset
//!   default, which this driver keeps). This is why
//!   [`crate::adc::Adc::read_temperature_raw`] reads a flat ~0 without REF_A —
//!   verified on hardware 2026-06-27 — and why the [`Ref`]-taking read methods
//!   exist.
//!
//! On this FR59xx generation REF_A is the *sole* owner of the reference —
//! older F5xx parts had duplicate legacy control bits inside the ADC plus a
//! `REFMSTR` arbitration bit; here there is nothing to arbitrate, the ADC just
//! consumes whatever REF_A produces.
//!
//! # `REFOUT` — the reference on a pin
//!
//! [`Ref::enable_output`] buffers VREF onto the **P1.1** pad (the package's
//! VREF+ function) for external circuitry — or, hands-free, for *other on-die
//! consumers that can only see pads*: [`crate::comp_e`]'s input channels tap
//! pads, so REFOUT on P1.1 = C1 is the one way to present a known mid-rail
//! analog voltage to the comparator with no wiring at all (the comp
//! integration fixture sweeps the comparator's VCC ladder against it). Takes
//! the P1.1 `Analog` typestate pin as proof the digital path is disconnected.
//! LaunchPad caveat: P1.1 is also button S2 — a held button shorts REFOUT to
//! ground.
//!
//! # Choosing the voltage
//!
//! Full scale is VR+, so pick the smallest reference that still covers the
//! signal — smaller reference, smaller µV/count, finer resolution:
//!
//! - **1.2 V**: the temperature sensor (~0.7 V) at best resolution.
//! - **2.0 V**: the sweet spot when one reference must serve both the
//!   temperature sensor and the supply monitor — AVCC/2 at a 3.3 V supply is
//!   1.65 V, which *clips at 1.2 V* but fits under 2.0 V (covers AVCC to 4 V).
//! - **2.5 V**: headroom for external signals up to 2.5 V.
//!
//! The factory TLV calibration ([`crate::tlv`]) stores constants for each of
//! the three settings, so any choice can be fully corrected.
//!
//! # Example
//!
//! ```ignore
//! let vref = Ref::new(p.shared_reference, ReferenceVoltage::V2_0); // settled on return
//! let mut adc = Adc::new(p.adc12, Config::default().sample_time(SampleTime::Cycles256));
//! let t_raw = adc.read_temperature(&vref);       // sensor powered, VREF-referenced
//! let avcc = adc.read_supply_millivolts(&vref);  // an actual voltage, not a ratio
//! ```

use crate::pac;

/// The three output voltages the reference buffer can produce (`REFVSEL`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ReferenceVoltage {
    /// 1.2 V — finest resolution; the bandgap's native scale.
    V1_2,
    /// 2.0 V — covers the supply monitor (AVCC/2) at a 3.3 V supply.
    V2_0,
    /// 2.5 V — largest full scale.
    V2_5,
}

impl ReferenceVoltage {
    /// Nominal output in millivolts. The TLV `REF` factor ([`crate::tlv::RefCal`])
    /// corrects for this device's actual deviation from nominal.
    pub const fn millivolts(self) -> u16 {
        match self {
            ReferenceVoltage::V1_2 => 1200,
            ReferenceVoltage::V2_0 => 2000,
            ReferenceVoltage::V2_5 => 2500,
        }
    }

    /// Index into the per-voltage TLV calibration arrays (1.2 V, 2.0 V, 2.5 V
    /// in storage order — see [`crate::tlv`]).
    pub(crate) const fn index(self) -> usize {
        match self {
            ReferenceVoltage::V1_2 => 0,
            ReferenceVoltage::V2_0 => 1,
            ReferenceVoltage::V2_5 => 2,
        }
    }
}

/// The REF_A reference generator, on and settled.
///
/// Constructing a `Ref` performs the full bring-up handshake, so *holding one
/// is proof the reference (and the temperature sensor it biases) is powered
/// and stable* — which is why the VREF-referenced [`crate::adc::Adc`] read
/// methods take `&Ref`: a conversion against an unpowered reference is
/// unrepresentable, and the borrow keeps the reference alive (un-`free`-able)
/// for the duration of the read.
pub struct Ref {
    ref_a: pac::SharedReference,
    voltage: ReferenceVoltage,
    /// `REFOUT` state, carried so [`program`](Ref::program)'s whole-register
    /// write preserves it across [`set_voltage`](Ref::set_voltage).
    output_enabled: bool,
}

impl Ref {
    /// Turn the reference generator on at `voltage` and wait for it to settle.
    ///
    /// The datasheet sequence: wait out `REFGENBUSY` (writes are *ignored*,
    /// not queued, while it is set), program `REFVSEL` + `REFON` in one write
    /// (`REFTCOFF` stays 0, keeping the temperature sensor powered), then poll
    /// `REFGENRDY` until the output is settled (tens of µs). Both polls are
    /// bounded, internal waits — no external dependency, cannot hang.
    pub fn new(ref_a: pac::SharedReference, voltage: ReferenceVoltage) -> Self {
        let mut r = Ref {
            ref_a,
            voltage,
            output_enabled: false,
        };
        r.program(voltage);
        r
    }

    /// Reprogram the output voltage (same busy-gate + settle handshake as
    /// construction). Any TLV constants already fetched stay valid — they are
    /// per-voltage, selected at use via [`voltage`](Ref::voltage).
    pub fn set_voltage(&mut self, voltage: ReferenceVoltage) {
        self.voltage = voltage;
        self.program(voltage);
    }

    /// The configured output voltage.
    pub fn voltage(&self) -> ReferenceVoltage {
        self.voltage
    }

    /// The configured output in millivolts (nominal — see
    /// [`ReferenceVoltage::millivolts`]).
    pub fn millivolts(&self) -> u16 {
        self.voltage.millivolts()
    }

    /// Buffer VREF onto the **P1.1** pad (`REFOUT`). The pin must already be
    /// in the `Analog` typestate (digital path disconnected) — it is only
    /// borrowed, since REF_A holds no per-pin state; keep it analog for as
    /// long as the output is enabled. The output tracks later
    /// [`set_voltage`](Ref::set_voltage) calls.
    pub fn enable_output(&mut self, _pin: &crate::gpio::Pin<crate::gpio::P1, 1, crate::gpio::Analog>) {
        self.output_enabled = true;
        self.program(self.voltage);
    }

    /// Stop driving P1.1 (`REFOUT` off); the reference itself stays on.
    pub fn disable_output(&mut self) {
        self.output_enabled = false;
        self.program(self.voltage);
    }

    /// The busy-gate → program → settle sequence shared by
    /// `new`/`set_voltage`/`enable_output`.
    fn program(&mut self, voltage: ReferenceVoltage) {
        while self.ref_a.refctl0().read().refgenbusy().bit_is_set() {}
        self.ref_a.refctl0().write(|w| {
            match voltage {
                ReferenceVoltage::V1_2 => w.refvsel().refvsel_0(),
                ReferenceVoltage::V2_0 => w.refvsel().refvsel_1(),
                ReferenceVoltage::V2_5 => w.refvsel().refvsel_2(),
            };
            w.refout().bit(self.output_enabled);
            w.refon().set_bit()
        });
        while self.ref_a.refctl0().read().refgenrdy().bit_is_clear() {}
    }

    /// Turn the reference generator (and with it the temperature sensor) off
    /// and return the PAC peripheral. Gated on `REFGENBUSY` like every other
    /// `REFCTL0` write; consuming `self` guarantees no `&Ref`-taking
    /// conversion can still be pending.
    pub fn free(self) -> pac::SharedReference {
        while self.ref_a.refctl0().read().refgenbusy().bit_is_set() {}
        self.ref_a.refctl0().write(|w| w.refon().clear_bit());
        self.ref_a
    }
}
