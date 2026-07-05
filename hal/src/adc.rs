//! ADC12_B — the 12-bit successive-approximation analog-to-digital converter.
//!
//! This drives the converter in its simplest useful mode: **single-channel,
//! single-conversion, software-triggered.** The `read*` methods are polled —
//! pick a channel, kick off one conversion, busy-wait the few microseconds it
//! takes, read the result. The `start_*` methods are the same conversions
//! **without the wait**: arm [`enable_conversion_interrupt`]
//! (Adc::enable_conversion_interrupt) and completion fires the `ADC12` vector,
//! where [`read_result`] collects the count (canonically after
//! [`crate::power::enter_lpm0`] — MODOSC self-clocks the conversion while the
//! CPU sleeps). The `*_repeated_dma` methods add the one sequencing mode this
//! driver uses: repeat-single-channel free-running conversions with a DMA
//! channel draining MEM0 per completion (see the DMA section below). No
//! multi-channel sequences yet.
//!
//! # How a conversion works (the mechanism this module programs)
//!
//! A SAR ADC binary-searches the input voltage against its reference. Each step
//! halves the remaining range, so an N-bit result needs N comparator decisions,
//! one per ADC clock. Before that search can begin the input must be **sampled**:
//! an internal capacitor is connected to the pin and given time to charge to the
//! input voltage (the *sample-and-hold* period). Get this backwards and you read
//! a stale or half-charged capacitor — so the sample time, not the 12 compare
//! cycles, is usually what dominates and what you tune for source impedance.
//!
//! The control registers encode exactly those two phases:
//!
//! - **`ADC12CTL0`** — `ADC12ON` powers the core; `ADC12SHT0x` sets the
//!   sample-and-hold time (in ADC-clock cycles) for memory registers MEM0–7.
//! - **`ADC12CTL1`** — `ADC12SHP = 1` selects *pulse* sample mode, where the
//!   sampling timer (not the duration of an external signal) defines the S&H
//!   period from `ADC12SHT0x`; `ADC12SSEL` picks the ADC clock; `ADC12CONSEQ = 0`
//!   selects single-channel-single-conversion.
//! - **`ADC12CTL2`** — `ADC12RES` sets 8/10/12-bit resolution; the result is
//!   unsigned right-justified binary (`ADC12DF = 0`).
//! - **`ADC12MCTL0`** — `ADC12INCH` routes one of the input channels to the
//!   conversion, and `ADC12VRSEL` picks the reference. The pin-based `read`
//!   methods use `VRSEL = 0` — **VR+ = AVCC, VR- = AVSS** — so a result reads
//!   `round(4095 · Vin / AVCC)` at 12-bit: *ratiometric*, a fraction of
//!   whatever the supply is. The `&Ref`-taking methods use `VRSEL = 1` —
//!   **VR+ = VREF (the buffered [`crate::ref_a`] output), VR- = AVSS** — so a
//!   count is worth a fixed `vref/4095` volts and results are *absolute*.
//! - **`ADC12MEM0`** — the conversion result; reading it also clears the
//!   per-channel interrupt flag `ADC12IFG0`.
//!
//! Conversions are clocked from **MODOSC** (the ~4.8 MHz internal oscillator,
//! requested automatically while the ADC is on) by default, so a conversion has
//! no external dependency and completes deterministically — the busy-wait here
//! cannot hang the way an I2C bus wait can.
//!
//! # No `embedded-hal` trait?
//!
//! `embedded-hal` **1.0 deliberately ships no ADC trait** — the `adc::OneShot` /
//! `adc::Channel` traits from 0.2 were dropped because the team had not settled
//! on a good abstraction, and its modules are only `digital`, `i2c`, `spi`,
//! `pwm`, and `delay`. So there is no upstream trait to implement here. This API
//! instead follows the *conventions* that ADC trait would have used: a converter
//! object that owns the peripheral, and **typed pins** ([`AdcPin`]) so a channel
//! read is checked against real silicon at compile time — you cannot read a pin
//! that has no ADC function.
//!
//! # Channel ↔ pin map (MSP430FR5969, ADC12_B)
//!
//! | Ch | Pin  | Ch | Pin  | Ch  | Pin  | Ch  | Pin  |
//! |----|------|----|------|-----|------|-----|------|
//! | A0 | P1.0 | A4 | P1.4 | A8  | P4.0 | A12 | P3.0 |
//! | A1 | P1.1 | A5 | P1.5 | A9  | P4.1 | A13 | P3.1 |
//! | A2 | P1.2 | A6 | P2.3 | A10 | P4.2 | A14 | P3.2 |
//! | A3 | P1.3 | A7 | P2.4 | A11 | P4.3 | A15 | P3.3 |
//!
//! Two internal sources have no pin: the **temperature sensor** (A30, powered
//! by REF_A — see [`read_temperature`](Adc::read_temperature)) and the
//! **(AVCC–AVSS)/2 supply monitor** (A31, see
//! [`read_supply_millivolts`](Adc::read_supply_millivolts)).
//!
//! # Example
//!
//! ```ignore
//! let (port1, _port2) = p.port_1_2.split();
//! let mut a4 = port1.pin4.into_analog();              // P1.4 = channel A4
//! let mut adc = Adc::new(p.adc12, Config::default()); // 12-bit, AVCC reference
//! let counts = adc.read(&mut a4);                     // 0..=4095
//! let millivolts = (counts as u32 * 3300) / 4095;     // assuming AVCC = 3.3 V
//! ```

use crate::gpio::{Analog, Pin, P1, P2, P3, P4};
use crate::pac;
use crate::ref_a::Ref;
use crate::tlv::AdcCal;

/// Which on-chip source (if any) to route to the converter via `ADC12CTL3`.
/// Internal use only — exposed to callers through the dedicated read methods.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Internal {
    /// External pin: no internal mapping.
    None,
    /// (AVCC–AVSS)/2 supply monitor on channel A31 (`ADC12BATMAP`).
    SupplyHalf,
    /// Temperature sensor on channel A30 (`ADC12TCMAP`).
    Temperature,
}

/// What VR+ the conversion measures against (`ADC12VRSEL`). Internal — chosen
/// by whether the caller went through a plain or a `&Ref`-taking read method.
#[derive(Clone, Copy, PartialEq, Eq)]
enum RefSel {
    /// `VRSEL = 0`: VR+ = AVCC (ratiometric).
    Avcc,
    /// `VRSEL = 1`: VR+ = buffered VREF from REF_A (absolute).
    VRef,
}

/// Conversion resolution. Fewer bits convert (slightly) faster; more bits give
/// finer steps. The driver defaults to the full 12 bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Resolution {
    /// 8-bit result, `0..=255`.
    Bits8,
    /// 10-bit result, `0..=1023`.
    Bits10,
    /// 12-bit result, `0..=4095`.
    Bits12,
}

impl Resolution {
    /// The largest value a conversion can return at this resolution (full scale,
    /// i.e. an input at VR+). Useful for scaling counts to a voltage.
    pub const fn max(self) -> u16 {
        match self {
            Resolution::Bits8 => 255,
            Resolution::Bits10 => 1023,
            Resolution::Bits12 => 4095,
        }
    }
}

/// The clock that drives the conversion (the SAR comparator steps and, in pulse
/// sample mode, the sample-and-hold timer).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// MODOSC, the ~4.8 MHz internal oscillator dedicated to the ADC. Always
    /// available (it spins up on demand while the ADC is on) and the right
    /// default — the conversion needs no other clock tree to be running.
    ModOsc,
    /// ACLK.
    Aclk,
    /// MCLK.
    Mclk,
    /// SMCLK.
    Smclk,
}

/// Sample-and-hold time, expressed as the number of ADC clock cycles the input
/// capacitor is allowed to charge before the SAR search begins. Longer is safer
/// for higher source impedance; the datasheet gives the minimum for a given
/// source resistance. The variants are the values the `ADC12SHT0x` field can
/// encode (a subset of the common ones).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SampleTime {
    /// 4 ADC clocks.
    Cycles4,
    /// 8 ADC clocks.
    Cycles8,
    /// 16 ADC clocks.
    Cycles16,
    /// 32 ADC clocks (driver default — comfortable for typical low-impedance
    /// sources at MODOSC ≈ 4.8 MHz, ~6.7 µs).
    Cycles32,
    /// 64 ADC clocks.
    Cycles64,
    /// 128 ADC clocks.
    Cycles128,
    /// 256 ADC clocks.
    Cycles256,
}

impl SampleTime {
    /// The 4-bit `ADC12SHT0x` field code for this sample time.
    const fn code(self) -> u8 {
        match self {
            SampleTime::Cycles4 => 0,
            SampleTime::Cycles8 => 1,
            SampleTime::Cycles16 => 2,
            SampleTime::Cycles32 => 3,
            SampleTime::Cycles64 => 4,
            SampleTime::Cycles128 => 6,
            SampleTime::Cycles256 => 8,
        }
    }
}

/// ADC configuration. Build with [`Config::default`] (12-bit, MODOSC, 32-cycle
/// sample) and override with the builder methods.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    resolution: Resolution,
    clock: ClockSource,
    sample_time: SampleTime,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            resolution: Resolution::Bits12,
            clock: ClockSource::ModOsc,
            sample_time: SampleTime::Cycles32,
        }
    }
}

impl Config {
    /// A default configuration: 12-bit, MODOSC-clocked, 32-cycle sample.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the conversion resolution.
    pub fn resolution(mut self, resolution: Resolution) -> Self {
        self.resolution = resolution;
        self
    }

    /// Set the conversion clock source.
    pub fn clock(mut self, clock: ClockSource) -> Self {
        self.clock = clock;
        self
    }

    /// Set the sample-and-hold time.
    pub fn sample_time(mut self, sample_time: SampleTime) -> Self {
        self.sample_time = sample_time;
        self
    }
}

/// The ADC12_B converter. Owns the PAC peripheral; configured once at
/// construction, then [`read`](Adc::read) one channel at a time.
pub struct Adc {
    adc: pac::Adc12,
    /// Kept from [`Config`] so count→millivolt scaling knows full scale.
    resolution: Resolution,
}

impl Adc {
    /// Configure and power on the ADC.
    ///
    /// Programs the control registers for single-channel-single-conversion, in
    /// pulse sample mode at the requested resolution/clock/sample time, and turns
    /// the core on. `ADC12ENC` is left clear so [`read_channel`](Adc::read_channel)
    /// can program the channel select before each conversion (the input-channel
    /// and reference fields are writable only while conversions are disabled).
    pub fn new(adc: pac::Adc12, config: Config) -> Self {
        // Conversions disabled while we program the control registers.
        adc.adc12ctl0().modify(|_, w| w.adc12enc().clear_bit());

        // CTL0: sample-and-hold time for MEM0..7, core on.
        adc.adc12ctl0().write(|w| {
            w.adc12sht0().set(config.sample_time.code());
            w.adc12on().set_bit()
        });

        // CTL1: pulse sample mode (sampling timer sets the S&H period), clock
        // source, single-channel-single-conversion (CONSEQ = 0).
        adc.adc12ctl1().write(|w| {
            w.adc12shp().set_bit();
            w.adc12conseq().adc12conseq_0();
            match config.clock {
                ClockSource::ModOsc => w.adc12ssel().adc12ssel_0(),
                ClockSource::Aclk => w.adc12ssel().adc12ssel_1(),
                ClockSource::Mclk => w.adc12ssel().adc12ssel_2(),
                ClockSource::Smclk => w.adc12ssel().adc12ssel_3(),
            }
        });

        // CTL2: resolution. DF = 0 (unsigned binary) and PWRMD = 0 (regular)
        // are the reset defaults that `write` restores.
        adc.adc12ctl2().write(|w| match config.resolution {
            Resolution::Bits8 => w.adc12res().adc12res_0(),
            Resolution::Bits10 => w.adc12res().adc12res_1(),
            Resolution::Bits12 => w.adc12res().adc12res_2(),
        });

        Adc {
            adc,
            resolution: config.resolution,
        }
    }

    /// Run one conversion on a typed analog pin and return the raw count.
    ///
    /// The pin must implement [`AdcPin`] — i.e. be a `Pin<_, _, Analog>` that is
    /// actually wired to an ADC channel — so an unsupported pin is a compile
    /// error rather than a silent wrong reading. Takes `&mut` to require
    /// exclusive use of the pin for the duration of the conversion.
    pub fn read<P: AdcPin>(&mut self, _pin: &mut P) -> u16 {
        self.read_channel(P::CHANNEL)
    }

    /// Run one conversion on a typed analog pin **against the REF_A
    /// reference** (`VRSEL = 1`) and return the raw count.
    ///
    /// Where [`read`](Adc::read) is ratiometric (a fraction of AVCC), this is
    /// absolute: full scale is the reference voltage, so
    /// `mV = counts · vref / max` — [`to_millivolts`](Adc::to_millivolts) does
    /// exactly that. The `&Ref` both proves the reference is on and settled
    /// (constructing one performs the handshake) and borrows it so it cannot
    /// be `free`d mid-conversion. The input must not exceed the reference
    /// (readings clip at full scale).
    pub fn read_vref<P: AdcPin>(&mut self, _pin: &mut P, vref: &Ref) -> u16 {
        let _ = vref;
        self.convert(P::CHANNEL, Internal::None, RefSel::VRef)
    }

    /// Run one conversion on a raw channel number (`0..=15` for A0..A15) and
    /// return the count, right-justified to the configured resolution.
    ///
    /// Prefer [`read`](Adc::read) for external pins — it proves the channel is
    /// real and that you hold the pin. This lower-level entry point exists for
    /// channels not tied to a single pin.
    pub fn read_channel(&mut self, channel: u8) -> u16 {
        self.convert(channel, Internal::None, RefSel::Avcc)
    }

    /// Convert the internal **(AVCC–AVSS)/2** supply monitor. **No external pin
    /// is involved.**
    ///
    /// Because the reference here is AVCC itself, the result is ratiometric and
    /// essentially fixed at **half full-scale** (≈ 2048 at 12-bit) no matter what
    /// the supply actually is — so this cannot measure the supply *voltage*
    /// (that is [`read_supply_millivolts`](Adc::read_supply_millivolts),
    /// against the REF_A reference), but it is an
    /// excellent **self-contained functional check**: a freshly-built ADC that
    /// reads ~half scale here is converting correctly with nothing wired up.
    ///
    /// The internal divider is relatively high-impedance, so configure the
    /// [`Adc`] with a long [`SampleTime`] (e.g. [`SampleTime::Cycles256`]);
    /// the default 32 cycles may under-sample it and read low.
    pub fn read_supply_half(&mut self) -> u16 {
        // The supply monitor maps onto channel A31 when ADC12BATMAP is set.
        self.convert(31, Internal::SupplyHalf, RefSel::Avcc)
    }

    /// Measure the supply voltage: convert the internal **(AVCC–AVSS)/2
    /// monitor against the REF_A reference** and return **AVCC in
    /// millivolts**. **No external pin is involved.**
    ///
    /// This is the absolute measurement [`read_supply_half`](Adc::read_supply_half)
    /// cannot make: against a fixed reference the divider's output is a real
    /// voltage, and doubling it recovers AVCC. The reference must exceed
    /// AVCC/2 or the reading clips — at a 3.3 V supply that rules out 1.2 V;
    /// use [`crate::ref_a::ReferenceVoltage::V2_0`] (covers AVCC up to 4 V).
    ///
    /// Uncalibrated (nominal reference, uncorrected gain/offset) — good to a
    /// couple of percent. For the calibrated chain take
    /// [`read_supply_raw`](Adc::read_supply_raw) and run it through
    /// [`crate::tlv::AdcCal::correct_gain_offset`] and
    /// [`crate::tlv::RefCal::correct`] before scaling.
    ///
    /// The divider is high-impedance: configure a long [`SampleTime`]
    /// (e.g. [`SampleTime::Cycles256`]).
    pub fn read_supply_millivolts(&mut self, vref: &Ref) -> u32 {
        let counts = self.read_supply_raw_inner();
        self.to_millivolts(counts, vref) * 2
    }

    /// Convert the internal (AVCC–AVSS)/2 monitor against the REF_A reference
    /// and return the **raw count** — the entry point for the fully
    /// calibrated supply measurement (see
    /// [`read_supply_millivolts`](Adc::read_supply_millivolts)).
    pub fn read_supply_raw(&mut self, vref: &Ref) -> u16 {
        let _ = vref;
        self.read_supply_raw_inner()
    }

    fn read_supply_raw_inner(&mut self) -> u16 {
        self.convert(31, Internal::SupplyHalf, RefSel::VRef)
    }

    /// Convert the internal **temperature sensor** channel *against AVCC,
    /// without touching REF_A*, and return the raw count. **No external pin is
    /// involved.**
    ///
    /// The sensor is part of the **REF_A** module, not the ADC: it is a diode
    /// stack biased by the reference generator, unpowered unless `REFON` is
    /// set. So unless something else brought REF_A up, **this reads a flat ~0**
    /// (verified on hardware 2026-06-27) — which is exactly what the
    /// `adc_internal` fixture asserts to prove the ADC converts a dead channel
    /// honestly. For an actual temperature use
    /// [`read_temperature`](Adc::read_temperature), which takes the [`Ref`]
    /// that powers the sensor.
    pub fn read_temperature_raw(&mut self) -> u16 {
        // The temperature sensor maps onto channel A30 when ADC12TCMAP is set.
        self.convert(30, Internal::Temperature, RefSel::Avcc)
    }

    /// Convert the internal **temperature sensor against the REF_A
    /// reference** and return the raw count. **No external pin is involved.**
    ///
    /// The `&Ref` is what makes the reading real: constructing it set `REFON`,
    /// which biases the sensor, and the conversion measures against the same
    /// reference the factory characterization used — so the raw count feeds
    /// straight into [`crate::tlv::AdcCal::temp_deci_celsius`] (or use
    /// [`read_temperature_deci_celsius`](Adc::read_temperature_deci_celsius)
    /// for the pair). The sensor output (~0.7 V, ~2.5 mV/°C) fits under any of
    /// the three reference voltages; it is high-impedance, so configure a long
    /// [`SampleTime`] — the datasheet asks for ≥ 30 µs of acquisition
    /// ([`SampleTime::Cycles256`] ≈ 53 µs at MODOSC).
    pub fn read_temperature(&mut self, vref: &Ref) -> u16 {
        let _ = vref;
        self.convert(30, Internal::Temperature, RefSel::VRef)
    }

    /// Measure the die temperature in **deci-°C** (273 = 27.3 °C):
    /// [`read_temperature`](Adc::read_temperature) interpolated between the
    /// factory 30 °C / 85 °C points for this `vref`'s voltage. `None` only if
    /// the calibration words are corrupt (see
    /// [`crate::tlv::AdcCal::temp_deci_celsius`]).
    ///
    /// Requires 12-bit [`Resolution`] — the TLV points are 12-bit conversion
    /// results, so an 8/10-bit reading would interpolate on the wrong scale.
    pub fn read_temperature_deci_celsius(&mut self, vref: &Ref, cal: &AdcCal) -> Option<i16> {
        let raw = self.read_temperature(vref);
        cal.temp_deci_celsius(vref.voltage(), raw)
    }

    /// Scale a count from a `&Ref`-taking read to **millivolts**, rounded to
    /// nearest: `counts · vref / full_scale` at this converter's configured
    /// [`Resolution`]. (Counts from the AVCC-referenced methods have no fixed
    /// millivolt worth — that is the point of the reference.)
    pub fn to_millivolts(&self, counts: u16, vref: &Ref) -> u32 {
        crate::adc_cal::counts_to_millivolts(counts, self.resolution.max(), vref.millivolts())
    }

    /// The shared single-conversion sequence: optionally route an internal
    /// source, select the channel and reference, trigger, busy-wait, and read
    /// MEM0.
    fn convert(&mut self, channel: u8, internal: Internal, refsel: RefSel) -> u16 {
        self.arm(channel, internal, refsel);

        // Self-completing (MODOSC-clocked, no external dependency): this poll is
        // bounded by the conversion time, a few microseconds.
        while self.adc.adc12ctl1().read().adc12busy().bit_is_set() {}

        // Reading MEM0 returns the result and clears ADC12IFG0.
        self.adc.adc12mem0().read().bits()
    }

    /// Program-and-trigger, no wait: everything [`convert`](Adc::convert) does
    /// up to and including the `ENC|SC` write. Completion sets `ADC12IFG0`
    /// (and fires the `ADC12` vector when enabled); MEM0 holds the result.
    fn arm(&mut self, channel: u8, internal: Internal, refsel: RefSel) {
        self.program(channel, internal, refsel);

        // Arm (ENC) and trigger (SC) in one write.
        self.adc
            .adc12ctl0()
            .modify(|_, w| w.adc12enc().set_bit().adc12sc().set_bit());
    }

    /// Route a channel/reference pair to MEM0, leaving `ENC` clear (the
    /// front half of [`arm`](Adc::arm), shared with the DMA methods, which
    /// must slot more setup between programming and the `ENC|SC` go).
    fn program(&mut self, channel: u8, internal: Internal, refsel: RefSel) {
        // CTL3 (internal-source mapping) and MCTL0 (INCH/VRSEL) are writable only
        // while ENC = 0; a prior conversion leaves ENC set, so clear it first.
        self.adc.adc12ctl0().modify(|_, w| w.adc12enc().clear_bit());

        // CTL3: connect the (AVCC–AVSS)/2 monitor (A31) or temperature sensor
        // (A30) to the converter, or neither for an external pin. CSTARTADD stays
        // 0 (MEM0) — the `write` resets it.
        self.adc.adc12ctl3().write(|w| match internal {
            Internal::None => w.adc12batmap().clear_bit().adc12tcmap().clear_bit(),
            Internal::SupplyHalf => w.adc12batmap().set_bit().adc12tcmap().clear_bit(),
            Internal::Temperature => w.adc12batmap().clear_bit().adc12tcmap().set_bit(),
        });

        // Route MEM0 to this channel with the requested reference pair, and
        // end-of-sequence (the single channel is also the last one).
        self.adc.adc12mctl0().write(|w| {
            match refsel {
                RefSel::Avcc => w.adc12vrsel().adc12vrsel_0(), // VR+ = AVCC, VR- = AVSS
                RefSel::VRef => w.adc12vrsel().adc12vrsel_1(), // VR+ = VREF buffered, VR- = AVSS
            };
            w.adc12eos().set_bit();
            w.adc12inch().set(channel)
        });
    }

    /// Enable the MEM0 conversion-complete interrupt (`ADC12IER0.ADC12IE0`):
    /// from now on a finishing conversion fires the `ADC12` vector (once GIE
    /// is set).
    ///
    /// `ADC12IER0` is ADC-private and this driver owns `pac::Adc12`, so a
    /// plain `modify` suffices — no critical section, unlike the shared
    /// `SFRIE1`/`PxIE` cases.
    pub fn enable_conversion_interrupt(&mut self) {
        self.adc.adc12ier0().modify(|_, w| w.adc12ie0().set_bit());
    }

    /// Disable the MEM0 conversion-complete interrupt. A pending `ADC12IFG0`
    /// stays latched (a completed-but-uncollected result still sits in MEM0;
    /// [`read_result`] collects and clears it).
    pub fn disable_conversion_interrupt(&mut self) {
        self.adc.adc12ier0().modify(|_, w| w.adc12ie0().clear_bit());
    }

    /// Start one conversion on a typed analog pin and **return immediately**.
    ///
    /// The non-blocking sibling of [`read`](Adc::read): same channel routing
    /// and AVCC reference, but instead of busy-waiting, completion sets
    /// `ADC12IFG0` — poll [`conversion_pending`] or take the `ADC12` interrupt
    /// and collect with [`read_result`]. With the default MODOSC clock the
    /// conversion self-completes even in LPM0 (the ADC requests its oscillator
    /// on demand), so `start… → enter_lpm0() → read_result()` samples while
    /// the CPU sleeps.
    ///
    /// Starting a new conversion before collecting the previous result simply
    /// overwrites MEM0.
    pub fn start_conversion<P: AdcPin>(&mut self, _pin: &mut P) {
        self.arm(P::CHANNEL, Internal::None, RefSel::Avcc);
    }

    /// Start one conversion on a raw channel number and return immediately —
    /// the non-blocking sibling of [`read_channel`](Adc::read_channel).
    pub fn start_channel(&mut self, channel: u8) {
        self.arm(channel, Internal::None, RefSel::Avcc);
    }

    /// Start one conversion of the internal (AVCC–AVSS)/2 monitor and return
    /// immediately — the non-blocking sibling of
    /// [`read_supply_half`](Adc::read_supply_half) (same ~half-full-scale
    /// expectation, same long-[`SampleTime`] advice).
    pub fn start_supply_half(&mut self) {
        self.arm(31, Internal::SupplyHalf, RefSel::Avcc);
    }

    /// Power the ADC core down and return the PAC peripheral.
    ///
    /// Clears `ADC12ENC` first (the core must not be converting when `ADC12ON`
    /// is cleared) then drops `ADC12ON`.
    pub fn free(self) -> pac::Adc12 {
        self.adc.adc12ctl0().modify(|_, w| w.adc12enc().clear_bit());
        self.adc.adc12ctl0().modify(|_, w| w.adc12on().clear_bit());
        self.adc
    }
}

// ---------------------------------------------------------------------------
// DMA-drained repeated conversions
// ---------------------------------------------------------------------------
//
// `ADC12IFG0` is DMA trigger 26 (`dma::TriggerSource::Adc12`), and the DMA's
// word read of `MEM0` clears it — the same read-collects-and-acknowledges
// contract `read_result` documents, performed by hardware. So a channel armed
// on that trigger turns the converter's *repeat-single-channel* mode
// (`CONSEQ = 2` with `ADC12MSC`, where each completed conversion immediately
// starts the next) into a free-running sampler that fills a buffer with zero
// per-sample CPU work: complete → IFG0 edge → DMA collects MEM0 (clearing
// IFG0) → converter is already sampling again.
//
// **The trigger is a latch, not the flag** (hardware-observed 2026-07-05; a
// known undocumented erratum, TI E2E #401588): the ADC12's DMA trigger line
// is set by a conversion completing while `ADC12IE0` is clear, and is reset
// ONLY by a DMA transfer that the ADC trigger itself fires — a CPU read of
// MEM0 clears `ADC12IFG0` but leaves the trigger latch high. Consequence:
// any conversion not collected by DMA (a polled `read`, or simply the free-
// running conversion that completes right after a run's last DMA transfer)
// parks the latch high, and an edge-sensitive channel then never sees
// another rising edge — on any channel, surviving even an ADC12ON power
// cycle. Only the first run after reset would ever work. The engine
// therefore *scrubs* the latch at the start of every run with a one-word
// level-sensitive dummy transfer (`Channel::consume_stale_trigger_word`),
// which a stuck-high latch fires immediately — a genuine ADC-triggered DMA
// service, so the latch resets.
//
// Ordering (the project's DMA lessons apply):
// - The scrub doubles as the MEM0/IFG0 drain (its dummy read of MEM0 clears
//   the flag), so IFG0 idles low and every completion presents a fresh edge
//   (normal pacing stays edge-sensitive).
// - The channel is armed *before* `ENC|SC` starts the converter, so even the
//   first result meets an armed channel — the ADC equivalent of arm-first,
//   announce-second.
//
// Timing budget: the DMA services a trigger within a few MCLK cycles, and the
// shortest conversion here is ~45 ADC clocks ≈ 9 µs at MODOSC — an order of
// magnitude of headroom before a result could be overwritten, at any MCLK
// this crate configures.
//
// Do not combine with `enable_conversion_interrupt`: the DMA and the ISR
// would race to consume the one flag (collect-once, same as ever) — and per
// the erratum above, an ISR-collected conversion also leaves the DMA trigger
// latch parked high (the next DMA run's scrub absorbs that).

#[cfg(feature = "critical-section")]
impl Adc {
    /// Convert a typed analog pin `buf.len()` times back-to-back
    /// (ratiometric, AVCC reference), the results DMA-drained into `buf`.
    /// Blocks until the buffer is full — bounded work: MODOSC self-clocks
    /// the conversions, so `len × (sample + 13 clocks)` and no external
    /// dependency. The converter is returned to single-conversion mode
    /// before returning.
    pub fn read_repeated_dma<P: AdcPin, const N: u8>(
        &mut self,
        _pin: &mut P,
        ch: &mut crate::dma::Channel<N>,
        buf: &mut [u16],
    ) {
        self.read_repeated_dma_inner(ch, P::CHANNEL, Internal::None, RefSel::Avcc, buf);
    }

    /// [`read_supply_half`](Adc::read_supply_half) `buf.len()` times,
    /// DMA-drained — every sample should sit near half full-scale (same
    /// long-[`SampleTime`] advice; the divider is high-impedance).
    pub fn read_supply_half_repeated_dma<const N: u8>(
        &mut self,
        ch: &mut crate::dma::Channel<N>,
        buf: &mut [u16],
    ) {
        self.read_repeated_dma_inner(ch, 31, Internal::SupplyHalf, RefSel::Avcc, buf);
    }

    /// [`read_temperature`](Adc::read_temperature) `buf.len()` times,
    /// DMA-drained: raw counts against the REF_A reference, each one valid
    /// input for [`crate::tlv::AdcCal::temp_deci_celsius`]. The `&Ref`
    /// keeps the sensor biased for the whole run.
    pub fn read_temperature_repeated_dma<const N: u8>(
        &mut self,
        vref: &Ref,
        ch: &mut crate::dma::Channel<N>,
        buf: &mut [u16],
    ) {
        let _ = vref;
        self.read_repeated_dma_inner(ch, 30, Internal::Temperature, RefSel::VRef, buf);
    }

    /// The shared engine: program the channel/reference, flip the converter
    /// into repeat-single-channel + multiple-sample-and-convert, arm the DMA
    /// for the whole buffer, start, wait, and restore single-conversion mode.
    fn read_repeated_dma_inner<const N: u8>(
        &mut self,
        ch: &mut crate::dma::Channel<N>,
        channel: u8,
        internal: Internal,
        refsel: RefSel,
        buf: &mut [u16],
    ) {
        if buf.is_empty() {
            return;
        }
        self.program(channel, internal, refsel);

        // Free-running: repeat-single-channel (CONSEQ = 2), with MSC so each
        // completed conversion starts the next sample immediately — the
        // ADC12SC trigger below is needed only once.
        self.adc
            .adc12ctl1()
            .modify(|_, w| w.adc12conseq().adc12conseq_2());
        self.adc.adc12ctl0().modify(|_, w| w.adc12msc().set_bit());

        // Scrub the trigger latch (see the section comment: one unserviced
        // conversion anywhere — including the tail of the previous run —
        // parks it high and deafens every future edge-sensitive run). The
        // scrub's dummy MEM0 read also clears a stale ADC12IFG0, so IFG0
        // idles low and every completion below presents a fresh edge.
        let mut scratch = 0u16;
        unsafe {
            ch.consume_stale_trigger_word(
                crate::dma::TriggerSource::Adc12,
                self.adc.adc12mem0().as_ptr() as *const u16,
                &mut scratch,
            );
        }

        // Arm first, start second: the first completion already finds the
        // channel listening.
        unsafe {
            ch.arm_single_words(
                crate::dma::TriggerSource::Adc12,
                self.adc.adc12mem0().as_ptr() as *const u16,
                crate::dma::AddrMode::Fixed,
                buf.as_mut_ptr(),
                crate::dma::AddrMode::Increment,
                buf.len() as u16,
            );
        }
        self.adc
            .adc12ctl0()
            .modify(|_, w| w.adc12enc().set_bit().adc12sc().set_bit());

        // `buf.len()` collected transfers later the channel completes; the
        // converter is still free-running until stopped below.
        ch.wait_done();

        // Stop (a conversion in flight completes, then the sequencer halts)
        // and restore the single-conversion contract the rest of the driver
        // assumes, draining the result that finished after the last collect.
        self.adc.adc12ctl0().modify(|_, w| w.adc12enc().clear_bit());
        while self.adc.adc12ctl1().read().adc12busy().bit_is_set() {}
        self.adc.adc12ctl0().modify(|_, w| w.adc12msc().clear_bit());
        self.adc
            .adc12ctl1()
            .modify(|_, w| w.adc12conseq().adc12conseq_0());
        let _ = self.adc.adc12mem0().read().bits();
    }
}

// ---------------------------------------------------------------------------
// ISR-side free functions
// ---------------------------------------------------------------------------
//
// The `Adc` driver owns `pac::Adc12`, so the `ADC12` ISR reaches the handful
// of registers it needs through `steal()` — sound because these touch only
// the result/flag side (MEM0, IFGR0, IV) that the owner leaves alone between
// `start_*` and collection.
//
// Flag-clearing semantics matter here: `ADC12IFG0` is cleared by *either*
// reading `ADC12IV` (which reports it) *or* reading `MEM0`. The canonical ISR
// body is just `let counts = adc::read_result();` — one read, result
// collected, flag down. Don't also call `read_iv` "to be safe": the IV read
// clears the flag first and a subsequent IV read reports 0, which is
// confusing at best. `read_iv` exists for when multiple ADC interrupt sources
// (window monitor, overflows, more MEMs) are enabled and the handler must
// demux — not needed while this driver only ever arms IFG0.

/// ISR-side: read `ADC12IV` — 0x0C means "MEM0 conversion complete"
/// (`ADC12IFG0`), 0 means nothing pending. Reading clears the reported
/// source's flag (for IFG0, *without* collecting MEM0 — prefer
/// [`read_result`], which does both).
pub fn read_iv() -> u16 {
    let adc = unsafe { pac::Adc12::steal() };
    adc.adc12iv().read().bits()
}

/// ISR-side: read the MEM0 conversion result. The read also clears
/// `ADC12IFG0` in hardware — result collected, interrupt acknowledged, one
/// bus access.
pub fn read_result() -> u16 {
    let adc = unsafe { pac::Adc12::steal() };
    adc.adc12mem0().read().bits()
}

/// Has a MEM0 conversion completed (`ADC12IFG0` latched)? The polling
/// companion to the `start_*` methods, for consumers that want non-blocking
/// starts without enabling the interrupt.
pub fn conversion_pending() -> bool {
    let adc = unsafe { pac::Adc12::steal() };
    adc.adc12ifgr0().read().adc12ifg0().bit_is_set()
}

// ---------------------------------------------------------------------------
// Typed analog pins → ADC channel numbers
// ---------------------------------------------------------------------------

mod sealed {
    pub trait Sealed {}
}

/// A pin that is wired to an ADC12_B input channel. Implemented only for the
/// `Pin<_, _, Analog>` types that correspond to real silicon channels; the
/// associated constant is that channel's `ADC12INCH` value. Sealed — the set of
/// analog pins is fixed by the package.
pub trait AdcPin: sealed::Sealed {
    /// The `ADC12INCH` channel number this pin feeds.
    const CHANNEL: u8;
}

macro_rules! adc_pins {
    ($($Port:ident $N:literal => $ch:literal),+ $(,)?) => {$(
        impl sealed::Sealed for Pin<$Port, $N, Analog> {}
        impl AdcPin for Pin<$Port, $N, Analog> {
            const CHANNEL: u8 = $ch;
        }
    )+};
}

// MSP430FR5969 ADC12_B external analog inputs (datasheet SLAS704, Table 4-1).
adc_pins! {
    P1 0 => 0,  P1 1 => 1,  P1 2 => 2,  P1 3 => 3,
    P1 4 => 4,  P1 5 => 5,  P2 3 => 6,  P2 4 => 7,
    P4 0 => 8,  P4 1 => 9,  P4 2 => 10, P4 3 => 11,
    P3 0 => 12, P3 1 => 13, P3 2 => 14, P3 3 => 15,
}
