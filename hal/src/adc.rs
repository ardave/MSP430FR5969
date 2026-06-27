//! ADC12_B — the 12-bit successive-approximation analog-to-digital converter.
//!
//! This drives the converter in its simplest useful mode: **single-channel,
//! single-conversion, software-triggered, polled.** You pick a channel, kick off
//! one conversion, busy-wait the few microseconds it takes, and read the result.
//! It is the ADC analog of [`crate::i2c`]'s blocking `probe` — no interrupts, no
//! DMA, no sequences yet.
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
//!   conversion, and `ADC12VRSEL` picks the reference. We use `VRSEL = 0`:
//!   **VR+ = AVCC, VR- = AVSS**, the only reference available without bringing up
//!   the on-chip REF_A module (not yet supported). A result therefore reads
//!   `round(4095 · Vin / AVCC)` at 12-bit.
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
//! (The internal temperature-sensor and battery-monitor channels need the REF_A
//! reference to be meaningful, so they are intentionally left out for now.)
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

        Adc { adc }
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

    /// Run one conversion on a raw channel number (`0..=15` for A0..A15) and
    /// return the count, right-justified to the configured resolution.
    ///
    /// Prefer [`read`](Adc::read) for external pins — it proves the channel is
    /// real and that you hold the pin. This lower-level entry point exists for
    /// channels not tied to a single pin.
    pub fn read_channel(&mut self, channel: u8) -> u16 {
        self.convert(channel, Internal::None)
    }

    /// Convert the internal **(AVCC–AVSS)/2** supply monitor. **No external pin
    /// is involved.**
    ///
    /// Because the reference here is AVCC itself, the result is ratiometric and
    /// essentially fixed at **half full-scale** (≈ 2048 at 12-bit) no matter what
    /// the supply actually is — so this cannot measure the supply *voltage*
    /// (that needs the on-chip REF_A reference, not yet supported), but it is an
    /// excellent **self-contained functional check**: a freshly-built ADC that
    /// reads ~half scale here is converting correctly with nothing wired up.
    ///
    /// The internal divider is relatively high-impedance, so configure the
    /// [`Adc`] with a long [`SampleTime`] (e.g. [`SampleTime::Cycles256`]);
    /// the default 32 cycles may under-sample it and read low.
    pub fn read_supply_half(&mut self) -> u16 {
        // The supply monitor maps onto channel A31 when ADC12BATMAP is set.
        self.convert(31, Internal::SupplyHalf)
    }

    /// Convert the internal **temperature sensor** and return the raw count. **No
    /// external pin is involved.**
    ///
    /// **This currently reads ~0 and is not yet usable** — verified on hardware
    /// 2026-06-27. The temperature sensor is part of the **REF_A** module, not
    /// the ADC: it is biased by the reference generator and produces no output
    /// unless `REFON` is set in `REFCTL0`. This driver does not bring up REF_A
    /// (it runs off the AVCC reference), so the sensor is unpowered and the
    /// channel reads a flat zero. The method is kept so the call site exists, but
    /// a meaningful reading — let alone °C, which additionally needs the factory
    /// TLV constants characterized against the internal reference — is blocked on
    /// REF_A support. When that lands, the sensor is also high-impedance and
    /// needs a long acquisition ([`SampleTime::Cycles256`] or more).
    pub fn read_temperature_raw(&mut self) -> u16 {
        // The temperature sensor maps onto channel A30 when ADC12TCMAP is set.
        self.convert(30, Internal::Temperature)
    }

    /// The shared single-conversion sequence: optionally route an internal
    /// source, select the channel, trigger, busy-wait, and read MEM0.
    fn convert(&mut self, channel: u8, internal: Internal) -> u16 {
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

        // Route MEM0 to this channel: AVCC/AVSS reference, end-of-sequence (the
        // single channel is also the last one).
        self.adc.adc12mctl0().write(|w| {
            w.adc12vrsel().adc12vrsel_0(); // VR+ = AVCC, VR- = AVSS
            w.adc12eos().set_bit();
            w.adc12inch().set(channel)
        });

        // Arm (ENC) and trigger (SC) in one write.
        self.adc
            .adc12ctl0()
            .modify(|_, w| w.adc12enc().set_bit().adc12sc().set_bit());

        // Self-completing (MODOSC-clocked, no external dependency): this poll is
        // bounded by the conversion time, a few microseconds.
        while self.adc.adc12ctl1().read().adc12busy().bit_is_set() {}

        // Reading MEM0 returns the result and clears ADC12IFG0.
        self.adc.adc12mem0().read().bits()
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
