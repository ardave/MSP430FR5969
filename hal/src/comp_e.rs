//! Comp_E — the analog comparator.
//!
//! A comparator is the simplest measurement circuit on the die: two analog
//! inputs, one digital output. `CEOUT` is 1 while V+ is above V−, 0 while it
//! is below — a **1-bit, continuous-time quantizer**. Unlike the
//! [`crate::adc`] (which is this same comparator wrapped in a clocked SAR
//! control loop asking it 12 questions per sample), Comp_E needs **no clock**:
//! it keeps deciding in LPM3/LPM4 with every oscillator stopped, burning ~1 µA,
//! and its output edge can fire the `COMP_E` interrupt vector and wake the CPU.
//! That is its whole reason to exist next to a 12-bit ADC: *sleep until a
//! voltage crosses a threshold* instead of *wake periodically and measure*.
//!
//! # Signal routing (what the config actually wires up)
//!
//! - **V+ gets the watched signal.** `CEIPSEL` closes one of 16 analog
//!   switches connecting an external channel pad (C0–C15) to the + terminal;
//!   `CEIPEN` enables the terminal. This driver's convention fixes the
//!   *channel on V+* and the *reference on V−* (`CERSEL = 1`), so
//!   [`output`](CompE::output)` == true` always reads "input above
//!   threshold". [`exchange_inputs`](CompE::exchange_inputs) (`CEEX`) swaps
//!   the terminals in silicon when the opposite sense is wanted.
//! - **V− gets the threshold** from the reference block, one of
//!   ([`Threshold`]):
//!   - the **resistor ladder fed by VCC** (`CERS = 01`): 32 equal taps, tap
//!     `n` = (n+1)/32 of VCC — ratiometric, tracks the supply;
//!   - the **ladder fed by the shared bandgap reference** (`CERS = 10`,
//!     `CEREFL` = 1.2/2.0/2.5 V): absolute thresholds. Comp_E *requests* the
//!     bandgap from the REF module itself — no [`crate::ref_a::Ref`] needs to
//!     be alive;
//!   - the **bandgap directly** (`CERS = 11`), bypassing the ladder.
//! - **Built-in hysteresis by tap switching.** The ladder has *two* tap
//!   registers: `CEREF0` is selected while `CEOUT = 0`, `CEREF1` while
//!   `CEOUT = 1` (`CEMRVS = 0`, this driver's fixed choice). Program
//!   `CEREF1` a few taps below `CEREF0` and the threshold drops the moment
//!   the output goes high — the input must fall meaningfully to flip it
//!   back. That is a Schmitt trigger, and it is why a noisy signal hovering
//!   at the threshold does not chatter thousands of interrupts. The
//!   [`Threshold`] constructors name them by *edge*: `rising_tap` (`CEREF0`,
//!   armed while the output is low) and `falling_tap` (`CEREF1`).
//!
//! The tap↔millivolt arithmetic lives in `comp_ladder.rs` (pure,
//! host-tested — see `unit_tests/`), re-exported here as
//! [`ladder_millivolts`]/[`tap_for_millivolts`].
//!
//! # Interrupts
//!
//! Both output edges have a flag: with the driver's fixed `CEIES = 0`,
//! **`CEIFG` latches on the rising edge** of `CEOUT` and **`CEIIFG` on the
//! falling edge**. Both route to the single `COMP_E` vector, demuxed by
//! [`read_iv`] whose `CEIV` read atomically clears the served flag in silicon
//! (the same lost-flag-immune pattern as `PxIV`/`RTCIV`/`DMAIV`). Because
//! `CEINT` holds flags and enables in one register, every thread-mode RMW of
//! it in this driver runs inside `critical_section::with` — a plain `modify`
//! racing the ISR's `CEIV` read could resurrect a just-cleared flag.
//! (`CERDYIFG` exists too, but its `CEIV` slot is not implemented here — its
//! encoding is unverified on hardware.)
//!
//! # Settling, not readiness
//!
//! The comparator has no polled busy flag the way REF_A does. After `CEON`,
//! an input-mux change, or a large reference step, the output needs a
//! settling time that grows as the power mode shrinks (`CEPWRMD`: high-speed
//! settles fastest, ultra-low-power slowest — microseconds either way, see
//! the device datasheet's t\_(EN\_CMP)). Give it a few tens of microseconds
//! before trusting [`output`](CompE::output), and expect a spurious edge flag
//! from reconfiguration: arm interrupts *after* configuring, or call
//! [`clear_pending_interrupts`](CompE::clear_pending_interrupts) (the
//! [`enable_output_interrupts`](CompE::enable_output_interrupts) sequence
//! does the clear internally, mirroring the GPIO arming rule).
//!
//! # Pins
//!
//! Proper analog use routes a `Pin<_, _, Analog>` (PxSEL = 11 disconnects the
//! digital path) implementing [`CompPin`] through
//! [`watch_pin`](CompE::watch_pin). Either way the driver sets the channel's
//! `CEPD` bit: SLAU367 sells it as the input-buffer disable (stops a mid-rail
//! analog level from burning shoot-through current in the pin's CMOS input
//! stage), but on hardware it is also the **pad→comparator connection
//! enable** — a channel without its `CEPD` bit reads nothing at all
//! (HW-established 2026-07-05). `CEPD` kills the input *buffer*, not the
//! output *driver*, so the raw [`watch_channel`](CompE::watch_channel)
//! sibling lets a fixture compare a pad the firmware is simultaneously
//! driving as a digital output. `COUT` (the comparator output on a pin) and
//! the `CESHORT` sample/hold switch are not implemented.
//!
//! Channel map on this package (SLAS704, shared with the ADC `Ax` inputs):
//! P1.0–P1.5 = C0–C5, P3.0–P3.3 = C12–C15. (P2.3/P2.4 and P4.x carry ADC
//! channels whose comparator channel numbers this driver does not pin down —
//! left unimplemented rather than guessed.)
//!
//! # Example: battery-sag alarm that sleeps
//!
//! ```ignore
//! let mut comp = CompE::new(p.comparator_e, Config::default());
//! // Wake when the divided battery on P1.3 falls below ~1.0 V (rising back
//! // above ~1.1 V re-arms — one ladder step of hysteresis at 3.6 V VCC).
//! let pin = port1.pin3.into_analog();
//! comp.watch_pin(&pin, Threshold::vcc_ladder_hysteresis(9, 8));
//! comp.enable_output_interrupts();          // COMP_E vector, both edges
//! power::enter_lpm3();                      // ~1 µA until the crossing
//! ```

use crate::gpio::{Analog, Pin, P1, P3};
use crate::pac;
use crate::ref_a::ReferenceVoltage;

// Pure tap↔millivolt arithmetic (host-tested; see unit_tests/src/comp_ladder.rs).
pub use crate::comp_ladder::{ladder_millivolts, tap_for_millivolts, LADDER_TAPS};

/// `CEPWRMD` — the speed/current trade. Every step down roughly trades an
/// order of magnitude of supply current for a slower output (longer
/// propagation delay and settling).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PowerMode {
    /// Fastest response, highest current.
    HighSpeed,
    /// The middle setting — the reset default and this driver's
    /// [`Config::default`].
    Normal,
    /// Slowest, ~µA class; the mode for always-on threshold watching in LPM3/4.
    UltraLowPower,
}

/// `CEFDLY` — analog glitch filter on the output path. The comparator's raw
/// output can sliver when the inputs cross slowly; the filter suppresses
/// pulses shorter than the selected delay (at the cost of that much latency).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FilterDelay {
    /// ~450 ns.
    Ns450,
    /// ~900 ns.
    Ns900,
    /// ~1800 ns.
    Ns1800,
    /// ~3600 ns.
    Ns3600,
}

/// What the V− terminal compares against — the reference block's three
/// source arrangements (`CERS`/`CEREFL` + the two ladder tap registers).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Threshold {
    /// Resistor ladder fed by **VCC** — thresholds are *fractions of the
    /// supply* (ratiometric). Tap `n` = (n+1)/32 of VCC.
    ///
    /// `rising_tap` (`CEREF0`) is armed while the output is low — the level
    /// the input must rise above. `falling_tap` (`CEREF1`) is armed while
    /// the output is high — the level the input must fall below. Equal taps
    /// = no hysteresis.
    VccLadder { rising_tap: u8, falling_tap: u8 },
    /// Resistor ladder fed by the **shared bandgap reference** at 1.2/2.0/
    /// 2.5 V — *absolute* thresholds, immune to supply sag. Comp_E requests
    /// the bandgap from the REF module on its own (`CEREFL`); no
    /// [`crate::ref_a::Ref`] is required.
    RefLadder {
        level: ReferenceVoltage,
        rising_tap: u8,
        falling_tap: u8,
    },
    /// The bandgap reference applied to V− **directly**, bypassing the
    /// ladder — exactly 1.2/2.0/2.5 V, no taps, no hysteresis.
    RefDirect { level: ReferenceVoltage },
}

impl Threshold {
    /// VCC ladder, one tap, no hysteresis: flip at (tap+1)/32 of VCC.
    pub const fn vcc_ladder(tap: u8) -> Self {
        Threshold::VccLadder {
            rising_tap: tap,
            falling_tap: tap,
        }
    }

    /// VCC ladder with hysteresis. `rising_tap` must be ≥ `falling_tap` for
    /// a useful band (the driver does not enforce it — an inverted band just
    /// makes the comparator *more* eager, not broken).
    pub const fn vcc_ladder_hysteresis(rising_tap: u8, falling_tap: u8) -> Self {
        Threshold::VccLadder {
            rising_tap,
            falling_tap,
        }
    }

    /// Bandgap-fed ladder, one tap, no hysteresis: flip at (tap+1)/32 of
    /// `level`.
    pub const fn ref_ladder(level: ReferenceVoltage, tap: u8) -> Self {
        Threshold::RefLadder {
            level,
            rising_tap: tap,
            falling_tap: tap,
        }
    }

    /// VCC ladder, tap chosen as the closest to `threshold_mv` given the
    /// actual supply (measure it with
    /// [`crate::adc::Adc::read_supply_millivolts`] — on this LaunchPad the
    /// eZ-FET LDO delivers ~3.63 V, not 3.3 V).
    pub fn vcc_ladder_millivolts(threshold_mv: u16, avcc_mv: u16) -> Self {
        Threshold::vcc_ladder(tap_for_millivolts(threshold_mv, avcc_mv))
    }
}

/// Static comparator configuration (everything that is not signal routing).
#[derive(Clone, Copy, Debug)]
pub struct Config {
    power_mode: PowerMode,
    filter: Option<FilterDelay>,
    invert_output: bool,
}

impl Default for Config {
    /// Normal power mode, no output filter, non-inverted output.
    fn default() -> Self {
        Config {
            power_mode: PowerMode::Normal,
            filter: None,
            invert_output: false,
        }
    }
}

impl Config {
    /// Select the speed/current trade (`CEPWRMD`).
    pub fn power_mode(mut self, mode: PowerMode) -> Self {
        self.power_mode = mode;
        self
    }

    /// Enable the output glitch filter (`CEF` + `CEFDLY`).
    pub fn filter(mut self, delay: FilterDelay) -> Self {
        self.filter = Some(delay);
        self
    }

    /// Invert `CEOUT` (`CEOUTPOL`) — output reads 1 while the input is
    /// *below* the threshold. Note the interrupt-edge naming in [`read_iv`]
    /// follows the (possibly inverted) output, not the analog crossing.
    pub fn invert_output(mut self) -> Self {
        self.invert_output = true;
        self
    }
}

/// The Comp_E comparator, configured but with no input watched yet — call
/// [`watch_pin`](CompE::watch_pin)/[`watch_channel`](CompE::watch_channel)
/// to route a signal and turn it on.
pub struct CompE {
    ce: pac::ComparatorE,
}

impl CompE {
    /// Program the static configuration; the comparator stays **off**
    /// (`CEON = 0`, no channel routed, ~zero current) until a `watch_*`
    /// call.
    pub fn new(ce: pac::ComparatorE, config: Config) -> Self {
        ce.cectl1().write(|w| {
            match config.power_mode {
                PowerMode::HighSpeed => w.cepwrmd().cepwrmd_0(),
                PowerMode::Normal => w.cepwrmd().cepwrmd_1(),
                PowerMode::UltraLowPower => w.cepwrmd().cepwrmd_2(),
            };
            if let Some(delay) = config.filter {
                w.cef().set_bit();
                match delay {
                    FilterDelay::Ns450 => w.cefdly().cefdly_0(),
                    FilterDelay::Ns900 => w.cefdly().cefdly_1(),
                    FilterDelay::Ns1800 => w.cefdly().cefdly_2(),
                    FilterDelay::Ns3600 => w.cefdly().cefdly_3(),
                };
            }
            w.ceoutpol().bit(config.invert_output)
            // CEIES/CEEX/CESHORT/CEMRVS/CEON all deliberately 0.
        });
        CompE { ce }
    }

    /// Watch a typed analog pin: route its channel to V+, the threshold to
    /// V−, and turn the comparator on.
    ///
    /// The pin is only borrowed: the `Analog` typestate already proves the
    /// digital path is disconnected for good, and Comp_E holds no per-pin
    /// state worth owning.
    pub fn watch_pin<P: CompPin>(&mut self, _pin: &P, threshold: Threshold) {
        self.route(P::CHANNEL, threshold);
    }

    /// Watch a bare channel number (C0–C15) without touching the pin mux.
    ///
    /// This is the escape hatch for inputs that are not conventional analog
    /// pins — e.g. a pad the firmware itself drives as a digital output
    /// (`CEPD` kills the pad's input *buffer*, not its output *driver*, so
    /// the driven rail is visible to the comparator), which is exactly how
    /// the hands-free integration fixture stimulates the comparator with no
    /// wiring. Channels above 15 are masked.
    pub fn watch_channel(&mut self, channel: u8, threshold: Threshold) {
        self.route(channel & 0x0F, threshold);
    }

    /// The shared routing sequence: comparator off, program input mux and
    /// reference, then on. Reconfiguring can latch a spurious edge flag —
    /// arm interrupts afterwards (or re-arm; the enable sequence clears
    /// pending flags).
    fn route(&mut self, channel: u8, threshold: Threshold) {
        self.ce.cectl1().modify(|_, w| w.ceon().clear_bit());

        // V+ ← channel, V− ← reference block (CERSEL = 1).
        self.ce.cectl0().write(|w| {
            w.ceipsel().set(channel);
            w.ceipen().set_bit()
        });
        // CEPD is not optional: hardware-established here 2026-07-05 — a
        // channel whose CEPD bit is clear never reaches the comparator (the
        // fixture's driven-pad rails test read a dead output until this was
        // set). So the bit is *both* the pad→comparator connection enable
        // and the digital input-buffer disable, not merely the
        // shoot-through guard SLAU367's name suggests.
        // SAFETY: raw register-level RMW — CECTL3 is 16 independent CEPD
        // bits, one per channel; only the watched channel's bit is set.
        self.ce
            .cectl3()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1u16 << channel)) });

        self.ce.cectl2().write(|w| {
            w.cersel().set_bit(); // reference → V− terminal
            match threshold {
                Threshold::VccLadder {
                    rising_tap,
                    falling_tap,
                } => {
                    w.cers().cers_1(); // ladder fed by VCC
                    w.cerefl().cerefl_0(); // no bandgap request
                    w.ceref0().set(rising_tap & 0x1F);
                    w.ceref1().set(falling_tap & 0x1F)
                }
                Threshold::RefLadder {
                    level,
                    rising_tap,
                    falling_tap,
                } => {
                    w.cers().cers_2(); // ladder fed by the shared reference
                    match level {
                        ReferenceVoltage::V1_2 => w.cerefl().cerefl_1(),
                        ReferenceVoltage::V2_0 => w.cerefl().cerefl_2(),
                        ReferenceVoltage::V2_5 => w.cerefl().cerefl_3(),
                    };
                    w.ceref0().set(rising_tap & 0x1F);
                    w.ceref1().set(falling_tap & 0x1F)
                }
                Threshold::RefDirect { level } => {
                    w.cers().cers_3(); // reference direct, ladder bypassed
                    match level {
                        ReferenceVoltage::V1_2 => w.cerefl().cerefl_1(),
                        ReferenceVoltage::V2_0 => w.cerefl().cerefl_2(),
                        ReferenceVoltage::V2_5 => w.cerefl().cerefl_3(),
                    }
                }
            }
        });

        self.ce.cectl1().modify(|_, w| w.ceon().set_bit());
    }

    /// Move the ladder taps of an already-routed threshold (`CEREF0` =
    /// rising/output-low tap, `CEREF1` = falling/output-high tap) without
    /// the off/on reconfiguration cycle — tap stepping while running is what
    /// the hysteresis hardware itself does on every output flip. This is the
    /// sweep primitive: step the tap, wait a settling time, read
    /// [`output`](CompE::output), and the comparator becomes a manual SAR.
    pub fn set_taps(&mut self, rising_tap: u8, falling_tap: u8) {
        self.ce.cectl2().modify(|_, w| {
            w.ceref0().set(rising_tap & 0x1F);
            w.ceref1().set(falling_tap & 0x1F)
        });
    }

    /// The comparator output `CEOUT`, after polarity and filter: with the
    /// default configuration, `true` = input above threshold. Instantaneous
    /// level, not a latched event — for edges, use the interrupt flags.
    pub fn output(&self) -> bool {
        self.ce.cectl1().read().ceout().bit_is_set()
    }

    /// `CEEX`: exchange which terminal each input reaches *and* invert the
    /// output, both in silicon — so the **logical comparison is preserved**
    /// (swap and invert cancel; HW-verified 2026-07-05 at both output
    /// states). What changes is which physical terminal carries which
    /// signal: measuring both ways and averaging cancels the comparator's
    /// input offset voltage, the register's actual purpose. For a sense
    /// inversion, use [`Config::invert_output`].
    pub fn exchange_inputs(&mut self, exchange: bool) {
        self.ce.cectl1().modify(|_, w| w.ceex().bit(exchange));
    }

    /// Clear both latched output-edge flags (`CEIFG`, `CEIIFG`).
    ///
    /// Inside a critical section because `CEINT` mixes flags and enables in
    /// one register and the ISR's `CEIV` read clears flags concurrently.
    #[cfg(feature = "critical-section")]
    pub fn clear_pending_interrupts(&mut self) {
        critical_section::with(|_| {
            self.ce
                .ceint()
                .modify(|_, w| w.ceifg().clear_bit().ceiifg().clear_bit());
        });
    }

    /// Arm the `COMP_E` vector for **both output edges**: fix `CEIES = 0`
    /// (so `CEIFG` = rising `CEOUT` edge → IV 0x02, `CEIIFG` = falling →
    /// IV 0x04), clear both stale flags, then set both enables — the same
    /// program-edge / clear / enable order the GPIO arming rule mandates,
    /// because the edge-select write can itself latch a flag. Nothing fires
    /// until GIE is set (`msp430::interrupt::enable()` or a
    /// `power::enter_lpm*` entry).
    #[cfg(feature = "critical-section")]
    pub fn enable_output_interrupts(&mut self) {
        critical_section::with(|_| {
            self.ce.cectl1().modify(|_, w| w.ceies().clear_bit());
            self.ce.ceint().modify(|_, w| {
                w.ceifg().clear_bit();
                w.ceiifg().clear_bit();
                w.ceie().set_bit();
                w.ceiie().set_bit()
            });
        });
    }

    /// Disarm both output-edge interrupt enables. Latched flags stay pending
    /// (they would fire on re-enable) — the re-arm sequence in
    /// [`enable_output_interrupts`](CompE::enable_output_interrupts) clears
    /// them.
    #[cfg(feature = "critical-section")]
    pub fn disable_output_interrupts(&mut self) {
        critical_section::with(|_| {
            self.ce
                .ceint()
                .modify(|_, w| w.ceie().clear_bit().ceiie().clear_bit());
        });
    }

    /// Comparator off (`CEON = 0`, both edge enables cleared) and the PAC
    /// peripheral back.
    #[cfg(feature = "critical-section")]
    pub fn free(self) -> pac::ComparatorE {
        self.ce.cectl1().modify(|_, w| w.ceon().clear_bit());
        critical_section::with(|_| {
            self.ce
                .ceint()
                .modify(|_, w| w.ceie().clear_bit().ceiie().clear_bit());
        });
        self.ce
    }
}

/// `CEIV` value for a `CEIFG` service — with the driver's `CEIES = 0`, the
/// comparator output **rose** (input crossed above the threshold).
pub const IV_OUTPUT_ROSE: u16 = 0x0002;
/// `CEIV` value for a `CEIIFG` service — output **fell** (input crossed
/// below).
pub const IV_OUTPUT_FELL: u16 = 0x0004;

/// ISR-side: read `CEIV`.
///
/// Returns [`IV_OUTPUT_ROSE`]/[`IV_OUTPUT_FELL`] for the highest-priority
/// pending *enabled* flag (or 0), and atomically clears that flag in the
/// same silicon read — immune to the lost-flag RMW hazard, like every other
/// IV register. Call in a loop until 0 if both edges may be pending. A free
/// function on a stolen handle because ISRs do not own the [`CompE`] driver
/// (the interrupt-conventions rule).
pub fn read_iv() -> u16 {
    // SAFETY: a stolen handle reading the self-clearing CEIV — the architected
    // way to service the COMP_E vector; touches no state the CompE owner writes.
    let ce = unsafe { pac::ComparatorE::steal() };
    ce.ceiv().read().bits()
}

// ---------------------------------------------------------------------------
// Typed analog pins → Comp_E channel numbers
// ---------------------------------------------------------------------------

mod sealed {
    pub trait Sealed {}
}

/// A pin wired to a Comp_E input channel. Implemented only for the
/// `Pin<_, _, Analog>` types whose comparator channel number the SLAS704
/// datasheet pins down unambiguously (the `Cx` on the same pad as ADC `Ax`);
/// the associated constant is that channel's `CEIPSEL`/`CEIMSEL` value.
/// Sealed — the set of comparator pins is fixed by the package.
pub trait CompPin: sealed::Sealed {
    /// The Comp_E input channel this pin feeds.
    const CHANNEL: u8;
}

macro_rules! comp_pins {
    ($($Port:ident $N:literal => $ch:literal),+ $(,)?) => {$(
        impl sealed::Sealed for Pin<$Port, $N, Analog> {}
        impl CompPin for Pin<$Port, $N, Analog> {
            const CHANNEL: u8 = $ch;
        }
    )+};
}

// MSP430FR5969 Comp_E external inputs (SLAS704 pin functions: C0–C5 share
// P1.0–P1.5 with A0–A5, C12–C15 share P3.0–P3.3 with A12–A15).
comp_pins! {
    P1 0 => 0,  P1 1 => 1,  P1 2 => 2,  P1 3 => 3,
    P1 4 => 4,  P1 5 => 5,
    P3 0 => 12, P3 1 => 13, P3 2 => 14, P3 3 => 15,
}
