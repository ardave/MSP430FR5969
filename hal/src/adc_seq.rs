// Pure ADC12_B sequence-of-channels math for the `adc` module.
//
// Like `rtc_alarm.rs` and `capture_math.rs`, this file is intentionally
// dependency-free (pure `core` integer arithmetic, no PAC/HAL types) so the
// exact same source can be `include!`d by the host-side test crate in
// `unit_tests/`. The `adc::Adc` sequence methods are a thin wrapper over
// these conversions — a regression here (a shifted `EOS` bit, a transposed
// `VRSEL` nibble, an accepted over-long sequence) fails the host tests
// without a board on the desk. Do not add external `use`s. (Regular `//`
// comments, not `//!`, so the file can be `include!`d mid-crate.)
//
// # The hardware's sequence model (SLAU367P, "ADC12_B Conversion Modes")
//
// A *sequence of channels* (`ADC12CONSEQ = 1`) converts the memory-control
// registers `ADC12MCTLx` in address order, starting at `ADC12CSTARTADD`
// (this driver always starts at 0, i.e. `MCTL0`), each result landing in the
// matching `ADC12MEMx`, until it converts a register whose **`ADC12EOS`**
// bit is set — the end-of-sequence marker. Each `MCTLx` carries its *own*
// input channel (`ADC12INCH`) **and its own reference pair** (`ADC12VRSEL`),
// so one hardware-triggered scan can mix a ratiometric supply check with
// absolute VREF-referenced measurements. With `ADC12MSC` set, one trigger
// runs the whole scan back-to-back.
//
// The sequence length is capped at **8 members** (`MEM0..MEM7`) because
// `ADC12SHT0x` — the one sample-time field this driver programs — covers
// exactly those registers; `MEM8..MEM31` sample under `ADC12SHT1x`, which
// the driver deliberately leaves alone (one sample-time knob, uniformly
// applied, is the whole configuration story).

/// Longest supported sequence: `MEM0..MEM7`, the registers whose
/// sample-and-hold time `ADC12SHT0x` (the driver's one sample-time setting)
/// governs. Longer scans would silently sample under the unprogrammed
/// `ADC12SHT1x` field, so they are rejected instead.
pub const MAX_SEQUENCE: usize = 8;

// ADC12MCTLx field layout (SLAU367P): input channel select in bits 4:0,
// end-of-sequence in bit 7, reference select in bits 11:8 (0 = VR+ is AVCC,
// 1 = VR+ is the buffered VREF from REF_A; VR- = AVSS in both).
const INCH_MASK: u16 = 0x001F;
const EOS: u16 = 0x0080;
const VRSEL_VREF: u16 = 0x0100;

// The two pinless internal sources, routed onto the top channel numbers by
// the `ADC12CTL3` map bits (see `map_bits`).
const CH_TEMPERATURE: u8 = 30;
const CH_SUPPLY_HALF: u8 = 31;

/// Which on-chip source (if any) a sequence member routes via `ADC12CTL3`.
/// Mirrors `adc::Internal`, redeclared here so this file stays
/// dependency-free.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum SeqInternal {
    /// External pin (or a raw channel number): no internal mapping.
    None,
    /// (AVCC–AVSS)/2 supply monitor on channel A31 (`ADC12BATMAP`).
    SupplyHalf,
    /// Temperature sensor on channel A30 (`ADC12TCMAP`).
    Temperature,
}

/// One member of a hardware-sequenced scan: an input channel plus the
/// reference it is converted against. Build them with the constructors and
/// hand a slice to [`adc::Adc::read_sequence`] / `read_sequence_vref` —
/// results land in the output buffer **in member order** (member `i` →
/// `ADC12MEMi` → `results[i]`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SeqMember {
    pub(crate) channel: u8,
    /// `true` = convert against the buffered REF_A reference
    /// (`ADC12VRSEL = 1`, absolute); `false` = against AVCC (ratiometric).
    pub(crate) vref: bool,
    pub(crate) internal: SeqInternal,
}

impl SeqMember {
    /// An external channel (`0..=31`, see the module channel↔pin map),
    /// converted ratiometrically against AVCC — the sequence sibling of
    /// [`adc::Adc::read_channel`]. Prefer the typed [`SeqMember::pin`]
    /// constructor (defined in `adc.rs`) where you hold the pin.
    pub const fn channel(channel: u8) -> SeqMember {
        SeqMember { channel, vref: false, internal: SeqInternal::None }
    }

    /// An external channel converted against the REF_A reference (absolute;
    /// requires the `_vref` read methods, which take the `&Ref` proof).
    pub const fn channel_vref(channel: u8) -> SeqMember {
        SeqMember { channel, vref: true, internal: SeqInternal::None }
    }

    /// The internal (AVCC–AVSS)/2 supply monitor against AVCC — ratiometric,
    /// so ~half full-scale by construction (the sequence sibling of
    /// [`adc::Adc::read_supply_half`], same long-sample-time advice).
    pub const fn supply_half() -> SeqMember {
        SeqMember { channel: CH_SUPPLY_HALF, vref: false, internal: SeqInternal::SupplyHalf }
    }

    /// The internal (AVCC–AVSS)/2 supply monitor against the REF_A
    /// reference — an absolute supply measurement (the sequence sibling of
    /// [`adc::Adc::read_supply_raw`]; double the millivolt worth to recover
    /// AVCC).
    pub const fn supply_vref() -> SeqMember {
        SeqMember { channel: CH_SUPPLY_HALF, vref: true, internal: SeqInternal::SupplyHalf }
    }

    /// The internal temperature sensor against the REF_A reference (the
    /// sequence sibling of [`adc::Adc::read_temperature`]; the raw count
    /// feeds `tlv::AdcCal::temp_deci_celsius`). The `&Ref` the `_vref` read
    /// methods take is also what biases the sensor.
    pub const fn temperature() -> SeqMember {
        SeqMember { channel: CH_TEMPERATURE, vref: true, internal: SeqInternal::Temperature }
    }

    /// Does this member convert against the REF_A reference (and therefore
    /// require the `_vref` flavor of the sequence read methods)?
    pub const fn needs_vref(&self) -> bool {
        self.vref
    }
}

/// Why a sequence cannot be run.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SequenceError {
    /// No members: nothing to convert.
    Empty,
    /// More than [`MAX_SEQUENCE`] members (only `MEM0..MEM7` sample under
    /// the driver-programmed `ADC12SHT0x`).
    TooLong,
    /// The results buffer is shorter than the member list, so the tail of
    /// the scan would have nowhere to land.
    BufferTooShort,
    /// A member's channel number exceeds 31 (`ADC12INCH` is 5 bits).
    ChannelOutOfRange,
    /// A member converts against the REF_A reference but the caller used a
    /// plain (no-`&Ref`) read method — the reference may be off, and for the
    /// temperature sensor unpowered. Use the `_vref` sibling.
    NeedsRef,
    /// (DMA reads only.) The drain DMA channel did not collect every result
    /// within the driver's generous spin bound — the ADC12→DMA trigger did
    /// not deliver one edge per sequence member. Not produced by the pure
    /// validation here; declared alongside the other errors because the
    /// driver's sequence methods share the one error type.
    DmaIncomplete,
}

/// Validate a member list against the results buffer and the caller's
/// reference proof. `have_ref` is whether the calling read method holds a
/// `&Ref` (the `_vref` flavors).
pub(crate) fn validate(
    members: &[SeqMember],
    results_len: usize,
    have_ref: bool,
) -> Result<(), SequenceError> {
    if members.is_empty() {
        return Err(SequenceError::Empty);
    }
    if members.len() > MAX_SEQUENCE {
        return Err(SequenceError::TooLong);
    }
    if results_len < members.len() {
        return Err(SequenceError::BufferTooShort);
    }
    for m in members {
        if m.channel > 31 {
            return Err(SequenceError::ChannelOutOfRange);
        }
        if m.vref && !have_ref {
            return Err(SequenceError::NeedsRef);
        }
    }
    Ok(())
}

/// The `ADC12MCTLx` register word for one member: channel in `INCH`, the
/// reference pair in `VRSEL` (0 = AVCC/AVSS, 1 = VREF-buffered/AVSS), and
/// `EOS` on the final member so the sequencer stops after it.
pub(crate) fn mctl_word(member: &SeqMember, last: bool) -> u16 {
    (member.channel as u16 & INCH_MASK)
        | if member.vref { VRSEL_VREF } else { 0 }
        | if last { EOS } else { 0 }
}

/// Encode a whole (pre-validated) member list into its `MCTL0..MCTLn`
/// register words — `EOS` lands on the last member and nowhere else.
/// `out` must be at least `members.len()` long.
pub(crate) fn encode_mctl(members: &[SeqMember], out: &mut [u16]) {
    let last = members.len() - 1;
    for (i, m) in members.iter().enumerate() {
        out[i] = mctl_word(m, i == last);
    }
}

/// The `ADC12CTL3` internal-source map bits a member list needs:
/// `(ADC12BATMAP, ADC12TCMAP)`. The maps are chip-global routing switches —
/// `BATMAP` puts the (AVCC–AVSS)/2 monitor on A31, `TCMAP` the temperature
/// sensor on A30 — and they steer *different* channels, so a sequence
/// containing both internal sources simply sets both.
pub(crate) fn map_bits(members: &[SeqMember]) -> (bool, bool) {
    let mut batmap = false;
    let mut tcmap = false;
    for m in members {
        match m.internal {
            SeqInternal::None => {}
            SeqInternal::SupplyHalf => batmap = true,
            SeqInternal::Temperature => tcmap = true,
        }
    }
    (batmap, tcmap)
}
