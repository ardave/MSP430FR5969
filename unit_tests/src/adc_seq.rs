//! Host-side tests for `hal/src/adc_seq.rs` — the ADC12_B
//! sequence-of-channels member validation, `ADC12MCTLx` word encoding, and
//! `ADC12CTL3` internal-source map-bit derivation that
//! `hal::adc::Adc::read_sequence*` delegate to.
//!
//! Anchors (SLAU367P, ADC12MCTLx): the input channel select occupies bits
//! 4:0, **end-of-sequence is bit 7**, and the reference select is the nibble
//! at bits 11:8 (0 = VR+ is AVCC, 1 = VR+ is the buffered VREF) — so a
//! shifted EOS, a transposed VRSEL nibble, EOS on the wrong member (a scan
//! that stops early or runs into unprogrammed MCTLs), or an accepted
//! over-long sequence can't survive these tests.

include!("../../hal/src/adc_seq.rs");

// ------------------------------------------------------------ constructors

#[test]
fn constructors_pick_the_documented_channels_and_references() {
    // The supply monitor is A31; the temperature sensor is A30 (routed by
    // BATMAP/TCMAP respectively). Reference choice mirrors the
    // single-conversion methods: supply_half ratiometric, supply_vref and
    // temperature absolute.
    let half = SeqMember::supply_half();
    assert_eq!(half.channel, 31);
    assert!(!half.needs_vref());
    assert_eq!(half.internal, SeqInternal::SupplyHalf);

    let sup = SeqMember::supply_vref();
    assert_eq!(sup.channel, 31);
    assert!(sup.needs_vref());
    assert_eq!(sup.internal, SeqInternal::SupplyHalf);

    let temp = SeqMember::temperature();
    assert_eq!(temp.channel, 30);
    assert!(temp.needs_vref());
    assert_eq!(temp.internal, SeqInternal::Temperature);

    // Raw channel constructors carry no internal mapping.
    let ext = SeqMember::channel(4);
    assert_eq!(ext.channel, 4);
    assert!(!ext.needs_vref());
    assert_eq!(ext.internal, SeqInternal::None);
    assert!(SeqMember::channel_vref(4).needs_vref());
}

// ----------------------------------------------------------- MCTL encoding

#[test]
fn mctl_word_places_inch_vrsel_and_eos() {
    // Channel in bits 4:0, nothing else set for a non-last AVCC member.
    assert_eq!(mctl_word(&SeqMember::channel(4), false), 0x0004);
    // VRSEL = 1 is the nibble at bits 8:11 → 0x0100.
    assert_eq!(mctl_word(&SeqMember::channel_vref(4), false), 0x0104);
    // EOS is bit 7 → 0x0080.
    assert_eq!(mctl_word(&SeqMember::channel(4), true), 0x0084);
    // All three together, on the top channel: A30 | VREF | EOS.
    assert_eq!(mctl_word(&SeqMember::temperature(), true), 0x019E);
    // The supply monitor at A31, ratiometric, last: INCH = 31 | EOS.
    assert_eq!(mctl_word(&SeqMember::supply_half(), true), 0x009F);
}

#[test]
fn encode_places_eos_on_the_last_member_only() {
    // A 3-member mixed-reference scan — the fixture's own shape. EOS on
    // word 2 and nowhere else: EOS on an earlier member truncates the scan,
    // a missing final EOS runs the sequencer into unprogrammed MCTLs.
    let members = [
        SeqMember::temperature(),
        SeqMember::supply_vref(),
        SeqMember::supply_half(),
    ];
    let mut words = [0u16; MAX_SEQUENCE];
    encode_mctl(&members, &mut words);
    assert_eq!(words[0], 0x011E); // A30, VREF, no EOS
    assert_eq!(words[1], 0x011F); // A31, VREF, no EOS
    assert_eq!(words[2], 0x009F); // A31, AVCC, EOS
}

#[test]
fn single_member_sequence_is_immediately_eos() {
    let mut words = [0u16; MAX_SEQUENCE];
    encode_mctl(&[SeqMember::channel(0)], &mut words);
    assert_eq!(words[0], EOS);
}

// ----------------------------------------------------------------- CTL3 map

#[test]
fn map_bits_reflect_the_internal_sources_used() {
    let none = [SeqMember::channel(4), SeqMember::channel(5)];
    assert_eq!(map_bits(&none), (false, false));

    let bat = [SeqMember::supply_half()];
    assert_eq!(map_bits(&bat), (true, false));

    let tc = [SeqMember::temperature()];
    assert_eq!(map_bits(&tc), (false, true));

    // Both internal sources in one scan: both maps up (they steer
    // different channels — A31 vs A30 — so they don't conflict).
    let both = [SeqMember::temperature(), SeqMember::supply_vref(), SeqMember::channel(4)];
    assert_eq!(map_bits(&both), (true, true));

    // A raw channel number 30/31 without the internal constructor does NOT
    // imply a map bit — the caller asked for the (unmapped) pad channel.
    let raw_top = [SeqMember::channel(30), SeqMember::channel(31)];
    assert_eq!(map_bits(&raw_top), (false, false));
}

// --------------------------------------------------------------- validation

#[test]
fn validate_accepts_the_full_size_range() {
    let one = [SeqMember::channel(0)];
    assert_eq!(validate(&one, 1, false), Ok(()));

    let eight = [SeqMember::channel(0); MAX_SEQUENCE];
    assert_eq!(validate(&eight, 8, false), Ok(()));
}

#[test]
fn validate_rejects_empty_and_overlong() {
    assert_eq!(validate(&[], 8, false), Err(SequenceError::Empty));

    // Nine members would put member 8 into MEM8, outside ADC12SHT0x's
    // coverage — rejected, not silently under-sampled.
    let nine = [SeqMember::channel(0); 9];
    assert_eq!(validate(&nine, 9, false), Err(SequenceError::TooLong));
}

#[test]
fn validate_requires_the_results_buffer_to_fit() {
    let three = [SeqMember::channel(0); 3];
    assert_eq!(validate(&three, 2, false), Err(SequenceError::BufferTooShort));
    // A larger buffer is fine — only the first three slots are written.
    assert_eq!(validate(&three, 8, false), Ok(()));
}

#[test]
fn validate_rejects_out_of_range_channels() {
    // INCH is 5 bits: 31 is the top channel, 32 doesn't exist.
    let top = [SeqMember::channel(31)];
    assert_eq!(validate(&top, 1, false), Ok(()));
    let bogus = [SeqMember::channel(32)];
    assert_eq!(validate(&bogus, 1, false), Err(SequenceError::ChannelOutOfRange));
}

#[test]
fn validate_gates_vref_members_on_the_ref_proof() {
    // A VREF member through the plain (no-&Ref) path: refused — the
    // reference may be off and the temp sensor unpowered.
    let temp = [SeqMember::temperature()];
    assert_eq!(validate(&temp, 1, false), Err(SequenceError::NeedsRef));
    assert_eq!(validate(&temp, 1, true), Ok(()));

    // Mixed scan: one VREF member is enough to require the proof; with it,
    // AVCC members ride along freely.
    let mixed = [SeqMember::supply_half(), SeqMember::temperature()];
    assert_eq!(validate(&mixed, 2, false), Err(SequenceError::NeedsRef));
    assert_eq!(validate(&mixed, 2, true), Ok(()));

    // An all-AVCC scan never needs the proof.
    let plain = [SeqMember::supply_half(), SeqMember::channel(4)];
    assert_eq!(validate(&plain, 2, false), Ok(()));
}
