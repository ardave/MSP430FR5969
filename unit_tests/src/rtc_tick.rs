//! Host-side tests for `hal/src/rtc_tick.rs` — the RTC_B prescaler-tick
//! rate↔register math that `hal::rtc::Rtc::enable_tick_interrupt` delegates
//! to.
//!
//! Anchors: the RT0PS bank divides the 32768 Hz crystal and the RT1PS bank
//! divides RT0PS's 128 Hz output (SLAU367P "RTC_B Operation"), the 3-bit
//! `RTxIP` code selects `/2^(code+1)` of the bank's input, and the sixteen
//! rates chain seamlessly across the bank boundary — so a rate mapped to
//! the wrong bank, an off-by-one interval code, or a broken period can't
//! survive these tests.

include!("../../hal/src/rtc_tick.rs");

/// All sixteen rates in declaration (descending-frequency) order.
const ALL: [TickRate; 16] = [
    TickRate::Hz16384,
    TickRate::Hz8192,
    TickRate::Hz4096,
    TickRate::Hz2048,
    TickRate::Hz1024,
    TickRate::Hz512,
    TickRate::Hz256,
    TickRate::Hz128,
    TickRate::Hz64,
    TickRate::Hz32,
    TickRate::Hz16,
    TickRate::Hz8,
    TickRate::Hz4,
    TickRate::Hz2,
    TickRate::Hz1,
    TickRate::HalfHz,
];

// ------------------------------------------------------------ bank mapping

#[test]
fn first_eight_rates_are_rt0ps_last_eight_rt1ps() {
    for (i, rate) in ALL.iter().enumerate() {
        assert_eq!(
            rate.uses_rt1ps(),
            i >= 8,
            "{rate:?} mapped to the wrong prescaler"
        );
    }
}

#[test]
fn ip_codes_walk_0_to_7_in_each_bank() {
    // TI's RT0IP__2..RT0IP__256 constants: code n = /2^(n+1) of the bank
    // input. Each bank's rates must walk 0..=7 in order.
    for (i, rate) in ALL.iter().enumerate() {
        assert_eq!(rate.ip_code(), (i % 8) as u8, "{rate:?} has the wrong IP code");
    }
}

#[test]
fn boundary_rates_pin_the_bank_split() {
    // 128 Hz is RT0PS's slowest tap (/256 of 32768), 64 Hz is RT1PS's
    // fastest (/2 of 128) — the seam where a >= vs > mistake would land.
    assert!(!TickRate::Hz128.uses_rt1ps());
    assert_eq!(TickRate::Hz128.ip_code(), 7);
    assert!(TickRate::Hz64.uses_rt1ps());
    assert_eq!(TickRate::Hz64.ip_code(), 0);
}

// ------------------------------------------------------------- frequencies

#[test]
fn frequencies_halve_down_the_table() {
    // hz_x2 keeps 0.5 Hz integral: 16384 Hz -> 32768, 0.5 Hz -> 1.
    let mut expect = 32768u32;
    for rate in ALL {
        assert_eq!(rate.hz_x2(), expect, "{rate:?} frequency wrong");
        expect /= 2;
    }
    // The table ends exactly at 0.5 Hz.
    assert_eq!(TickRate::HalfHz.hz_x2(), 1);
}

#[test]
fn banks_chain_by_a_factor_of_256() {
    // RT1PS at code n runs 256x slower than RT0PS at the same code — the
    // divider-chain fact the whole single-enum design leans on.
    assert_eq!(
        TickRate::Hz16384.hz_x2(),
        TickRate::Hz64.hz_x2() * 256
    );
    assert_eq!(TickRate::Hz128.hz_x2(), TickRate::HalfHz.hz_x2() * 256);
}

// ------------------------------------------------------------------ period

#[test]
fn periods_match_the_crystal() {
    assert_eq!(TickRate::Hz1.period_us(), 1_000_000);
    assert_eq!(TickRate::HalfHz.period_us(), 2_000_000);
    assert_eq!(TickRate::Hz128.period_us(), 7_812); // 7812.5 truncated
    assert_eq!(TickRate::Hz32.period_us(), 31_250);
    assert_eq!(TickRate::Hz16384.period_us(), 61); // 61.035... truncated
}

#[test]
fn period_and_frequency_are_consistent() {
    // period_us = floor(2e6 / hz_x2), so the round trip loses strictly less
    // than one hz_x2 unit: 2e6 - hz_x2 < period_us * hz_x2 <= 2e6.
    for rate in ALL {
        let product = rate.period_us() as u64 * rate.hz_x2() as u64;
        let floor = 2_000_000 - rate.hz_x2() as u64;
        assert!(
            product > floor && product <= 2_000_000,
            "{rate:?}: period/frequency drifted apart (product {product})"
        );
    }
}
