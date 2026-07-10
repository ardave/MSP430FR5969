// Pure RTC_B prescaler-tick math for the `rtc` module.
//
// Like `rtc_alarm.rs`, this file is intentionally dependency-free (pure
// `core` integer arithmetic, no PAC/HAL types) so the exact same source can
// be `include!`d by the host-side test crate in `unit_tests/`. The
// `rtc::Rtc` tick methods are a thin wrapper over these conversions — a
// regression here (a rate mapped to the wrong prescaler, an off-by-one
// interval code, a wrong period) fails the host tests without a board on
// the desk. Do not add external `use`s. (Regular `//` comments, not `//!`,
// so the file can be `include!`d mid-crate.)
//
// # The hardware's prescaler chain (SLAU367P, "RTC_B Operation")
//
// In calendar mode the RTC_B divider chain is hardwired: **RT0PS** divides
// the 32768 Hz crystal and **RT1PS** divides RT0PS's /256 output (128 Hz);
// RT1PS's own /128 output is the calendar's 1 Hz. The chain cannot be
// re-sourced or re-cascaded on this part — what *is* programmable is one
// interrupt tap per prescaler: the 3-bit `RTxIP` field picks a divide of
// that prescaler's input clock (`/2, /4, … /256` for `IP = 0..7`, TI's
// `RT0IP__2`..`RT0IP__256` constants), and `RTxPSIFG` latches at that rate,
// firing the shared `RTC` vector when `RTxPSIE` is set.
//
// Two taps, sixteen rates, all powers of two:
//
// - RT0PS (input 32768 Hz): 16384, 8192, 4096, 2048, 1024, 512, 256, 128 Hz
// - RT1PS (input 128 Hz):   64, 32, 16, 8, 4, 2, 1, 0.5 Hz
//
// The two banks chain seamlessly — RT1PS at `IP = n` runs 256× slower than
// RT0PS at the same `n` — so a single ordered enum covers the full range,
// with the discriminant encoding everything: bit 3 picks the prescaler,
// bits 2:0 are the `RTxIP` code, and the frequency is
// `32768 >> (discriminant + 1)` (in half-hertz units so 0.5 Hz stays
// integral).

/// A periodic tick rate available from the RTC_B prescaler chain — sixteen
/// power-of-two rates from 16.384 kHz down to one tick every two seconds,
/// crystal-accurate and alive in LPM3 (the prescalers run off ACLK's LFXT
/// like the calendar itself).
///
/// The first eight rates come from prescaler RT0PS (RTCIV slot `0x08`), the
/// last eight from RT1PS (slot `0x0A`); the two are independent, so one of
/// each bank can run concurrently.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TickRate {
    /// 16384 Hz (RT0PS, /2). At a 1 MHz MCLK this leaves ~61 CPU cycles per
    /// tick — see the ISR-starvation warning on the enable method.
    Hz16384 = 0,
    /// 8192 Hz (RT0PS, /4).
    Hz8192 = 1,
    /// 4096 Hz (RT0PS, /8).
    Hz4096 = 2,
    /// 2048 Hz (RT0PS, /16).
    Hz2048 = 3,
    /// 1024 Hz (RT0PS, /32).
    Hz1024 = 4,
    /// 512 Hz (RT0PS, /64).
    Hz512 = 5,
    /// 256 Hz (RT0PS, /128).
    Hz256 = 6,
    /// 128 Hz (RT0PS, /256).
    Hz128 = 7,
    /// 64 Hz (RT1PS, /2).
    Hz64 = 8,
    /// 32 Hz (RT1PS, /4).
    Hz32 = 9,
    /// 16 Hz (RT1PS, /8).
    Hz16 = 10,
    /// 8 Hz (RT1PS, /16).
    Hz8 = 11,
    /// 4 Hz (RT1PS, /32).
    Hz4 = 12,
    /// 2 Hz (RT1PS, /64).
    Hz2 = 13,
    /// 1 Hz (RT1PS, /128).
    Hz1 = 14,
    /// 0.5 Hz — one tick every two seconds (RT1PS, /256).
    HalfHz = 15,
}

impl TickRate {
    /// Which prescaler serves this rate: `false` = RT0PS (`RTCPS0CTL`,
    /// RTCIV `0x08`), `true` = RT1PS (`RTCPS1CTL`, RTCIV `0x0A`).
    pub fn uses_rt1ps(self) -> bool {
        (self as u8) >= 8
    }

    /// The 3-bit `RTxIP` interrupt-interval code for this rate's prescaler
    /// (`0` = /2 … `7` = /256 of that prescaler's input clock).
    pub fn ip_code(self) -> u8 {
        (self as u8) & 0x7
    }

    /// The tick frequency in **half-hertz** (`hz × 2`, so the 0.5 Hz rate
    /// stays integral): `32768 Hz` is `65536`, `0.5 Hz` is `1`. The whole
    /// table is one shift because the banks chain: each discriminant step
    /// halves the rate.
    pub fn hz_x2(self) -> u32 {
        65536 >> ((self as u8) + 1)
    }

    /// Nominal tick period in **microseconds** at a 32768 Hz crystal,
    /// truncated (16384 Hz → 61 µs, the exact value being 61.035…).
    pub fn period_us(self) -> u32 {
        2_000_000 / self.hz_x2()
    }
}
