#![no_std]
#![no_main]

//! TA0 channel-count probe — **no wiring at all**, driven by the host-side
//! `ta0_probe_tests` runner. Settles a three-way documentation conflict on
//! silicon:
//!
//! - SLAS704G's TA0 register table lists `TA0CCTL3/4` (0x0348/0x034A) and
//!   `TA0CCR3/4` (0x0358/0x035A) — a five-channel TA0.
//! - SLAS704G's own section 6.10.10 prose ("three capture/compare registers
//!   each"), its Table 6-13 signal connections, TI's `msp430fr5969.h`
//!   (`Timer0_A3`, CCR0–2 only), and the SVD all say **three** channels.
//!
//! The registers either exist at those addresses or they don't, so the fixture
//! probes them directly via raw volatile pointers (the PAC has no CCR3/CCR4
//! accessors — that absence is the question under test), the same technique
//! `hal::mpu` uses for its un-generated block. Three independent probes per
//! channel, which must agree:
//!
//! - **readback** — write/read-back patterns in the putative `TA0CCRn` and
//!   `TA0CCTLn` (read-only/latch bits masked). Unimplemented FR-family
//!   register addresses don't hold state.
//! - **functional capture** — arm the putative `TA0CCTLn` for a synchronized
//!   both-edge capture of the software `CCIS` input and fire the GND→VCC
//!   toggle (the same trick `capture_test_runner` uses). A real channel
//!   latches `CCIFG` and stamps `TA0CCRn` inside a `TA0R` bracket; absent
//!   silicon cannot fake that behavior.
//! - **IV demux** — with only that channel's `CCIE` armed (GIE off, so the
//!   flag pends without needing an ISR), `TA0IV` must read the channel's slot
//!   (CCR3 → 0x06, CCR4 → 0x08) if it exists, 0x00 if not. This is the part
//!   no documentation table can vouch for — the RTCIV lesson.
//!
//! **Channel 2 runs first as a positive control** through the identical probe
//! code (expected: readback sticks, capture fires bracketed, `TA0IV` = 0x04).
//! If the control fails, the method is broken and no finding is trustworthy.
//! An **alias check** then proves the CCR3/CCR4 probe writes didn't disturb
//! CCR0–CCR2/CCTL0–CCTL2 (a partially-decoded address bus would fold the
//! probed addresses onto real registers).
//!
//! # Framed output for the host runner
//!
//! ```text
//! ta0 ch2 iv=04 | ch3 rbccr=0 rbctl=0 cap=0 brk=0 iv=00 | ch4 rbccr=0 rbctl=0 cap=0 brk=0 iv=00 | alias=1
//! TA0_PROBE_BEGIN
//! TA0 CH2 CONTROL OK
//! TA0 CH3 CONSISTENT OK
//! TA0 CH4 CONSISTENT OK
//! TA0 NO ALIAS OK
//! TA0 CH3 ABSENT
//! TA0 CH4 ABSENT
//! TA0_PROBE_END
//! ```
//!
//! The `CH3`/`CH4` lines are **findings**, not pass/fail verdicts: `PRESENT`
//! (all three probes affirm), `ABSENT` (all three deny), or `MIXED` (probes
//! disagree — also fails the CONSISTENT verdict). The host runner pins the
//! expected finding; the fixture itself has no opinion. **GREEN** while the
//! control, both consistency checks, and the alias check pass — whichever way
//! the findings land — **RED** otherwise.

use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::gpio::GpioExt;
use hal::serial::{Config as UartConfig, SerialExt};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

// TA0 register block (base 0x0340), per the SLAS704G memory map. CCTL3/CCTL4
// and CCR3/CCR4 are the addresses under test — the PAC deliberately has no
// accessors for them.
const TA0CTL: *mut u16 = 0x0340 as *mut u16;
const TA0R: *mut u16 = 0x0350 as *mut u16;
const TA0IV: *mut u16 = 0x036E as *mut u16;

/// `TA0CCTLn` for n = 0..=4 (offset 0x02 + 2n).
const fn cctl(n: usize) -> *mut u16 {
    (0x0342 + 2 * n) as *mut u16
}

/// `TA0CCRn` for n = 0..=4 (offset 0x12 + 2n).
const fn ccr(n: usize) -> *mut u16 {
    (0x0352 + 2 * n) as *mut u16
}

// TA0CTL fields.
const TASSEL_SMCLK: u16 = 0x0200;
const MC_CONT: u16 = 0x0020;
const TACLR: u16 = 0x0004;

// TAxCCTLn fields.
const CM_BOTH: u16 = 0xC000;
const CCIS_GND: u16 = 0x2000;
const CCIS_VCC: u16 = 0x3000;
const SCS: u16 = 0x0800;
const CAP: u16 = 0x0100;
const CCIE: u16 = 0x0010;
const CCIFG: u16 = 0x0001;
/// Read-only / hardware-latched CCTL bits excluded from readback compare:
/// CCI (live input level), COV, CCIFG.
const CCTL_RO_MASK: u16 = !(0x0008 | 0x0002 | 0x0001);

/// Everything the three probes observed about one putative channel.
struct ChannelProbe {
    /// `TA0CCRn` held both write/read-back patterns.
    rb_ccr: bool,
    /// `TA0CCTLn` held both patterns (read-only bits masked).
    rb_cctl: bool,
    /// `CCIFG` latched after the software `CCIS` GND→VCC fire.
    cap_fired: bool,
    /// ...and the captured stamp landed inside the `TA0R` bracket.
    cap_bracketed: bool,
    /// Raw `TA0IV` read with only this channel's `CCIE` armed.
    iv: u16,
}

impl ChannelProbe {
    /// All three probes affirm the channel (IV at the channel's own slot).
    fn present(&self, n: usize) -> bool {
        self.rb_ccr
            && self.rb_cctl
            && self.cap_fired
            && self.cap_bracketed
            && self.iv == (2 * n) as u16
    }

    /// All three probes deny the channel.
    fn absent(&self) -> bool {
        !self.rb_ccr && !self.rb_cctl && !self.cap_fired && self.iv == 0
    }
}

/// Write/read-back probe of one 16-bit register. Restores 0 afterwards.
/// `mask` excludes read-only/latched bits from the comparison.
unsafe fn readback16(reg: *mut u16, a: u16, b: u16, mask: u16) -> bool {
    unsafe {
        reg.write_volatile(a);
        let ra = reg.read_volatile();
        reg.write_volatile(b);
        let rb = reg.read_volatile();
        reg.write_volatile(0);
        (ra & mask) == (a & mask) && (rb & mask) == (b & mask)
    }
}

/// Run all three probes against putative channel `n`. Leaves the timer
/// stopped and the channel's CCTL zeroed.
unsafe fn probe_channel(n: usize) -> ChannelProbe {
    unsafe {
        // --- Readback, timer stopped -----------------------------------
        // CCR: plain 16-bit patterns. CCTL: two patterns exercising every
        // read-write field (CM/CCIS/SCS/CAP/OUTMOD/CCIE/OUT), RO bits masked.
        let rb_ccr = readback16(ccr(n), 0xA5A5, 0x5A5A, 0xFFFF);
        let rb_cctl = readback16(cctl(n), 0x6900, 0xB174, CCTL_RO_MASK);

        // --- Functional capture + IV demux ------------------------------
        // Fresh continuous run on SMCLK (synchronous to MCLK — same DCO — so
        // plain TA0R reads don't tear). Arm the channel for a synchronized
        // both-edge capture of the software CCIS input, with CCIE so the
        // latched flag (GIE off) is visible to TA0IV.
        TA0CTL.write_volatile(TASSEL_SMCLK | TACLR);
        cctl(n).write_volatile(CM_BOTH | CCIS_GND | SCS | CAP | CCIE);
        TA0CTL.write_volatile(TASSEL_SMCLK | MC_CONT);
        let t0 = TA0R.read_volatile();

        // Fire: CCIS GND→VCC is a rising edge on the selected input.
        cctl(n).write_volatile(CM_BOTH | CCIS_VCC | SCS | CAP | CCIE);
        // SCS synchronizes the capture to the next timer clock; at SMCLK
        // 8 MHz vs 1 MHz MCLK the next instruction is already past it, but
        // give it an explicit couple of timer reads' worth of slack.
        let _ = TA0R.read_volatile();
        let _ = TA0R.read_volatile();

        let stamp = ccr(n).read_volatile();
        let ctlv = cctl(n).read_volatile();
        let t1 = TA0R.read_volatile();
        let cap_fired = ctlv & CCIFG != 0;
        // No wrap: the run started from TACLR microseconds ago (a continuous-
        // mode wrap takes 8.19 ms at 8 MHz).
        let cap_bracketed = cap_fired && t0 <= stamp && stamp <= t1;

        // TA0IV shows the highest-priority *enabled* pending source; only
        // channel n's CCIE is set, so a real channel reads its own slot
        // (0x02·n) and the read auto-clears the served flag.
        let iv = TA0IV.read_volatile();

        // Disarm and stop.
        cctl(n).write_volatile(0);
        TA0CTL.write_volatile(0);

        ChannelProbe {
            rb_ccr,
            rb_cctl,
            cap_fired,
            cap_bracketed,
            iv,
        }
    }
}

/// Prove the CCR3/CCR4/CCTL3/CCTL4 probe writes don't alias onto the real
/// channels 0–2 (a partially-decoded address bus would fold them). Timer
/// stopped throughout; restores everything to 0.
unsafe fn alias_check() -> bool {
    unsafe {
        TA0CTL.write_volatile(0);
        for i in 0..3 {
            cctl(i).write_volatile(0);
        }
        ccr(0).write_volatile(0x1111);
        ccr(1).write_volatile(0x2222);
        ccr(2).write_volatile(0x3333);

        // Hammer the putative channel 3/4 registers.
        ccr(3).write_volatile(0xA5A5);
        ccr(4).write_volatile(0x5A5A);
        cctl(3).write_volatile(0x6900);
        cctl(4).write_volatile(0xB174);

        let ok = ccr(0).read_volatile() == 0x1111
            && ccr(1).read_volatile() == 0x2222
            && ccr(2).read_volatile() == 0x3333
            && cctl(0).read_volatile() & CCTL_RO_MASK == 0
            && cctl(1).read_volatile() & CCTL_RO_MASK == 0
            && cctl(2).read_volatile() & CCTL_RO_MASK == 0;

        for i in 0..5 {
            cctl(i).write_volatile(0);
            ccr(i).write_volatile(0);
        }
        ok
    }
}

/// Firmware entry point.
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // MCLK 1 MHz, SMCLK 8 MHz — both DCO-derived, so TA0-on-SMCLK is
    // synchronous with the CPU and TA0R reads don't tear.
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // UART (eUSCI_A0): 9600 8N1, BRCLK = SMCLK = 8 MHz.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, _rx) = serial.split();

    // LEDs: P1.0 = GREEN (LED2), P4.6 = RED (LED1).
    let (port1, _port2) = p.port_1_2.split();
    let (_port3, port4) = p.port_3_4.split();
    let mut green_led = port1.pin0.into_output();
    let mut red_led = port4.pin6.into_output();

    let mut delay = Delay::new(clocks.mclk());

    tx.write_all(b"MSP430FR5969 TA0 channel-count probe: does silicon have CCR3/CCR4? (no wiring)\r\n")
        .ok();

    loop {
        // Channel 2 first: the positive control that validates the probe
        // method itself (its registers and IV slot 0x04 are undisputed).
        let ch2 = unsafe { probe_channel(2) };
        let control_ok = ch2.present(2);

        // The channels under test.
        let ch3 = unsafe { probe_channel(3) };
        let ch4 = unsafe { probe_channel(4) };
        let ch3_consistent = ch3.present(3) || ch3.absent();
        let ch4_consistent = ch4.present(4) || ch4.absent();

        let alias_ok = unsafe { alias_check() };

        // Raw observations, one line, for diagnosis from the transcript alone.
        tx.write_all(b"ta0 ch2 iv=").ok();
        write_hex8(&mut tx, ch2.iv as u8);
        write_channel_info(&mut tx, b" | ch3 ", &ch3);
        write_channel_info(&mut tx, b" | ch4 ", &ch4);
        tx.write_all(b" | alias=").ok();
        tx.write_all(if alias_ok { b"1" as &[u8] } else { b"0" }).ok();
        tx.write_all(b"\r\n").ok();

        // The framed burst: four verdicts, then the two findings.
        tx.write_all(b"TA0_PROBE_BEGIN\r\n").ok();
        verdict(&mut tx, b"TA0 CH2 CONTROL", control_ok);
        verdict(&mut tx, b"TA0 CH3 CONSISTENT", ch3_consistent);
        verdict(&mut tx, b"TA0 CH4 CONSISTENT", ch4_consistent);
        verdict(&mut tx, b"TA0 NO ALIAS", alias_ok);
        finding(&mut tx, b"TA0 CH3", &ch3, 3);
        finding(&mut tx, b"TA0 CH4", &ch4, 4);
        tx.write_all(b"TA0_PROBE_END\r\n").ok();

        // GREEN = the probe machinery is sound (whichever way the findings
        // land); RED = the method itself failed, believe nothing.
        let all_ok = control_ok && ch3_consistent && ch4_consistent && alias_ok;
        if all_ok {
            green_led.set_high().ok();
            red_led.set_low().ok();
        } else {
            red_led.set_high().ok();
            green_led.set_low().ok();
        }

        delay.delay_ms(1000);
    }
}

/// Write one channel's raw probe bits for the info line.
fn write_channel_info<W: hal::embedded_io::Write>(tx: &mut W, label: &[u8], p: &ChannelProbe) {
    tx.write_all(label).ok();
    tx.write_all(b"rbccr=").ok();
    write_bit(tx, p.rb_ccr);
    tx.write_all(b" rbctl=").ok();
    write_bit(tx, p.rb_cctl);
    tx.write_all(b" cap=").ok();
    write_bit(tx, p.cap_fired);
    tx.write_all(b" brk=").ok();
    write_bit(tx, p.cap_bracketed);
    tx.write_all(b" iv=").ok();
    write_hex8(tx, p.iv as u8);
}

/// Write `name` + ` PRESENT`/` ABSENT`/` MIXED` + CRLF.
fn finding<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], p: &ChannelProbe, n: usize) {
    tx.write_all(name).ok();
    tx.write_all(if p.present(n) {
        b" PRESENT\r\n" as &[u8]
    } else if p.absent() {
        b" ABSENT\r\n"
    } else {
        b" MIXED\r\n"
    })
    .ok();
}

/// Write `name` + ` OK`/` FAIL` + CRLF.
fn verdict<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], ok: bool) {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" as &[u8] } else { b" FAIL\r\n" })
        .ok();
}

/// Write a bool as `1`/`0`.
fn write_bit<W: hal::embedded_io::Write>(tx: &mut W, b: bool) {
    tx.write_all(if b { b"1" as &[u8] } else { b"0" }).ok();
}

/// Write a byte as two uppercase hex digits. `core::fmt` is deliberately
/// avoided project-wide (FRAM budget).
fn write_hex8<W: hal::embedded_io::Write>(tx: &mut W, b: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    tx.write_all(&[HEX[(b >> 4) as usize], HEX[(b & 0xF) as usize]])
        .ok();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// compiler-builtins' memcpy/memcmp reference `abort` on their safety paths.
// Provide a minimal one so we don't link newlib's libc (and its syscall stubs).
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
