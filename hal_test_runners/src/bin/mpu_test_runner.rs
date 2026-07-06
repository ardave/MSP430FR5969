#![no_std]
#![no_main]
#![feature(abi_msp430_interrupt)]

//! FRAM MPU integration fixture — **no wiring at all**: the protected memory,
//! the violating accesses, and all three consequence paths (latched flag,
//! `SYSNMI`, PUC reset) are software-only.
//!
//! ```text
//! cargo +nightly build --bin mpu_test_runner
//! DSLite load ... -f target/msp430-none-elf/debug/mpu_test_runner
//! ```
//!
//! # Shape: a state machine across a deliberate reset
//!
//! The PUC-on-violation path can only be tested by *taking* the PUC, so (like
//! `lpmx5_test_runner`) this fixture persists its progress in Info FRAM and
//! runs across reboots — one flash covers all lives:
//!
//! - **Cold phase** (fresh flash / reset button / power-on): six verdicts
//!   computed live, recorded as bits in FRAM — write-through proof before
//!   protection, blocked write (memory unchanged), flag latching + clearing,
//!   `SYSNMI` delivery with exact `SYSSNIV` demux, info-segment protection,
//!   write-through again after disable. Then segment 3 is armed with
//!   PUC-on-violation and deliberately violated. No burst is printed yet.
//! - **After the PUC**: the boot finds `SYSRSTIV` reporting `MPU seg 3`
//!   (`ResetReason::MpuSeg3`) — that *is* the reset verdict. Then the lock
//!   test: `MPULOCK` set, and a raw register write probed. SLAU367 does not
//!   spell out whether a locked write is ignored or PUCs, so **either**
//!   outcome passes — unchanged registers in this life, or one more reboot
//!   that lands here with the state byte saying "lock probe armed" (the
//!   evidence is reported in the info line). Locked-ness surviving the PUC is
//!   part of the verdict (lock outlives everything but BOR). HW-established
//!   2026-07-05: the write is silently **ignored** (`lock=ignored`, borders
//!   unchanged) — no PUC on this silicon.
//! - **Steady state**: the framed verdict burst, once per second, forever.
//!   Reflash or the reset button (both BOR-class — which also clears
//!   `MPULOCK`) restarts the machine cold.
//!
//! The protected target is the upper FRAM bank (`0x10000..`, `HighFram`) —
//! borders at the bank split put all code, `.rodata`, and vectors in segment
//! 1, so the fixture can never fence itself off. The info-segment verdict
//! briefly write-protects Info FRAM itself; the state record is only written
//! while that segment is open.
//!
//! # Framed output for the host runner (`mpu` suite)
//!
//! ```text
//! mpu state=2 causes=MPU seg 3 flags=7F borders=14000/14000 lock=ignored
//! MPU_TEST_BEGIN
//! MPU WRITE PRE OK
//! MPU WRITE BLOCKED OK
//! MPU FLAG LATCH OK
//! MPU NMI DEMUX OK
//! MPU INFO BLOCKED OK
//! MPU WRITE POST OK
//! MPU RESET ON VIOLATION OK
//! MPU LOCK OK
//! MPU_TEST_END
//! ```
//!
//! **GREEN** LED while all eight pass, **RED** otherwise.

use core::cell::Cell;

use critical_section::Mutex;
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::OutputPin;
use hal::embedded_io::Write as _;
use hal::embedded_storage::{ReadStorage, Storage};
use hal::fram::{HighFram, InfoFram};
use hal::gpio::GpioExt;
use hal::interrupt;
use hal::mpu::{Access, Config, Mpu};
use hal::serial::{Config as UartConfig, SerialExt};
use hal::sys::{NmiSource, ResetReason, ResetReasons};
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve for
// pac's Peripherals::take().
use msp430 as _;

/// State record home in Info FRAM (lpmx5_test_runner owns 0x60..0x64; this
/// fixture takes 0x70..0x74). Layout: `[b'M', b'P', state, flags]`.
const FRAM_OFFSET: u32 = 0x70;
const MAGIC: [u8; 2] = [b'M', b'P'];

/// States: cold run in progress / PUC-on-violation armed / lock probe armed.
const STATE_COLD: u8 = 0;
const STATE_AWAIT_PUC: u8 = 1;
const STATE_LOCK_PROBED: u8 = 2;

// Verdict bits in the persisted flags byte.
const F_WRITE_PRE: u8 = 1 << 0;
const F_WRITE_BLOCKED: u8 = 1 << 1;
const F_FLAG_LATCH: u8 = 1 << 2;
const F_NMI_DEMUX: u8 = 1 << 3;
const F_INFO_BLOCKED: u8 = 1 << 4;
const F_WRITE_POST: u8 = 1 << 5;
const F_RESET: u8 = 1 << 6;
const ALL_COLD: u8 =
    F_WRITE_PRE | F_WRITE_BLOCKED | F_FLAG_LATCH | F_NMI_DEMUX | F_INFO_BLOCKED | F_WRITE_POST;

/// Border for every phase: the bank split. Segment 1 = all lower-bank FRAM
/// (code, .rodata, vectors — everything the program needs), segment 2 empty,
/// segment 3 = exactly the HighFram bank.
const BANK_SPLIT: u32 = 0x1_0000;

/// Scratch offsets *within* HighFram (absolute 0x12000) and Info FRAM (0x1900
/// — well clear of both state records).
const HIGH_SCRATCH: u32 = 0x2000;
const INFO_SCRATCH: u32 = 0x100;

const PATTERN_A: [u8; 8] = [0xA5, 0x5A, 0xC3, 0x3C, 0x0F, 0xF0, 0x69, 0x96];
const PATTERN_B: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

/// SYSNMI evidence, ISR → main. (`Mutex<Cell>` per the repo convention; an
/// NMI can preempt a critical section by definition, but these are single
/// byte/flag cells written in the ISR and read by main strictly afterwards.)
static NMI_COUNT: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));
static NMI_LAST: Mutex<Cell<u8>> = Mutex::new(Cell::new(NMI_NONE));

const NMI_NONE: u8 = 0;
const NMI_SEG3: u8 = 1;
const NMI_INFO: u8 = 2;
const NMI_OTHER: u8 = 3;

/// The `SYSNMI` vector: demux `SYSSNIV`, record, and clear the MPU flags —
/// the clear is mandatory (SLAU367 doesn't promise the `SYSSNIV` read does
/// it) or the source stays pending. Non-maskable: fires with GIE never set.
#[msp430_rt::interrupt]
fn SYSNMI() {
    let code = match hal::sys::read_nmi_iv() {
        Some(NmiSource::MpuSeg3) => NMI_SEG3,
        Some(NmiSource::MpuSegInfo) => NMI_INFO,
        Some(_) => NMI_OTHER,
        None => NMI_NONE,
    };
    hal::mpu::clear_violation_flags();
    critical_section::with(|cs| {
        NMI_LAST.borrow(cs).set(code);
        let n = NMI_COUNT.borrow(cs);
        n.set(n.get().saturating_add(1));
    });
}

fn nmi_snapshot() -> (u8, u8) {
    critical_section::with(|cs| (NMI_COUNT.borrow(cs).get(), NMI_LAST.borrow(cs).get()))
}

/// A segment-3 config: everything open except the HighFram bank, which gets
/// `access`. `nmi` = escalate flag-violations to the SYSNMI vector.
fn seg3_config(access: Access, nmi: bool) -> Config {
    Config {
        border1: BANK_SPLIT,
        border2: BANK_SPLIT,
        seg1: Access::rwx(),
        seg2: Access::rwx(),
        seg3: access,
        info: Access::rwx(),
        nmi,
    }
}

fn save_state(fram: &mut InfoFram, state: u8, flags: u8) {
    let record = [MAGIC[0], MAGIC[1], state, flags];
    fram.write(FRAM_OFFSET, &record).ok();
}

/// Firmware entry point — runs once per life (cold boot or post-PUC).
#[entry]
fn main() -> ! {
    // Stop the watchdog (default ~32 ms fuse) and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Why did this life begin? An MPU-violation PUC reports MpuSeg3; a
    // reflash or reset-button press latches ResetPin (a power cycle
    // Brownout) — those are BOR-class, clear MPULOCK, and force the cold
    // path even if stale FRAM state says otherwise.
    let reasons = ResetReasons::drain(&p.sys);
    let bor_class =
        reasons.contains(ResetReason::ResetPin) || reasons.contains(ResetReason::Brownout);

    let mut fram = InfoFram::new();
    let mut record = [0u8; 4];
    fram.read(FRAM_OFFSET, &mut record).ok();
    let valid = record[0] == MAGIC[0] && record[1] == MAGIC[1];
    let state = if valid && !bor_class { record[2] } else { STATE_COLD };
    let mut flags = if valid && !bor_class { record[3] } else { 0 };

    // MCLK 1 MHz, SMCLK 8 MHz (SMCLK feeds the UART BRCLK below).
    let clocks = hal::clocks::configure(p.cs);

    // Unlock GPIO (clear LOCKLPM5) so the UART pin mux takes effect. (The MPU
    // itself has no pins.)
    p.pmm.pm5ctl0().modify(|_, w| w.locklpm5().clear_bit());

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
    let mut mpu = Mpu::new();
    let mut high = HighFram::new();

    tx.write_all(b"\r\nMSP430FR5969 FRAM MPU fixture (no wiring)\r\n").ok();
    tx.write_all(b"mpu boot state=").ok();
    write_dec(&mut tx, state as u32);
    tx.write_all(b" causes=").ok();
    for (i, r) in reasons.iter().enumerate() {
        if i > 0 {
            tx.write_all(b",").ok();
        }
        tx.write_all(r.as_str().as_bytes()).ok();
    }
    tx.write_all(b"\r\n").ok();

    // Evidence of how the lock defended itself: register writes ignored in
    // this life, or a PUC that landed us back here with state=LOCK_PROBED.
    let mut lock_evidence: &[u8] = b"untested";
    let mut lock_ok = false;
    let mut reset_ok = flags & F_RESET != 0;

    if state == STATE_COLD {
        // ---- Cold phase: the six in-life verdicts --------------------------
        flags = 0;
        // A previous life can't have left MPULOCK set here (any path to a
        // cold boot is BOR-class), but an enabled MPU survives a plain PUC —
        // start from a clean slate.
        let _ = mpu.disable();
        mpu.clear_violations();

        // WRITE PRE: with the MPU off, the HighFram scratch takes a write.
        high.write(HIGH_SCRATCH, &PATTERN_A).ok();
        let mut buf = [0u8; 8];
        high.read(HIGH_SCRATCH, &mut buf).ok();
        if buf == PATTERN_A {
            flags |= F_WRITE_PRE;
        }

        // WRITE BLOCKED: fence the bank (r+x, no write, flag-only, no NMI);
        // the same write must now be suppressed — FRAM keeps PATTERN_A.
        mpu.enable(&seg3_config(Access::rx(), false)).unwrap();
        high.write(HIGH_SCRATCH, &PATTERN_B).ok();
        high.read(HIGH_SCRATCH, &mut buf).ok();
        if buf == PATTERN_A {
            flags |= F_WRITE_BLOCKED;
        }

        // FLAG LATCH: exactly the seg3 flag is up (no NMI fired — SEGIE was
        // off), and clearing takes it down.
        let v = mpu.violations();
        let latched = v.seg3() && !v.seg1() && !v.seg2() && !v.info();
        mpu.clear_violations();
        if latched && !mpu.violations().any() {
            flags |= F_FLAG_LATCH;
        }

        // NMI DEMUX: same fence with escalation on. One violating byte write
        // → exactly one SYSNMI, demuxed to MpuSeg3, flags cleared by the
        // handler; and still exactly one a beat later (no refire storm).
        mpu.enable(&seg3_config(Access::rx(), true)).unwrap();
        high.write(HIGH_SCRATCH, &[0xEE]).ok();
        let (n, last) = nmi_snapshot();
        delay.delay_ms(10);
        let (n2, _) = nmi_snapshot();
        if n == 1 && n2 == 1 && last == NMI_SEG3 && !mpu.violations().any() {
            flags |= F_NMI_DEMUX;
        }

        // INFO BLOCKED: fence Info FRAM itself (the state record is not
        // touched while this is up). One violating byte write → suppressed,
        // one more NMI, demuxed to the info segment.
        let mut info_cfg = seg3_config(Access::rwx(), true);
        info_cfg.info = Access::rx();
        mpu.enable(&info_cfg).unwrap();
        let mut orig = [0u8; 1];
        fram.read(INFO_SCRATCH, &mut orig).ok();
        fram.write(INFO_SCRATCH, &[orig[0] ^ 0xFF]).ok();
        let mut after = [0u8; 1];
        fram.read(INFO_SCRATCH, &mut after).ok();
        let (n3, last3) = nmi_snapshot();
        if after == orig && n3 == 2 && last3 == NMI_INFO {
            flags |= F_INFO_BLOCKED;
        }

        // WRITE POST: disable, and the original blocked write goes through —
        // proof the fence (not a wedged bank) did the blocking.
        mpu.disable().unwrap();
        high.write(HIGH_SCRATCH, &PATTERN_B).ok();
        high.read(HIGH_SCRATCH, &mut buf).ok();
        if buf == PATTERN_B {
            flags |= F_WRITE_POST;
        }

        tx.write_all(b"mpu cold verdict flags=").ok();
        write_hex8(&mut tx, flags);
        tx.write_all(b" arming PUC-on-violation...\r\n").ok();
        tx.flush().ok(); // the PUC preempts mid-character otherwise

        // RESET ON VIOLATION: re-arm with VS=reset and violate. No return
        // expected — the chip reboots into the state==AWAIT_PUC path.
        save_state(&mut fram, STATE_AWAIT_PUC, flags);
        mpu.enable(&seg3_config(Access::rx().reset_on_violation(), false))
            .unwrap();
        high.write(HIGH_SCRATCH, &[0xEE]).ok();

        // Still here: the violation did not reset the chip. Fall through to
        // the report loop with the reset verdict failed.
        delay.delay_ms(10);
        let _ = mpu.disable();
        save_state(&mut fram, STATE_COLD, flags);
        tx.write_all(b"mpu ERROR: survived VS=reset violation\r\n").ok();
    } else if state == STATE_AWAIT_PUC {
        // ---- After the deliberate PUC --------------------------------------
        // The reset cause is the verdict. (The MPU registers survive a PUC;
        // stand enforcement down before touching the bank again.)
        reset_ok = reasons.contains(ResetReason::MpuSeg3);
        if reset_ok {
            flags |= F_RESET;
        }
        let _ = mpu.disable();
        mpu.clear_violations();

        // LOCK: freeze a benign config, then probe it. Whether a locked
        // write is ignored or PUCs is not spelled out in SLAU367 — both
        // defend the registers, so both pass (a PUC lands in the
        // LOCK_PROBED arm below with the flags byte already carrying seven
        // verdicts).
        mpu.enable(&Config::allow_all()).unwrap();
        mpu.lock();
        save_state(&mut fram, STATE_LOCK_PROBED, flags);
        let before = mpu.borders();
        tx.write_all(b"mpu locked, probing register write...\r\n").ok();
        tx.flush().ok();
        // The driver refuses locked writes in software (Err(Locked)) — that
        // alone would leave the hardware unprobed. Go around it: a raw
        // password-bracketed MPUSEGB1 write, the same registers the driver
        // uses. Outcomes: write ignored (expected here), or a PUC (caught by
        // the LOCK_PROBED arm above — if the lock freezes even the password
        // lane, the border write lands "while closed", which PUCs). Either
        // way the registers defended themselves; only a changed border fails.
        let probe = mpu.enable(&seg3_config(Access::rx(), false));
        unsafe {
            (0x05A1 as *mut u8).write_volatile(0xA5); // MPUCTL0_H: open
            (0x05A6 as *mut u16).write_volatile(0x0480); // MPUSEGB1: try 0x4800
            (0x05A1 as *mut u8).write_volatile(0x00); // MPUCTL0_H: close
        }
        lock_ok = mpu.is_locked() && probe.is_err() && mpu.borders() == before;
        lock_evidence = b"ignored";
    } else {
        // ---- STATE_LOCK_PROBED: the lock probe PUC'd ------------------------
        // Locked-write-causes-PUC semantics: the registers defended
        // themselves with a reset, and MPULOCK must have survived it.
        lock_ok = mpu.is_locked();
        lock_evidence = b"puc";
    }

    // ---- Steady state: the framed burst, once per second, forever ----------
    let all_ok = flags & ALL_COLD == ALL_COLD && reset_ok && lock_ok;
    loop {
        let borders = mpu.borders();
        tx.write_all(b"mpu state=").ok();
        write_dec(&mut tx, state as u32);
        tx.write_all(b" flags=").ok();
        write_hex8(&mut tx, flags);
        tx.write_all(b" borders=").ok();
        write_hex20(&mut tx, borders.0);
        tx.write_all(b"/").ok();
        write_hex20(&mut tx, borders.1);
        tx.write_all(b" lock=").ok();
        tx.write_all(lock_evidence).ok();
        tx.write_all(b"\r\n").ok();

        tx.write_all(b"MPU_TEST_BEGIN\r\n").ok();
        verdict(&mut tx, b"MPU WRITE PRE", flags & F_WRITE_PRE != 0);
        verdict(&mut tx, b"MPU WRITE BLOCKED", flags & F_WRITE_BLOCKED != 0);
        verdict(&mut tx, b"MPU FLAG LATCH", flags & F_FLAG_LATCH != 0);
        verdict(&mut tx, b"MPU NMI DEMUX", flags & F_NMI_DEMUX != 0);
        verdict(&mut tx, b"MPU INFO BLOCKED", flags & F_INFO_BLOCKED != 0);
        verdict(&mut tx, b"MPU WRITE POST", flags & F_WRITE_POST != 0);
        verdict(&mut tx, b"MPU RESET ON VIOLATION", reset_ok);
        verdict(&mut tx, b"MPU LOCK", lock_ok);
        tx.write_all(b"MPU_TEST_END\r\n").ok();

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

/// Write `name` + ` OK`/` FAIL` + CRLF.
fn verdict<W: hal::embedded_io::Write>(tx: &mut W, name: &[u8], ok: bool) {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" as &[u8] } else { b" FAIL\r\n" })
        .ok();
}

/// Write a byte as two uppercase hex digits. `core::fmt` is deliberately
/// avoided project-wide (FRAM budget).
fn write_hex8<W: hal::embedded_io::Write>(tx: &mut W, b: u8) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    tx.write_all(&[HEX[(b >> 4) as usize], HEX[(b & 0xF) as usize]])
        .ok();
}

/// Write a 20-bit address as five uppercase hex digits.
fn write_hex20<W: hal::embedded_io::Write>(tx: &mut W, v: u32) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    tx.write_all(&[HEX[((v >> 16) & 0xF) as usize]]).ok();
    write_hex8(tx, (v >> 8) as u8);
    write_hex8(tx, v as u8);
}

/// Write an unsigned integer in decimal.
fn write_dec<W: hal::embedded_io::Write>(tx: &mut W, mut v: u32) {
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    tx.write_all(&buf[i..]).ok();
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
