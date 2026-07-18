#![no_std]
#![no_main]
// msp430-rt's #[interrupt] emits `extern "msp430-interrupt"` handlers; the
// `wake_cpu` variant additionally patches the stacked SR with inline asm.
#![feature(abi_msp430_interrupt)]
#![feature(asm_experimental_arch)]

//! Two-board integration fixture: ONE binary, flashed to BOTH LaunchPads of
//! the permanently-wired two-board rig, driven by the host-side
//! `two_board_integration_tests` runner over each board's own eUSCI_A0 USB
//! backchannel (9600 8N1).
//!
//! # Identity
//!
//! Which physical board is "parent" and which is "child" is stored in **Info
//! FRAM at offset 0xA0** (`[b'2', b'B', b'P'|b'C', 0]`; the offset map so far:
//! 0x60 lpmx5, 0x70 mpu, 0x80 clock_speed, 0x90 vlo_soak). Info FRAM is not
//! part of the flashed image, so the role survives reflashes and USB replugs —
//! the host discovers which serial port is which board by *asking*, never by
//! device path. Roles are written once with the runner's `provision`
//! subcommand (`P`/`C` commands below).
//!
//! The role is an *identity*, not a personality: every board answers every
//! command, and the host decides who does what. The wiring makes that safe —
//! see "pin direction discipline" below.
//!
//! # Command protocol (host → board, single bytes; CR/LF ignored)
//!
//! | cmd | action | response |
//! |-----|--------|----------|
//! | `i` | identify | `2B_ID role=… fw=…` |
//! | `P`/`C` | provision role parent/child into Info FRAM | `2B_PROVISIONED role=…` |
//! | `s` | eUSCI_B0 **I2C slave** register file at 0x48 until `q` | `2B_SLAVE_ON …`, then `2B_SLAVE_STATS …` |
//! | `m` | eUSCI_B0 **I2C master** test against the peer's 0x48 | `2B_I2C_TEST_BEGIN` … verdicts … `END` |
//! | `e` | eUSCI_A1 UART **echo+1 server** until `q` | `2B_UARTECHO_ON`, then `2B_UARTECHO_STATS …` |
//! | `t` | eUSCI_A1 UART **initiator**: pattern out, expect echo+1 | `2B_UART_TEST_BEGIN` … verdicts … `END` |
//! | `g` | arm P3.5 rising-edge interrupt counter until `q` | `2B_GPIO_ARMED`, then `2B_GPIO_STATS edges=… badiv=…` |
//! | `p` | 10 pulses out of P3.4 (1 ms high / 2 ms low) | `2B_PULSED n=10` |
//! | `1` | one pulse out of P3.4 | `2B_PULSED n=1` |
//! | `w` | arm P3.5 rising edge, **enter LPM4** until the peer pulses | `2B_SLEEPING`, then `2B_WOKE edges=…` |
//! | `f`/`F` | 1 kHz PWM on P1.4 (TB0.1) at 25 % / 75 % | `2B_PWM freq=… duty=…` |
//! | `d`/`D` | 1 kHz PWM on P1.5 (TB0.2, the W10/W11 line) at 30 % / 60 % — the RC-DAC stimulus for the name-only `adc_dac` suite, meaningful once the optional 10 µF caps are fitted | `2B_DAC duty=…` |
//! | `x` | park both PWM outputs low | `2B_PWM_OFF` |
//! | `c` | measure P1.2 (TA1.CCI1A): frequency + duty | `2B_CAP freq=… duty=…` |
//! | `a` | ADC: A7 (P2.4) millivolts + own AVCC millivolts | `2B_ADC a7_mv=… avcc_mv=…` |
//!
//! Until the first command arrives the board emits `2B_READY role=… fw=…`
//! once per second (the eZ-FET gates TX on host DTR, so earlier output is
//! lost — the announce-until-poked convention).
//!
//! # Pin direction discipline (why no command sequence can cause contention)
//!
//! Every cross-board wire has exactly ONE driving pin and one receiving pin,
//! and the assignment is identical on both boards because the wiring is
//! *crossed* (out→in in each direction):
//!
//! - P3.4 = always output (pulse out), P3.5 = always input (pull-down) —
//!   wired parent-P3.4→child-P3.5 and child-P3.4→parent-P3.5.
//! - P1.4/P1.5 = always TB0 PWM outputs, P1.2 = always TA1 capture input,
//!   P2.4 = always ADC analog input — same crossing.
//! - P2.5/P2.6 = eUSCI_A1 TXD/RXD, direction fixed by the peripheral, wired
//!   TXD→RXD both ways.
//! - P1.6/P1.7 = I2C SDA/SCL: open-drain by protocol on both sides (a shared
//!   bus, never push-pull), with external pull-ups.
//!
//! So even if both boards were told to run the same command at the same time,
//! no wire ever has two push-pull drivers. The series resistors specified in
//! the wiring table are a second, electrical layer of the same guarantee
//! (they bound the current of any miswiring or fault to a safe level).

use core::cell::Cell;

use critical_section::Mutex;
use hal::adc::{Adc, Config as AdcConfig};
use hal::capture::{CaptureTimer, Edge as CapEdge};
use hal::delay::Delay;
use hal::embedded_hal::delay::DelayNs as _;
use hal::embedded_hal::digital::{OutputPin, StatefulOutputPin};
use hal::embedded_hal::i2c::I2c as _;
use hal::embedded_hal::pwm::SetDutyCycle as _;
use hal::embedded_hal_nb::serial::Read as _;
use hal::embedded_io::Write as _;
use hal::embedded_storage::{ReadStorage, Storage};
use hal::fram::InfoFram;
use hal::gpio::{Edge as PinEdge, GpioExt};
use hal::i2c::{
    Config as I2cConfig, I2c, I2cExt, I2cSlave, I2cSlaveExt, SlaveConfig, SlaveEvent,
};
use hal::interrupt;
use hal::pwm::Pwm;
use hal::ref_a::{Ref, ReferenceVoltage};
use hal::serial::{Config as UartConfig, Rx, SerialExt, Tx, UsciA0, UsciA1};
use hal::timer::Divider;
use msp430_rt::entry;

// Force-link the msp430 crate so its critical-section impl symbols resolve
// for pac's Peripherals::take().
use msp430 as _;

/// Protocol/firmware revision, reported in `2B_READY`/`2B_ID`. Bump when the
/// command protocol changes so a stale flash on one board is detectable.
const FW: u32 = 1;

/// Info-FRAM role record: offset (clear of 0x60 lpmx5, 0x70 mpu,
/// 0x80 clock_speed, 0x90 vlo_soak) and magic.
const FRAM_OFFSET: u32 = 0xA0;
const MAGIC: [u8; 2] = [b'2', b'B'];

/// I2C slave register file: address, size, fixed read-only ID in register 0.
/// Same shape as the single-board `i2c_slave_test_runner` fixture.
const I2C_ADDR: u8 = 0x48;
const I2C_REGS: usize = 16;
const I2C_ID_BYTE: u8 = 0x69;
/// An address nothing on the bus answers — the master's NACK check.
const I2C_EMPTY_ADDR: u8 = 0x22;

/// Cross-link UART pattern (eUSCI_A1). The echoing side returns each byte
/// +1, so a wire short/loopback (which would return the byte unchanged)
/// cannot pass. 24 bytes, like the single-board serial_irq suite.
const UART_PATTERN: &[u8] = b"2B-UART-XLINK-0123456789";

/// Both cross-board PWM channels run at this frequency (TB0.1 and TB0.2
/// share TB0CCR0, so they necessarily share a period).
const PWM_HZ: u32 = 1_000;

/// P3.5 rising/falling edges land in PxIV slot 2·(5+1) = 0x0C.
const P35_IV: u16 = 0x0C;

/// Edge tally from the PORT3 ISR (P3.5, the cross-board edge input).
static EDGES: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
/// PORT3 IV reads that were NOT the P3.5 slot — any nonzero value here means
/// a spurious source fired, which the GPIO suite treats as a failure.
static BAD_IV: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// PORT3 ISR: served for every armed P3.x edge. Reading PxIV atomically
/// clears the served flag; `wake_cpu` lets the same handler end an LPM4 park
/// (the `w` command) as well as count edges in active mode (the `g` command).
#[msp430_rt::interrupt(wake_cpu)]
fn PORT3() {
    let iv = hal::gpio::read_iv::<hal::gpio::P3>();
    critical_section::with(|cs| {
        if iv == P35_IV {
            let e = EDGES.borrow(cs);
            e.set(e.get().wrapping_add(1));
        } else {
            let b = BAD_IV.borrow(cs);
            b.set(b.get().saturating_add(1));
        }
    });
}

/// The board's provisioned identity.
#[derive(Clone, Copy, PartialEq)]
enum Role {
    Parent,
    Child,
    Unset,
}

impl Role {
    fn as_str(self) -> &'static [u8] {
        match self {
            Role::Parent => b"parent",
            Role::Child => b"child",
            Role::Unset => b"unset",
        }
    }
}

/// eUSCI_B0 is one register block that can be the I2C master OR the I2C
/// slave. The slave can hand the block back (`free`) and become the master
/// later; the master driver has no `free`, so master→slave is refused (the
/// host never asks for it — each board plays one bus role per flash session).
enum B0 {
    Free(hal::pac::UsciB0I2cMode),
    Master(I2c),
    Slave(I2cSlave),
    /// Transient state during `core::mem::replace` handoffs only.
    Gone,
}

#[entry]
fn main() -> ! {
    // Stop the watchdog and take the peripherals, in that order.
    let p = hal::init(hal::watchdog::WdtMode::Hold).unwrap();

    // Performance profile: MCLK = 1 MHz, SMCLK = 8 MHz (UART BRCLK, PWM and
    // capture timebase).
    let clocks = hal::clocks::configure(p.cs);

    hal::gpio::unlock_pins(&p.pmm);

    // Backchannel to the host: eUSCI_A0, 9600 8N1.
    let serial = p
        .usci_a0_uart_mode
        .into_uart(UartConfig::new(clocks.smclk()).baud(9600));
    let (mut tx, mut rx) = serial.split();

    let (port1, port2) = p.port_1_2.split();
    let (port3, port4) = p.port_3_4.split();

    let mut green = port1.pin0.into_output(); // LED2: heartbeat / activity
    let mut red = port4.pin6.into_output(); // LED1: latched on any local FAIL
    green.set_low().ok();
    red.set_low().ok();

    // Cross-board pins, directions fixed for the life of the firmware (see
    // the module docs): P3.4 always drives, P3.5 always listens. The
    // receiving side is pulled DOWN so the line has a defined idle level
    // even while the peer's driver is still high-impedance (unpowered peer,
    // peer between commands at boot, …); a pulse is then a clean rising edge.
    let mut pulse_out = port3.pin4.into_output();
    pulse_out.set_low().ok();
    let mut edge_in = port3.pin5.into_pull_down_input();

    // PWM out: TB0.1 = P1.4 (capture stimulus wire), TB0.2 = P1.5 (RC-DAC
    // wire). Both parked at 0 % (OUTMOD=0, pin low — clean rail) until told.
    let pwm = Pwm::new_smclk(p.timer_0_b7, &clocks, PWM_HZ);
    let mut pwm_cap = pwm.channel(port1.pin4.into_timer_b_output());
    let mut pwm_dac = pwm.channel(port1.pin5.into_timer_b_output());

    // Capture in: TA1.CCI1A = P1.2, timestamped at SMCLK/1 = 8 MHz ticks
    // (a 1 kHz period = 8000 ticks; duty resolution ~0.1 ‰).
    let cap_timer = CaptureTimer::new_smclk(p.timer_1_a3, &clocks, Divider::Div1);
    let mut cap_ch = cap_timer.capture_pin(port1.pin2.into_timer_a_capture(), CapEdge::Rising);

    // ADC in: A7 = P2.4 (the peer's RC-filtered PWM-DAC), read ratiometric
    // against AVCC; AVCC itself is read absolutely via the 2.0 V reference
    // so the host can close the cross-board analog loop in millivolts.
    let mut adc = Adc::new(p.adc12, AdcConfig::new());
    let mut a7 = port2.pin4.into_analog();
    let vref = Ref::new(p.shared_reference, ReferenceVoltage::V2_0);

    let mut delay = Delay::new(clocks.mclk());
    let mut fram = InfoFram::new();
    let mut role = load_role(&mut fram);

    // eUSCI_B0 (I2C) and eUSCI_A1 (cross UART) are built lazily on first
    // use, so an idle board leaves those pins high-impedance / bus-released.
    let mut b0 = B0::Free(p.usci_b0_i2c_mode);
    let mut a1_src = Some(p.usci_a1_uart_mode);
    let mut a1: Option<(Tx<UsciA1>, Rx<UsciA1>)> = None;

    // GIE up front: the PORT3 counter (command `g`) counts in active mode.
    // SAFETY: all ISR-shared state is in critical-section Mutexes.
    unsafe {
        msp430::interrupt::enable();
    }

    tx.write_all(b"\r\nMSP430FR5969 two-board fixture\r\n").ok();

    // Announce until the first command, then serve commands forever.
    let mut poked = false;
    let mut idle_ticks: u16 = 0;
    loop {
        let Some(cmd) = poll_cmd(&mut rx) else {
            delay.delay_ms(10);
            idle_ticks += 1;
            if !poked && idle_ticks >= 100 {
                idle_ticks = 0;
                green.toggle().ok();
                tx.write_all(b"2B_READY role=").ok();
                tx.write_all(role.as_str()).ok();
                tx.write_all(b" fw=").ok();
                write_dec(&mut tx, FW);
                tx.write_all(b"\r\n").ok();
            }
            continue;
        };
        poked = true;
        green.toggle().ok();

        match cmd {
            // ---- identity / provisioning -------------------------------
            b'i' => {
                tx.write_all(b"2B_ID role=").ok();
                tx.write_all(role.as_str()).ok();
                tx.write_all(b" fw=").ok();
                write_dec(&mut tx, FW);
                tx.write_all(b"\r\n").ok();
            }
            b'P' | b'C' => {
                role = if cmd == b'P' { Role::Parent } else { Role::Child };
                save_role(&mut fram, role);
                // Read back through the same path so the ack proves the
                // record actually landed in FRAM.
                role = load_role(&mut fram);
                tx.write_all(b"2B_PROVISIONED role=").ok();
                tx.write_all(role.as_str()).ok();
                tx.write_all(b"\r\n").ok();
            }

            // ---- I2C: slave register file ------------------------------
            b's' => {
                b0 = match core::mem::replace(&mut b0, B0::Gone) {
                    B0::Free(block) => match block.into_i2c_slave(SlaveConfig::new(I2C_ADDR)) {
                        Ok(slave) => B0::Slave(slave),
                        // Unreachable for 0x48 (validated range), but never
                        // panic-loop silently in a fixture.
                        Err(_) => {
                            tx.write_all(b"2B_ERR slave=addr\r\n").ok();
                            red.set_high().ok();
                            continue;
                        }
                    },
                    B0::Slave(slave) => B0::Slave(slave),
                    B0::Master(_) | B0::Gone => {
                        tx.write_all(b"2B_ERR b0=master\r\n").ok();
                        red.set_high().ok();
                        continue;
                    }
                };
                let B0::Slave(ref mut slave) = b0 else {
                    unreachable!()
                };
                run_i2c_slave(slave, &mut tx, &mut rx, &mut green);
            }

            // ---- I2C: master test burst --------------------------------
            b'm' => {
                b0 = match core::mem::replace(&mut b0, B0::Gone) {
                    B0::Free(block) => B0::Master(
                        block.into_i2c(I2cConfig::new(clocks.smclk()).scl_freq(50_000)),
                    ),
                    B0::Slave(slave) => B0::Master(
                        slave
                            .free()
                            .into_i2c(I2cConfig::new(clocks.smclk()).scl_freq(50_000)),
                    ),
                    B0::Master(master) => B0::Master(master),
                    B0::Gone => unreachable!(),
                };
                let B0::Master(ref mut i2c) = b0 else {
                    unreachable!()
                };
                run_i2c_master_test(i2c, &mut tx, &mut red);
            }

            // ---- cross UART (eUSCI_A1) ---------------------------------
            b'e' | b't' => {
                if a1.is_none() {
                    if let Some(src) = a1_src.take() {
                        let s = src.into_uart(UartConfig::new(clocks.smclk()).baud(9600));
                        a1 = Some(s.split());
                    }
                }
                let Some((ref mut tx1, ref mut rx1)) = a1 else {
                    tx.write_all(b"2B_ERR a1=gone\r\n").ok();
                    continue;
                };
                if cmd == b'e' {
                    run_uart_echo(tx1, rx1, &mut tx, &mut rx, &mut green);
                } else {
                    run_uart_initiator(tx1, rx1, &mut tx, &mut delay, &mut red);
                }
            }

            // ---- GPIO edges / LPM4 wake --------------------------------
            b'g' => {
                critical_section::with(|cs| {
                    EDGES.borrow(cs).set(0);
                    BAD_IV.borrow(cs).set(0);
                });
                edge_in.enable_interrupt(PinEdge::Rising);
                tx.write_all(b"2B_GPIO_ARMED\r\n").ok();
                // Count until 'q'; edges land in the ISR meanwhile.
                loop {
                    if poll_cmd(&mut rx) == Some(b'q') {
                        break;
                    }
                    delay.delay_ms(5);
                }
                edge_in.disable_interrupt();
                let (edges, bad) =
                    critical_section::with(|cs| (EDGES.borrow(cs).get(), BAD_IV.borrow(cs).get()));
                tx.write_all(b"2B_GPIO_STATS edges=").ok();
                write_dec(&mut tx, edges as u32);
                tx.write_all(b" badiv=").ok();
                write_dec(&mut tx, bad as u32);
                tx.write_all(b"\r\n").ok();
            }
            b'p' | b'1' => {
                let n: u16 = if cmd == b'p' { 10 } else { 1 };
                for _ in 0..n {
                    pulse_out.set_high().ok();
                    delay.delay_ms(1);
                    pulse_out.set_low().ok();
                    delay.delay_ms(2);
                }
                tx.write_all(b"2B_PULSED n=").ok();
                write_dec(&mut tx, n as u32);
                tx.write_all(b"\r\n").ok();
            }
            b'w' => {
                critical_section::with(|cs| {
                    EDGES.borrow(cs).set(0);
                    BAD_IV.borrow(cs).set(0);
                });
                tx.write_all(b"2B_SLEEPING\r\n").ok();
                tx.flush().ok();
                // flush() waits for TXBUF, not the shift register — give the
                // final character time to leave before every clock stops.
                delay.delay_ms(5);
                edge_in.enable_interrupt(PinEdge::Rising);
                // Everything stops here (LPM4: no MCLK/SMCLK/ACLK). Only the
                // peer's rising edge on P3.5 — an asynchronous port wake —
                // can resume us, which is exactly what this proves.
                hal::power::enter_lpm4();
                edge_in.disable_interrupt();
                let edges = critical_section::with(|cs| EDGES.borrow(cs).get());
                tx.write_all(b"2B_WOKE edges=").ok();
                write_dec(&mut tx, edges as u32);
                tx.write_all(b"\r\n").ok();
            }

            // ---- PWM out / capture in / ADC ----------------------------
            b'f' | b'F' => {
                let percent: u8 = if cmd == b'f' { 25 } else { 75 };
                pwm_cap.set_duty_cycle_percent(percent).ok();
                tx.write_all(b"2B_PWM freq=").ok();
                write_dec(&mut tx, pwm.frequency());
                tx.write_all(b" duty=").ok();
                write_dec(&mut tx, percent as u32 * 10);
                tx.write_all(b"\r\n").ok();
            }
            b'd' | b'D' => {
                let percent: u8 = if cmd == b'd' { 30 } else { 60 };
                pwm_dac.set_duty_cycle_percent(percent).ok();
                tx.write_all(b"2B_DAC duty=").ok();
                write_dec(&mut tx, percent as u32 * 10);
                tx.write_all(b"\r\n").ok();
            }
            b'x' => {
                pwm_cap.set_duty_cycle_fully_off().ok();
                pwm_dac.set_duty_cycle_fully_off().ok();
                tx.write_all(b"2B_PWM_OFF\r\n").ok();
            }
            b'c' => {
                // Frequency over 8 periods, then duty (needs both edges).
                // Per-edge budget 20000 ticks = 2.5 ms at 8 MHz — generous
                // for the 1 kHz stimulus, still promptly reports a dead wire.
                let freq = cap_ch.frequency_hz(8, 20_000);
                cap_ch.set_edge(CapEdge::Both);
                let duty = cap_ch.measure_duty_permille(20_000);
                cap_ch.set_edge(CapEdge::Rising);
                match (freq, duty) {
                    (Ok(f), Ok(d)) => {
                        tx.write_all(b"2B_CAP freq=").ok();
                        write_dec(&mut tx, f);
                        tx.write_all(b" duty=").ok();
                        write_dec(&mut tx, d as u32);
                        tx.write_all(b"\r\n").ok();
                    }
                    (freq, duty) => {
                        // Which measurement failed, and how (the capture
                        // Error variants, or the out-of-range value).
                        let code = |r: &Result<u32, hal::capture::Error>| -> &'static [u8] {
                            match r {
                                Ok(_) => b"ok",
                                Err(hal::capture::Error::Timeout) => b"tmo",
                                Err(hal::capture::Error::Overcapture) => b"ovc",
                                Err(hal::capture::Error::LevelSync) => b"lvl",
                            }
                        };
                        tx.write_all(b"X_CAP_DETAIL f=").ok();
                        tx.write_all(code(&freq)).ok();
                        if let Ok(f) = freq {
                            tx.write_all(b":").ok();
                            write_dec(&mut tx, f);
                        }
                        tx.write_all(b" d=").ok();
                        tx.write_all(code(&duty.map(|d| d as u32))).ok();
                        if let Ok(d) = duty {
                            tx.write_all(b":").ok();
                            write_dec(&mut tx, d as u32);
                        }
                        tx.write_all(b"\r\n").ok();
                        // No capture — trace the pad→capture route. P1IN
                        // reads the pin regardless of the mux; TA1CCTL1's CCI
                        // bit (0x08) is the live input level as the capture
                        // unit sees it. Pad toggling with CCI dead = the mux
                        // isn't routing; both toggling = the capture latch
                        // itself. ~30k polls cover several 1 kHz periods.
                        let mut pad_tog: u16 = 0;
                        let mut cci_tog: u16 = 0;
                        let mut pad_last = unsafe { core::ptr::read_volatile(0x0200 as *const u8) }
                            & (1 << 2);
                        let mut cci_last = unsafe { core::ptr::read_volatile(0x0384 as *const u16) }
                            & 0x0008;
                        for _ in 0..30_000u16 {
                            let pad = unsafe { core::ptr::read_volatile(0x0200 as *const u8) }
                                & (1 << 2);
                            if pad != pad_last {
                                pad_tog = pad_tog.saturating_add(1);
                                pad_last = pad;
                            }
                            let cci = unsafe { core::ptr::read_volatile(0x0384 as *const u16) }
                                & 0x0008;
                            if cci != cci_last {
                                cci_tog = cci_tog.saturating_add(1);
                                cci_last = cci;
                            }
                        }
                        // Both-armed edge stream, buffered (NO UART between
                        // waits — this mirrors measure_duty_permille's
                        // wait→level cadence exactly). Timestamp deltas
                        // should alternate ~2000/6000 ticks (250/750 µs at
                        // 8 MHz); a near-zero delta = phantom double capture.
                        cap_ch.set_edge(CapEdge::Both);
                        let mut trace: [(u16, bool, u8); 6] = [(0, false, 0); 6];
                        for slot in trace.iter_mut() {
                            *slot = match cap_ch.wait_edge(20_000) {
                                Ok(t) => (t, cap_ch.input_level(), 0),
                                Err(hal::capture::Error::Overcapture) => (0, false, 1),
                                Err(_) => (0, false, 2),
                            };
                        }
                        cap_ch.set_edge(CapEdge::Rising);
                        tx.write_all(b"X_CAP_EDGES").ok();
                        for (t, lvl, err) in trace {
                            tx.write_all(b" ").ok();
                            match err {
                                0 => {
                                    write_hex16(&mut tx, t);
                                    tx.write_all(if lvl { b":h" } else { b":l" }).ok();
                                }
                                1 => {
                                    tx.write_all(b"ovc").ok();
                                }
                                _ => {
                                    tx.write_all(b"err").ok();
                                }
                            }
                        }
                        tx.write_all(b"\r\n").ok();
                        let cctl1 = unsafe { core::ptr::read_volatile(0x0384 as *const u16) };
                        let sel0 = unsafe { core::ptr::read_volatile(0x020A as *const u8) };
                        let sel1 = unsafe { core::ptr::read_volatile(0x020C as *const u8) };
                        let dir = unsafe { core::ptr::read_volatile(0x0204 as *const u8) };
                        tx.write_all(b"2B_CAP err=nosignal tog=").ok();
                        write_dec(&mut tx, pad_tog as u32);
                        tx.write_all(b" ccitog=").ok();
                        write_dec(&mut tx, cci_tog as u32);
                        tx.write_all(b" cctl1=").ok();
                        write_hex16(&mut tx, cctl1);
                        tx.write_all(b" sel0=").ok();
                        write_hex16(&mut tx, sel0 as u16);
                        tx.write_all(b" sel1=").ok();
                        write_hex16(&mut tx, sel1 as u16);
                        tx.write_all(b" dir=").ok();
                        write_hex16(&mut tx, dir as u16);
                        tx.write_all(b"\r\n").ok();
                    }
                }
            }
            b'a' => {
                let avcc = adc.read_supply_millivolts(&vref);
                let counts = adc.read(&mut a7) as u32;
                let a7_mv = counts * avcc / 4095;
                tx.write_all(b"2B_ADC a7_mv=").ok();
                write_dec(&mut tx, a7_mv);
                tx.write_all(b" avcc_mv=").ok();
                write_dec(&mut tx, avcc);
                tx.write_all(b"\r\n").ok();
            }

            // ---- B0 register dump (bus post-mortem) --------------------
            // Raw volatile reads, deliberately outside the HAL: works no
            // matter which mode (master/slave/unconfigured) B0 is in, and
            // perturbs nothing. STATW's UCBBUSY/UCSCLLOW say whether the
            // bus is mid-transaction and who is holding SCL.
            b'B' => {
                let ctlw0 = unsafe { core::ptr::read_volatile(0x0640 as *const u16) };
                let statw = unsafe { core::ptr::read_volatile(0x0648 as *const u16) };
                let ifg = unsafe { core::ptr::read_volatile(0x066C as *const u16) };
                // P1IN reads the pad level even with the pin muxed to the
                // eUSCI — the live electrical state of SDA (P1.6) and SCL
                // (P1.7).
                let p1in = unsafe { core::ptr::read_volatile(0x0200 as *const u8) };
                tx.write_all(b"2B_B0 ctlw0=").ok();
                write_hex16(&mut tx, ctlw0);
                tx.write_all(b" statw=").ok();
                write_hex16(&mut tx, statw);
                tx.write_all(b" ifg=").ok();
                write_hex16(&mut tx, ifg);
                tx.write_all(b" sda=").ok();
                tx.write_all(if p1in & (1 << 6) != 0 { b"1" } else { b"0" })
                    .ok();
                tx.write_all(b" scl=").ok();
                tx.write_all(if p1in & (1 << 7) != 0 { b"1" } else { b"0" })
                    .ok();
                tx.write_all(b"\r\n").ok();
            }

            // 'q' outside a sub-mode is harmless; anything else is noise —
            // answer so host-side protocol bugs are visible, not silent.
            b'q' => {}
            _ => {
                tx.write_all(b"2B_UNK\r\n").ok();
            }
        }
    }
}

/// Non-blocking poll of the backchannel for one command byte. CR/LF (line
/// endings from a human in `screen`) are swallowed; receive errors read as
/// "no command".
fn poll_cmd(rx: &mut Rx<UsciA0>) -> Option<u8> {
    match rx.read() {
        Ok(b'\r') | Ok(b'\n') => None,
        Ok(byte) => Some(byte),
        Err(_) => None,
    }
}

/// `s`: serve the 16-register I2C slave file at 0x48 until `q` arrives on
/// the backchannel. Register 0 is the read-only ID byte; the register
/// pointer autoincrements through both phases. The eUSCI stretches SCL until
/// we service RXBUF/TXBUF, so this loop is correct at any polling latency —
/// including the backchannel poll interleaved into it.
fn run_i2c_slave(
    slave: &mut I2cSlave,
    tx: &mut Tx<UsciA0>,
    rx: &mut Rx<UsciA0>,
    green: &mut impl StatefulOutputPin,
) {
    #[derive(Clone, Copy)]
    enum Phase {
        Idle,
        WritePtr,
        WriteData,
        Read,
    }

    let mut file = [0u8; I2C_REGS];
    file[0] = I2C_ID_BYTE;
    let mut ptr: usize = 0;
    let mut phase = Phase::Idle;
    let mut transactions: u16 = 0;
    let mut writes: u16 = 0;
    let mut reads: u16 = 0;
    // Raw event tallies: which bus events actually reached this loop. On a
    // wedged bus these localize the stall (e.g. starts seen but no RX byte
    // vs a read-turnaround with no TX request).
    let mut ev_start_wr: u16 = 0;
    let mut ev_start_rd: u16 = 0;
    let mut ev_rx: u16 = 0;
    let mut ev_txreq: u16 = 0;

    tx.write_all(b"2B_SLAVE_ON addr=0x48\r\n").ok();

    loop {
        if poll_cmd(rx) == Some(b'q') {
            break;
        }
        let Some(event) = slave.poll() else { continue };
        match event {
            SlaveEvent::Start { read } => {
                if read {
                    ev_start_rd += 1;
                } else {
                    ev_start_wr += 1;
                }
                phase = if read { Phase::Read } else { Phase::WritePtr };
            }
            SlaveEvent::Received(byte) => {
                ev_rx += 1;
                match phase {
                    Phase::WritePtr => {
                        ptr = (byte as usize) % I2C_REGS;
                        phase = Phase::WriteData;
                    }
                    Phase::WriteData => {
                        // Register 0 is read-only: the master's overwrite
                        // attempt must bounce off (the ROREG verdict on the
                        // master side).
                        if ptr != 0 {
                            file[ptr] = byte;
                        }
                        ptr = (ptr + 1) % I2C_REGS;
                    }
                    _ => {}
                }
            }
            SlaveEvent::TxRequest => {
                ev_txreq += 1;
                slave.write_byte(file[ptr]);
                ptr = (ptr + 1) % I2C_REGS;
            }
            SlaveEvent::Stop => {
                match phase {
                    Phase::Read => reads += 1,
                    Phase::WritePtr | Phase::WriteData => writes += 1,
                    Phase::Idle => {}
                }
                phase = Phase::Idle;
                transactions += 1;
                green.toggle().ok();
            }
        }
    }

    tx.write_all(b"2B_SLAVE_STATS trans=").ok();
    write_dec(tx, transactions as u32);
    tx.write_all(b" wr=").ok();
    write_dec(tx, writes as u32);
    tx.write_all(b" rd=").ok();
    write_dec(tx, reads as u32);
    tx.write_all(b" stw=").ok();
    write_dec(tx, ev_start_wr as u32);
    tx.write_all(b" str=").ok();
    write_dec(tx, ev_start_rd as u32);
    tx.write_all(b" rxb=").ok();
    write_dec(tx, ev_rx as u32);
    tx.write_all(b" txr=").ok();
    write_dec(tx, ev_txreq as u32);
    tx.write_all(b"\r\n").ok();
}

/// `m`: drive the peer's register file as the bus master and emit a framed
/// verdict burst. The expectations are protocol-fixed (the peer is our own
/// fixture in `s` mode), so the verdicts are computed on-board like the
/// single-board suites; the host just asserts the `OK` lines.
/// One `X_I2C_PRE` bus-state line: the master's view of STATW/CTLW0 and the
/// live pad levels, emitted between gauntlet steps to localize a wedge.
fn bus_state_line(tx: &mut Tx<UsciA0>) {
    let ctlw0 = unsafe { core::ptr::read_volatile(0x0640 as *const u16) };
    let statw = unsafe { core::ptr::read_volatile(0x0648 as *const u16) };
    let p1in = unsafe { core::ptr::read_volatile(0x0200 as *const u8) };
    tx.write_all(b"X_I2C_PRE statw=").ok();
    write_hex16(tx, statw);
    tx.write_all(b" ctlw0=").ok();
    write_hex16(tx, ctlw0);
    tx.write_all(b" sda=").ok();
    tx.write_all(if p1in & (1 << 6) != 0 { b"1" } else { b"0" }).ok();
    tx.write_all(b" scl=").ok();
    tx.write_all(if p1in & (1 << 7) != 0 { b"1" } else { b"0" }).ok();
    tx.write_all(b"\r\n").ok();
}

fn run_i2c_master_test(i2c: &mut I2c, tx: &mut Tx<UsciA0>, red: &mut impl OutputPin) {
    tx.write_all(b"2B_I2C_TEST_BEGIN\r\n").ok();
    let mut all_ok = true;

    // 1. The peer ACKs its address — the two-board bus is alive.
    bus_state_line(tx);
    let probe_ok = i2c.probe(I2C_ADDR);
    all_ok &= verdict(tx, b"X_I2C PROBE", probe_ok);

    // 2. An empty address NACKs: the ACK above is the peer, not a stuck-low
    //    SDA or a shorted bus (which would "ACK" everything).
    bus_state_line(tx);
    let nodev_ok = !i2c.probe(I2C_EMPTY_ADDR);
    all_ok &= verdict(tx, b"X_I2C NODEV", nodev_ok);

    // 3. write_read of register 0 returns the fixed ID: the full
    //    write → repeated-START → read turnaround against real silicon.
    bus_state_line(tx);
    let mut id = [0u8; 1];
    let id_res = i2c.write_read(I2C_ADDR, &[0x00], &mut id);
    let id_ok = id_res.is_ok() && id[0] == I2C_ID_BYTE;
    all_ok &= verdict(tx, b"X_I2C ID", id_ok);
    if !id_ok {
        i2c_detail(tx, id_res, &id);
    }

    // 4. Write three bytes at register 2, read them back: RX-phase pointer
    //    autoincrement, then TX-phase autoincrement, in one verdict. This
    //    read following the ID read is also the stale-speculative-byte
    //    check: the eUSCI requests TX bytes one ahead, so the ID read
    //    parked a byte in the peer's TXBUF at STOP — had the peer's driver
    //    not flushed it, it would lead (and corrupt) this read.
    bus_state_line(tx);
    let wr_res = i2c.write(I2C_ADDR, &[0x02, 0xA5, 0x5A, 0xC3]);
    let mut back = [0u8; 3];
    let rd_res = i2c.write_read(I2C_ADDR, &[0x02], &mut back);
    let wrrd_ok = wr_res.is_ok() && rd_res.is_ok() && back == [0xA5, 0x5A, 0xC3];
    all_ok &= verdict(tx, b"X_I2C WRRD", wrrd_ok);
    if !wrrd_ok {
        i2c_detail(tx, wr_res.and(rd_res), &back);
    }

    // 5. Register 0 is read-only in the peer: an overwrite attempt must
    //    leave the ID intact.
    bus_state_line(tx);
    let ro_wr_res = i2c.write(I2C_ADDR, &[0x00, 0xFF]);
    let mut ro = [0u8; 1];
    let ro_rd_res = i2c.write_read(I2C_ADDR, &[0x00], &mut ro);
    let ro_ok = ro_wr_res.is_ok() && ro_rd_res.is_ok() && ro[0] == I2C_ID_BYTE;
    all_ok &= verdict(tx, b"X_I2C ROREG", ro_ok);
    if !ro_ok {
        i2c_detail(tx, ro_wr_res.and(ro_rd_res), &ro);
    }

    // 6. Pointer wrap, via a standalone read transaction (START-read after a
    //    STOP, not a repeated START): set the pointer to the last register
    //    in its own write, then read two bytes — register 15 (never written,
    //    0x00) then wrap to register 0 (the ID). Deliberately NOT a
    //    continue-from-previous-pointer read: the speculative TX request
    //    advances the peer's pointer by a timing-dependent amount after
    //    every read, so only an explicitly re-pointed read is deterministic.
    bus_state_line(tx);
    let ptr_res = i2c.write(I2C_ADDR, &[0x0F]);
    let mut wrap = [0u8; 2];
    let wrap_rd_res = i2c.read(I2C_ADDR, &mut wrap);
    let wrap_ok = ptr_res.is_ok() && wrap_rd_res.is_ok() && wrap == [0x00, I2C_ID_BYTE];
    all_ok &= verdict(tx, b"X_I2C WRAP", wrap_ok);
    if !wrap_ok {
        i2c_detail(tx, ptr_res.and(wrap_rd_res), &wrap);
    }

    if !all_ok {
        red.set_high().ok();
    }
    tx.write_all(b"2B_I2C_TEST_END\r\n").ok();
}

/// `e`: echo every byte received on the A1 cross-link back +1 until `q`
/// arrives on the backchannel. `+1` proves software reception on this board
/// — the crossed TX/RX wiring alone could never produce it.
fn run_uart_echo(
    tx1: &mut Tx<UsciA1>,
    rx1: &mut Rx<UsciA1>,
    tx: &mut Tx<UsciA0>,
    rx: &mut Rx<UsciA0>,
    green: &mut impl StatefulOutputPin,
) {
    let mut received: u16 = 0;
    let mut errors: u16 = 0;

    tx.write_all(b"2B_UARTECHO_ON\r\n").ok();
    loop {
        if poll_cmd(rx) == Some(b'q') {
            break;
        }
        match rx1.read() {
            Ok(byte) => {
                tx1.write_all(&[byte.wrapping_add(1)]).ok();
                received += 1;
                green.toggle().ok();
            }
            Err(hal::embedded_hal_nb::nb::Error::WouldBlock) => {}
            Err(_) => errors = errors.saturating_add(1),
        }
    }
    tx.write_all(b"2B_UARTECHO_STATS rx=").ok();
    write_dec(tx, received as u32);
    tx.write_all(b" err=").ok();
    write_dec(tx, errors as u32);
    tx.write_all(b"\r\n").ok();
}

/// `t`: send the pattern down the A1 cross-link one byte at a time and
/// require each echo to come back +1, in order. Two independent DCOs clock
/// the two ends — this is a real baud-tolerance test, which a single-board
/// loopback (same clock both ways) can never be.
fn run_uart_initiator(
    tx1: &mut Tx<UsciA1>,
    rx1: &mut Rx<UsciA1>,
    tx: &mut Tx<UsciA0>,
    delay: &mut Delay,
    red: &mut impl OutputPin,
) {
    // Drop any stale bytes a previous run left in RXBUF.
    while rx1.read().is_ok() {}

    let mut echo_ok = true;
    let mut errors: u16 = 0;
    for &byte in UART_PATTERN {
        tx1.write_all(&[byte]).ok();
        // A byte at 9600 baud takes ~1.04 ms each way; allow 50 ms.
        let mut got: Option<u8> = None;
        for _ in 0..500 {
            match rx1.read() {
                Ok(b) => {
                    got = Some(b);
                    break;
                }
                Err(hal::embedded_hal_nb::nb::Error::WouldBlock) => delay.delay_us(100),
                Err(_) => errors = errors.saturating_add(1),
            }
        }
        if got != Some(byte.wrapping_add(1)) {
            echo_ok = false;
        }
    }

    tx.write_all(b"2B_UART_TEST_BEGIN\r\n").ok();
    let mut all_ok = true;
    all_ok &= verdict(tx, b"X_UART ECHO", echo_ok);
    all_ok &= verdict(tx, b"X_UART CLEAN", errors == 0);
    if !all_ok {
        red.set_high().ok();
    }
    tx.write_all(b"2B_UART_TEST_END\r\n").ok();
}

/// After a FAIL verdict on an I2C step: emit the transfer outcome and the
/// bytes actually read, so a one-line host log localizes the fault (wire vs
/// NACK vs timeout vs wrong data).
fn i2c_detail(tx: &mut Tx<UsciA0>, res: Result<(), hal::i2c::Error>, got: &[u8]) {
    tx.write_all(b"X_I2C_DETAIL err=").ok();
    let code: &[u8] = match res {
        Ok(()) => b"none",
        Err(hal::i2c::Error::AddressNack) => b"anack",
        Err(hal::i2c::Error::DataNack) => b"dnack",
        Err(hal::i2c::Error::Timeout) => b"tmo",
    };
    tx.write_all(code).ok();
    tx.write_all(b" got=").ok();
    for byte in got {
        let hex = |n: u8| if n < 10 { b'0' + n } else { b'a' + n - 10 };
        tx.write_all(&[hex(byte >> 4), hex(byte & 0xF), b' ']).ok();
    }
    tx.write_all(b"\r\n").ok();
}

/// Emit `<name> OK` / `<name> FAIL` and return whether it passed.
fn verdict(tx: &mut Tx<UsciA0>, name: &[u8], ok: bool) -> bool {
    tx.write_all(name).ok();
    tx.write_all(if ok { b" OK\r\n" } else { b" FAIL\r\n" }).ok();
    ok
}

/// Load the provisioned role from Info FRAM (magic-guarded; anything else —
/// fresh silicon, another fixture's leavings — reads as Unset).
fn load_role(fram: &mut InfoFram) -> Role {
    let mut record = [0u8; 4];
    fram.read(FRAM_OFFSET, &mut record).ok();
    if record[0] != MAGIC[0] || record[1] != MAGIC[1] {
        return Role::Unset;
    }
    match record[2] {
        b'P' => Role::Parent,
        b'C' => Role::Child,
        _ => Role::Unset,
    }
}

/// Persist the role to Info FRAM.
fn save_role(fram: &mut InfoFram, role: Role) {
    let byte = match role {
        Role::Parent => b'P',
        Role::Child => b'C',
        Role::Unset => 0,
    };
    let record = [MAGIC[0], MAGIC[1], byte, 0];
    fram.write(FRAM_OFFSET, &record).ok();
}

/// Write a 16-bit value as four hex digits (no core::fmt).
fn write_hex16<W: hal::embedded_io::Write>(tx: &mut W, value: u16) {
    let hex = |n: u8| if n < 10 { b'0' + n } else { b'a' + n - 10 };
    let b = [
        hex((value >> 12) as u8 & 0xF),
        hex((value >> 8) as u8 & 0xF),
        hex((value >> 4) as u8 & 0xF),
        hex(value as u8 & 0xF),
    ];
    tx.write_all(&b).ok();
}

/// Write an unsigned value as decimal ASCII (no padding, no core::fmt).
fn write_dec<W: hal::embedded_io::Write>(tx: &mut W, mut value: u32) {
    let mut buf = [0u8; 10];
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (value % 10) as u8;
        value /= 10;
        if value == 0 {
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
#[unsafe(no_mangle)]
pub extern "C" fn abort() -> ! {
    loop {}
}
