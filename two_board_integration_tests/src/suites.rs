//! The cross-board test suites. Each drives BOTH boards through their
//! backchannels: one board is put into a serving/generating mode, the other
//! runs the measurement or protocol exercise, and the host holds every
//! expectation that spans the two clock domains.
//!
//! All suites run each direction where the wiring is symmetric, so both
//! boards' silicon gets exercised in both the driving and the observing role.

use std::error::Error;
use std::thread::sleep;
use std::time::{Duration, Instant};

use crate::boards::{Board, Rig};
use crate::serial::{self, field};

const T: Duration = Duration::from_secs(10);

/// Read the next non-empty line and require it to be exactly `want`.
fn expect_exact(board: &mut Board, want: &str) -> Result<(), Box<dyn Error>> {
    let deadline = Instant::now() + T;
    loop {
        let line = serial::read_line(board.port.as_mut(), deadline)?;
        if line.is_empty() {
            continue;
        }
        println!("  [{}] {line}", board.role.as_str());
        if line == want {
            return Ok(());
        }
        return Err(format!(
            "[{}] expected {want:?}, got {line:?}",
            board.role.as_str()
        )
        .into());
    }
}

/// Require `key=value` in a report line.
fn expect_field(line: &str, key: &str, want: u32, who: &str) -> Result<(), Box<dyn Error>> {
    match field(line, key) {
        Some(got) if got == want => Ok(()),
        got => Err(format!("[{who}] expected {key}={want}, got {got:?} in {line:?}").into()),
    }
}

/// Require `key=value` within ±tol of `want`.
fn expect_field_near(
    line: &str,
    key: &str,
    want: u32,
    tol: u32,
    who: &str,
) -> Result<(), Box<dyn Error>> {
    match field(line, key) {
        Some(got) if got.abs_diff(want) <= tol => Ok(()),
        got => Err(format!(
            "[{who}] expected {key}={want}±{tol}, got {got:?} in {line:?}"
        )
        .into()),
    }
}

/// Identity: both boards answer `i` with their provisioned role (already how
/// discovery paired them) and the SAME firmware revision — a stale flash on
/// one board (e.g. the probe-pinned flashing only reached the other) fails
/// here, before any cross-board behavior can mislead.
pub fn identity(rig: &mut Rig) -> Result<(), Box<dyn Error>> {
    println!("== identity ==");
    let p = rig.parent.cmd_expect(b'i', "2B_ID role=parent", T)?;
    let c = rig.child.cmd_expect(b'i', "2B_ID role=child", T)?;
    let (pfw, cfw) = (field(&p, "fw"), field(&c, "fw"));
    if pfw.is_none() || pfw != cfw {
        return Err(format!("firmware mismatch: parent {pfw:?} vs child {cfw:?}").into());
    }
    Ok(())
}

/// I2C bridge — the flagship: the FIRST on-silicon exercise of the HAL's
/// eUSCI_B0 I2C slave driver, with the HAL's own master on the other board.
/// Child serves the 16-register file at 0x48; parent runs the protocol
/// gauntlet (probe / empty-address NACK / ID / write-read / read-only reg /
/// pointer wrap; back-to-back reads within it double as the speculative-TX
/// flush check). The child's transaction tally is then asserted exactly:
/// 8 STOPs — probe + pointer-set writes and the ID/WRRD/ROREG/WRAP reads —
/// of which 4 ended in a write phase and 4 in a read phase.
pub fn i2c_bridge(rig: &mut Rig) -> Result<(), Box<dyn Error>> {
    println!("== i2c_bridge ==");
    rig.child.cmd_expect(b's', "2B_SLAVE_ON addr=0x48", T)?;

    rig.parent.cmd_expect(b'm', "2B_I2C_TEST_BEGIN", T)?;
    for want in [
        "X_I2C PROBE OK",
        "X_I2C NODEV OK",
        "X_I2C ID OK",
        "X_I2C WRRD OK",
        "X_I2C ROREG OK",
        "X_I2C WRAP OK",
        "2B_I2C_TEST_END",
    ] {
        expect_exact(&mut rig.parent, want)?;
    }

    let stats = rig.child.cmd_expect(b'q', "2B_SLAVE_STATS", T)?;
    expect_field(&stats, "trans", 8, "child")?;
    expect_field(&stats, "wr", 4, "child")?;
    expect_field(&stats, "rd", 4, "child")?;
    Ok(())
}

/// UART cross-link (eUSCI_A1): a 24-byte pattern echoed back +1, each
/// direction of initiation. Two independent DCOs clock the two ends, so this
/// is a genuine baud-tolerance test no single-board loopback can perform;
/// the +1 proves software reception (a wire short would echo unchanged).
pub fn uart_link(rig: &mut Rig) -> Result<(), Box<dyn Error>> {
    println!("== uart_link ==");
    for flip in [false, true] {
        let (echoer, initiator) = pair(rig, flip);
        echoer.cmd_expect(b'e', "2B_UARTECHO_ON", T)?;

        initiator.cmd_expect(b't', "2B_UART_TEST_BEGIN", T)?;
        expect_exact(initiator, "X_UART ECHO OK")?;
        expect_exact(initiator, "X_UART CLEAN OK")?;
        expect_exact(initiator, "2B_UART_TEST_END")?;

        let (echoer, _) = pair(rig, flip);
        let stats = echoer.cmd_expect(b'q', "2B_UARTECHO_STATS", T)?;
        let who = echoer.role.as_str().to_string();
        expect_field(&stats, "rx", 24, &who)?;
        expect_field(&stats, "err", 0, &who)?;
    }
    Ok(())
}

/// GPIO edges: ten real wire pulses (not software-set PxIFG, which is all the
/// single-board fixture can do) counted by the peer's PORT3 ISR through the
/// PxIV demux — exactly ten, with zero foreign IV slots, each direction.
pub fn gpio_edge(rig: &mut Rig) -> Result<(), Box<dyn Error>> {
    println!("== gpio_edge ==");
    for flip in [false, true] {
        let (counter, pulser) = pair(rig, flip);
        counter.cmd_expect(b'g', "2B_GPIO_ARMED", T)?;

        let pulsed = pulser.cmd_expect(b'p', "2B_PULSED", T)?;
        expect_field(&pulsed, "n", 10, pulser.role.as_str())?;

        let (counter, _) = pair(rig, flip);
        let stats = counter.cmd_expect(b'q', "2B_GPIO_STATS", T)?;
        let who = counter.role.as_str().to_string();
        expect_field(&stats, "edges", 10, &who)?;
        expect_field(&stats, "badiv", 0, &who)?;
    }
    Ok(())
}

/// LPM4 wake: one board parks in LPM4 (every clock stopped), the OTHER board
/// wakes it with a single wire edge — a genuinely external wake source,
/// which the on-board buttons could only fake with a human present. Exactly
/// one edge must be tallied, each direction.
pub fn lpm4_wake(rig: &mut Rig) -> Result<(), Box<dyn Error>> {
    println!("== lpm4_wake ==");
    for flip in [false, true] {
        let (sleeper, waker) = pair(rig, flip);
        sleeper.cmd_expect(b'w', "2B_SLEEPING", T)?;

        // Give the sleeper time to flush its UART and actually reach LPM4 —
        // waking a board that hasn't slept yet would still count the edge,
        // but the point is to prove the wake-from-LPM4 path.
        sleep(Duration::from_millis(500));
        waker.cmd_expect(b'1', "2B_PULSED", T)?;

        let (sleeper, _) = pair(rig, flip);
        let woke = sleeper.expect_prefix("2B_WOKE", T)?;
        expect_field(&woke, "edges", 1, sleeper.role.as_str())?;
    }
    Ok(())
}

/// PWM ↔ capture across clock domains: one board generates 1 kHz PWM
/// (Timer_B0), the peer timestamps it with Timer_A1 capture. The frequency
/// gate (±5 %) is the two DCOs measured against each other; the 25 %/75 %
/// duty points are asymmetric, so a transposed or inverted line cannot pass.
pub fn pwm_cross(rig: &mut Rig) -> Result<(), Box<dyn Error>> {
    println!("== pwm_cross ==");
    for flip in [false, true] {
        let (generator, _) = pair(rig, flip);
        let on = generator.cmd_expect(b'f', "2B_PWM", T)?;
        let gen_freq = field(&on, "freq").ok_or("no freq in 2B_PWM line")?;

        let (measurer, _) = pair_rev(rig, flip);
        let cap = measurer.cmd_expect(b'c', "2B_CAP", T)?;
        let who = measurer.role.as_str().to_string();
        expect_field_near(&cap, "freq", gen_freq, gen_freq / 20, &who)?;
        expect_field_near(&cap, "duty", 250, 25, &who)?;

        let (generator, _) = pair(rig, flip);
        generator.cmd_expect(b'F', "2B_PWM", T)?;
        let (measurer, _) = pair_rev(rig, flip);
        let cap = measurer.cmd_expect(b'c', "2B_CAP", T)?;
        expect_field_near(&cap, "duty", 750, 25, &who)?;

        let (generator, _) = pair(rig, flip);
        generator.cmd_expect(b'x', "2B_PWM_OFF", T)?;
    }
    Ok(())
}

/// Cross-board analog: the generator's PWM through the wiring's RC becomes a
/// DC level; the measurer's ADC reads it in millivolts. Expected value =
/// duty × the generator's own ADC-measured rail, so the assertion closes a
/// loop through BOTH chips' calibrated analog chains. Two duty points catch
/// gain and offset errors independently.
pub fn adc_dac(rig: &mut Rig) -> Result<(), Box<dyn Error>> {
    println!("== adc_dac ==");
    for flip in [false, true] {
        let (generator, _) = pair(rig, flip);
        let rail = generator.cmd_expect(b'a', "2B_ADC", T)?;
        let gen_avcc = field(&rail, "avcc_mv").ok_or("no avcc_mv in 2B_ADC line")?;
        if !(2900..=3700).contains(&gen_avcc) {
            return Err(format!("implausible generator rail: {gen_avcc} mV").into());
        }

        for (cmd, permille) in [(b'd', 300u32), (b'D', 600u32)] {
            let (generator, _) = pair(rig, flip);
            generator.cmd_expect(cmd, "2B_DAC", T)?;
            // RC settle: τ = 2.2 kΩ × 10 µF = 22 ms; 700 ms ≈ 30 τ.
            sleep(Duration::from_millis(700));

            let (measurer, _) = pair_rev(rig, flip);
            let adc = measurer.cmd_expect(b'a', "2B_ADC", T)?;
            let want = permille * gen_avcc / 1000;
            let tol = want / 20 + 30; // ±5 % + 30 mV (ripple + cal floors)
            let who = measurer.role.as_str().to_string();
            expect_field_near(&adc, "a7_mv", want, tol, &who)?;
        }

        let (generator, _) = pair(rig, flip);
        generator.cmd_expect(b'x', "2B_PWM_OFF", T)?;
    }
    Ok(())
}

/// Pick (A, B) = (child, parent) or flipped — the "serving" board first.
fn pair(rig: &mut Rig, flip: bool) -> (&mut Board, &mut Board) {
    if flip {
        (&mut rig.parent, &mut rig.child)
    } else {
        (&mut rig.child, &mut rig.parent)
    }
}

/// The opposite selection: the board whose turn it is to observe.
fn pair_rev(rig: &mut Rig, flip: bool) -> (&mut Board, &mut Board) {
    pair(rig, !flip)
}
