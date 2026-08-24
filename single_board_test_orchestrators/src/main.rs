use std::error::Error;

mod deployment;
mod serial;

mod serial_port_test_orchestrator;
mod serial_irq_test_orchestrator;
mod gpio_test_orchestrator;
mod ta_pwm_test_orchestrator;
mod timer_test_orchestrator;
mod delay_test_orchestrator;
mod deep_sleep_test_orchestrator;
mod lpmx5_test_orchestrator;
mod spi_test_orchestrator;
mod adc_test_orchestrator;
mod adc_irq_test_orchestrator;
mod adc_dma_test_orchestrator;
mod adc_seq_test_orchestrator;
mod adc_window_test_orchestrator;
mod accel_test_orchestrator;
mod capture_test_orchestrator;
mod captio_test_orchestrator;
mod clock_speed_test_orchestrator;
mod comp_test_orchestrator;
mod mpu_test_orchestrator;
mod dma_test_orchestrator;
mod ref_a_test_orchestrator;
mod fram_test_orchestrator;
mod rtc_test_orchestrator;
mod rtc_tick_test_orchestrator;
mod vlo_soak_test_orchestrator;
mod watchdog_test_orchestrator;
mod wdt_interval_test_orchestrator;

fn main() -> Result<(), Box<dyn Error>> {
    // With no args, run everything; with args, run only the named suites
    // (e.g. `cargo run -- lpmx5 rtc` after touching those drivers).
    let only: Vec<String> = std::env::args().skip(1).collect();
    let wanted = |name: &str| only.is_empty() || only.iter().any(|o| o == name);

    println!("Starting Tests");

    let suites: [(&str, fn() -> Result<(), Box<dyn Error>>); 27] = [
        ("serial_port", serial_port_test_orchestrator::run),
        ("serial_irq", serial_irq_test_orchestrator::run),
        ("dma", dma_test_orchestrator::run),
        ("accel", accel_test_orchestrator::run),
        ("capture", capture_test_orchestrator::run),
        ("captio", captio_test_orchestrator::run),
        ("clock_speed", clock_speed_test_orchestrator::run),
        ("clock_high_speed", clock_speed_test_orchestrator::run_high_speed),
        ("comp", comp_test_orchestrator::run),
        ("mpu", mpu_test_orchestrator::run),
        ("adc", adc_test_orchestrator::run),
        ("adc_irq", adc_irq_test_orchestrator::run),
        ("adc_dma", adc_dma_test_orchestrator::run),
        ("adc_seq", adc_seq_test_orchestrator::run),
        ("adc_window", adc_window_test_orchestrator::run),
        ("ref_a", ref_a_test_orchestrator::run),
        ("fram", fram_test_orchestrator::run),
        ("rtc", rtc_test_orchestrator::run),
        ("rtc_tick", rtc_tick_test_orchestrator::run),
        ("watchdog", watchdog_test_orchestrator::run),
        ("wdt_interval", wdt_interval_test_orchestrator::run),
        ("gpio", gpio_test_orchestrator::run),
        ("timer", timer_test_orchestrator::run),
        ("ta_pwm", ta_pwm_test_orchestrator::run),
        ("delay", delay_test_orchestrator::run),
        ("deep_sleep", deep_sleep_test_orchestrator::run),
        ("lpmx5", lpmx5_test_orchestrator::run),
        // spi is interactive (loopback-jumper prompt); run it by name only.
    ];

    for (name, run) in suites {
        if wanted(name) {
            run()?;
        }
    }
    if wanted("spi") && !only.is_empty() {
        spi_test_orchestrator::run()?;
    }
    // capture_jumper is interactive (PWM-loopback-jumper prompt); by name only.
    // (`capture` above is the hands-free variant: PWM verdicts may SKIP.)
    if wanted("capture_jumper") && !only.is_empty() {
        capture_test_orchestrator::run_with_jumper()?;
    }
    // vlo_soak is an instrument (200 self-reboots measuring the ACLK boot
    // race), not a regression gate; run it by name only.
    if wanted("vlo_soak") && !only.is_empty() {
        vlo_soak_test_orchestrator::run()?;
    }

    Ok(())
}
