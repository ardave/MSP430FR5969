use std::error::Error;

mod deployment;
mod serial;

mod serial_port_tests;
mod serial_irq_tests;
mod gpio_tests;
mod ta_pwm_tests;
mod timer_tests;
mod delay_tests;
mod deep_sleep_tests;
mod lpmx5_tests;
mod i2c_tests;
mod spi_tests;
mod adc_tests;
mod adc_irq_tests;
mod adc_dma_tests;
mod adc_seq_tests;
mod adc_window_tests;
mod accel_tests;
mod capture_tests;
mod clock_speed_tests;
mod comp_tests;
mod mpu_tests;
mod dma_tests;
mod ref_a_tests;
mod fram_tests;
mod rtc_tests;
mod rtc_tick_tests;
mod vlo_soak_tests;
mod watchdog_tests;
mod wdt_interval_tests;

fn main() -> Result<(), Box<dyn Error>> {
    // With no args, run everything; with args, run only the named suites
    // (e.g. `cargo run -- lpmx5 rtc` after touching those drivers).
    let only: Vec<String> = std::env::args().skip(1).collect();
    let wanted = |name: &str| only.is_empty() || only.iter().any(|o| o == name);

    println!("Starting Tests");

    let suites: [(&str, fn() -> Result<(), Box<dyn Error>>); 27] = [
        ("serial_port", serial_port_tests::run),
        ("serial_irq", serial_irq_tests::run),
        ("dma", dma_tests::run),
        ("accel", accel_tests::run),
        ("capture", capture_tests::run),
        ("clock_speed", clock_speed_tests::run),
        ("clock_high_speed", clock_speed_tests::run_high_speed),
        ("comp", comp_tests::run),
        ("mpu", mpu_tests::run),
        ("adc", adc_tests::run),
        ("adc_irq", adc_irq_tests::run),
        ("adc_dma", adc_dma_tests::run),
        ("adc_seq", adc_seq_tests::run),
        ("adc_window", adc_window_tests::run),
        ("ref_a", ref_a_tests::run),
        ("fram", fram_tests::run),
        ("rtc", rtc_tests::run),
        ("rtc_tick", rtc_tick_tests::run),
        ("watchdog", watchdog_tests::run),
        ("wdt_interval", wdt_interval_tests::run),
        ("gpio", gpio_tests::run),
        ("timer", timer_tests::run),
        ("ta_pwm", ta_pwm_tests::run),
        ("delay", delay_tests::run),
        ("deep_sleep", deep_sleep_tests::run),
        ("lpmx5", lpmx5_tests::run),
        ("i2c", i2c_tests::run),
        // spi is interactive (loopback-jumper prompt); run it by name only.
    ];

    for (name, run) in suites {
        if wanted(name) {
            run()?;
        }
    }
    if wanted("spi") && !only.is_empty() {
        spi_tests::run()?;
    }
    // capture_jumper is interactive (PWM-loopback-jumper prompt); by name only.
    // (`capture` above is the hands-free variant: PWM verdicts may SKIP.)
    if wanted("capture_jumper") && !only.is_empty() {
        capture_tests::run_with_jumper()?;
    }
    // vlo_soak is an instrument (200 self-reboots measuring the ACLK boot
    // race), not a regression gate; run it by name only.
    if wanted("vlo_soak") && !only.is_empty() {
        vlo_soak_tests::run()?;
    }

    Ok(())
}
