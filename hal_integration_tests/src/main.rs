use std::error::Error;

mod deployment;
mod serial;

mod serial_port_tests;
mod serial_irq_tests;
mod gpio_tests;
mod timer_tests;
mod delay_tests;
mod deep_sleep_tests;
mod lpmx5_tests;
mod i2c_tests;
mod spi_tests;
mod adc_tests;
mod adc_irq_tests;
mod ref_a_tests;
mod fram_tests;
mod rtc_tests;
mod watchdog_tests;
mod wdt_interval_tests;

fn main() -> Result<(), Box<dyn Error>> {
    // With no args, run everything; with args, run only the named suites
    // (e.g. `cargo run -- lpmx5 rtc` after touching those drivers).
    let only: Vec<String> = std::env::args().skip(1).collect();
    let wanted = |name: &str| only.is_empty() || only.iter().any(|o| o == name);

    println!("Starting Tests");

    let suites: [(&str, fn() -> Result<(), Box<dyn Error>>); 15] = [
        ("serial_port", serial_port_tests::run),
        ("serial_irq", serial_irq_tests::run),
        ("adc", adc_tests::run),
        ("adc_irq", adc_irq_tests::run),
        ("ref_a", ref_a_tests::run),
        ("fram", fram_tests::run),
        ("rtc", rtc_tests::run),
        ("watchdog", watchdog_tests::run),
        ("wdt_interval", wdt_interval_tests::run),
        ("gpio", gpio_tests::run),
        ("timer", timer_tests::run),
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

    Ok(())
}
