use std::error::Error;

mod deployment;
mod serial;

mod serial_port_tests;
mod gpio_tests;
mod timer_tests;
mod delay_tests;
mod deep_sleep_tests;
mod i2c_tests;
mod spi_tests;
mod adc_tests;
mod ref_a_tests;
mod fram_tests;
mod rtc_tests;
mod watchdog_tests;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Starting Tests");

    serial_port_tests::run()?;
    adc_tests::run()?;
    ref_a_tests::run()?;
    fram_tests::run()?;
    rtc_tests::run()?;
    watchdog_tests::run()?;
    gpio_tests::run()?;
    timer_tests::run()?;
    delay_tests::run()?;
    deep_sleep_tests::run()?;
    i2c_tests::run()?;
    // spi_tests::run()?;

    Ok(())
}
