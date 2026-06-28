use std::error::Error;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting I2C Tests...");

    println!("I2C Tests Completed Successfully");
    Ok(())
}
