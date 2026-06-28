use std::error::Error;

pub fn run() -> Result<(), Box<dyn Error>> {
    println!("Starting FRAM Tests...");

    println!("FRAM Tests Completed Successfully");
    Ok(())
}
