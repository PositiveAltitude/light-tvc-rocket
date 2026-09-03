use esp_idf_hal::gpio::{Gpio21, Output, PinDriver};
use esp_idf_sys::EspError;

pub struct VoltageRegulator<'a> {
    driver: PinDriver<'a, Gpio21, Output>
}

impl<'a> VoltageRegulator<'a> {
    pub fn new(pin21: Gpio21) -> Self {
        let mut driver = esp_idf_hal::gpio::PinDriver::output(pin21).unwrap();
        driver.set_low().unwrap();

        Self{driver}
    }

    pub fn on(&mut self) -> Result<(), EspError> {
        self.driver.set_high()
    }

    pub fn off(&mut self) -> Result<(), EspError> {
        self.driver.set_low()
    }
}