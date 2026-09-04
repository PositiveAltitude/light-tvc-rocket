use esp_idf_hal::gpio::{Gpio21, Output, PinDriver};
use esp_idf_sys::EspError;

pub struct VoltageRegulator<'a> {
    driver: PinDriver<'a, Output>,
}

impl<'a> VoltageRegulator<'a> {
    pub fn new(pin21: Gpio21<'a>) -> Self {
        let mut driver = esp_idf_hal::gpio::PinDriver::output(pin21).unwrap();
        driver.set_low().unwrap();

        Self { driver }
    }

    pub fn on(&mut self) -> Result<(), EspError> {
        self.driver.set_high()
    }
}
