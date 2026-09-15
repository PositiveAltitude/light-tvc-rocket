use esp_idf_hal::i2c::{I2cDriver, I2cError};
use std::sync::{Arc, Mutex};

/// Cloneable, transaction-serialized access to the board's shared I²C bus.
///
/// The MAX17048 fuel gauge, ICM-42688-P IMU, and BMP280 barometer share I2C0.
/// Each high-level driver receives a handle while every bus transaction stays
/// atomic, so no device can interleave bytes with another device.
#[derive(Clone)]
pub struct SharedI2c {
    inner: Arc<Mutex<I2cDriver<'static>>>,
}

impl SharedI2c {
    pub fn new(driver: I2cDriver<'static>) -> Self {
        Self {
            inner: Arc::new(Mutex::new(driver)),
        }
    }
}

impl embedded_hal::i2c::ErrorType for SharedI2c {
    type Error = I2cError;
}

impl embedded_hal::i2c::I2c for SharedI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        embedded_hal::i2c::I2c::transaction(&mut *self.inner.lock().unwrap(), address, operations)
    }
}
