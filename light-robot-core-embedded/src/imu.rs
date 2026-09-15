use embedded_hal::i2c::I2c;
use esp_idf_hal::delay::FreeRtos;
use light_robot_core_api::ImuState;

use crate::shared_i2c::SharedI2c;

const WHO_AM_I: u8 = 0x75;
const WHO_AM_I_VALUE: u8 = 0x47;
const I2C_ADDRESS: u8 = 0x69;
const PWR_MGMT0: u8 = 0x4e;
const GYRO_CONFIG0: u8 = 0x4f;
const ACCEL_CONFIG0: u8 = 0x50;
const ACCEL_DATA_X1: u8 = 0x1f;

// ±16 g and ±2000 °/s, both with a 100 Hz ODR. The ICM-42688-P encodes
// 16 g / 2000 °/s as FS_SEL 0, and 100 Hz as ODR 7.
const CONFIG_100_HZ: u8 = 0x07;
const ACCEL_SCALE_MPS2: f32 = 16.0 * 9.80665 / 32768.0;
const GYRO_SCALE_RADPS: f32 = 2000.0 * core::f32::consts::PI / 180.0 / 32768.0;

pub enum ImuError {
    IdentityRead,
    UnexpectedIdentity(u8),
    PowerMode,
    GyroConfiguration,
    AccelConfiguration,
    DataRead,
}

impl core::fmt::Debug for ImuError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::IdentityRead => formatter.write_str("IdentityRead"),
            Self::UnexpectedIdentity(identity) => formatter
                .debug_tuple("UnexpectedIdentity")
                .field(identity)
                .finish(),
            Self::PowerMode => formatter.write_str("PowerMode"),
            Self::GyroConfiguration => formatter.write_str("GyroConfiguration"),
            Self::AccelConfiguration => formatter.write_str("AccelConfiguration"),
            Self::DataRead => formatter.write_str("DataRead"),
        }
    }
}

/// ICM-42688-P raw accel/gyro reader at the board's AP_AD0-high I²C address.
pub struct Imu {
    bus: SharedI2c,
}

impl Imu {
    pub fn new(bus: SharedI2c) -> Self {
        Self { bus }
    }

    pub fn initialize(&mut self) -> Result<(), ImuError> {
        let mut value = [0_u8];
        self.bus
            .write_read(I2C_ADDRESS, &[WHO_AM_I], &mut value)
            .map_err(|_| ImuError::IdentityRead)?;
        if value[0] != WHO_AM_I_VALUE {
            return Err(ImuError::UnexpectedIdentity(value[0]));
        }
        // The hardware reset/power-on sequence has already completed before
        // this task starts. Avoid a software reset here: some boards NACK the
        // first configuration write while that reset is still settling.
        self.bus
            .write(I2C_ADDRESS, &[PWR_MGMT0, 0x0f])
            .map_err(|_| ImuError::PowerMode)?;
        // Gyro startup is the longer of the two paths; do it only on a
        // reconnect, not in the 100 Hz acquisition loop.
        FreeRtos::delay_ms(50);
        self.bus
            .write(I2C_ADDRESS, &[GYRO_CONFIG0, CONFIG_100_HZ])
            .map_err(|_| ImuError::GyroConfiguration)?;
        self.bus
            .write(I2C_ADDRESS, &[ACCEL_CONFIG0, CONFIG_100_HZ])
            .map_err(|_| ImuError::AccelConfiguration)?;
        Ok(())
    }

    pub fn read(&mut self, sample_count: u32) -> Result<ImuState, ImuError> {
        // Bank 0 lays out six big-endian accel bytes followed by six gyro
        // bytes, permitting an atomic 12-byte I²C register read.
        let mut data = [0_u8; 12];
        self.bus
            .write_read(I2C_ADDRESS, &[ACCEL_DATA_X1], &mut data)
            .map_err(|_| ImuError::DataRead)?;
        let axis = |offset| i16::from_be_bytes([data[offset], data[offset + 1]]) as f32;
        Ok(ImuState {
            present: true,
            acceleration_mps2: [
                axis(0) * ACCEL_SCALE_MPS2,
                axis(2) * ACCEL_SCALE_MPS2,
                axis(4) * ACCEL_SCALE_MPS2,
            ],
            angular_velocity_radps: [
                axis(6) * GYRO_SCALE_RADPS,
                axis(8) * GYRO_SCALE_RADPS,
                axis(10) * GYRO_SCALE_RADPS,
            ],
            sample_count,
            average_rate_hz: 0.0,
        })
    }
}
