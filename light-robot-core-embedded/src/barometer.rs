use embedded_hal::i2c::I2c;
use light_robot_core_api::BarometerState;

use crate::shared_i2c::SharedI2c;

const I2C_ADDRESS: u8 = 0x76;
const CHIP_ID: u8 = 0x58;
const REG_CHIP_ID: u8 = 0xd0;
const REG_CALIBRATION: u8 = 0x88;
const REG_CONTROL: u8 = 0xf4;
const REG_CONFIG: u8 = 0xf5;
const REG_PRESSURE: u8 = 0xf7;
/// First-order IIR coefficient for a 5 Hz cutoff at the 100 Hz task rate.
/// This keeps altitude responsive to flight-scale motion while filtering the
/// single-sample pressure noise visible on a stationary vehicle.
const LOW_PASS_ALPHA: f32 = 0.269_597;

#[derive(Debug)]
pub enum BarometerError {
    IdentityRead,
    UnexpectedIdentity(u8),
    CalibrationRead,
    Configuration,
    DataRead,
}

#[derive(Clone, Copy)]
struct Calibration {
    t1: u16,
    t2: i16,
    t3: i16,
    p1: u16,
    p2: i16,
    p3: i16,
    p4: i16,
    p5: i16,
    p6: i16,
    p7: i16,
    p8: i16,
    p9: i16,
}

impl Calibration {
    fn from_registers(bytes: [u8; 24]) -> Self {
        let u16_at = |offset| u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        let i16_at = |offset| i16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        Self {
            t1: u16_at(0),
            t2: i16_at(2),
            t3: i16_at(4),
            p1: u16_at(6),
            p2: i16_at(8),
            p3: i16_at(10),
            p4: i16_at(12),
            p5: i16_at(14),
            p6: i16_at(16),
            p7: i16_at(18),
            p8: i16_at(20),
            p9: i16_at(22),
        }
    }
}

/// BMP280 reader in normal mode, configured for pressure and temperature x1
/// oversampling. The 0.5 ms standby setting keeps fresh conversions available
/// for the 100 Hz sampling task without blocking I²C for conversion time.
pub struct Barometer {
    bus: SharedI2c,
    calibration: Option<Calibration>,
    filtered_pressure_hpa: Option<f32>,
    filtered_temperature_c: Option<f32>,
}

impl Barometer {
    pub fn new(bus: SharedI2c) -> Self {
        Self {
            bus,
            calibration: None,
            filtered_pressure_hpa: None,
            filtered_temperature_c: None,
        }
    }

    pub fn initialize(&mut self) -> Result<(), BarometerError> {
        let mut id = [0];
        self.bus
            .write_read(I2C_ADDRESS, &[REG_CHIP_ID], &mut id)
            .map_err(|_| BarometerError::IdentityRead)?;
        if id[0] != CHIP_ID {
            return Err(BarometerError::UnexpectedIdentity(id[0]));
        }
        let mut calibration = [0; 24];
        self.bus
            .write_read(I2C_ADDRESS, &[REG_CALIBRATION], &mut calibration)
            .map_err(|_| BarometerError::CalibrationRead)?;
        let calibration = Calibration::from_registers(calibration);
        if calibration.p1 == 0 {
            return Err(BarometerError::CalibrationRead);
        }
        // t_sb=0.5 ms, IIR filter disabled. osrs_t=x1, osrs_p=x1, normal mode.
        self.bus
            .write(I2C_ADDRESS, &[REG_CONFIG, 0x00])
            .map_err(|_| BarometerError::Configuration)?;
        self.bus
            .write(I2C_ADDRESS, &[REG_CONTROL, 0x27])
            .map_err(|_| BarometerError::Configuration)?;
        self.calibration = Some(calibration);
        self.filtered_pressure_hpa = None;
        self.filtered_temperature_c = None;
        Ok(())
    }

    pub fn read(&mut self, sample_count: u32) -> Result<BarometerState, BarometerError> {
        let calibration = self.calibration.ok_or(BarometerError::CalibrationRead)?;
        let mut data = [0; 6];
        self.bus
            .write_read(I2C_ADDRESS, &[REG_PRESSURE], &mut data)
            .map_err(|_| BarometerError::DataRead)?;
        let raw_pressure =
            ((data[0] as i32) << 12) | ((data[1] as i32) << 4) | ((data[2] as i32) >> 4);
        let raw_temperature =
            ((data[3] as i32) << 12) | ((data[4] as i32) << 4) | ((data[5] as i32) >> 4);
        let (temperature, fine_temperature) = compensate_temperature(raw_temperature, calibration);
        let pressure_pa = compensate_pressure(raw_pressure, fine_temperature, calibration)
            .ok_or(BarometerError::DataRead)?;
        let pressure_hpa = low_pass(&mut self.filtered_pressure_hpa, pressure_pa / 100.0);
        let temperature = low_pass(&mut self.filtered_temperature_c, temperature);
        // International Standard Atmosphere barometric formula (P0 = 1013.25 hPa).
        let altitude = 44_330.0 * (1.0 - (pressure_hpa / 1013.25).powf(0.190_294_95));
        Ok(BarometerState {
            present: true,
            altitude,
            temperature,
            pressure_hpa,
            sample_count,
            average_rate_hz: 0.0,
        })
    }
}

fn low_pass(filtered: &mut Option<f32>, sample: f32) -> f32 {
    let next = match filtered {
        Some(previous) => *previous + LOW_PASS_ALPHA * (sample - *previous),
        None => sample,
    };
    *filtered = Some(next);
    next
}

fn compensate_temperature(raw: i32, c: Calibration) -> (f32, i32) {
    let var1 = (((raw >> 3) - ((c.t1 as i32) << 1)) * c.t2 as i32) >> 11;
    let var2 =
        (((((raw >> 4) - c.t1 as i32) * ((raw >> 4) - c.t1 as i32)) >> 12) * c.t3 as i32) >> 14;
    let fine = var1 + var2;
    ((fine * 5 + 128 >> 8) as f32 / 100.0, fine)
}

fn compensate_pressure(raw: i32, fine: i32, c: Calibration) -> Option<f32> {
    let mut var1 = fine as i64 - 128_000;
    let mut var2 = var1 * var1 * c.p6 as i64;
    var2 += (var1 * c.p5 as i64) << 17;
    var2 += (c.p4 as i64) << 35;
    var1 = ((var1 * var1 * c.p3 as i64) >> 8) + ((var1 * c.p2 as i64) << 12);
    var1 = (((1_i64 << 47) + var1) * c.p1 as i64) >> 33;
    if var1 == 0 {
        return None;
    }
    let mut pressure = 1_048_576_i64 - raw as i64;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = (c.p9 as i64 * (pressure >> 13) * (pressure >> 13)) >> 25;
    var2 = (c.p8 as i64 * pressure) >> 19;
    pressure = ((pressure + var1 + var2) >> 8) + ((c.p7 as i64) << 4);
    Some(pressure as f32 / 256.0)
}
