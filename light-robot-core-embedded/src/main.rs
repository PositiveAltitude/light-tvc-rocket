extern crate core;

mod barometer;
mod external_flash;
mod imu;
mod led_driver;
mod server;
mod shared_i2c;
mod voltage_regulator;
mod wifi;

use crate::barometer::Barometer;
use crate::imu::Imu;
use crate::led_driver::LedDriver;
use crate::server::Server;
use crate::shared_i2c::SharedI2c;
use crate::voltage_regulator::VoltageRegulator;
use crate::wifi::WiFi;
use bldc_servo_protocol::{
    ApiEncodeDecode, GeneralCommandFrame, GeneralResponseFrame, ServoCommandFrame,
    ServoResponseFrame,
};
use enumset::EnumSet;
use esp_idf_hal::adc::{
    attenuation,
    oneshot::{config::AdcChannelConfig, AdcChannelDriver, AdcDriver},
};
use esp_idf_hal::can::{CanDriver, Frame};
use esp_idf_hal::cpu::Core;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::Pull;
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriverConfig};
use esp_idf_hal::units::*;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition};
use esp_idf_svc::timer::EspTaskTimerService;
use light_robot_core_api as api;
use light_robot_core_api::*;
use light_robot_core_flight_control::PidControlSystem;
use light_robot_core_simulation::{
    ControlInputs, ControlSystem, Environment, NumericalSimulation, Quat, RocketParameters,
    RocketState, SensorData, Vec3,
};
use log::{error, info, warn};
use max170xx::Max17048;
use std::collections::HashSet;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const WIFI_CONFIGURATION_NVS_KEY: &str = "wifi";
const PRELAUNCH_CHECKLIST_NVS_KEY: &str = "prelaunch";
const PYRO_CONFIGURATION_NVS_KEY: &str = "pyro_cfg";
const ENCODER_COUNTS_PER_TURN: i32 = 16_384;
const SERVO_TEST_CAPTURE_MS: u64 = 250;
const INERTIA_CAPTURE_DURATION: Duration = Duration::from_secs(5);
const ORIENTATION_CHECK_MAX_COMMAND: f32 = 0.75;
const ORIENTATION_CHECK_PERIOD: Duration = Duration::from_millis(20);
const SERVO_TELEMETRY_TIMEOUT: Duration = Duration::from_millis(250);
/// Host polling cadence for the ICM-42688-P output registers.
const IMU_SAMPLE_PERIOD: Duration = Duration::from_millis(10);
/// BMP280 and ADC telemetry are acquired at the same 100 Hz cadence as IMU.
const SENSOR_SAMPLE_PERIOD: Duration = Duration::from_millis(10);
/// MAX17048 telemetry is intentionally low-rate to minimize shared-I²C load.
const BATTERY_SAMPLE_PERIOD: Duration = Duration::from_millis(250);
/// The continuity-test dividers scale their source voltage by 2.
const ADC_DIVIDER_SCALE: f32 = 2.0;
/// An intact igniter produces roughly 5 V before the divider (2.5 V ADC).
const CONTINUITY_THRESHOLD_VOLTS: f32 = 1.0;

trait ApiTransmitter {
    fn transmit_api<T: ApiEncodeDecode>(&mut self, address: u32, data: &T);
}

impl ApiTransmitter for CanDriver<'_> {
    fn transmit_api<T: ApiEncodeDecode>(&mut self, address: u32, data: &T) {
        let encoded = data.api_encode().expect("CAN API encoding failed");
        let frame = Frame::new(address, EnumSet::new(), encoded.as_slice())
            .expect("CAN API frame exceeded eight bytes");
        // A bare bench bus has no peer to acknowledge frames. Treat a timeout
        // as an offline device, not a reason to stop the flight-control task.
        let _ = self.transmit(&frame, 10);
    }
}

fn normalized_position_to_encoder(configuration: &ServoConfiguration, position: f32) -> u16 {
    let counts_per_degree = ENCODER_COUNTS_PER_TURN as f32 / 360.0;
    let delta = (counts_per_degree
        * configuration.max_turn_degrees.max(0.01)
        * position.clamp(-1.0, 1.0)) as i32;
    (configuration.zero_encoder_count as i32 + delta).rem_euclid(ENCODER_COUNTS_PER_TURN) as u16
}

/// Applies the flight-controller-level direction setting. `reverse_motor`
/// remains a servo-firmware configuration and is intentionally not used here.
fn flight_position_to_encoder(configuration: &ServoConfiguration, position: f32) -> u16 {
    normalized_position_to_encoder(
        configuration,
        if configuration.reverse_control {
            -position
        } else {
            position
        },
    )
}

/// Keep the configuration protocol work out of the flight-control loop's stack
/// frame. `api_encode` uses temporary bincode buffers, so this must not be
/// inlined into the already large CAN task.
#[inline(never)]
fn apply_servo_configuration(
    can: &mut CanDriver<'static>,
    address: u16,
    configuration: &ServoConfiguration,
) {
    can.transmit_api(
        address as u32,
        &ServoCommandFrame::SetDataRate { data_rate: 1000 },
    );
    can.transmit_api(
        address as u32,
        &ServoCommandFrame::SetGeneralConfig1 {
            reverse_motor: configuration.reverse_motor,
            duty_cycle_limit: configuration.duty_cycle_limit,
        },
    );
    can.transmit_api(
        address as u32,
        &ServoCommandFrame::SetPositionPGain {
            p: configuration.position_p,
        },
    );
    can.transmit_api(
        address as u32,
        &ServoCommandFrame::SetPositionIGain {
            i: configuration.position_i,
            cycles_to_max_out: 1000,
        },
    );
    can.transmit_api(
        address as u32,
        &ServoCommandFrame::SetPositionDGain {
            d: configuration.position_d,
        },
    );
}

/// Selects a servo by its persistent identity and restores its logical CAN
/// channel. Channel assignments live in the servo and must be repeated after
/// the servo controller powers up.
#[inline(never)]
fn assign_servo_channel(
    can: &mut CanDriver<'static>,
    master: u16,
    slave: u16,
    device: &ServoDeviceId,
) {
    can.transmit_api(
        0,
        &GeneralCommandFrame::ChipID1 {
            chip_id1: device.chip_id1,
        },
    );
    can.transmit_api(
        0,
        &GeneralCommandFrame::ChipID2 {
            chip_id2: device.chip_id2,
        },
    );
    can.transmit_api(0, &GeneralCommandFrame::SetChannel { master, slave });
}

fn reset_and_assign_servo_channels(
    can: &mut CanDriver<'static>,
    x_address: (u16, u16),
    y_address: (u16, u16),
    x_device: Option<&ServoDeviceId>,
    y_device: Option<&ServoDeviceId>,
) {
    // Every servo accepts this broadcast even when it still has an address
    // from a previous host session. The servo disables its output before
    // forgetting that address.
    can.transmit_api(0, &GeneralCommandFrame::UnassignChannels);
    FreeRtos::delay_ms(20);
    if let Some(device) = x_device {
        assign_servo_channel(can, x_address.0, x_address.1, device);
    }
    if let Some(device) = y_device {
        assign_servo_channel(can, y_address.0, y_address.1, device);
    }
}

fn discover_servos(can: &mut CanDriver<'static>) -> Vec<ServoDeviceId> {
    can.transmit_api(0, &GeneralCommandFrame::RequestChipId1);
    let mut chip_id1s = HashSet::new();
    let started = Instant::now();
    while started.elapsed() < Duration::from_millis(100) {
        if let Ok(frame) = can.receive(5) {
            if frame.identifier() == 1 {
                if let Ok(GeneralResponseFrame::ChipID1 { chip_id1 }) =
                    GeneralResponseFrame::api_decode(frame.data())
                {
                    chip_id1s.insert(chip_id1);
                }
            }
        }
    }

    let mut devices = HashSet::new();
    for chip_id1 in chip_id1s {
        can.transmit_api(0, &GeneralCommandFrame::RequestChipId2 { chip_id1 });
        let started = Instant::now();
        while started.elapsed() < Duration::from_millis(100) {
            if let Ok(frame) = can.receive(5) {
                if frame.identifier() == 1 {
                    if let Ok(GeneralResponseFrame::ChipID2 { chip_id2 }) =
                        GeneralResponseFrame::api_decode(frame.data())
                    {
                        devices.insert(ServoDeviceId { chip_id1, chip_id2 });
                        break;
                    }
                }
            }
        }
    }
    devices.into_iter().collect()
}

fn encoder_to_normalized_position(configuration: &ServoConfiguration, encoder: u16) -> f32 {
    let half_turn = ENCODER_COUNTS_PER_TURN / 2;
    let delta = (encoder as i32 - configuration.zero_encoder_count as i32 + half_turn)
        .rem_euclid(ENCODER_COUNTS_PER_TURN)
        - half_turn;
    delta as f32
        / ((ENCODER_COUNTS_PER_TURN as f32 / 360.0) * configuration.max_turn_degrees.max(0.01))
}

fn validate_servo_configuration(configuration: &ServoConfiguration) -> anyhow::Result<()> {
    anyhow::ensure!(
        configuration.zero_encoder_count < ENCODER_COUNTS_PER_TURN as u16,
        "servo zero must be between 0 and 16383"
    );
    anyhow::ensure!(
        configuration.max_turn_degrees.is_finite()
            && (0.0..=180.0).contains(&configuration.max_turn_degrees)
            && configuration.max_turn_degrees > 0.0,
        "servo turn range must be between 0 and 180 degrees"
    );
    anyhow::ensure!(
        [
            configuration.position_p,
            configuration.position_i,
            configuration.position_d,
        ]
        .iter()
        .all(|gain| gain.is_finite() && *gain >= 0.0),
        "servo PID gains must be finite and non-negative"
    );
    anyhow::ensure!(
        configuration.duty_cycle_limit.is_finite()
            && (0.0..=1.0).contains(&configuration.duty_cycle_limit),
        "servo duty-cycle limit must be between 0 and 1"
    );
    Ok(())
}

fn validate_inertia_configuration(configuration: &InertiaConfiguration) -> anyhow::Result<()> {
    anyhow::ensure!(
        configuration.mass_g.is_finite() && configuration.mass_g > 0.0,
        "rocket mass must be finite and positive"
    );
    anyhow::ensure!(
        configuration.center_of_mass_offset_mm.is_finite()
            && configuration.center_of_mass_offset_mm >= 0.0,
        "center-of-mass offset must be finite and non-negative"
    );
    anyhow::ensure!(
        configuration
            .moment_of_inertia_kgm2
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0),
        "moments of inertia must be finite and non-negative"
    );
    Ok(())
}

fn validate_simulation_configuration(
    configuration: &SimulationConfiguration,
) -> anyhow::Result<()> {
    let non_negative = [
        configuration.thrust_newtons,
        configuration.burn_time_s,
        configuration.max_tvc_angle_degrees,
        configuration.max_tvc_rate_degrees_per_s,
        configuration.drag_coefficient_axial,
        configuration.drag_coefficient_sideways,
        configuration.reference_area_m2,
    ];
    anyhow::ensure!(
        non_negative
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0),
        "simulation magnitudes must be finite and non-negative"
    );
    anyhow::ensure!(
        configuration
            .tvc_misalignment_degrees
            .iter()
            .chain(configuration.wind_mps.iter())
            .chain(configuration.initial_tilt_degrees.iter())
            .all(|value| value.is_finite()),
        "simulation vectors must contain only finite values"
    );
    anyhow::ensure!(
        configuration.center_of_pressure_offset_mm.is_finite(),
        "center-of-pressure offset must be finite"
    );
    anyhow::ensure!(
        configuration
            .pid_gains
            .iter()
            .all(|gain| gain.is_finite() && *gain >= 0.0),
        "flight PID gains must be finite and non-negative"
    );
    Ok(())
}

fn load_wifi_credentials(nvs: &EspDefaultNvs) -> Option<WifiCredentials> {
    let length = nvs.blob_len(WIFI_CONFIGURATION_NVS_KEY).ok().flatten()?;
    let mut bytes = vec![0; length];
    let bytes = nvs
        .get_blob(WIFI_CONFIGURATION_NVS_KEY, &mut bytes)
        .ok()
        .flatten()?;
    let credentials: WifiCredentials = serde_json::from_slice(bytes).ok()?;
    if credentials.ssid.is_empty()
        || credentials.ssid.len() > 32
        || (!credentials.password.is_empty() && !(8..=63).contains(&credentials.password.len()))
    {
        None
    } else {
        Some(credentials)
    }
}

#[derive(Clone)]
enum CalibrationAction {
    Apply(ServoAxis, ServoConfiguration),
    CaptureZero(ServoAxis),
    Disable(ServoAxis),
    Position(ServoAxis, f32),
    StartTest(ServoAxis, ServoConfiguration),
    StartHil(SimulationConfiguration),
    AssignDevice(ServoAxis, ServoDeviceId),
    StopOrientationCheck,
    Save,
}

fn enqueue_calibration_action(
    sender: &mpsc::SyncSender<CalibrationAction>,
    action: CalibrationAction,
) -> anyhow::Result<()> {
    match sender.try_send(action) {
        Ok(()) => Ok(()),
        Err(mpsc::TrySendError::Full(_)) => {
            anyhow::bail!("flight-control command queue is busy")
        }
        Err(mpsc::TrySendError::Disconnected(_)) => {
            anyhow::bail!("flight-control task is offline")
        }
    }
}

fn motor_action_active(state: &api::State) -> bool {
    state.servo_calibration.test_running
        || state.servo_calibration.orientation_check_running
        || state.hil_simulation.running
}

fn any_servo_enabled(state: &api::State) -> bool {
    state.servo_calibration.x_enabled || state.servo_calibration.y_enabled
}

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let state = Arc::new(Mutex::new(api::State::default()));

    let test_data = Arc::new(Mutex::new(api::ServoTestResult::default()));
    let inertia_capture_result = Arc::new(Mutex::new(api::InertiaCaptureResult::default()));
    let hil_result = Arc::new(Mutex::new(api::HilSimulationResult::default()));
    let inertia_capture_request = Arc::new(AtomicBool::new(false));
    let restart_request = Arc::new(AtomicBool::new(false));
    let (calibration_action_sender, calibration_action_receiver) =
        mpsc::sync_channel::<CalibrationAction>(8);

    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();

    // Wi-Fi's PHY calibration and servo calibration persistence both use NVS.
    // Take/initialize the default partition before constructing EspWifi so RF
    // calibration does not fall back to the slow, non-persistent path.
    let servo_nvs = Arc::new(Mutex::new(
        EspDefaultNvs::new(EspDefaultNvsPartition::take().unwrap(), "servo_cfg", true).unwrap(),
    ));
    // Do not keep the first MutexGuard alive across the body of `if let`:
    // get_blob needs to take the same mutex and would otherwise deadlock.
    let saved_configuration_length = { servo_nvs.lock().unwrap().blob_len("axes").ok().flatten() };
    if let Some(length) = saved_configuration_length {
        let mut bytes = vec![0; length];
        let saved_configuration = {
            servo_nvs
                .lock()
                .unwrap()
                .get_blob("axes", &mut bytes)
                .ok()
                .flatten()
                .map(Vec::from)
        };
        if let Some(bytes) = saved_configuration {
            if let Ok(mut calibration) = serde_json::from_slice::<ServoCalibrationState>(&bytes) {
                if validate_servo_configuration(&calibration.x).is_err() {
                    calibration.x = ServoConfiguration::default();
                }
                if validate_servo_configuration(&calibration.y).is_err() {
                    calibration.y = ServoConfiguration::default();
                }
                calibration.x_enabled = false;
                calibration.y_enabled = false;
                calibration.detected_x = false;
                calibration.detected_y = false;
                calibration.test_running = false;
                calibration.orientation_check_running = false;
                calibration.orientation_check_command = [0.0; 2];
                calibration.discovered_devices.clear();
                calibration.test_result_revision = 0;
                if calibration.x_device == calibration.y_device && calibration.x_device.is_some() {
                    calibration.y_device = None;
                }
                state.lock().unwrap().servo_calibration = calibration;
            }
        }
    }
    let saved_inertia_length = { servo_nvs.lock().unwrap().blob_len("inertia").ok().flatten() };
    if let Some(length) = saved_inertia_length {
        let mut bytes = vec![0; length];
        let saved_inertia = {
            servo_nvs
                .lock()
                .unwrap()
                .get_blob("inertia", &mut bytes)
                .ok()
                .flatten()
                .map(Vec::from)
        };
        if let Some(bytes) = saved_inertia {
            if let Ok(configuration) = serde_json::from_slice(&bytes) {
                if validate_inertia_configuration(&configuration).is_ok() {
                    state.lock().unwrap().inertia_configuration = configuration;
                }
            }
        }
    }
    let saved_simulation_length = {
        servo_nvs
            .lock()
            .unwrap()
            .blob_len("simulation")
            .ok()
            .flatten()
    };
    if let Some(length) = saved_simulation_length {
        let mut bytes = vec![0; length];
        let saved_simulation = servo_nvs
            .lock()
            .unwrap()
            .get_blob("simulation", &mut bytes)
            .ok()
            .flatten()
            .map(Vec::from);
        if let Some(bytes) = saved_simulation {
            if let Ok(configuration) = serde_json::from_slice(&bytes) {
                if validate_simulation_configuration(&configuration).is_ok() {
                    state.lock().unwrap().simulation_configuration = configuration;
                }
            }
        }
    }
    let saved_checklist_length = {
        servo_nvs
            .lock()
            .unwrap()
            .blob_len(PRELAUNCH_CHECKLIST_NVS_KEY)
            .ok()
            .flatten()
    };
    if let Some(length) = saved_checklist_length {
        let mut bytes = vec![0; length];
        let saved = servo_nvs
            .lock()
            .unwrap()
            .get_blob(PRELAUNCH_CHECKLIST_NVS_KEY, &mut bytes)
            .ok()
            .flatten()
            .map(Vec::from);
        if let Some(bytes) = saved {
            if let Ok(checklist) = serde_json::from_slice::<PrelaunchChecklistState>(&bytes) {
                if checklist.completed_steps <= PrelaunchChecklistStep::COUNT {
                    state.lock().unwrap().prelaunch_checklist = checklist;
                }
            }
        }
    }
    let saved_pyro_length = {
        servo_nvs
            .lock()
            .unwrap()
            .blob_len(PYRO_CONFIGURATION_NVS_KEY)
            .ok()
            .flatten()
    };
    if let Some(length) = saved_pyro_length {
        let mut bytes = vec![0; length];
        let saved = servo_nvs
            .lock()
            .unwrap()
            .get_blob(PYRO_CONFIGURATION_NVS_KEY, &mut bytes)
            .ok()
            .flatten()
            .map(Vec::from);
        if let Some(bytes) = saved {
            if let Ok(configuration) = serde_json::from_slice::<PyroConfiguration>(&bytes) {
                if (1..=10_000).contains(&configuration.parachute_duration_ms)
                    && (1..=10_000).contains(&configuration.igniter_duration_ms)
                {
                    state.lock().unwrap().pyro_configuration = configuration;
                }
            }
        }
    }
    let wifi_credentials = { load_wifi_credentials(&servo_nvs.lock().unwrap()) };
    if let Some(credentials) = &wifi_credentials {
        state.lock().unwrap().configured_wifi_ssid = credentials.ssid.clone();
    }

    let mut voltage_regulator = VoltageRegulator::new(peripherals.pins.gpio21);
    voltage_regulator.on().unwrap();

    let _button = esp_idf_hal::gpio::PinDriver::input(peripherals.pins.gpio15, Pull::Up).unwrap();

    let mut pyro1 = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio5).unwrap();
    pyro1.set_low().unwrap();

    let mut pyro2 = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio7).unwrap();
    pyro2.set_low().unwrap();

    // One task owns both firing GPIOs. This makes pulses mutually exclusive
    // and prevents duplicate HTTP requests from creating delayed extra pulses.
    let (pyro_pulse_sender, pyro_pulse_receiver) = mpsc::sync_channel::<(u8, u16)>(1);
    let pyro_pulse_state = state.clone();
    let default_pyro_thread_config = esp_idf_hal::task::thread::ThreadSpawnConfiguration::get();
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(c"pyro-pulse"),
        stack_size: 8 * 1024,
        inherit: true,
        pin_to_core: Some(Core::Core0),
        ..Default::default()
    }
    .set()
    .unwrap();
    thread::spawn(move || {
        while let Ok((channel, duration_ms)) = pyro_pulse_receiver.recv() {
            let raised = match channel {
                1 => pyro1.set_high(),
                2 => pyro2.set_high(),
                _ => continue,
            };
            if let Err(error) = raised {
                error!("PYR{} could not drive high: {:?}", channel, error);
            } else {
                info!("PYR{} active for {} ms", channel, duration_ms);
                FreeRtos::delay_ms(duration_ms as u32);
            }
            let lowered = match channel {
                1 => pyro1.set_low(),
                2 => pyro2.set_low(),
                _ => unreachable!(),
            };
            if let Err(error) = lowered {
                error!("PYR{} could not drive low: {:?}", channel, error);
            }
            let mut state = pyro_pulse_state.lock().unwrap();
            if channel == 1 {
                state.pyro.channel1.fire = false;
            } else {
                state.pyro.channel2.fire = false;
            }
        }
    });
    if let Some(default_pyro_thread_config) = default_pyro_thread_config {
        default_pyro_thread_config.set().unwrap();
    }

    // Keep /WP and /HOLD high for ordinary single-SPI operation and retain
    // every driver for the lifetime of the external NAND store.
    let mut _flash_wp = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio14).unwrap();
    let mut _flash_hold = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio9).unwrap();
    _flash_wp.set_high().unwrap();
    _flash_hold.set_high().unwrap();
    let flash_spi = SpiDeviceDriver::new_single(
        peripherals.spi2,
        peripherals.pins.gpio12,
        peripherals.pins.gpio11,
        Some(peripherals.pins.gpio13),
        Some(peripherals.pins.gpio10),
        &SpiDriverConfig::new(),
        &SpiConfig::new().baudrate(10_u32.MHz().into()),
    )
    .unwrap();
    let mut external_flash =
        external_flash::ArtifactStore::new(external_flash::W25N01GV::new(flash_spi));
    let external_flash_status = external_flash
        .reset()
        .and_then(|_| external_flash.read_id())
        .and_then(|id| {
            external_flash
                .clear_write_protection()
                .map(|protection| (id, protection))
        });
    match external_flash_status {
        Ok((id, protection)) => info!(
            "External NAND ready: JEDEC {:02x?}, protection {:02x}",
            id, protection
        ),
        Err(error) => warn!("External NAND unavailable: {:?}", error),
    }
    let _external_flash = external_flash;

    let i2c = esp_idf_hal::i2c::I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio17,
        peripherals.pins.gpio18,
        &esp_idf_hal::i2c::I2cConfig::default(),
    )
    .unwrap();
    let i2c = SharedI2c::new(i2c);
    let mut max17048 = Max17048::new(i2c.clone());

    let battery_state = state.clone();
    let default_battery_thread_config = esp_idf_hal::task::thread::ThreadSpawnConfiguration::get();
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(c"battery-sampler"),
        stack_size: 8 * 1024,
        inherit: true,
        pin_to_core: Some(Core::Core0),
        ..Default::default()
    }
    .set()
    .unwrap();
    thread::spawn(move || {
        let mut failure_reported = false;
        loop {
            match (max17048.voltage(), max17048.soc()) {
                (Ok(voltage), Ok(soc)) => {
                    battery_state.lock().unwrap().battery = BatteryState {
                        present: true,
                        voltage,
                        soc: soc.clamp(0.0, 100.0),
                    };
                    failure_reported = false;
                }
                (Err(error), _) | (_, Err(error)) => {
                    if !failure_reported {
                        warn!("MAX17048 telemetry unavailable: {:?}", error);
                        failure_reported = true;
                    }
                    battery_state.lock().unwrap().battery.present = false;
                }
            }
            FreeRtos::delay_ms(BATTERY_SAMPLE_PERIOD.as_millis() as u32);
        }
    });
    if let Some(default_battery_thread_config) = default_battery_thread_config {
        default_battery_thread_config.set().unwrap();
    }

    #[allow(deprecated)]
    let mut led_driver: LedDriver<'static> =
        LedDriver::new(peripherals.pins.gpio8, peripherals.rmt.channel0).unwrap();
    led_driver.set_rgb(20, 0, 0).unwrap();

    let wifi_configuration = wifi_credentials
        .map(|credentials| api::WifiConnectionConfiguration {
            connection_type: api::WifiConnectionType::ConnectToExternal,
            credentials,
        })
        .unwrap_or_default();

    let _wifi = WiFi::new(
        wifi_configuration,
        peripherals.modem,
        sysloop.clone(),
        state.clone(),
    )
    .unwrap();

    // Green means the configured network is connected; amber means the
    // recovery access point is serving the control panel. Red is shown while
    // connection is still being attempted above.
    match state.lock().unwrap().wifi_state.connection_type {
        WifiConnectionType::ConnectToExternal => led_driver.set_rgb(0, 20, 0).unwrap(),
        _ => led_driver.set_rgb(10, 10, 0).unwrap(),
    }

    let state_ = state.clone();

    let calibration_action_sender_ = calibration_action_sender.clone();
    let calibration_state = state.clone();
    // Persistence is intentionally owned by a separate task. The CAN/HIL task
    // must never serialize JSON or enter NVS: both have deep ESP-IDF stacks.
    let (servo_save_sender, servo_save_receiver) = mpsc::sync_channel::<ServoCalibrationState>(1);
    let servo_save_nvs = servo_nvs.clone();
    let default_persistence_thread_config =
        esp_idf_hal::task::thread::ThreadSpawnConfiguration::get();
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(c"servo-save"),
        stack_size: 32 * 1024,
        inherit: true,
        pin_to_core: Some(Core::Core0),
        ..Default::default()
    }
    .set()
    .unwrap();
    thread::spawn(move || {
        while let Ok(mut configuration) = servo_save_receiver.recv() {
            // Runtime actuation is never persisted, regardless of the state at
            // the moment the save button was pressed.
            configuration.x_enabled = false;
            configuration.y_enabled = false;
            configuration.test_running = false;
            configuration.orientation_check_running = false;
            configuration.orientation_check_command = [0.0; 2];
            match serde_json::to_vec(&configuration) {
                Ok(bytes) => match servo_save_nvs.lock().unwrap().set_blob("axes", &bytes) {
                    Ok(()) => {}
                    Err(error) => warn!("Unable to save servo configuration: {:?}", error),
                },
                Err(error) => warn!("Unable to serialize servo configuration: {:?}", error),
            }
        }
    });
    if let Some(default_persistence_thread_config) = default_persistence_thread_config {
        default_persistence_thread_config.set().unwrap();
    }
    let inertia_capture_request_ = inertia_capture_request.clone();
    let inertia_capture_state = state.clone();
    let inertia_nvs = servo_nvs.clone();
    let servo_save_sender_ = servo_save_sender.clone();
    let hil_action_sender = calibration_action_sender.clone();
    let wifi_nvs = servo_nvs.clone();
    let checklist_nvs = servo_nvs.clone();
    let pyro_configuration_state = state.clone();
    let wifi_state = state.clone();
    let restart_request_ = restart_request.clone();
    let command_handler = move |c: &api::Command| -> anyhow::Result<()> {
        match c {
            Command::Reset => {
                restart_request_.store(true, Ordering::Release);
            }
            Command::SetWifi { ssid, password } => {
                anyhow::ensure!(!ssid.is_empty(), "Wi-Fi network name cannot be empty");
                anyhow::ensure!(
                    ssid.len() <= 32,
                    "Wi-Fi network name must be 32 bytes or fewer"
                );
                anyhow::ensure!(
                    password.is_empty() || (8..=63).contains(&password.len()),
                    "Wi-Fi password must be empty or 8–63 bytes"
                );

                let credentials = WifiCredentials {
                    ssid: ssid.clone(),
                    password: password.clone(),
                };
                let bytes = serde_json::to_vec(&credentials)?;
                wifi_nvs
                    .lock()
                    .unwrap()
                    .set_blob(WIFI_CONFIGURATION_NVS_KEY, &bytes)?;
                // Do not expose the password through the state endpoint.
                wifi_state.lock().unwrap().configured_wifi_ssid = ssid.clone();
            }
            Command::SetServoConfiguration {
                axis,
                configuration,
            } => {
                validate_servo_configuration(configuration)?;
                let mut calibration = calibration_state.lock().unwrap();
                anyhow::ensure!(
                    !motor_action_active(&calibration) && !any_servo_enabled(&calibration),
                    "servo configuration cannot change during an active motor operation"
                );
                let locked_step = if *axis == ServoAxis::Y { 0 } else { 1 };
                anyhow::ensure!(calibration.prelaunch_checklist.completed_steps <= locked_step,
                    "this servo configuration is signed off; return to its checklist step before changing it");
                enqueue_calibration_action(
                    &calibration_action_sender_,
                    CalibrationAction::Apply(*axis, configuration.clone()),
                )?;
                if *axis == ServoAxis::X {
                    calibration.servo_calibration.x = configuration.clone();
                } else {
                    calibration.servo_calibration.y = configuration.clone();
                }
            }
            Command::CaptureServoZero { axis } => {
                let locked_step = if *axis == ServoAxis::Y { 0 } else { 1 };
                let calibration = calibration_state.lock().unwrap();
                anyhow::ensure!(
                    !motor_action_active(&calibration) && !any_servo_enabled(&calibration),
                    "servo zero cannot be captured during an active motor operation"
                );
                anyhow::ensure!(calibration.prelaunch_checklist.completed_steps <= locked_step,
                    "this servo configuration is signed off; return to its checklist step before changing it");
                enqueue_calibration_action(
                    &calibration_action_sender_,
                    CalibrationAction::CaptureZero(*axis),
                )?;
            }
            Command::SetServoEnabled { axis, enabled } => {
                let mut calibration = calibration_state.lock().unwrap();
                anyhow::ensure!(
                    !motor_action_active(&calibration),
                    "manual servo control is unavailable during an active motor operation"
                );
                let assigned = if *axis == ServoAxis::X {
                    calibration.servo_calibration.x_device.is_some()
                } else {
                    calibration.servo_calibration.y_device.is_some()
                };
                anyhow::ensure!(!enabled || assigned, "assign this servo before enabling it");
                let detected = if *axis == ServoAxis::X {
                    calibration.servo_calibration.detected_x
                } else {
                    calibration.servo_calibration.detected_y
                };
                anyhow::ensure!(!enabled || detected, "servo feedback is unavailable");
                if !enabled {
                    enqueue_calibration_action(
                        &calibration_action_sender_,
                        CalibrationAction::Disable(*axis),
                    )?;
                }
                if *axis == ServoAxis::X {
                    calibration.servo_calibration.x_enabled = *enabled;
                } else {
                    calibration.servo_calibration.y_enabled = *enabled;
                }
            }
            Command::SetServoPosition { axis, position } => {
                anyhow::ensure!(position.is_finite(), "servo position must be finite");
                let calibration = calibration_state.lock().unwrap();
                anyhow::ensure!(
                    !motor_action_active(&calibration),
                    "manual servo control is unavailable during an active motor operation"
                );
                let enabled = if *axis == ServoAxis::X {
                    calibration.servo_calibration.x_enabled
                } else {
                    calibration.servo_calibration.y_enabled
                };
                anyhow::ensure!(enabled, "enable this servo before commanding a position");
                enqueue_calibration_action(
                    &calibration_action_sender_,
                    CalibrationAction::Position(*axis, position.clamp(-1.0, 1.0)),
                )?;
            }
            Command::StartServoPerformanceTest {
                axis,
                configuration,
            } => {
                validate_servo_configuration(configuration)?;
                let mut calibration = calibration_state.lock().unwrap();
                anyhow::ensure!(
                    !motor_action_active(&calibration) && !any_servo_enabled(&calibration),
                    "servo test cannot start during another motor operation"
                );
                let assigned = if *axis == ServoAxis::X {
                    calibration.servo_calibration.x_device.is_some()
                } else {
                    calibration.servo_calibration.y_device.is_some()
                };
                anyhow::ensure!(assigned, "assign this servo before testing it");
                let detected = if *axis == ServoAxis::X {
                    calibration.servo_calibration.detected_x
                } else {
                    calibration.servo_calibration.detected_y
                };
                anyhow::ensure!(detected, "servo feedback is unavailable");
                let locked_step = if *axis == ServoAxis::Y { 0 } else { 1 };
                anyhow::ensure!(calibration.prelaunch_checklist.completed_steps <= locked_step,
                    "this servo configuration is signed off; return to its checklist step before changing it");
                enqueue_calibration_action(
                    &calibration_action_sender_,
                    CalibrationAction::StartTest(*axis, configuration.clone()),
                )?;
                if *axis == ServoAxis::X {
                    calibration.servo_calibration.x = configuration.clone();
                } else {
                    calibration.servo_calibration.y = configuration.clone();
                }
                calibration.servo_calibration.test_running = true;
            }
            Command::AssignServoDevice { axis, device } => {
                let locked_step = if *axis == ServoAxis::Y { 0 } else { 1 };
                let calibration = calibration_state.lock().unwrap();
                anyhow::ensure!(
                    !motor_action_active(&calibration) && !any_servo_enabled(&calibration),
                    "servo assignment cannot change during an active motor operation"
                );
                anyhow::ensure!(calibration.prelaunch_checklist.completed_steps <= locked_step,
                    "this servo configuration is signed off; return to its checklist step before changing it");
                anyhow::ensure!(
                    calibration
                        .servo_calibration
                        .discovered_devices
                        .contains(device),
                    "servo is not present in the latest discovery result"
                );
                let other_device = if *axis == ServoAxis::X {
                    &calibration.servo_calibration.y_device
                } else {
                    &calibration.servo_calibration.x_device
                };
                anyhow::ensure!(
                    other_device.as_ref() != Some(device),
                    "the same servo cannot be assigned to both axes"
                );
                enqueue_calibration_action(
                    &calibration_action_sender_,
                    CalibrationAction::AssignDevice(*axis, device.clone()),
                )?;
            }
            Command::SaveServoConfigurations => {
                enqueue_calibration_action(&calibration_action_sender_, CalibrationAction::Save)?;
            }
            Command::SetServoOrientationCheck { running } => {
                let mut calibration = calibration_state.lock().unwrap();
                if *running {
                    anyhow::ensure!(
                        calibration.imu.present,
                        "cannot start servo orientation check while the IMU is offline"
                    );
                    anyhow::ensure!(
                        !motor_action_active(&calibration) && !any_servo_enabled(&calibration),
                        "cannot start the orientation check during another motor operation"
                    );
                    anyhow::ensure!(
                        calibration.servo_calibration.x_device.is_some()
                            && calibration.servo_calibration.y_device.is_some(),
                        "assign both servos before starting the orientation check"
                    );
                    anyhow::ensure!(
                        calibration.servo_calibration.detected_x
                            && calibration.servo_calibration.detected_y,
                        "feedback from both servos is required for the orientation check"
                    );
                } else {
                    enqueue_calibration_action(
                        &calibration_action_sender_,
                        CalibrationAction::StopOrientationCheck,
                    )?;
                }
                calibration.servo_calibration.orientation_check_running = *running;
                if *running {
                    calibration.servo_calibration.x_enabled = true;
                    calibration.servo_calibration.y_enabled = true;
                } else {
                    calibration.servo_calibration.orientation_check_command = [0.0; 2];
                }
            }
            Command::StartInertiaCapture => {
                // Ignore duplicate clicks while the previous five-second
                // capture is running; the sampler consumes this request.
                let capture_running = inertia_capture_state
                    .lock()
                    .unwrap()
                    .inertia_capture
                    .running;
                if !capture_running {
                    inertia_capture_request_.store(true, Ordering::Release);
                }
            }
            Command::SetInertiaConfiguration { configuration } => {
                validate_inertia_configuration(configuration)?;
                anyhow::ensure!(inertia_capture_state.lock().unwrap().prelaunch_checklist.completed_steps <= 6,
                    "moment of inertia is signed off; return to that checklist step before changing it");
                inertia_capture_state.lock().unwrap().inertia_configuration = configuration.clone();
            }
            Command::SaveInertiaConfiguration { configuration } => {
                validate_inertia_configuration(configuration)?;
                anyhow::ensure!(inertia_capture_state.lock().unwrap().prelaunch_checklist.completed_steps <= 6,
                    "moment of inertia is signed off; return to that checklist step before changing it");
                inertia_capture_state.lock().unwrap().inertia_configuration = configuration.clone();
                match serde_json::to_vec(configuration) {
                    Ok(bytes) => match inertia_nvs.lock().unwrap().set_blob("inertia", &bytes) {
                        Ok(()) => {}
                        Err(error) => warn!(
                            "Unable to save moment-of-inertia configuration: {:?}",
                            error
                        ),
                    },
                    Err(error) => warn!(
                        "Unable to serialize moment-of-inertia configuration: {:?}",
                        error
                    ),
                }
            }
            Command::SaveSimulationConfiguration { configuration } => {
                validate_simulation_configuration(configuration)?;
                anyhow::ensure!(inertia_capture_state.lock().unwrap().prelaunch_checklist.completed_steps <= 7,
                    "simulation parameters are signed off; return to that checklist step before changing them");
                inertia_capture_state
                    .lock()
                    .unwrap()
                    .simulation_configuration = configuration.clone();
                match serde_json::to_vec(configuration) {
                    Ok(bytes) => match inertia_nvs.lock().unwrap().set_blob("simulation", &bytes) {
                        Ok(()) => {}
                        Err(error) => warn!("Unable to save simulation configuration: {:?}", error),
                    },
                    Err(error) => {
                        warn!("Unable to serialize simulation configuration: {:?}", error)
                    }
                }
            }
            Command::StartHilSimulation { configuration } => {
                validate_simulation_configuration(configuration)?;
                let mut s = inertia_capture_state.lock().unwrap();
                anyhow::ensure!(
                    !motor_action_active(&s) && !any_servo_enabled(&s),
                    "HIL cannot start while another motor action is active"
                );
                anyhow::ensure!(
                    s.servo_calibration.x_device.is_some()
                        && s.servo_calibration.y_device.is_some(),
                    "assign both servos before starting HIL"
                );
                anyhow::ensure!(
                    s.servo_calibration.detected_x && s.servo_calibration.detected_y,
                    "feedback from both servos is required for HIL"
                );
                enqueue_calibration_action(
                    &hil_action_sender,
                    CalibrationAction::StartHil(configuration.clone()),
                )?;
                s.hil_simulation.running = true;
            }

            Command::SetPyroConfiguration { configuration } => {
                let mut s = pyro_configuration_state.lock().unwrap();
                anyhow::ensure!(
                    (1..=10_000).contains(&configuration.parachute_duration_ms)
                        && (1..=10_000).contains(&configuration.igniter_duration_ms),
                    "pyro pulse durations must be between 1 and 10000 ms"
                );
                if configuration.parachute_duration_ms != s.pyro_configuration.parachute_duration_ms
                {
                    anyhow::ensure!(s.prelaunch_checklist.completed_steps <= 4,
                        "parachute timing is signed off; return to its checklist step before changing it");
                }
                if configuration.igniter_duration_ms != s.pyro_configuration.igniter_duration_ms {
                    anyhow::ensure!(s.prelaunch_checklist.completed_steps <= 5,
                        "igniter timing is signed off; return to its checklist step before changing it");
                }
                s.pyro_configuration = configuration.clone();
                drop(s);
                let bytes = serde_json::to_vec(configuration)?;
                checklist_nvs
                    .lock()
                    .unwrap()
                    .set_blob(PYRO_CONFIGURATION_NVS_KEY, &bytes)?;
            }
            Command::TestPyro { channel } => {
                let channel = *channel;
                let duration_ms = {
                    let mut s = pyro_configuration_state.lock().unwrap();
                    anyhow::ensure!(channel == 1 || channel == 2, "unknown pyro channel");
                    anyhow::ensure!(
                        !s.pyro.channel1.fire && !s.pyro.channel2.fire,
                        "another pyro pulse is already active"
                    );
                    if channel == 2 {
                        anyhow::ensure!(
                            s.prelaunch_checklist.active
                                && s.prelaunch_checklist.completed_steps == 5,
                            "PYR2 test is allowed only on its active checklist step"
                        );
                    }
                    if channel == 1 {
                        s.pyro.channel1.fire = true;
                        s.pyro_configuration.parachute_duration_ms
                    } else {
                        s.pyro.channel2.fire = true;
                        s.pyro_configuration.igniter_duration_ms
                    }
                };
                if pyro_pulse_sender.try_send((channel, duration_ms)).is_err() {
                    let mut s = pyro_configuration_state.lock().unwrap();
                    if channel == 1 {
                        s.pyro.channel1.fire = false;
                    } else {
                        s.pyro.channel2.fire = false;
                    }
                    anyhow::bail!("pyro pulse task is unavailable");
                }
            }
            Command::StartPrelaunchChecklist => {
                let mut s = pyro_configuration_state.lock().unwrap();
                s.prelaunch_checklist = PrelaunchChecklistState {
                    active: true,
                    completed_steps: 0,
                };
                let bytes = serde_json::to_vec(&s.prelaunch_checklist)?;
                drop(s);
                checklist_nvs
                    .lock()
                    .unwrap()
                    .set_blob(PRELAUNCH_CHECKLIST_NVS_KEY, &bytes)?;
            }
            Command::AbortPrelaunchChecklist => {
                let mut s = pyro_configuration_state.lock().unwrap();
                s.prelaunch_checklist = PrelaunchChecklistState::default();
                let bytes = serde_json::to_vec(&s.prelaunch_checklist)?;
                drop(s);
                checklist_nvs
                    .lock()
                    .unwrap()
                    .set_blob(PRELAUNCH_CHECKLIST_NVS_KEY, &bytes)?;
            }
            Command::SignOffPrelaunchStep { step } => {
                let mut s = pyro_configuration_state.lock().unwrap();
                anyhow::ensure!(
                    s.prelaunch_checklist.active,
                    "start the pre-launch checklist first"
                );
                anyhow::ensure!(
                    s.prelaunch_checklist.completed_steps == step.index(),
                    "steps must be signed off in order"
                );
                if *step == PrelaunchChecklistStep::CheckParachuteConnection {
                    anyhow::ensure!(s.pyro.channel1.continuity && !s.pyro.channel2.continuity,
                        "PYR1 must show continuity and PYR2 must be open before signing off parachute-only connection");
                }
                s.prelaunch_checklist.completed_steps += 1;
                let bytes = serde_json::to_vec(&s.prelaunch_checklist)?;
                drop(s);
                checklist_nvs
                    .lock()
                    .unwrap()
                    .set_blob(PRELAUNCH_CHECKLIST_NVS_KEY, &bytes)?;
            }
            Command::ReturnToPrelaunchStep { step } => {
                let mut s = pyro_configuration_state.lock().unwrap();
                anyhow::ensure!(
                    s.prelaunch_checklist.active,
                    "start the pre-launch checklist first"
                );
                anyhow::ensure!(
                    step.index() < s.prelaunch_checklist.completed_steps,
                    "that step has not been signed off"
                );
                s.prelaunch_checklist.completed_steps = step.index();
                let bytes = serde_json::to_vec(&s.prelaunch_checklist)?;
                drop(s);
                checklist_nvs
                    .lock()
                    .unwrap()
                    .set_blob(PRELAUNCH_CHECKLIST_NVS_KEY, &bytes)?;
            }
        }
        Ok(())
    };

    let _server = Server::new(
        state.clone(),
        test_data.clone(),
        inertia_capture_result.clone(),
        hil_result.clone(),
        command_handler,
    )
    .unwrap();

    let mut mdns = esp_idf_svc::mdns::EspMdns::take().unwrap();
    mdns.set_hostname("lrc").unwrap();
    mdns.set_instance_name("Light Robot Core web server")
        .unwrap();
    mdns.add_service(None, "_http", "_tcp", 80, &[("board", "{esp32}")])
        .unwrap();

    // A high-resolution ESP timer produces the 100 Hz cadence. It only
    // enqueues a one-slot tick, so an overloaded consumer drops stale samples
    // rather than running a catch-up burst.
    let (imu_tick_sender, imu_tick_receiver) = mpsc::sync_channel::<()>(1);
    let imu_timer_service = EspTaskTimerService::new().unwrap();
    let imu_timer = imu_timer_service
        .timer(move || {
            let _ = imu_tick_sender.try_send(());
        })
        .unwrap();
    imu_timer.every(IMU_SAMPLE_PERIOD).unwrap();

    let imu_i2c = i2c.clone();
    let imu_state = state.clone();
    let imu_inertia_capture_request = inertia_capture_request.clone();
    let imu_inertia_capture_result = inertia_capture_result.clone();
    let default_imu_thread_config = esp_idf_hal::task::thread::ThreadSpawnConfiguration::get();
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(c"imu-sampler"),
        stack_size: 16 * 1024,
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }
    .set()
    .unwrap();
    thread::spawn(move || {
        let mut imu = Imu::new(imu_i2c);
        let mut initialized = false;
        let mut sample_count = 0_u32;
        let mut first_sample_at = None::<Instant>;
        let mut failure_reported = false;
        let mut capture_started_at = None::<Instant>;
        let mut capture = InertiaCaptureResult::default();
        while imu_tick_receiver.recv().is_ok() {
            if !initialized {
                match imu.initialize() {
                    Ok(()) => {
                        initialized = true;
                        sample_count = 0;
                        first_sample_at = None;
                    }
                    Err(error) => {
                        if !failure_reported {
                            warn!("ICM-42688-P unavailable: {:?}", error);
                            failure_reported = true;
                        }
                        continue;
                    }
                }
            }

            let next_sample_count = sample_count.wrapping_add(1);
            match imu.read(next_sample_count) {
                Ok(mut reading) => {
                    failure_reported = false;
                    let first = *first_sample_at.get_or_insert_with(Instant::now);
                    sample_count = next_sample_count;
                    reading.average_rate_hz = if sample_count > 1 {
                        (sample_count - 1) as f32 / first.elapsed().as_secs_f32()
                    } else {
                        0.0
                    };
                    let gyro_x_radps = reading.angular_velocity_radps[0];
                    let gyro_y_radps = reading.angular_velocity_radps[1];
                    imu_state.lock().unwrap().imu = reading;

                    if imu_inertia_capture_request.swap(false, Ordering::AcqRel) {
                        capture_started_at = Some(Instant::now());
                        capture = InertiaCaptureResult::default();
                        imu_state.lock().unwrap().inertia_capture.running = true;
                    }
                    if let Some(started_at) = capture_started_at {
                        capture.samples.push(InertiaGyroSample {
                            time_us: started_at.elapsed().as_micros() as u32,
                            gyro_x_radps,
                            gyro_y_radps,
                        });
                        if started_at.elapsed() >= INERTIA_CAPTURE_DURATION {
                            let (mut x_min, mut x_max) = (f32::INFINITY, f32::NEG_INFINITY);
                            let (mut y_min, mut y_max) = (f32::INFINITY, f32::NEG_INFINITY);
                            for sample in &capture.samples {
                                x_min = x_min.min(sample.gyro_x_radps);
                                x_max = x_max.max(sample.gyro_x_radps);
                                y_min = y_min.min(sample.gyro_y_radps);
                                y_max = y_max.max(sample.gyro_y_radps);
                            }
                            capture.dominant_axis = Some(if x_max - x_min >= y_max - y_min {
                                InertiaAxis::X
                            } else {
                                InertiaAxis::Y
                            });
                            capture.measured_rate_hz = if capture.samples.len() > 1 {
                                (capture.samples.len() - 1) as f32
                                    / started_at.elapsed().as_secs_f32()
                            } else {
                                0.0
                            };
                            *imu_inertia_capture_result.lock().unwrap() = capture.clone();
                            let mut state = imu_state.lock().unwrap();
                            state.inertia_capture.running = false;
                            state.inertia_capture.result_revision =
                                state.inertia_capture.result_revision.wrapping_add(1);
                            capture_started_at = None;
                        }
                    }
                }
                Err(error) => {
                    if !failure_reported {
                        warn!(
                            "ICM-42688-P read failed: {:?}; retrying initialization",
                            error
                        );
                        failure_reported = true;
                    }
                    initialized = false;
                    imu_state.lock().unwrap().imu.present = false;
                }
            }
        }
    });
    if let Some(default_imu_thread_config) = default_imu_thread_config {
        default_imu_thread_config.set().unwrap();
    }

    // Sample the barometer and passive continuity telemetry from a separate
    // one-slot 100 Hz ticker. As with the IMU, overload drops an old tick
    // instead of creating a burst of delayed samples.
    let (sensor_tick_sender, sensor_tick_receiver) = mpsc::sync_channel::<()>(1);
    let sensor_timer_service = EspTaskTimerService::new().unwrap();
    let sensor_timer = sensor_timer_service
        .timer(move || {
            let _ = sensor_tick_sender.try_send(());
        })
        .unwrap();
    sensor_timer.every(SENSOR_SAMPLE_PERIOD).unwrap();

    let sensor_i2c = i2c.clone();
    let sensor_state = state.clone();
    let sensor_adc = peripherals.adc1;
    let pyro1_test_pin = peripherals.pins.gpio4;
    let pyro2_test_pin = peripherals.pins.gpio6;
    let default_sensor_thread_config = esp_idf_hal::task::thread::ThreadSpawnConfiguration::get();
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(c"environment-sampler"),
        stack_size: 16 * 1024,
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }
    .set()
    .unwrap();
    thread::spawn(move || {
        let adc = AdcDriver::new(sensor_adc).expect("ADC1 initialization failed");
        let adc_config = AdcChannelConfig {
            attenuation: attenuation::DB_12,
            ..Default::default()
        };
        let mut pyro1_test = AdcChannelDriver::new(&adc, pyro1_test_pin, &adc_config)
            .expect("PYRO1 continuity ADC initialization failed");
        let mut pyro2_test = AdcChannelDriver::new(&adc, pyro2_test_pin, &adc_config)
            .expect("PYRO2 continuity ADC initialization failed");
        let mut barometer = Barometer::new(sensor_i2c);
        let mut barometer_initialized = false;
        let mut sample_count = 0_u32;
        let mut first_sample_at = None::<Instant>;
        let mut barometer_failure_reported = false;
        let mut adc_failure_reported = false;
        while sensor_tick_receiver.recv().is_ok() {
            if !barometer_initialized {
                match barometer.initialize() {
                    Ok(()) => {
                        barometer_initialized = true;
                        sample_count = 0;
                        first_sample_at = None;
                    }
                    Err(error) => {
                        if !barometer_failure_reported {
                            warn!("BMP280 unavailable: {:?}", error);
                            barometer_failure_reported = true;
                        }
                    }
                }
            }
            if barometer_initialized {
                let next_sample_count = sample_count.wrapping_add(1);
                match barometer.read(next_sample_count) {
                    Ok(mut reading) => {
                        barometer_failure_reported = false;
                        let first = *first_sample_at.get_or_insert_with(Instant::now);
                        sample_count = next_sample_count;
                        reading.average_rate_hz = if sample_count > 1 {
                            (sample_count - 1) as f32 / first.elapsed().as_secs_f32()
                        } else {
                            0.0
                        };
                        sensor_state.lock().unwrap().barometer = reading;
                    }
                    Err(error) => {
                        if !barometer_failure_reported {
                            warn!("BMP280 read failed; retrying initialization: {:?}", error);
                            barometer_failure_reported = true;
                        }
                        barometer_initialized = false;
                        sensor_state.lock().unwrap().barometer.present = false;
                    }
                }
            }

            match (pyro1_test.read(), pyro2_test.read()) {
                (Ok(pyro1_mv), Ok(pyro2_mv)) => {
                    let pyro1_voltage = pyro1_mv as f32 / 1000.0 * ADC_DIVIDER_SCALE;
                    let pyro2_voltage = pyro2_mv as f32 / 1000.0 * ADC_DIVIDER_SCALE;
                    let mut state = sensor_state.lock().unwrap();
                    state.pyro.channel1.test_voltage = pyro1_voltage;
                    state.pyro.channel1.continuity = pyro1_voltage >= CONTINUITY_THRESHOLD_VOLTS;
                    state.pyro.channel2.test_voltage = pyro2_voltage;
                    state.pyro.channel2.continuity = pyro2_voltage >= CONTINUITY_THRESHOLD_VOLTS;
                    adc_failure_reported = false;
                }
                (Err(error), _) | (_, Err(error)) => {
                    if !adc_failure_reported {
                        warn!("Continuity ADC read failed: {:?}", error);
                        adc_failure_reported = true;
                    }
                }
            }
        }
    });
    if let Some(default_sensor_thread_config) = default_sensor_thread_config {
        default_sensor_thread_config.set().unwrap();
    }

    // Keep deterministic flight-control work off the core that runs Wi-Fi and
    // ESP-IDF's HTTP server. This task is the sole CAN owner.
    //
    // `twai_driver_install` performs ESP-IDF driver setup synchronously. Do
    // that from the startup task, then transfer the unopened driver to the
    // dedicated CAN owner below. This keeps the install path out of the
    // FreeRTOS control task where ESP-IDF's stack canary has reported faults.
    let rs_pin = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio1).unwrap();
    let filter = esp_idf_hal::can::config::Filter::standard_allow_all();
    let timing = esp_idf_hal::can::config::Timing::B1M;
    let mode = esp_idf_hal::can::config::Mode::Normal;
    let can_configuration = esp_idf_hal::can::config::Config::new()
        .filter(filter)
        .timing(timing)
        .mode(mode);
    let can = esp_idf_hal::can::CanDriver::new(
        peripherals.can,
        peripherals.pins.gpio43,
        peripherals.pins.gpio2,
        &can_configuration,
    )
    .unwrap();

    let default_flight_thread_config = esp_idf_hal::task::thread::ThreadSpawnConfiguration::get();
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(c"flight-control"),
        // HIL keeps a sizeable physics state and capture buffer on this task.
        stack_size: 128 * 1024,
        // `esp_pthread_set_cfg` configures the spawning task; the child only
        // receives it when inheritance is explicitly enabled.
        inherit: true,
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }
    .set()
    .unwrap();
    thread::spawn(move || {
        // Keep the transceiver-enable pin configured for this task's lifetime.
        let mut rs_pin = rs_pin;
        rs_pin.set_low().unwrap();

        let mut can = can;

        can.start().unwrap();
        FreeRtos::delay_ms(1000);

        let servo1_address_master = 2u16;
        let servo1_address_slave = 12u16;
        let servo2_address_master = 3u16;
        let servo2_address_slave = 13u16;

        reset_and_assign_servo_channels(
            &mut can,
            (servo1_address_master, servo1_address_slave),
            (servo2_address_master, servo2_address_slave),
            None,
            None,
        );
        let discovered_devices = discover_servos(&mut can);
        info!("CAN ready; {} servo(s) detected", discovered_devices.len());
        state.lock().unwrap().servo_calibration.discovered_devices = discovered_devices;

        // The state was populated from NVS before CAN was initialized. Apply
        // both saved axis configurations now that the servo addresses exist.
        let startup_configuration = state.lock().unwrap().servo_calibration.clone();
        for (axis, master, slave, assigned_device) in [
            (
                ServoAxis::X,
                servo1_address_master,
                servo1_address_slave,
                startup_configuration.x_device.as_ref(),
            ),
            (
                ServoAxis::Y,
                servo2_address_master,
                servo2_address_slave,
                startup_configuration.y_device.as_ref(),
            ),
        ] {
            if let Some(device) = assigned_device {
                if startup_configuration.discovered_devices.contains(device) {
                    assign_servo_channel(&mut can, master, slave, device);
                } else {
                    warn!(
                        "Saved {} servo is unavailable; channel not assigned",
                        if axis == ServoAxis::X { "X" } else { "Y" }
                    );
                }
            }
        }
        apply_servo_configuration(&mut can, servo1_address_master, &startup_configuration.x);
        apply_servo_configuration(&mut can, servo2_address_master, &startup_configuration.y);

        let test_data_ = test_data.clone();
        let servo_save_sender_ = servo_save_sender_.clone();
        let hil_result_ = hil_result.clone();
        let restart_request = restart_request.clone();
        let mut last_orientation_command_at = Instant::now() - ORIENTATION_CHECK_PERIOD;
        let mut servo1_response_at = None::<Instant>;
        let mut servo2_response_at = None::<Instant>;
        loop {
            if restart_request.swap(false, Ordering::AcqRel) {
                can.transmit_api(servo1_address_master as u32, &ServoCommandFrame::Disable);
                can.transmit_api(servo2_address_master as u32, &ServoCommandFrame::Disable);
                FreeRtos::delay_ms(20);
                esp_idf_hal::reset::restart();
            }
            if let Ok(action) = calibration_action_receiver.try_recv() {
                match action {
                    CalibrationAction::AssignDevice(axis, device) => {
                        let (x_device, y_device) = {
                            let mut state = state_.lock().unwrap();
                            if axis == ServoAxis::X {
                                state.servo_calibration.x_device = Some(device);
                            } else {
                                state.servo_calibration.y_device = Some(device);
                            }
                            state.servo_calibration.detected_x = false;
                            state.servo_calibration.detected_y = false;
                            (
                                state.servo_calibration.x_device.clone(),
                                state.servo_calibration.y_device.clone(),
                            )
                        };
                        reset_and_assign_servo_channels(
                            &mut can,
                            (servo1_address_master, servo1_address_slave),
                            (servo2_address_master, servo2_address_slave),
                            x_device.as_ref(),
                            y_device.as_ref(),
                        );
                    }
                    CalibrationAction::StopOrientationCheck => {
                        can.transmit_api(servo1_address_master as u32, &ServoCommandFrame::Disable);
                        can.transmit_api(servo2_address_master as u32, &ServoCommandFrame::Disable);
                        let mut state = state_.lock().unwrap();
                        state.servo_calibration.x_enabled = false;
                        state.servo_calibration.y_enabled = false;
                    }
                    CalibrationAction::Apply(axis, configuration) => {
                        let address = if axis == ServoAxis::X {
                            servo1_address_master
                        } else {
                            servo2_address_master
                        };
                        apply_servo_configuration(&mut can, address, &configuration);
                    }
                    CalibrationAction::CaptureZero(axis) => {
                        let mut s = state_.lock().unwrap();
                        if axis == ServoAxis::X {
                            s.servo_calibration.x.zero_encoder_count = s.servo1.position;
                        } else {
                            s.servo_calibration.y.zero_encoder_count = s.servo2.position;
                        }
                    }
                    CalibrationAction::Disable(axis) => {
                        let address = if axis == ServoAxis::X {
                            servo1_address_master
                        } else {
                            servo2_address_master
                        };
                        can.transmit_api(address as u32, &ServoCommandFrame::Disable);
                    }
                    CalibrationAction::Position(axis, position) => {
                        let encoder = {
                            let s = state_.lock().unwrap();
                            let configuration = if axis == ServoAxis::X {
                                &s.servo_calibration.x
                            } else {
                                &s.servo_calibration.y
                            };
                            normalized_position_to_encoder(configuration, position)
                        };
                        let address = if axis == ServoAxis::X {
                            servo1_address_master
                        } else {
                            servo2_address_master
                        };
                        can.transmit_api(
                            address as u32,
                            &ServoCommandFrame::BrushedHoldPosition { position: encoder },
                        );
                    }
                    CalibrationAction::StartTest(axis, configuration) => {
                        state_.lock().unwrap().servo_calibration.test_running = true;
                        let (address, slave) = if axis == ServoAxis::X {
                            (servo1_address_master, servo1_address_slave)
                        } else {
                            (servo2_address_master, servo2_address_slave)
                        };
                        let zero = configuration.zero_encoder_count;
                        // A test always starts from precisely the configuration shown
                        // in the UI, even when the user did not press Apply first.
                        can.transmit_api(
                            address as u32,
                            &ServoCommandFrame::SetDataRate { data_rate: 1000 },
                        );
                        can.transmit_api(
                            address as u32,
                            &ServoCommandFrame::SetGeneralConfig1 {
                                reverse_motor: configuration.reverse_motor,
                                duty_cycle_limit: configuration.duty_cycle_limit,
                            },
                        );
                        can.transmit_api(
                            address as u32,
                            &ServoCommandFrame::SetPositionPGain {
                                p: configuration.position_p,
                            },
                        );
                        can.transmit_api(
                            address as u32,
                            &ServoCommandFrame::SetPositionIGain {
                                i: configuration.position_i,
                                cycles_to_max_out: 1000,
                            },
                        );
                        can.transmit_api(
                            address as u32,
                            &ServoCommandFrame::SetPositionDGain {
                                d: configuration.position_d,
                            },
                        );
                        can.transmit_api(
                            address as u32,
                            &ServoCommandFrame::BrushedHoldPosition { position: zero },
                        );
                        // Keep the CAN RX FIFO empty while the actuator settles.
                        // Otherwise pre-step telemetry is consumed at capture time and
                        // appears as a vertical spike at t=0 in the response plot.
                        let settling_started = Instant::now();
                        while settling_started.elapsed() < Duration::from_secs(2)
                            && !restart_request.load(Ordering::Acquire)
                        {
                            let _ = can.receive(1);
                        }
                        while can.receive(0).is_ok() {}
                        let target = normalized_position_to_encoder(&configuration, 0.75);
                        let mut result = ServoTestResult {
                            axis,
                            configuration: configuration.clone(),
                            samples: Vec::new(),
                        };
                        let started = Instant::now();
                        can.transmit_api(
                            address as u32,
                            &ServoCommandFrame::BrushedHoldPosition { position: target },
                        );
                        while started.elapsed() < Duration::from_millis(SERVO_TEST_CAPTURE_MS)
                            && !restart_request.load(Ordering::Acquire)
                        {
                            if let Ok(frame) = can.receive(1) {
                                if frame.identifier() == slave as u32 {
                                    if let Ok(ServoResponseFrame::State { position, .. }) =
                                        ServoResponseFrame::api_decode(frame.data())
                                    {
                                        let normalized = encoder_to_normalized_position(
                                            &configuration,
                                            position,
                                        );
                                        result.samples.push(ServoTestSample {
                                            time_us: started.elapsed().as_micros() as u32,
                                            position: normalized,
                                        });
                                    }
                                }
                            }
                        }
                        *test_data_.lock().unwrap() = result;
                        can.transmit_api(address as u32, &ServoCommandFrame::Disable);
                        let mut state = state_.lock().unwrap();
                        state.servo_calibration.test_running = false;
                        if axis == ServoAxis::X {
                            state.servo_calibration.x_enabled = false;
                        } else {
                            state.servo_calibration.y_enabled = false;
                        }
                        state.servo_calibration.test_result_revision =
                            state.servo_calibration.test_result_revision.wrapping_add(1);
                    }
                    CalibrationAction::StartHil(configuration) => {
                        // Bench-only HIL: CAN remains exclusively owned here for the
                        // full run. Servo feedback, not a gimbal model, drives TVC.
                        let (x_config, y_config, inertia) = {
                            let s = state_.lock().unwrap();
                            (
                                s.servo_calibration.x.clone(),
                                s.servo_calibration.y.clone(),
                                s.inertia_configuration.clone(),
                            )
                        };
                        apply_servo_configuration(&mut can, servo1_address_master, &x_config);
                        apply_servo_configuration(&mut can, servo2_address_master, &y_config);
                        can.transmit_api(
                            servo1_address_master as u32,
                            &ServoCommandFrame::BrushedHoldPosition {
                                position: flight_position_to_encoder(&x_config, 0.0),
                            },
                        );
                        can.transmit_api(
                            servo2_address_master as u32,
                            &ServoCommandFrame::BrushedHoldPosition {
                                position: flight_position_to_encoder(&y_config, 0.0),
                            },
                        );
                        {
                            let mut s = state_.lock().unwrap();
                            s.hil_simulation.running = true;
                            s.hil_simulation.missed_deadlines = 0;
                            s.servo_calibration.x_enabled = true;
                            s.servo_calibration.y_enabled = true;
                        }
                        let settling = Instant::now();
                        while settling.elapsed() < Duration::from_secs(2)
                            && !restart_request.load(Ordering::Acquire)
                        {
                            let _ = can.receive(1);
                        }
                        let env = Environment {
                            max_time: 8.0,
                            dt: 0.001,
                            substeps: 1,
                            wind: Vec3::new(
                                configuration.wind_mps[0],
                                configuration.wind_mps[1],
                                configuration.wind_mps[2],
                            ),
                            ..Environment::default()
                        };
                        let rocket =
                            RocketParameters::from_configuration(&inertia, &configuration, env.dt);
                        let radians = std::f32::consts::PI / 180.0;
                        let mut model = RocketState::default();
                        model.rotation = Quat::axis_angle(
                            Vec3::new(1.0, 0.0, 0.0),
                            configuration.initial_tilt_degrees[0] * radians,
                        ) * Quat::axis_angle(
                            Vec3::new(0.0, 1.0, 0.0),
                            configuration.initial_tilt_degrees[1] * radians,
                        );
                        let mut physics = NumericalSimulation;
                        let mut controller = PidControlSystem::new(
                            configuration.pid_gains[0],
                            configuration.pid_gains[1],
                            configuration.pid_gains[2],
                        );
                        controller.reset();
                        controller.set_estimated_rotation(model.rotation);
                        let mut command = ControlInputs {
                            ignition: true,
                            ..Default::default()
                        };
                        let mut actual = [0.0; 2];
                        let mut result = HilSimulationResult::default();
                        let started = Instant::now();
                        let mut cycle = 0_u32;
                        while model.time < env.max_time && !restart_request.load(Ordering::Acquire)
                        {
                            let scheduled = Duration::from_micros(cycle as u64 * 1000);
                            while started.elapsed() < scheduled {
                                std::hint::spin_loop();
                            }
                            let elapsed = started.elapsed();
                            if elapsed > scheduled + Duration::from_micros(500) {
                                result.missed_deadlines += 1;
                            }
                            while let Ok(frame) = can.receive(0) {
                                if let Ok(ServoResponseFrame::State { position, .. }) =
                                    ServoResponseFrame::api_decode(frame.data())
                                {
                                    if frame.identifier() == servo1_address_slave as u32 {
                                        servo1_response_at = Some(Instant::now());
                                        actual[0] =
                                            encoder_to_normalized_position(&x_config, position)
                                                * if x_config.reverse_control { -1.0 } else { 1.0 };
                                    }
                                    if frame.identifier() == servo2_address_slave as u32 {
                                        servo2_response_at = Some(Instant::now());
                                        actual[1] =
                                            encoder_to_normalized_position(&y_config, position)
                                                * if y_config.reverse_control { -1.0 } else { 1.0 };
                                    }
                                }
                            }
                            let control_tick = cycle % 10 == 0;
                            if control_tick {
                                command = controller.update(&SensorData {
                                    time: model.time,
                                    acceleration: model.acceleration,
                                    angular_velocity: model.angular_velocity,
                                    barometric_height: model.position.z,
                                });
                            }
                            physics.step_with_actual_tvc(&mut model, command, actual, rocket, env);
                            can.transmit_api(
                                servo1_address_master as u32,
                                &ServoCommandFrame::BrushedHoldPosition {
                                    position: flight_position_to_encoder(&x_config, command.tvc[0]),
                                },
                            );
                            can.transmit_api(
                                servo2_address_master as u32,
                                &ServoCommandFrame::BrushedHoldPosition {
                                    position: flight_position_to_encoder(&y_config, command.tvc[1]),
                                },
                            );
                            // Log at the same 100 Hz cadence as the controller.
                            // Physics and real-servo feedback still run at 1 kHz.
                            if control_tick {
                                let up = model.rotation.rotate(Vec3::UP);
                                result.samples.push(HilSimulationSample {
                                    time_us: elapsed.as_micros() as u32,
                                    scheduled_time_us: scheduled.as_micros() as u32,
                                    position_m: [
                                        model.position.x,
                                        model.position.y,
                                        model.position.z,
                                    ],
                                    orientation_xy_degrees: [
                                        (-up.y).atan2(up.z).to_degrees(),
                                        up.x.atan2(up.z).to_degrees(),
                                    ],
                                    tvc_command: command.tvc,
                                    tvc_actual: actual,
                                });
                            }
                            cycle += 1;
                        }
                        can.transmit_api(servo1_address_master as u32, &ServoCommandFrame::Disable);
                        can.transmit_api(servo2_address_master as u32, &ServoCommandFrame::Disable);
                        // Move the large 1 kHz log into shared storage. Cloning it
                        // here allocated ~180 KiB on the CAN task's internal heap
                        // and aborted immediately after a completed HIL run.
                        let missed_deadlines = result.missed_deadlines;
                        *hil_result_.lock().unwrap() = result;
                        let mut s = state_.lock().unwrap();
                        s.hil_simulation.running = false;
                        s.hil_simulation.missed_deadlines = missed_deadlines;
                        s.hil_simulation.result_revision =
                            s.hil_simulation.result_revision.wrapping_add(1);
                        s.servo_calibration.x_enabled = false;
                        s.servo_calibration.y_enabled = false;
                    }
                    CalibrationAction::Save => {
                        let configuration = state_.lock().unwrap().servo_calibration.clone();
                        if servo_save_sender_.try_send(configuration).is_err() {
                            warn!("Servo configuration save is already pending");
                        }
                    }
                }
            }

            // While the horizontal rocket is rolled around its cylindrical Z
            // axis, gravity appears in the IMU X/Y plane. Command each TVC axis
            // from that vector, then let the FC-level per-axis reversal correct
            // a mechanically mirrored installation.
            if last_orientation_command_at.elapsed() >= ORIENTATION_CHECK_PERIOD {
                let (orientation, stop_for_imu) = {
                    let mut state = state_.lock().unwrap();
                    if state.servo_calibration.orientation_check_running && state.imu.present {
                        let command = [
                            (state.imu.acceleration_mps2[0] / 9.80665).clamp(-1.0, 1.0)
                                * ORIENTATION_CHECK_MAX_COMMAND,
                            (state.imu.acceleration_mps2[1] / 9.80665).clamp(-1.0, 1.0)
                                * ORIENTATION_CHECK_MAX_COMMAND,
                        ];
                        (
                            Some((
                                command,
                                state.servo_calibration.x.clone(),
                                state.servo_calibration.y.clone(),
                            )),
                            false,
                        )
                    } else {
                        let should_stop = state.servo_calibration.orientation_check_running;
                        if should_stop {
                            state.servo_calibration.orientation_check_running = false;
                            state.servo_calibration.orientation_check_command = [0.0; 2];
                            state.servo_calibration.x_enabled = false;
                            state.servo_calibration.y_enabled = false;
                            warn!("Servo orientation check stopped because the IMU is offline");
                        }
                        (None, should_stop)
                    }
                };
                if stop_for_imu {
                    can.transmit_api(servo1_address_master as u32, &ServoCommandFrame::Disable);
                    can.transmit_api(servo2_address_master as u32, &ServoCommandFrame::Disable);
                }
                if let Some((command, x_configuration, y_configuration)) = orientation {
                    can.transmit_api(
                        servo1_address_master as u32,
                        &ServoCommandFrame::BrushedHoldPosition {
                            position: flight_position_to_encoder(&x_configuration, command[0]),
                        },
                    );
                    can.transmit_api(
                        servo2_address_master as u32,
                        &ServoCommandFrame::BrushedHoldPosition {
                            position: flight_position_to_encoder(&y_configuration, command[1]),
                        },
                    );
                    state_
                        .lock()
                        .unwrap()
                        .servo_calibration
                        .orientation_check_command = command;
                }
                last_orientation_command_at = Instant::now();
            }
            if let Ok(frame) = can.receive(0) {
                if let Ok(ServoResponseFrame::State {
                    sensor_detected,
                    position,
                    ..
                }) = ServoResponseFrame::api_decode(frame.data())
                {
                    let mut state = state_.lock().unwrap();
                    if frame.identifier() == servo1_address_slave as u32 {
                        servo1_response_at = Some(Instant::now());
                        state.servo1.position = position;
                        state.servo_calibration.detected_x = sensor_detected;
                    } else if frame.identifier() == servo2_address_slave as u32 {
                        servo2_response_at = Some(Instant::now());
                        state.servo2.position = position;
                        state.servo_calibration.detected_y = sensor_detected;
                    }
                }
            }
            let servo1_timed_out = servo1_response_at
                .is_some_and(|received_at| received_at.elapsed() >= SERVO_TELEMETRY_TIMEOUT);
            let servo2_timed_out = servo2_response_at
                .is_some_and(|received_at| received_at.elapsed() >= SERVO_TELEMETRY_TIMEOUT);
            if servo1_timed_out || servo2_timed_out {
                let mut state = state_.lock().unwrap();
                if servo1_timed_out {
                    servo1_response_at = None;
                    state.servo_calibration.detected_x = false;
                }
                if servo2_timed_out {
                    servo2_response_at = None;
                    state.servo_calibration.detected_y = false;
                }
            }

            // Yield to the Core 1 idle task between CAN polling iterations.
            FreeRtos::delay_ms(1);
        }
    });
    if let Some(default_flight_thread_config) = default_flight_thread_config {
        default_flight_thread_config.set().unwrap();
    }

    // Keep the network, web server, and mDNS owners alive on Core 0.
    loop {
        FreeRtos::delay_ms(1000);
    }
}
