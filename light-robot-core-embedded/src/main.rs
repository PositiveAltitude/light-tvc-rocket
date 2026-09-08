extern crate core;

mod imu;
mod led_driver;
mod server;
mod shared_i2c;
mod voltage_regulator;
mod wifi;

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
use esp_idf_hal::can::{CanDriver, Frame};
use esp_idf_hal::cpu::Core;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::Pull;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::nvs::{EspDefaultNvs, EspDefaultNvsPartition};
use esp_idf_svc::timer::EspTaskTimerService;
use light_robot_core_api as api;
use light_robot_core_api::*;
use log::info;
use max170xx::Max17048;
use std::collections::HashSet;
use std::ffi::CStr;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const BMP_280_FILTER_GAIN: f32 = 0.05f32;
const ENCODER_COUNTS_PER_TURN: i32 = 16_384;
const SERVO_TEST_CAPTURE_MS: u64 = 250;
const INERTIA_CAPTURE_DURATION: Duration = Duration::from_secs(5);
/// Host polling cadence for the ICM-42688-P output registers.
const IMU_SAMPLE_PERIOD: Duration = Duration::from_millis(10);

fn normalized_position_to_encoder(configuration: &ServoConfiguration, position: f32) -> u16 {
    let counts_per_degree = ENCODER_COUNTS_PER_TURN as f32 / 360.0;
    let delta = (counts_per_degree
        * configuration.max_turn_degrees.max(0.01)
        * position.clamp(-1.0, 1.0)) as i32;
    (configuration.zero_encoder_count as i32 + delta).rem_euclid(ENCODER_COUNTS_PER_TURN) as u16
}

fn encoder_to_normalized_position(configuration: &ServoConfiguration, encoder: u16) -> f32 {
    let half_turn = ENCODER_COUNTS_PER_TURN / 2;
    let delta = (encoder as i32 - configuration.zero_encoder_count as i32 + half_turn)
        .rem_euclid(ENCODER_COUNTS_PER_TURN)
        - half_turn;
    delta as f32
        / ((ENCODER_COUNTS_PER_TURN as f32 / 360.0) * configuration.max_turn_degrees.max(0.01))
}

#[derive(Clone)]
enum CalibrationAction {
    None,
    Detect,
    Apply(ServoAxis, ServoConfiguration),
    CaptureZero(ServoAxis),
    Enable(ServoAxis, bool),
    Position(ServoAxis, f32),
    StartTest(ServoAxis),
    AssignDevice(ServoAxis, ServoDeviceId),
    Save,
}

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    info!("Startup: logger ready");

    let state = Arc::new(Mutex::new(api::State::default()));

    let test_data = Arc::new(Mutex::new(api::ServoTestResult::default()));
    let inertia_capture_result = Arc::new(Mutex::new(api::InertiaCaptureResult::default()));
    let inertia_capture_request = Arc::new(Mutex::new(false));
    let calibration_action = Arc::new(Mutex::new(CalibrationAction::None));

    let peripherals = Peripherals::take().unwrap();
    info!("Startup: peripherals taken");
    let sysloop = EspSystemEventLoop::take().unwrap();
    info!("Startup: system event loop ready");

    // Wi-Fi's PHY calibration and servo calibration persistence both use NVS.
    // Take/initialize the default partition before constructing EspWifi so RF
    // calibration does not fall back to the slow, non-persistent path.
    let servo_nvs = Arc::new(Mutex::new(
        EspDefaultNvs::new(EspDefaultNvsPartition::take().unwrap(), "servo_cfg", true).unwrap(),
    ));
    info!("Startup: NVS ready");
    info!("Startup: loading saved servo configuration");
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
            if let Ok(calibration) = serde_json::from_slice(&bytes) {
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
                state.lock().unwrap().inertia_configuration = configuration;
            }
        }
    }
    info!("Startup: saved servo configuration loaded");

    let mut voltage_regulator = VoltageRegulator::new(peripherals.pins.gpio21);
    info!("Startup: regulator GPIO ready");
    voltage_regulator.on().unwrap();
    info!("Startup: regulator enabled");

    let _button = esp_idf_hal::gpio::PinDriver::input(peripherals.pins.gpio15, Pull::Up).unwrap();

    let mut pyro1 = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio5).unwrap();
    pyro1.set_low().unwrap();

    let mut pyro2 = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio7).unwrap();
    pyro2.set_low().unwrap();

    let i2c = esp_idf_hal::i2c::I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio17,
        peripherals.pins.gpio18,
        &esp_idf_hal::i2c::I2cConfig::default(),
    )
    .unwrap();
    info!("Startup: I2C ready");

    let i2c = SharedI2c::new(i2c);
    let mut max17048 = Max17048::new(i2c.clone());

    info!("SOC: {:.2}", max17048.soc().unwrap());
    info!("MAX -- OK");

    // let bmp280 = bmp280_ehal::BMP280::new(shared_i2c.acquire_i2c())?;
    // let bmp280 = Arc::new(Mutex::new(bmp280));

    // thread::spawn(move || {
    //     loop {
    //         thread::sleep(Duration::from_millis(20));
    //         let mut bmp280 = bmp280.lock().unwrap();
    //         let temperature: f32  = bmp280.temp() as f32;
    //         let p0 = 101325f32;
    //         let pressure: f32 = bmp280.pressure_one_shot() as f32;
    //         //-44330f32 * (1f32 - f32::powf  (pressure / 101325f32).po powf(1f32/5.255f32));
    //         let altitude: f32 = -8435.775 * (pressure / p0 - 1f32);
    //         let mut state = state_.lock().unwrap();
    //         let new_altitude = state.barometer.altitude +
    //             (altitude - state.barometer.altitude) * BMP_280_FILTER_GAIN;
    //         state.barometer.temperature = temperature;
    //         state.barometer.altitude = new_altitude;
    //     }
    // });

    // let mut adc_driver = esp_idf_hal::adc::oneshot::AdcDriver::new(peripherals.adc1).unwrap();
    // let mut adc_channel_driver = esp_idf_hal::adc::oneshot::AdcChannelDriver::new(&adc_driver, peripherals.pins.gpio1, &adc_channel_config).unwrap();

    // thread::spawn(move || {
    //     loop {
    //         thread::sleep(Duration::from_millis(1000));
    //         let adc_reading = adc_channel_driver.read().unwrap();
    //         let voltage = adc_reading as f32;
    //         let mut state = state2.lock().unwrap();
    //         state.pyro.channel1.test_voltage = voltage / 1000f32;
    //     }
    // });

    //Drivers
    let mut led_driver: LedDriver<'static> =
        LedDriver::new(peripherals.pins.gpio8, peripherals.rmt.channel0).unwrap();
    info!("LED -- OK");
    led_driver.set_rgb(20, 0, 0).unwrap();

    let wifi_configuration = api::WifiConnectionConfiguration {
        connection_type: api::WifiConnectionType::StartAccessPoint,
        credentials: api::WifiCredentials {
            ssid: String::from("light-robot-core"),
            password: String::from("12345678"),
        },
    };

    info!("wifi config {:?}", wifi_configuration);

    #[allow(unused_variables)]
    let wifi = WiFi::new(
        wifi_configuration,
        peripherals.modem,
        sysloop.clone(),
        state.clone(),
    )
    .unwrap();

    match state.lock().unwrap().wifi_state.connection_type {
        WifiConnectionType::ConnectToExternal => led_driver.set_rgb(0, 20, 0).unwrap(),
        _ => led_driver.set_rgb(10, 10, 0).unwrap(),
    }

    let ld = Arc::new(Mutex::new(led_driver));
    let ld1 = ld.clone();

    let state_ = state.clone();

    let pyro1_ = Arc::new(Mutex::new(pyro1));
    let pyro2_ = Arc::new(Mutex::new(pyro2));

    let calibration_action_ = calibration_action.clone();
    let calibration_state = state.clone();
    let inertia_capture_request_ = inertia_capture_request.clone();
    let inertia_capture_state = state.clone();
    let inertia_nvs = servo_nvs.clone();
    let command_handler = move |c: &api::Command| -> anyhow::Result<()> {
        match c {
            Command::Reset => {}
            Command::SetWifi { ssid, password } => {}
            Command::SetLedColor { r, g, b } => {
                info!("color");
                ld.lock()
                    .unwrap()
                    .set_rgb(r.clone(), g.clone(), b.clone())
                    .unwrap();
            }
            Command::DetectServos => {
                *calibration_action_.lock().unwrap() = CalibrationAction::Detect
            }
            Command::SetServoConfiguration {
                axis,
                configuration,
            } => {
                let mut calibration = calibration_state.lock().unwrap();
                if *axis == ServoAxis::X {
                    calibration.servo_calibration.x = configuration.clone();
                } else {
                    calibration.servo_calibration.y = configuration.clone();
                }
                *calibration_action_.lock().unwrap() =
                    CalibrationAction::Apply(*axis, configuration.clone());
            }
            Command::CaptureServoZero { axis } => {
                *calibration_action_.lock().unwrap() = CalibrationAction::CaptureZero(*axis)
            }
            Command::SetServoEnabled { axis, enabled } => {
                let mut calibration = calibration_state.lock().unwrap();
                if *axis == ServoAxis::X {
                    calibration.servo_calibration.x_enabled = *enabled;
                } else {
                    calibration.servo_calibration.y_enabled = *enabled;
                }
                *calibration_action_.lock().unwrap() = CalibrationAction::Enable(*axis, *enabled);
            }
            Command::SetServoPosition { axis, position } => {
                *calibration_action_.lock().unwrap() =
                    CalibrationAction::Position(*axis, position.clamp(-1.0, 1.0));
            }
            Command::StartServoPerformanceTest {
                axis,
                configuration,
            } => {
                let mut calibration = calibration_state.lock().unwrap();
                if *axis == ServoAxis::X {
                    calibration.servo_calibration.x = configuration.clone();
                } else {
                    calibration.servo_calibration.y = configuration.clone();
                }
                *calibration_action_.lock().unwrap() = CalibrationAction::StartTest(*axis)
            }
            Command::AssignServoDevice { axis, device } => {
                *calibration_action_.lock().unwrap() =
                    CalibrationAction::AssignDevice(*axis, device.clone())
            }
            Command::SaveServoConfigurations => {
                *calibration_action_.lock().unwrap() = CalibrationAction::Save
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
                    *inertia_capture_request_.lock().unwrap() = true;
                }
            }
            Command::SetInertiaConfiguration { configuration } => {
                inertia_capture_state.lock().unwrap().inertia_configuration = configuration.clone();
            }
            Command::SaveInertiaConfiguration { configuration } => {
                inertia_capture_state.lock().unwrap().inertia_configuration = configuration.clone();
                match serde_json::to_vec(configuration) {
                    Ok(bytes) => match inertia_nvs.lock().unwrap().set_blob("inertia", &bytes) {
                        Ok(()) => info!("Moment-of-inertia configuration saved to NVS"),
                        Err(error) => info!(
                            "Unable to save moment-of-inertia configuration: {:?}",
                            error
                        ),
                    },
                    Err(error) => info!(
                        "Unable to serialize moment-of-inertia configuration: {:?}",
                        error
                    ),
                }
            }

            Command::ResetNvs => {}
            Command::TestServo { .. } => {}
        }
        Ok(())
    };

    #[allow(unused_variables)]
    let server = Server::new(
        state.clone(),
        test_data.clone(),
        inertia_capture_result.clone(),
        command_handler,
    )
    .unwrap();

    info!("HTTP server -- OK");
    info!("mDNS -- OK");

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
        name: Some(CStr::from_bytes_with_nul(b"imu-sampler\0").unwrap()),
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
                            info!("ICM-42688-P initialization failed: {:?}", error);
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

                    if std::mem::replace(&mut *imu_inertia_capture_request.lock().unwrap(), false) {
                        capture_started_at = Some(Instant::now());
                        capture = InertiaCaptureResult::default();
                        imu_state.lock().unwrap().inertia_capture.running = true;
                        info!("Moment-of-inertia gyro capture started");
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
                            info!("Moment-of-inertia gyro capture finished");
                        }
                    }
                }
                Err(error) => {
                    if !failure_reported {
                        info!(
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

    // Keep deterministic flight-control work off the core that runs Wi-Fi and
    // ESP-IDF's HTTP server. The task owns CAN and IMU polling together, so it
    // can update the shared state without cross-task control handoffs.
    let default_flight_thread_config = esp_idf_hal::task::thread::ThreadSpawnConfiguration::get();
    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(CStr::from_bytes_with_nul(b"flight-control\0").unwrap()),
        stack_size: 32 * 1024,
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }
    .set()
    .unwrap();
    thread::spawn(move || {
        let mut rs_pin = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio1).unwrap();
        rs_pin.set_low().unwrap();

        let filter = esp_idf_hal::can::config::Filter::standard_allow_all();

        let timing = esp_idf_hal::can::config::Timing::B1M;
        let mode = esp_idf_hal::can::config::Mode::Normal;
        let config = esp_idf_hal::can::config::Config::new()
            .filter(filter)
            .timing(timing)
            .mode(mode);
        let mut can = esp_idf_hal::can::CanDriver::new(
            peripherals.can,
            peripherals.pins.gpio43,
            peripherals.pins.gpio2,
            &config,
        )
        .unwrap();

        can.start().unwrap();

        FreeRtos::delay_ms(1000);

        info!("START!!!");

        trait ApiTransmitter {
            fn can_transmit<T: ApiEncodeDecode>(&mut self, address: u32, data: &T);
        }
        impl ApiTransmitter for CanDriver<'static> {
            fn can_transmit<T: ApiEncodeDecode>(&mut self, address: u32, data: &T) {
                let data_vec = data.api_encode().unwrap();
                let data = data_vec.as_slice();
                let frame = Frame::new(address, EnumSet::new(), data).unwrap();
                // CAN requires another active node to acknowledge a frame. A bare
                // bench bus therefore returns ESP_ERR_TIMEOUT, which is an offline
                // condition—not a reason for the flight computer to panic.
                let _ = self.transmit(&frame, 10);
            }
        }

        let mut chip_ids = HashSet::<([u8; 6], [u8; 6])>::new();

        //read all servos
        {
            info!("Servo auth started");
            can.can_transmit(0, &GeneralCommandFrame::RequestChipId1);

            let mut chip_id1s = HashSet::new();

            let mut channel_index = 100;

            let timestamp = Instant::now();

            loop {
                match can.receive(20) {
                    Ok(frame) => {
                        if frame.identifier() == 1 {
                            match GeneralResponseFrame::api_decode(frame.data()).unwrap() {
                                GeneralResponseFrame::ChipID1 { chip_id1 } => {
                                    chip_id1s.insert(chip_id1);
                                }
                                _ => {}
                            }
                        }
                    }
                    Err(_) => break,
                }
                if timestamp.elapsed().as_millis() > 20 {
                    break;
                }
            }

            for chip_id1 in chip_id1s {
                let data_vec = GeneralCommandFrame::RequestChipId2 { chip_id1 }
                    .api_encode()
                    .unwrap();
                let data = data_vec.as_slice();
                let frame = Frame::new(0, EnumSet::new(), data).unwrap();

                let _ = can.transmit(&frame, 10);

                loop {
                    let timestamp = Instant::now();
                    match can.receive(20) {
                        Ok(frame) => {
                            if frame.identifier() == 1 {
                                match GeneralResponseFrame::api_decode(frame.data()).unwrap() {
                                    GeneralResponseFrame::ChipID2 { chip_id2 } => {
                                        &chip_ids.insert((chip_id1, chip_id2));
                                    }
                                    _ => {}
                                }
                            }
                        }
                        Err(_) => break,
                    }
                    if timestamp.elapsed().as_millis() > 20 {
                        break;
                    }
                }

                // for chip_id2 in chip_id2s {
                //     let data_vec = GeneralCommandFrame::ChipID1 { chip_id1 }
                //         .api_encode()
                //         .unwrap();
                //     let data = data_vec.as_slice();
                //     let frame = Frame::new(0, EnumSet::new(), data).unwrap();
                //     can.transmit(&frame, 10).unwrap();
                //     info!("T! id:{:} data:{:?}", frame.identifier(), frame.data());
                //     thread::sleep(Duration::from_millis(2));
                //
                //     let data_vec = GeneralCommandFrame::ChipID2 { chip_id2 }
                //         .api_encode()
                //         .unwrap();
                //     let data = data_vec.as_slice();
                //     let frame = Frame::new(0, EnumSet::new(), data).unwrap();
                //     can.transmit(&frame, 10).unwrap();
                //     info!("T! id:{:} data:{:?}", frame.identifier(), frame.data());
                //     thread::sleep(Duration::from_millis(2));
                //
                //     let data_vec = GeneralCommandFrame::SetChannel {
                //         master: channel_index,
                //         slave: channel_index + 1,
                //     }
                //     .api_encode()
                //     .unwrap();
                //     let data = data_vec.as_slice();
                //     let frame = Frame::new(0, EnumSet::new(), data).unwrap();
                //     can.transmit(&frame, 10).unwrap();
                //     info!("T! id:{:} data:{:?}", frame.identifier(), frame.data());
                //     thread::sleep(Duration::from_millis(2));
                //
                //     let data_vec = ServoCommandFrame::SetLedConstant {
                //         r: 100,
                //         g: 100,
                //         b: 100,
                //     }
                //     .api_encode()
                //     .unwrap();
                //     let data = data_vec.as_slice();
                //     let frame = Frame::new(channel_index as u32, EnumSet::new(), data).unwrap();
                //     info!("T! id:{:} data:{:?}", frame.identifier(), frame.data());
                //     can.transmit(&frame, 10).unwrap();
                //
                //     channel_index = channel_index + 2;
                //     devices_connected += 1;
                // }
            }
            info!("Servos detected:");
            for chip_id in chip_ids.iter() {
                info!("{:?}", chip_id);
            }
        }

        info!("Servo detection END");
        state.lock().unwrap().servo_calibration.discovered_devices = chip_ids
            .iter()
            .map(|(chip_id1, chip_id2)| ServoDeviceId {
                chip_id1: *chip_id1,
                chip_id2: *chip_id2,
            })
            .collect();

        let servo1_address_master = 2u16;
        let servo1_address_slave = 12u16;
        let servo2_address_master = 3u16;
        let servo2_address_slave = 13u16;

        let test_data_ = test_data.clone();
        loop {
            let action = std::mem::replace(
                &mut *calibration_action.lock().unwrap(),
                CalibrationAction::None,
            );
            match action {
                CalibrationAction::None => {}
                CalibrationAction::Detect => {
                    // Query each identity half in sequence so ChipID2 is paired to
                    // the ChipID1 which requested it.
                    can.can_transmit(0, &GeneralCommandFrame::RequestChipId1);
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
                    let mut discovered = Vec::new();
                    for chip_id1 in chip_id1s {
                        can.can_transmit(0, &GeneralCommandFrame::RequestChipId2 { chip_id1 });
                        let started = Instant::now();
                        while started.elapsed() < Duration::from_millis(100) {
                            if let Ok(frame) = can.receive(5) {
                                if frame.identifier() == 1 {
                                    if let Ok(GeneralResponseFrame::ChipID2 { chip_id2 }) =
                                        GeneralResponseFrame::api_decode(frame.data())
                                    {
                                        discovered.push(ServoDeviceId { chip_id1, chip_id2 });
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    state_.lock().unwrap().servo_calibration.discovered_devices = discovered;
                }
                CalibrationAction::AssignDevice(axis, device) => {
                    let (master, slave) = if axis == ServoAxis::X {
                        (servo1_address_master, servo1_address_slave)
                    } else {
                        (servo2_address_master, servo2_address_slave)
                    };
                    can.can_transmit(
                        0,
                        &GeneralCommandFrame::ChipID1 {
                            chip_id1: device.chip_id1,
                        },
                    );
                    can.can_transmit(
                        0,
                        &GeneralCommandFrame::ChipID2 {
                            chip_id2: device.chip_id2,
                        },
                    );
                    can.can_transmit(0, &GeneralCommandFrame::SetChannel { master, slave });
                    let mut state = state_.lock().unwrap();
                    if axis == ServoAxis::X {
                        state.servo_calibration.x_device = Some(device);
                    } else {
                        state.servo_calibration.y_device = Some(device);
                    }
                }
                CalibrationAction::Apply(axis, configuration) => {
                    let address = if axis == ServoAxis::X {
                        servo1_address_master
                    } else {
                        servo2_address_master
                    };
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetDataRate { data_rate: 1000 },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetGeneralConfig1 {
                            reverse_motor: configuration.reverse_motor,
                            duty_cycle_limit: configuration.duty_cycle_limit,
                        },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetPositionPGain {
                            p: configuration.position_p,
                        },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetPositionIGain {
                            i: configuration.position_i,
                            cycles_to_max_out: 1000,
                        },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetPositionDGain {
                            d: configuration.position_d,
                        },
                    );
                }
                CalibrationAction::CaptureZero(axis) => {
                    let position = if axis == ServoAxis::X {
                        state_.lock().unwrap().servo1.position
                    } else {
                        state_.lock().unwrap().servo2.position
                    };
                    let mut s = state_.lock().unwrap();
                    if axis == ServoAxis::X {
                        s.servo_calibration.x.zero_encoder_count = position;
                    } else {
                        s.servo_calibration.y.zero_encoder_count = position;
                    }
                }
                CalibrationAction::Enable(axis, enabled) => {
                    if !enabled {
                        let address = if axis == ServoAxis::X {
                            servo1_address_master
                        } else {
                            servo2_address_master
                        };
                        can.can_transmit(address as u32, &ServoCommandFrame::Disable);
                    }
                }
                CalibrationAction::Position(axis, position) => {
                    let s = state_.lock().unwrap();
                    let c = if axis == ServoAxis::X {
                        &s.servo_calibration.x
                    } else {
                        &s.servo_calibration.y
                    };
                    let encoder = normalized_position_to_encoder(c, position);
                    let address = if axis == ServoAxis::X {
                        servo1_address_master
                    } else {
                        servo2_address_master
                    };
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::BrushedHoldPosition { position: encoder },
                    );
                }
                CalibrationAction::StartTest(axis) => {
                    info!("Servo step-response test started");
                    state_.lock().unwrap().servo_calibration.test_running = true;
                    let (address, slave, configuration) = {
                        let s = state_.lock().unwrap();
                        if axis == ServoAxis::X {
                            (
                                servo1_address_master,
                                servo1_address_slave,
                                s.servo_calibration.x.clone(),
                            )
                        } else {
                            (
                                servo2_address_master,
                                servo2_address_slave,
                                s.servo_calibration.y.clone(),
                            )
                        }
                    };
                    let zero = configuration.zero_encoder_count;
                    // A test always starts from precisely the configuration shown
                    // in the UI, even when the user did not press Apply first.
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetDataRate { data_rate: 1000 },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetGeneralConfig1 {
                            reverse_motor: configuration.reverse_motor,
                            duty_cycle_limit: configuration.duty_cycle_limit,
                        },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetPositionPGain {
                            p: configuration.position_p,
                        },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetPositionIGain {
                            i: configuration.position_i,
                            cycles_to_max_out: 1000,
                        },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::SetPositionDGain {
                            d: configuration.position_d,
                        },
                    );
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::BrushedHoldPosition { position: zero },
                    );
                    // Keep the CAN RX FIFO empty while the actuator settles.
                    // Otherwise pre-step telemetry is consumed at capture time and
                    // appears as a vertical spike at t=0 in the response plot.
                    let settling_started = Instant::now();
                    while settling_started.elapsed() < Duration::from_secs(2) {
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
                    can.can_transmit(
                        address as u32,
                        &ServoCommandFrame::BrushedHoldPosition { position: target },
                    );
                    while started.elapsed() < Duration::from_millis(SERVO_TEST_CAPTURE_MS) {
                        if let Ok(frame) = can.receive(1) {
                            if frame.identifier() == slave as u32 {
                                if let Ok(ServoResponseFrame::State { position, .. }) =
                                    ServoResponseFrame::api_decode(frame.data())
                                {
                                    let normalized =
                                        encoder_to_normalized_position(&configuration, position);
                                    result.samples.push(ServoTestSample {
                                        time_us: started.elapsed().as_micros() as u32,
                                        position: normalized,
                                    });
                                }
                            }
                        }
                    }
                    *test_data_.lock().unwrap() = result;
                    can.can_transmit(address as u32, &ServoCommandFrame::Disable);
                    let mut state = state_.lock().unwrap();
                    state.servo_calibration.test_running = false;
                    if axis == ServoAxis::X {
                        state.servo_calibration.x_enabled = false;
                    } else {
                        state.servo_calibration.y_enabled = false;
                    }
                    state.servo_calibration.test_result_revision =
                        state.servo_calibration.test_result_revision.wrapping_add(1);
                    info!("Servo step-response test finished");
                }
                CalibrationAction::Save => {
                    match serde_json::to_vec(&state_.lock().unwrap().servo_calibration) {
                        Ok(bytes) => match servo_nvs.lock().unwrap().set_blob("axes", &bytes) {
                            Ok(()) => info!("Servo configuration saved to NVS flash"),
                            Err(error) => info!("Unable to save servo configuration: {:?}", error),
                        },
                        Err(error) => info!("Unable to serialize servo configuration: {:?}", error),
                    }
                }
            }
            /*
                    // Historic test implementation intentionally retained as documentation only.
                    // let zero_servo1 = 4096i16 + 1358;
                    //
                    //
                    // let max_servo1 = (4096 * 5 * 116 / 360 / 8) as f32;
                    //
                    // can.can_transmit(servo2_address_master as u32, &ServoCommandFrame::Disable);
                    //
                    // can.can_transmit(
                    //     servo1_address_master as u32,
                    //     &ServoCommandFrame::HoldPosition {
                    //         setpoint: ((zero_servo1 + (max_servo1 * start) as i16) % 4096) as u16,
                    //     },
                    // );
                    //
                    // let timestamp = Instant::now();
                    //
                    // while timestamp.elapsed().as_millis() < 2000 {
                    //     match can.receive(0) {
                    //         Ok(_) => {}
                    //         Err(_) => {}
                    //     }
                    // }
                    //
                    // let mut test_data = test_data_.lock().unwrap();
                    // test_data.data.clear();
                    //
                    // let timestamp = Instant::now();
                    //
                    // can.can_transmit(
                    //     servo1_address_master as u32,
                    //     &ServoCommandFrame::HoldPosition {
                    //         setpoint: ((zero_servo1 + (max_servo1 * end) as i16) % 4096) as u16,
                    //     },
                    // );
                    //
                    // while timestamp.elapsed().as_millis() < 1000 {
                    //     match can.receive(0) {
                    //         Ok(frame) => {
                    //             if frame.identifier() == servo1_address_slave as u32 {
                    //                 match ServoResponseFrame::api_decode(frame.data()).unwrap() {
                    //                     ServoResponseFrame::State {
                    //                         sensor_detected,
                    //                         position,
                    //                         velocity,
                    //                         current,
                    //                     } => {
                    //                         test_data.data.push((
                    //                             timestamp.elapsed().as_millis() as u16,
                    //                             position,
                    //                         ));
                    //                     }
                    //                 }
                    //             }
                    //         }
                    //         Err(_) => {}
                    //     }
                    // }

            */

            let can_receive_result = can.receive(0);

            match can_receive_result {
                Ok(frame) => {
                    if frame.identifier() == servo1_address_slave as u32 {
                        match ServoResponseFrame::api_decode(frame.data()).unwrap() {
                            ServoResponseFrame::State {
                                sensor_detected,
                                position,
                                velocity,
                                current,
                            } => {
                                let mut state = state_.lock().unwrap();
                                (*state).servo1.position = position;
                                (*state).servo1.velocity = velocity;
                                (*state).servo1.current = current;
                                (*state).servo1.sensor_detected = sensor_detected;
                                (*state).servo_calibration.detected_x = sensor_detected;
                            }
                        }
                    }
                    if frame.identifier() == servo2_address_slave as u32 {
                        match ServoResponseFrame::api_decode(frame.data()).unwrap() {
                            ServoResponseFrame::State {
                                sensor_detected,
                                position,
                                velocity,
                                current,
                            } => {
                                let mut state = state_.lock().unwrap();
                                (*state).servo2.position = position;
                                (*state).servo2.velocity = velocity;
                                (*state).servo2.current = current;
                                (*state).servo2.sensor_detected = sensor_detected;
                                (*state).servo_calibration.detected_y = sensor_detected;
                            }
                        }
                    }
                }
                Err(_) => {}
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
