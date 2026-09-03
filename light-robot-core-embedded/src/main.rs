extern crate core;

mod led_driver;
mod server;
mod servo_api;
mod voltage_regulator;
mod wifi;

use crate::led_driver::LedDriver;
use crate::server::Server;
use crate::servo_api::*;
use crate::voltage_regulator::VoltageRegulator;
use crate::wifi::WiFi;
use embedded_hal::digital::OutputPin;
use enumset::EnumSet;
use esp_idf_hal::adc::Resolution;
use esp_idf_hal::can::{Alert, CanDriver, Frame};
use esp_idf_hal::gpio::{Level, Pull};
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_hal::ledc;
use esp_idf_hal::ledc::config::TimerConfig;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::prelude::FromValueType;
use esp_idf_hal::sys::{i2c_master_read, EspError};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::gpio::Pin;
use esp_idf_svc::hal::peripherals::Peripherals;
use light_robot_core_api as api;
use light_robot_core_api::*;
use log::info;
use max170xx::Max17048;
use smart_leds::hsv::{hsv2rgb, Hsv};
use smart_leds::{RGB, RGB8};
use smart_leds_trait::SmartLedsWrite;
use std::collections::HashSet;
use std::io::Read;
use std::sync::{Arc, Mutex, MutexGuard};
use std::thread;
use std::thread::Thread;
use std::time::{Duration, Instant};
use ws2812_esp32_rmt_driver::Ws2812Esp32RmtDriver;

const BMP_280_FILTER_GAIN: f32 = 0.05f32;

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let state = Arc::new(Mutex::new(api::State::default()));

    let test_data = Arc::new(Mutex::new(api::TestData::default()));

    let test_started = Arc::new(Mutex::new(None::<(f32, f32)>));

    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();

    let mut voltage_regulator = VoltageRegulator::new(peripherals.pins.gpio21);
    voltage_regulator.on().unwrap();

    let mut button = esp_idf_hal::gpio::PinDriver::input(peripherals.pins.gpio15).unwrap();
    button.set_pull(Pull::Up).unwrap();

    let mut pyro1 = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio5).unwrap();
    pyro1.set_low().unwrap();

    let mut pyro2 = esp_idf_hal::gpio::PinDriver::output(peripherals.pins.gpio7).unwrap();
    pyro2.set_low().unwrap();

    let i2c = esp_idf_hal::i2c::I2cDriver::new(
        peripherals.i2c0.into_ref(),
        peripherals.pins.gpio17,
        peripherals.pins.gpio18,
        &esp_idf_hal::i2c::I2cConfig::default(),
    )
    .unwrap();

    //let shared_i2c = shared_bus::new_std!(I2cDriver = i2c).unwrap();

    let mut max17048 = Max17048::new(i2c);

    info!("SOC: {:.2}", max17048.soc().unwrap());
    info!("MAX -- OK");

    let max17048 = Arc::new(Mutex::new(max17048));

    esp_idf_hal::task::thread::ThreadSpawnConfiguration {
        name: Some(b"max-thread\0"),
        ..Default::default()
    }
    .set()
    .unwrap();

    let max1 = max17048.clone();
    let max2 = max17048.clone();

    let state1 = state.clone();
    let state2 = state.clone();

    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        let mut state = state1.lock().unwrap();
        let mut max = max1.lock().unwrap();
        state.battery.soc = max.soc().unwrap();
        state.battery.voltage = max.voltage().unwrap();
        state.battery.charge_rate = max.charge_rate().unwrap();
    });

    // let bmp280 = bmp280_ehal::BMP280::new(shared_i2c.acquire_i2c())?;
    // let bmp280 = Arc::new(Mutex::new(bmp280));

    let state_ = state.clone();

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

    let mut adc_channel_config = esp_idf_hal::adc::oneshot::config::AdcChannelConfig::default();
    adc_channel_config.resolution = Resolution::Resolution12Bit;
    adc_channel_config.calibration = true;

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

    let cc: ServoCommand = ServoCommand::Disable;
    let car_command_ = Arc::new(Mutex::new(cc));

    let car_command = car_command_.clone();
    let test_started_ = test_started.clone();
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
            Command::ServoCommand { command } => {
                let mut car_command = car_command.lock().unwrap();
                let cc = (*command).clone();
                match cc {
                    ServoCommand::Disable => {
                        *car_command = cc;
                    }
                    ServoCommand::Update {
                        servo1: steering,
                        servo2: power,
                    } => {
                        let s = steering.clamp(-1.0, 1.0);
                        let p = power.clamp(-1.0, 1.0);
                        *car_command = ServoCommand::Update {
                            servo1: s,
                            servo2: p,
                        };
                    }
                }
                *car_command = (*command).clone();
            }

            Command::ResetNvs => {}
            Command::TestServo {
                servo_id,
                start,
                end,
            } => {
                let mut test_started = test_started_.lock().unwrap();
                test_started.replace((*start, *end));
            }
        }
        Ok(())
    };

    #[allow(unused_variables)]
    let server = Server::new(state.clone(), test_data.clone(), command_handler).unwrap();

    info!("HTTP server -- OK");
    info!("mDNS -- OK");

    let mut mdns = esp_idf_svc::mdns::EspMdns::take().unwrap();
    mdns.set_hostname("lrc").unwrap();
    mdns.set_instance_name("Light Robot Core web server")
        .unwrap();
    mdns.add_service(None, "_http", "_tcp", 80, &[("board", "{esp32}")])
        .unwrap();

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

    thread::sleep(Duration::from_millis(1000));

    info!("START!!!");

    trait ApiTransmitter {
        fn can_transmit<T: ApiEncodeDecode>(&mut self, address: u32, data: &T);
    }
    impl ApiTransmitter for CanDriver<'static> {
        fn can_transmit<T: ApiEncodeDecode>(&mut self, address: u32, data: &T) {
            let data_vec = data.api_encode().unwrap();
            let data = data_vec.as_slice();
            let frame = Frame::new(address, EnumSet::new(), data).unwrap();
            self.transmit(&frame, 10).unwrap();
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

            can.transmit(&frame, 10).unwrap();

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

    let servo1_address_master = 2u16;
    let servo1_address_slave = 12u16;
    let servo2_address_master = 3u16;
    let servo2_address_slave = 13u16;

    //servo1
    {
        let chip_id1: [u8; 6] = [57, 0, 64, 0, 14, 80];
        let chip_id2: [u8; 6] = [65, 50, 77, 54, 55, 32];

        can.can_transmit(0, &GeneralCommandFrame::ChipID1 { chip_id1 });
        can.can_transmit(0, &GeneralCommandFrame::ChipID2 { chip_id2 });
        can.can_transmit(
            0,
            &GeneralCommandFrame::SetChannel {
                master: servo1_address_master,
                slave: servo1_address_slave,
            },
        );
        can.can_transmit(
            servo1_address_master as u32,
            &ServoCommandFrame::SetDataRate { data_rate: 500 },
        );
        can.can_transmit(
            servo1_address_master as u32,
            &ServoCommandFrame::SetMotorConfig {
                duty_cycle_limit: 0.75,
                reverse_motor: false,
            },
        );
        can.can_transmit(
            servo1_address_master as u32,
            &ServoCommandFrame::SetPGain { p: 0.02 },
        );
    }
    //servo2
    {
        let chip_id1: [u8; 6] = [59, 0, 97, 0, 14, 80];
        let chip_id2: [u8; 6] = [65, 50, 77, 54, 55, 32];

        can.can_transmit(0, &GeneralCommandFrame::ChipID1 { chip_id1 });
        can.can_transmit(0, &GeneralCommandFrame::ChipID2 { chip_id2 });
        can.can_transmit(
            0,
            &GeneralCommandFrame::SetChannel {
                master: servo2_address_master,
                slave: servo2_address_slave,
            },
        );
        can.can_transmit(
            servo2_address_master as u32,
            &ServoCommandFrame::SetDataRate { data_rate: 10 },
        );
        can.can_transmit(
            servo2_address_master as u32,
            &ServoCommandFrame::SetMotorConfig {
                duty_cycle_limit: 0.75,
                reverse_motor: false,
            },
        );
        can.can_transmit(
            servo2_address_master as u32,
            &ServoCommandFrame::SetPGain { p: 0.01 },
        );
    }

    let car_command = car_command_.clone();
    let test_started_ = test_started.clone();
    let test_data_ = test_data.clone();
    loop {
        let mut test_started = test_started_.lock().unwrap();

        let ts = test_started.take();

        match ts {
            None => {}
            Some((start, end)) => {
                info!("Test started");
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

                let timestamp = Instant::now();

                let mut p1 = pyro1_.lock().unwrap();

                p1.set_high().unwrap();

                while timestamp.elapsed().as_millis() < 2000 {

                }

                p1.set_low().unwrap();

                info!("Test ended");
            }
        }

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
                        }
                    }
                }
            }
            Err(_) => {}
        }

        let car_command = car_command.lock().unwrap();

        let zero_servo1 = 4096i16 + 1358;
        let zero_servo2 = 4096i16 + 2376;

        let max_servo1 = (4096 * 5 * 116 / 360 / 8) as f32;
        let max_servo2 = (4096 * 5 * 66 / 360 / 8) as f32;

        match *car_command {
            ServoCommand::Disable => {
                can.can_transmit(servo2_address_master as u32, &ServoCommandFrame::Disable);
                can.can_transmit(servo1_address_master as u32, &ServoCommandFrame::Disable);
            }
            ServoCommand::Update {
                servo1: steering,
                servo2: power,
            } => {
                let steering_setpoint =
                    ((zero_servo1 + (max_servo1 * steering) as i16) % 4096) as u16;
                can.can_transmit(
                    servo1_address_master as u32,
                    &ServoCommandFrame::HoldPosition {
                        setpoint: steering_setpoint,
                    },
                );

                let steering_setpoint = ((zero_servo2 + (max_servo2 * power) as i16) % 4096) as u16;
                can.can_transmit(
                    servo2_address_master as u32,
                    &ServoCommandFrame::HoldPosition {
                        setpoint: steering_setpoint,
                    },
                );
            }
        }

        // can.can_transmit(2, &ServoCommandFrame::HoldPosition { setpoint: 500 });
        // thread::sleep(Duration::from_millis(2000));
        // can.can_transmit(2, &ServoCommandFrame::HoldPosition { setpoint: 2500 });
        //thread::sleep(Duration::from_millis(1));
        thread::yield_now();
    }

    // #[allow(unreachable_code)]
}
