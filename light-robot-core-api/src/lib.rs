use serde::{Deserialize, Serialize};

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct State {
    pub battery: BatteryState,
    pub pyro: PyroState,
    pub wifi_state: WifiConnectionConfiguration,
    pub barometer: BarometerState,
    pub servo1: ServoState,
    pub servo2: ServoState,
    pub weight: i32,
    pub chip_id1: [u8; 6],
    pub chip_id2: [u8; 6],
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ServoState {
    pub position: u16,
    pub velocity: i16,
    pub current: i16,
    pub sensor_detected: bool,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct BatteryState {
    pub soc: f32,
    pub voltage: f32,
    pub charge_rate: f32,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize, Debug)]
pub struct WifiCredentials {
    pub ssid: String,
    pub password: String,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize, Debug)]
pub struct WifiConnectionConfiguration {
    pub connection_type: WifiConnectionType,
    pub credentials: WifiCredentials,
}

#[derive(Clone, PartialEq, Serialize, Deserialize, Debug)]
pub enum WifiConnectionType {
    ConnectToExternal,
    StartAccessPoint,
}

impl Default for WifiConnectionType {
    fn default() -> Self { WifiConnectionType::StartAccessPoint }
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct PyroChannelState {
    pub fire: bool,
    pub test_voltage: f32,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct PyroState {
    pub channel1: PyroChannelState,
    pub channel2: PyroChannelState,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct BarometerState {
    pub altitude: f32,
    pub temperature: f32,
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub enum ServoCommand {
    Disable,
    Update { servo1: f32, servo2: f32}
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub enum Command {
    Reset,
    SetWifi { ssid: String, password: String },
    ResetNvs,
    SetLedColor { r: u8, g: u8, b: u8 },
    ServoCommand {command: ServoCommand },
    TestServo {servo_id: u8, start: f32, end: f32},
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct TestData {
    pub data: Vec<(u16, u16)>,
}