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
    pub servo_calibration: ServoCalibrationState,
}

#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum ServoAxis {
    X,
    Y,
}

impl Default for ServoAxis {
    fn default() -> Self {
        Self::X
    }
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct ServoConfiguration {
    /// Encoder counts at the mechanically defined TVC zero position.
    pub zero_encoder_count: u16,
    /// Physical deflection represented by command -1.0 / +1.0, in degrees.
    pub max_turn_degrees: f32,
    pub position_p: f32,
    pub position_i: f32,
    pub position_d: f32,
    pub velocity_p: f32,
    pub velocity_i: f32,
    pub duty_cycle_limit: f32,
    pub max_velocity: f32,
    pub reverse_motor: bool,
}

impl Default for ServoConfiguration {
    fn default() -> Self {
        Self {
            zero_encoder_count: 0,
            max_turn_degrees: 10.0,
            position_p: 0.02,
            position_i: 0.0,
            position_d: 0.0,
            velocity_p: 0.0,
            velocity_i: 0.0,
            duty_cycle_limit: 0.75,
            max_velocity: 0.0,
            reverse_motor: false,
        }
    }
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ServoCalibrationState {
    pub x: ServoConfiguration,
    pub y: ServoConfiguration,
    pub x_enabled: bool,
    pub y_enabled: bool,
    pub detected_x: bool,
    pub detected_y: bool,
    pub test_running: bool,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ServoTestSample {
    pub time_ms: u16,
    pub position: f32,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ServoTestResult {
    pub axis: ServoAxis,
    pub configuration: ServoConfiguration,
    pub samples: Vec<ServoTestSample>,
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
    fn default() -> Self {
        WifiConnectionType::StartAccessPoint
    }
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
    Update { servo1: f32, servo2: f32 },
}

#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub enum Command {
    Reset,
    SetWifi {
        ssid: String,
        password: String,
    },
    ResetNvs,
    SetLedColor {
        r: u8,
        g: u8,
        b: u8,
    },
    ServoCommand {
        command: ServoCommand,
    },
    TestServo {
        servo_id: u8,
        start: f32,
        end: f32,
    },
    DetectServos,
    SetServoConfiguration {
        axis: ServoAxis,
        configuration: ServoConfiguration,
    },
    CaptureServoZero {
        axis: ServoAxis,
    },
    SetServoEnabled {
        axis: ServoAxis,
        enabled: bool,
    },
    SetServoPosition {
        axis: ServoAxis,
        position: f32,
    },
    StartServoPerformanceTest {
        axis: ServoAxis,
    },
    SaveServoConfigurations,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct TestData {
    pub data: Vec<(u16, u16)>,
}
