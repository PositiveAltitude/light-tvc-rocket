use serde::{Deserialize, Serialize};

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct State {
    pub battery: BatteryState,
    pub pyro: PyroState,
    pub wifi_state: WifiConnectionConfiguration,
    pub barometer: BarometerState,
    pub imu: ImuState,
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

/// The two 48-bit identity halves reported by a servo during CAN discovery.
#[derive(Clone, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct ServoDeviceId {
    pub chip_id1: [u8; 6],
    pub chip_id2: [u8; 6],
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
    pub duty_cycle_limit: f32,
    /// Reverses the brushed H-bridge polarity for this physical motor.
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
            duty_cycle_limit: 0.75,
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
    /// Devices found by the latest CAN discovery scan.
    pub discovered_devices: Vec<ServoDeviceId>,
    /// The device currently assigned to each TVC axis, if any.
    pub x_device: Option<ServoDeviceId>,
    pub y_device: Option<ServoDeviceId>,
    pub test_result_revision: u32,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ServoTestSample {
    /// Capture timestamp relative to the commanded step, in microseconds.
    pub time_us: u32,
    pub position: f32,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ServoTestResult {
    pub axis: ServoAxis,
    pub configuration: ServoConfiguration,
    pub samples: Vec<ServoTestSample>,
}

/// Messages carried by the persistent UI WebSocket.
#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub enum SocketMessage {
    State(State),
    TestResult(ServoTestResult),
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

/// Latest raw inertial measurement from the board-mounted ICM-42688-P.
///
/// Acceleration is expressed in m/s² and angular velocity in rad/s, using the
/// ICM-42688-P's configured ±16 g and ±2000 °/s ranges respectively. The axes are
/// the physical sensor axes; board-to-vehicle alignment is intentionally not
/// applied here.
#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct ImuState {
    pub present: bool,
    pub acceleration_mps2: [f32; 3],
    pub angular_velocity_radps: [f32; 3],
    /// Monotonically wrapping count of successful 100 Hz samples.
    pub sample_count: u32,
    /// Average successful host acquisition rate since the last IMU initialization.
    pub average_rate_hz: f32,
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
        configuration: ServoConfiguration,
    },
    AssignServoDevice {
        axis: ServoAxis,
        device: ServoDeviceId,
    },
    SaveServoConfigurations,
}

#[derive(Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct TestData {
    pub data: Vec<(u16, u16)>,
}
