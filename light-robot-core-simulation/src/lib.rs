//! Deterministic, renderer-independent six-degree-of-freedom rocket model.
//!
//! This crate intentionally contains no browser, ESP-IDF, Bevy, or control
//! policy code.  It can therefore be used for frontend previews and firmware
//! bench simulations alike.

use std::collections::VecDeque;

use light_robot_core_api::{InertiaConfiguration, MotorThrustProfile, SimulationConfiguration};

/// Klima D3 manufacturer RASP thrust data, published by ThrustCurve.org:
/// <https://www.thrustcurve.org/simfiles/5f8b0767d5fa3b000447e837/>.
/// Each pair is `(seconds after ignition, thrust in newtons)`.
const KLIMA_D3_THRUST_CURVE: &[(f32, f32)] = &[
    (0.073, 0.229),
    (0.178, 0.686),
    (0.251, 1.287),
    (0.313, 2.203),
    (0.375, 3.633),
    (0.425, 5.006),
    (0.473, 6.465),
    (0.556, 8.181),
    (0.603, 9.010),
    (0.655, 6.922),
    (0.698, 5.463),
    (0.782, 4.291),
    (0.873, 3.576),
    (1.024, 3.146),
    (1.176, 2.946),
    (5.282, 2.918),
    (5.491, 2.832),
    (5.590, 2.517),
    (5.782, 1.859),
    (5.924, 1.287),
    (6.061, 0.715),
    (6.170, 0.286),
    (6.260, 0.0),
];

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
impl Vec3 {
    pub const UP: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }
    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }
}
impl std::ops::Add for Vec3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl std::ops::Sub for Vec3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl std::ops::Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}
impl std::ops::Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl std::ops::Div<f32> for Vec3 {
    type Output = Self;
    fn div(self, s: f32) -> Self {
        self * (1.0 / s)
    }
}
impl std::ops::AddAssign for Vec3 {
    fn add_assign(&mut self, o: Self) {
        *self = *self + o;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
impl Default for Quat {
    fn default() -> Self {
        Self::IDENTITY
    }
}
impl Quat {
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };
    pub fn axis_angle(axis: Vec3, angle: f32) -> Self {
        let (s, c) = (angle * 0.5).sin_cos();
        Self {
            x: axis.x * s,
            y: axis.y * s,
            z: axis.z * s,
            w: c,
        }
    }
    pub fn conjugate(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }
    pub fn normalized(self) -> Self {
        let l = (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt();
        if l > 0.0 {
            Self {
                x: self.x / l,
                y: self.y / l,
                z: self.z / l,
                w: self.w / l,
            }
        } else {
            Self::IDENTITY
        }
    }
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let qv = Self {
            x: v.x,
            y: v.y,
            z: v.z,
            w: 0.0,
        };
        let r = self * qv * self.conjugate();
        Vec3::new(r.x, r.y, r.z)
    }
}
impl std::ops::Mul for Quat {
    type Output = Self;
    fn mul(self, b: Self) -> Self {
        Self {
            w: self.w * b.w - self.x * b.x - self.y * b.y - self.z * b.z,
            x: self.w * b.x + self.x * b.w + self.y * b.z - self.z * b.y,
            y: self.w * b.y - self.x * b.z + self.y * b.w + self.z * b.x,
            z: self.w * b.z + self.x * b.y - self.y * b.x + self.z * b.w,
        }
    }
}
impl std::ops::Add for Quat {
    type Output = Self;
    fn add(self, b: Self) -> Self {
        Self {
            x: self.x + b.x,
            y: self.y + b.y,
            z: self.z + b.z,
            w: self.w + b.w,
        }
    }
}
impl std::ops::Mul<f32> for Quat {
    type Output = Self;
    fn mul(self, s: f32) -> Self {
        Self {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
            w: self.w * s,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ControlInputs {
    pub tvc: [f32; 2],
    pub ignition: bool,
    pub parachute: bool,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct SensorData {
    pub time: f32,
    pub acceleration: Vec3,
    pub angular_velocity: Vec3,
    pub barometric_height: f32,
}
pub trait ControlSystem {
    fn reset(&mut self);
    fn update(&mut self, sensor: &SensorData) -> ControlInputs;
}

#[derive(Clone, Debug)]
pub struct RocketState {
    pub time: f32,
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub rotation: Quat,
    pub angular_velocity: Vec3,
    pub tvc: [f32; 2],
    delay: VecDeque<ControlInputs>,
}
impl Default for RocketState {
    fn default() -> Self {
        Self {
            time: 0.0,
            position: Vec3::default(),
            velocity: Vec3::default(),
            acceleration: Vec3::default(),
            rotation: Quat::default(),
            angular_velocity: Vec3::default(),
            tvc: [0.0; 2],
            delay: VecDeque::new(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Environment {
    pub gravity: f32,
    pub wind: Vec3,
    pub max_time: f32,
    pub dt: f32,
    pub substeps: u16,
}
impl Default for Environment {
    fn default() -> Self {
        Self {
            gravity: 9.80665,
            wind: Vec3::default(),
            max_time: 8.0,
            dt: 0.002,
            substeps: 5,
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct RocketParameters {
    pub mass_kg: f32,
    pub inertia: Vec3,
    pub thrust_profile: MotorThrustProfile,
    pub thrust_n: f32,
    pub burn_time_s: f32,
    pub max_tvc_angle_rad: f32,
    pub gimbal_com_offset: Vec3,
    pub tvc_misalignment_rad: [f32; 2],
    pub max_tvc_rate_rad_s: f32,
    pub tvc_delay_steps: usize,
    pub drag_cd_axial: f32,
    pub drag_cd_sideways: f32,
    pub reference_area_m2: f32,
    pub center_of_pressure_offset: Vec3,
}
impl RocketParameters {
    pub fn from_configuration(
        inertia: &InertiaConfiguration,
        cfg: &SimulationConfiguration,
        dt: f32,
    ) -> Self {
        let radians = std::f32::consts::PI / 180.0;
        Self {
            mass_kg: (inertia.mass_g / 1000.0).max(0.001),
            inertia: Vec3::new(
                inertia.moment_of_inertia_kgm2[0].max(1e-7),
                inertia.moment_of_inertia_kgm2[1].max(1e-7),
                inertia.moment_of_inertia_kgm2[2].max(1e-7),
            ),
            thrust_profile: cfg.motor_thrust_profile,
            thrust_n: cfg.thrust_newtons.max(0.0),
            burn_time_s: cfg.burn_time_s.max(0.0),
            max_tvc_angle_rad: cfg.max_tvc_angle_degrees.max(0.0) * radians,
            gimbal_com_offset: Vec3::new(
                0.0,
                0.0,
                -inertia.center_of_mass_offset_mm.max(0.0) / 1000.0,
            ),
            tvc_misalignment_rad: [
                cfg.tvc_misalignment_degrees[0] * radians,
                cfg.tvc_misalignment_degrees[1] * radians,
            ],
            max_tvc_rate_rad_s: cfg.max_tvc_rate_degrees_per_s.max(0.0) * radians,
            tvc_delay_steps: ((cfg.tvc_delay_ms as f32 / 1000.0) / dt.max(0.0001)).round() as usize,
            drag_cd_axial: cfg.drag_coefficient_axial.max(0.0),
            drag_cd_sideways: cfg.drag_coefficient_sideways.max(0.0),
            reference_area_m2: cfg.reference_area_m2.max(0.0),
            center_of_pressure_offset: Vec3::new(
                0.0,
                0.0,
                cfg.center_of_pressure_offset_mm / 1000.0,
            ),
        }
    }
    pub fn burn_duration_s(self) -> f32 {
        match self.thrust_profile {
            MotorThrustProfile::Constant => self.burn_time_s,
            MotorThrustProfile::KlimaD3 => KLIMA_D3_THRUST_CURVE
                .last()
                .map(|point| point.0)
                .unwrap_or(0.0),
        }
    }
    pub fn thrust_at(self, time_s: f32) -> f32 {
        match self.thrust_profile {
            MotorThrustProfile::Constant => (time_s < self.burn_time_s)
                .then_some(self.thrust_n)
                .unwrap_or(0.0),
            MotorThrustProfile::KlimaD3 => interpolate_thrust(KLIMA_D3_THRUST_CURVE, time_s),
        }
    }
}

fn interpolate_thrust(curve: &[(f32, f32)], time_s: f32) -> f32 {
    let Some(&(first_time, first_thrust)) = curve.first() else {
        return 0.0;
    };
    if time_s < 0.0 || time_s > curve.last().map(|point| point.0).unwrap_or(0.0) {
        return 0.0;
    }
    if time_s <= first_time {
        return first_thrust * time_s / first_time;
    }
    for window in curve.windows(2) {
        let ((start_time, start_thrust), (end_time, end_thrust)) = (window[0], window[1]);
        if time_s <= end_time {
            return start_thrust
                + (end_thrust - start_thrust) * (time_s - start_time) / (end_time - start_time);
        }
    }
    0.0
}
#[derive(Clone, Copy, Debug)]
pub struct SimulationLog {
    pub time: f32,
    pub position: Vec3,
    pub rotation: Quat,
    pub tvc: [f32; 2],
}
pub struct NumericalSimulation;
impl NumericalSimulation {
    pub fn run<C: ControlSystem>(
        &mut self,
        env: Environment,
        rocket: RocketParameters,
        mut state: RocketState,
        control: &mut C,
        stop_on_failure: bool,
    ) -> Vec<SimulationLog> {
        state.time = 0.0;
        state.delay.clear();
        control.reset();
        let mut log = Vec::new();
        let mut sensor = SensorData::default();
        while state.time < env.max_time {
            log.push(SimulationLog {
                time: state.time,
                position: state.position,
                rotation: state.rotation,
                tvc: state.tvc,
            });
            if stop_on_failure && state.rotation.rotate(Vec3::UP).z < 0.5 {
                break;
            }
            let input = control.update(&sensor);
            for _ in 0..env.substeps.max(1) {
                self.step(&mut state, input, rocket, env);
            }
            sensor = SensorData {
                time: state.time,
                acceleration: -state.acceleration + Vec3::new(0.0, 0.0, env.gravity),
                angular_velocity: state.angular_velocity,
                barometric_height: state.position.z,
            };
        }
        log
    }
    pub fn step(
        &mut self,
        state: &mut RocketState,
        requested: ControlInputs,
        rocket: RocketParameters,
        env: Environment,
    ) {
        self.step_inner(state, requested, rocket, env, None);
    }
    /// HIL path: servo position is measured from the real actuator, so no
    /// gimbal delay/rate model is applied before the physics integration.
    pub fn step_with_actual_tvc(
        &mut self,
        state: &mut RocketState,
        requested: ControlInputs,
        actual_tvc: [f32; 2],
        rocket: RocketParameters,
        env: Environment,
    ) {
        self.step_inner(state, requested, rocket, env, Some(actual_tvc));
    }
    fn step_inner(
        &mut self,
        state: &mut RocketState,
        requested: ControlInputs,
        rocket: RocketParameters,
        env: Environment,
        actual_tvc: Option<[f32; 2]>,
    ) {
        let input = if let Some(actual) = actual_tvc {
            state.tvc = [actual[0].clamp(-1.0, 1.0), actual[1].clamp(-1.0, 1.0)];
            requested
        } else {
            state.delay.push_back(requested);
            let input = if state.delay.len() > rocket.tvc_delay_steps {
                state.delay.pop_front().unwrap_or_default()
            } else {
                ControlInputs::default()
            };
            let delta = env.dt * rocket.max_tvc_rate_rad_s;
            state.tvc[0] += (input.tvc[0].clamp(-1.0, 1.0) - state.tvc[0]).clamp(-delta, delta);
            state.tvc[1] += (input.tvc[1].clamp(-1.0, 1.0) - state.tvc[1]).clamp(-delta, delta);
            input
        };
        let thrust_value = if input.ignition {
            rocket.thrust_at(state.time)
        } else {
            0.0
        };
        let nozzle = Quat::axis_angle(
            Vec3::new(1.0, 0.0, 0.0),
            -(state.tvc[0] * rocket.max_tvc_angle_rad + rocket.tvc_misalignment_rad[0]),
        ) * Quat::axis_angle(
            Vec3::new(0.0, 1.0, 0.0),
            -(state.tvc[1] * rocket.max_tvc_angle_rad + rocket.tvc_misalignment_rad[1]),
        );
        let body_thrust = nozzle.rotate(Vec3::new(0.0, 0.0, thrust_value));
        let world_thrust = state.rotation.rotate(body_thrust);
        let relative_velocity = state.velocity - env.wind;
        let speed = relative_velocity.length();
        let direction = state.rotation.rotate(Vec3::UP);
        let aoa_cos = if speed > 1e-5 {
            relative_velocity.dot(direction) / speed
        } else {
            1.0
        };
        let cd = rocket.drag_cd_sideways
            + aoa_cos * aoa_cos * (rocket.drag_cd_axial - rocket.drag_cd_sideways);
        let drag = relative_velocity * (-0.5 * 1.225 * rocket.reference_area_m2 * cd * speed);
        state.acceleration =
            (world_thrust + drag + Vec3::new(0.0, 0.0, -env.gravity * rocket.mass_kg))
                / rocket.mass_kg;
        state.velocity += state.acceleration * env.dt;
        state.position += state.velocity * env.dt;
        state.time += env.dt;
        let moment = rocket.gimbal_com_offset.cross(body_thrust)
            + rocket
                .center_of_pressure_offset
                .cross(state.rotation.conjugate().rotate(drag));
        let w = state.angular_velocity;
        let i = rocket.inertia;
        let angular_acceleration = Vec3::new(
            ((i.y - i.z) * w.y * w.z + moment.x) / i.x,
            ((i.z - i.x) * w.z * w.x + moment.y) / i.y,
            ((i.x - i.y) * w.x * w.y + moment.z) / i.z,
        );
        state.angular_velocity += angular_acceleration * env.dt;
        let q_dot = state.rotation
            * Quat {
                x: state.angular_velocity.x,
                y: state.angular_velocity.y,
                z: state.angular_velocity.z,
                w: 0.0,
            }
            * 0.5;
        state.rotation = (state.rotation + q_dot * env.dt).normalized();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Ignited;
    impl ControlSystem for Ignited {
        fn reset(&mut self) {}
        fn update(&mut self, _: &SensorData) -> ControlInputs {
            ControlInputs {
                ignition: true,
                ..Default::default()
            }
        }
    }
    #[test]
    fn thrust_lifts_a_stable_rocket() {
        let mut sim = NumericalSimulation;
        let mut control = Ignited;
        let rocket = RocketParameters::from_configuration(
            &InertiaConfiguration {
                mass_g: 100.0,
                center_of_mass_offset_mm: 100.0,
                moment_of_inertia_kgm2: [0.001, 0.001, 0.0001],
            },
            &SimulationConfiguration::default(),
            0.002,
        );
        let result = sim.run(
            Environment {
                max_time: 1.0,
                ..Default::default()
            },
            rocket,
            RocketState::default(),
            &mut control,
            false,
        );
        assert!(result.last().unwrap().position.z > 0.0);
    }
    #[test]
    fn klima_d3_profile_interpolates_manufacturer_data() {
        let mut cfg = SimulationConfiguration::default();
        cfg.motor_thrust_profile = MotorThrustProfile::KlimaD3;
        let rocket =
            RocketParameters::from_configuration(&InertiaConfiguration::default(), &cfg, 0.002);
        assert_eq!(rocket.thrust_at(0.0), 0.0);
        assert!((rocket.thrust_at(0.603) - 9.01).abs() < 0.001);
        assert!((rocket.thrust_at(0.629) - 7.966).abs() < 0.002);
        assert_eq!(rocket.thrust_at(6.26), 0.0);
        assert_eq!(rocket.thrust_at(6.27), 0.0);
        assert!((rocket.burn_duration_s() - 6.26).abs() < f32::EPSILON);
    }
}
