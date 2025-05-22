use bevy::math::{Quat, Vec2, Vec3};
use std::collections::VecDeque;

pub struct RocketState {
    pub time: f32,
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub rotation: Quat,
    pub angular_velocity: Vec3,
    pub tvc: Vec2,
    pub tvc_delay_deque: VecDeque<ControlInputs>,
}

impl Clone for RocketState {
    fn clone(&self) -> Self {
        RocketState {
            time: self.time,
            position: self.position,
            velocity: self.velocity,
            acceleration: self.acceleration,
            rotation: self.rotation,
            angular_velocity: self.angular_velocity,
            tvc: self.tvc,
            tvc_delay_deque: VecDeque::new(),
        }
    }
}

pub struct RocketParameters {
    pub mass: f32,
    pub moment_of_inertia: Vec3,
    pub thrust: f32,
    pub thrust_duration: f32,
    pub max_tvc_angle: f32,
    pub motor_com_offset: Vec3,
    pub tvc_misalignment: Vec2,
    pub max_tvc_turn_rate: f32,
    pub tvc_delay: u8,
}

#[derive(Clone)]
pub struct Environment {
    pub g: f32,
    pub wind: Vec3,
    pub max_time: f32,
    pub dt: f32,
    pub simulation_substeps: u32,
}

pub struct SensorData {
    pub time: f32,
    pub acc: Vec3,
    pub gyr: Vec3,
    pub barometric_height: f32,
}

#[derive(Clone)]
pub struct ControlInputs {
    pub tvc: Vec2,
    pub ignition: bool,
    pub parachute: bool,
}

pub struct SimulationLog {
    pub time: f32,
    pub position: Vec3,
    pub rotation: Quat,
}

pub trait ControlSystem {
    fn reset(&mut self);
    fn update(&mut self, sd: &SensorData) -> ControlInputs;
}

pub trait Simulation {
    fn run(
        &mut self,
        environment: &Environment,
        rp: &RocketParameters,
        initial_state: &RocketState,
        control_system: &mut dyn ControlSystem,
        stop_of_failure: bool,
    ) -> Vec<SimulationLog>;
    fn make_step(
        &mut self,
        state: &mut RocketState,
        control_inputs: &ControlInputs,
        rocket_parameters: &RocketParameters,
        environment: &Environment,
    );
}
