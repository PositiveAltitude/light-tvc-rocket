use bevy::math::{Quat, Vec2, Vec3};

#[derive(Clone)]
pub struct RocketState {
    pub time: f32,
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
    pub rotation: Quat,
    pub angular_velocity: Quat,
}

// pub struct SimulationState {
//     pub rocket_states: RocketState,
//     pub tvc_angle: Vec2,
//     pub tvc_velocity: Vec2,
// }

pub struct RocketParameters {
    pub mass: f32,
    pub moment_of_inertia: Vec3,
    pub thrust: f32,
    pub thrust_duration: f32,

}
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
    pub gyr: Quat,
    pub barometric_height: f32,
}

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

    fn run(&mut self, environment: &Environment, rp: &RocketParameters, initial_state: &RocketState, control_system: &mut dyn ControlSystem) -> Vec<SimulationLog>;
    fn make_step(&mut self, state: &mut RocketState, control_inputs: &ControlInputs, rocket_parameters: &RocketParameters, environment: &Environment);
}