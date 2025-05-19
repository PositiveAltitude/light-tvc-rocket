use std::collections::VecDeque;
use std::f32::consts::PI;
use bevy::math::*;
use light_tvc_rocket_simulation::control_system::*;
use light_tvc_rocket_simulation::model::*;
use light_tvc_rocket_simulation::simulation::*;
pub fn run_simulation() -> Vec<SimulationLog> {
    let environment = Environment {
        g: 9.8,
        wind: Vec3::default(),
        max_time: 5.0,
        dt: 0.001,
        simulation_substeps: 10,
    };

    let rocket_parameters = RocketParameters {
        mass: 0.2,
        moment_of_inertia: Vec3::new(0.001, 0.001, 0.00005),
        thrust: 3.0,
        thrust_duration: 5.0,
        max_tvc_angle: 5.0_f32.to_radians(),
        motor_com_offset: Vec3::new(0.0, 0.0, -0.1),
        tvc_misalignment: Vec2::new(0.2_f32.to_radians(), 1.0_f32.to_radians()),
        max_tvc_turn_rate: 5.0,
        tvc_delay: 20,
    };

    let initial_state = RocketState {
        time: 0.0,
        position: Vec3::default(),
        velocity: Vec3::default(),
        acceleration: Vec3::default(),
        rotation: Quat::default(),
        angular_velocity: Vec3::new(0.00, -0.00, 0.0),
        tvc: Vec2::default(),
        tvc_delay_deque: VecDeque::new(),
    };

    let mut control_system = PIDControlSystem::new(1.0, 2.0, 0.2);

    let mut simulation = NumericalSimulation {};

    let ans = simulation.run(
        &environment,
        &rocket_parameters,
        &initial_state,
        &mut control_system,
    );

    ans
}
