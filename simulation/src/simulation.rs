use crate::model::{
    ControlInputs, ControlSystem, Environment, RocketParameters, RocketState, SensorData,
    Simulation, SimulationLog,
};
use bevy::math::{Quat, Vec3};
use bevy::prelude::Transform;
use std::ops::{Mul, MulAssign};

pub struct NumericalSimulation {}

impl Simulation for NumericalSimulation {
    fn run(
        &mut self,
        environment: &Environment,
        rp: &RocketParameters,
        initial_state: &RocketState,
        control_system: &mut dyn ControlSystem,
        stop_of_failure: bool,
    ) -> Vec<SimulationLog> {
        let mut current_state = (*initial_state).clone();
        current_state.time = 0.0;
        control_system.reset();

        let mut ans: Vec<SimulationLog> = Vec::new();

        let mut sensor_data = SensorData {
            time: 0.0,
            acc: Default::default(),
            gyr: Default::default(),
            barometric_height: 0.0,
        };

        let up = Vec3::new(0.0, 0.0, 1.0);

        while current_state.time < environment.max_time {
            ans.push(SimulationLog {
                time: current_state.time,
                position: current_state.position,
                rotation: current_state.rotation,
            });

            if stop_of_failure && (current_state.rotation * up).z < 0.5 {
                break;
            };

            let control_inputs = control_system.update(&sensor_data);
            for i in 0..environment.simulation_substeps {
                self.make_step(&mut current_state, &control_inputs, rp, environment);
            }
            sensor_data.time = current_state.time;
            sensor_data.acc = -current_state.acceleration - Vec3::new(0.0, 0.0, -environment.g);
            sensor_data.barometric_height = current_state.position.z;
            sensor_data.gyr = current_state.angular_velocity;
        }

        ans
    }

    fn make_step(
        &mut self,
        state: &mut RocketState,
        ci: &ControlInputs,
        rp: &RocketParameters,
        env: &Environment,
    ) {
        let mg = Vec3::new(0.0, 0.0, -env.g * rp.mass);
        let thrust_value = if rp.thrust_duration > state.time {
            rp.thrust
        } else {
            0.0
        };

        state.tvc_delay_deque.push_back((*ci).clone());
        let ci = if state.tvc_delay_deque.len() > rp.tvc_delay as usize {
            state.tvc_delay_deque.pop_front().unwrap()
        } else {
            ControlInputs {
                tvc: Default::default(),
                ignition: false,
                parachute: false,
            }
        };

        let max_tvc_movement = env.dt * rp.max_tvc_turn_rate;
        state.tvc.x += (ci.tvc.y - state.tvc.x).clamp(-max_tvc_movement, max_tvc_movement);
        state.tvc.y += (ci.tvc.y - state.tvc.y).clamp(-max_tvc_movement, max_tvc_movement);

        let mut tvc_rotation = Quat::from_axis_angle(
            Vec3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            -(ci.tvc.x * rp.max_tvc_angle + rp.tvc_misalignment.x),
        ) * Quat::from_axis_angle(
            Vec3 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
            -(ci.tvc.y * rp.max_tvc_angle + rp.tvc_misalignment.y),
        );

        let thrust = tvc_rotation * Vec3::new(0.0, 0.0, thrust_value);

        let rocket_direction = state.rotation * Vec3::new(0.0, 0.0, 1.0);

        let velocity_length = state.velocity.length();

        let drag_aoa_cos = if velocity_length != 0.0 {
            state.velocity.dot(rocket_direction) / state.velocity.length()
        } else {
            1.0
        };
        let drag_cd =
            rp.drag_cd_90 + (drag_aoa_cos * drag_aoa_cos) * (rp.drag_cd_0 - rp.drag_cd_90);
        let drag_force =
            -state.velocity * (state.velocity.length()) * rp.drag_a_and_density_half * drag_cd;

        let total_force = mg + state.rotation * thrust + drag_force;
        let a = total_force / rp.mass;
        
        state.time += env.dt;

        state.acceleration = a;
        state.velocity += a * env.dt;
        state.position += state.velocity * env.dt;

        let moment = rp.motor_com_offset.cross(thrust)
            + rp.cp_com_offset
                .cross(state.rotation.conjugate() * drag_force);

        fn d_omega(w: &Vec3, i: &Vec3, m: &Vec3) -> Vec3 {
            Vec3::new(
                ((i.y - i.z) * w.y * w.z + m.x) / i.x,
                ((i.z - i.x) * w.z * w.x + m.y) / i.y,
                ((i.x - i.y) * w.x * w.y + m.z) / i.z,
            )
        }

        let d_omega_rk2_step1 = d_omega(&state.angular_velocity, &rp.moment_of_inertia, &moment);
        let d_omega_rk2_step2 = state.angular_velocity + d_omega_rk2_step1 * env.dt / 2.0;
        let d_omega_rk2_step3 = d_omega(&d_omega_rk2_step2, &rp.moment_of_inertia, &moment);

        state.angular_velocity += d_omega_rk2_step3 * env.dt;

        let dq = state.rotation
            * Quat::from_xyzw(
                state.angular_velocity.x,
                state.angular_velocity.y,
                state.angular_velocity.z,
                0.0,
            )
            * (env.dt / 2.0);
        state.rotation = (state.rotation + dq).normalize();

        //TODO: Euler equations
        //TODO:
    }
}
