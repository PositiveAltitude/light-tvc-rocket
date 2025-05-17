use crate::model::{
    ControlInputs, ControlSystem, Environment, RocketParameters, RocketState, SensorData,
    Simulation, SimulationLog,
};
use bevy::math::Vec3;

pub struct NumericalSimulation {}

impl Simulation for NumericalSimulation {
    fn run(
        &mut self,
        environment: &Environment,
        rp: &RocketParameters,
        initial_state: &RocketState,
        control_system: &mut dyn ControlSystem,
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

        while current_state.time < environment.max_time {
            ans.push(SimulationLog {
                time: current_state.time,
                position: current_state.position,
                rotation: current_state.rotation,
            });
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
        control_inputs: &ControlInputs,
        rocket_parameters: &RocketParameters,
        environment: &Environment,
    ) {
        let mg = Vec3::new(0.0, 0.0, -environment.g * rocket_parameters.mass);
        let thrust_value = if rocket_parameters.thrust_duration > state.time {
            rocket_parameters.thrust
        } else {
            0.0
        };
        let thrust = Vec3::new(0.0, 0.0, thrust_value);
        
        let total_force = mg + thrust;
        let a = total_force / rocket_parameters.mass;
        
        state.time += environment.dt;
        
        state.acceleration = a;
        state.velocity += a * environment.dt;
        state.position += state.velocity * environment.dt;
        
        //TODO: orientation change
        //TODO: 
    }
}
