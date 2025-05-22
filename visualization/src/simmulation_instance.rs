use bevy::math::*;
use light_tvc_rocket_simulation::control_system::*;
use light_tvc_rocket_simulation::model::*;
use light_tvc_rocket_simulation::neural_network::nn::{
    GeneticAlgorithm, NeuralNetwork, NeuralNetworkConfig, Population, Problem,
};
use light_tvc_rocket_simulation::simulation::*;
use rand::{Rng, rng};
use rand_distr::Normal;
use std::collections::VecDeque;
use std::f32::consts::PI;
use std::sync::Arc;
use bevy::render::render_resource::encase::private::RuntimeSizedArray;

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
        false,
    );

    ans
}
pub fn run_nn() {
    fn nn_cfg() -> NeuralNetworkConfig {
        NeuralNetworkConfig {
            layers: vec![3, 5, 5, 5, 3],
            activation_function: |x| x / (1.0 + x.abs()),
        }
    }

    let ga = GeneticAlgorithm {
        elitism: 2,
        mutants: 10,
        mutation_rounds: 2,
        recombinations: 10,
        recombination_rounds: 2,
        initial_sigma: 0.5,
        mutation_rate: 0.05,
        mutation_sigma: 0.25,
        recombination_min: 2,
        recombination_max: 20,
    };

    let fluctuation = (0.7..1.3);

    fn randomize() -> f32 {
        let fluctuation = (0.7..1.3);
        rng().random_range(fluctuation.clone())
    }

    fn randomize_abs(a: f32) -> f32 {
        let fluctuation = (-1.0..1.0);
        a * rng().random_range(fluctuation.clone())
    }

    let environment = Environment {
        g: 9.8,
        wind: Vec3::default(),
        max_time: 5.0,
        dt: 0.001,
        simulation_substeps: 10,
    };

    let rocket_count = 20;

    let rockets = (0..rocket_count)
        .map(|_| {
            (
                environment.clone(),
                RocketParameters {
                    mass: 0.2 * randomize(),
                    moment_of_inertia: Vec3::new(
                        0.0006 * randomize(),
                        0.0006 * randomize(),
                        0.00005 * randomize(),
                    ),
                    thrust: 3.0,
                    thrust_duration: 5.0,
                    max_tvc_angle: 5.0_f32.to_radians(),
                    motor_com_offset: Vec3::new(randomize_abs(0.003), randomize_abs(0.003), -0.1),
                    tvc_misalignment: Vec2::new(
                        randomize_abs(0.5_f32.to_radians()),
                        randomize_abs(0.5_f32.to_radians()),
                    ),
                    max_tvc_turn_rate: 7.0 * randomize(),
                    tvc_delay: (20.0 * randomize()) as u8,
                },
            )
        })
        .collect::<Vec<_>>();

    let problem = Problem { rockets };

    let population = Population {
        generation: 0,
        agents: vec![],
    };

    let problem_arc = Arc::new(problem);

    let mut current_population =
        ga.create_new_population(&population, problem_arc.clone(), Some(nn_cfg()));

    let max_target = current_population
        .agents
        .iter()
        .map(|(_, _, x)| x)
        .max_by(|a, b| a.partial_cmp(b).unwrap())
        .unwrap();




    for i in  (0..1000) {
        current_population = ga.create_new_population(&current_population, problem_arc.clone(), None);

        current_population.agents.iter().for_each(|(_, _, x)|
            if x.is_nan() {
                println!("NAN!")
            }
        );



        let (_, log, max_target) = current_population
            .agents
            .iter()
            .max_by(|(_, _,a), (_, _, b)| a.partial_cmp(b).unwrap())
            .unwrap();


        let exploded = log
            .iter()
            .filter(|l| l.iter().len() < 501)
            .count();

        println!("generation: {} target: {:.2}% exploded:{}", current_population.generation, max_target / 501.0 / rocket_count as f64 * 100.0, exploded);
    }
}
