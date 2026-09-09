use crate::control_system::{NNControlSystem, PIDControlSystem};
use crate::model::{
    ControlSystem, Environment, RocketParameters, RocketState, Simulation, SimulationLog,
};
use crate::simulation::NumericalSimulation;
use bevy::math::{Quat, Vec3};
use bevy::utils::HashSet;
use rand::seq::SliceRandom;
use rand::{Rng, rng};
use rand_distr::Normal;
use std::sync::Arc;
use std::thread;

#[derive(Clone)]
pub struct NeuralNetworkConfig {
    pub layers: Vec<u32>,
    pub activation_function: fn(f32) -> f32,
}

impl NeuralNetworkConfig {
    pub fn parameters_count(&self) -> u32 {
        let mut current = self.layers[0];
        let mut sum = 0u32;

        (1..self.layers.len()).for_each(|i| {
            sum += self.layers[i] * (current + 1);
            current = self.layers[i];
        });

        sum
    }
}

pub struct NeuralNetwork {
    pub config: NeuralNetworkConfig,
    pub parameters: Vec<f32>,
}

impl Clone for NeuralNetwork {
    fn clone(&self) -> Self {
        NeuralNetwork {
            config: self.config.clone(),
            parameters: self.parameters.clone(),
        }
    }
}

impl NeuralNetwork {
    pub fn calculate(&self, inputs: Vec<f32>) -> Vec<f32> {



        let mut previous_layer: Vec<f32> = Vec::new();
        let mut current_layer: Vec<f32> = Vec::new();
        for i in inputs {
            previous_layer.push(i);
        }

        let mut parameter_index = 0u32;
        for layer_size in self.config.layers.iter().skip(1) {
            current_layer.clear();
            (0..*layer_size).for_each(|_| current_layer.push(0.0));

            for current in current_layer.iter_mut() {
                for previous in previous_layer.iter() {
                    *current += previous * self.parameters[parameter_index as usize];
                    parameter_index += 1;
                }
                *current += self.parameters[parameter_index as usize];
                parameter_index += 1;
                *current = (self.config.activation_function)(*current);
            }
            previous_layer = current_layer.clone();
        }

        current_layer
    }
}

#[derive(Clone)]
pub struct Population {
    pub generation: u64,
    pub agents: Vec<(NeuralNetwork, Vec<Vec<SimulationLog>>, f64)>,
}

pub struct Problem {
    pub rockets: Vec<(Environment, RocketParameters)>,
}

impl Problem {
    pub fn calculate_target(&self, nn: &NeuralNetwork) -> (Vec<Vec<SimulationLog>>, f64) {
        let mut control_system = NNControlSystem::new(nn);
        // let mut control_system = PIDControlSystem::new(2.0, 2.0, 0.8);

        let res = self
            .rockets
            .iter()
            .map(|(env, rp)| {
                let mut s = NumericalSimulation {};
                let is = RocketState {
                    time: 0.0,
                    position: Default::default(),
                    velocity: Default::default(),
                    acceleration: Default::default(),
                    rotation: Default::default(),
                    angular_velocity: Default::default(),
                    tvc: Default::default(),
                    tvc_delay_deque: Default::default(),
                };
                control_system.reset();
                let sl = s.run(env, rp, &is, &mut control_system, true);

                let mut target = sl
                    .iter()
                    .map(|sl| {
                        let up = Vec3::new(0.0, 0.0, 1.0);
                        (sl.rotation * up).z.powi(2) as f64
                    })
                    .sum::<f64>();
                target = target / 501.0;

                if sl.len() == 501 {target += 9.0};
                (sl, target)
            })
            .collect::<Vec<_>>();

        let mut total_target: f64 = res.iter().map(|(_, x)| x).sum();

        (
            res.into_iter().map(|(x, _)| x).collect::<Vec<_>>(),
            total_target,
        )
    }
}

pub struct GeneticAlgorithm {
    pub elitism: u32,
    pub mutants: u32,
    pub mutation_rounds: u32,
    pub recombinations: u32,
    pub recombination_rounds: u32,

    pub initial_sigma: f32,

    pub mutation_rate: f64,
    pub mutation_sigma: f64,

    pub recombination_min: u32,
    pub recombination_max: u32,
}

impl GeneticAlgorithm {
    pub fn population_size(&self) -> u32 {
        self.elitism
            + self.mutants * self.mutation_rounds
            + self.recombinations * self.recombination_rounds * 2
    }

    pub fn elimination_count(&self) -> u32 {
        self.elitism
            + self.mutants * (self.mutation_rounds - 1)
            + self.recombinations * (self.recombination_rounds - 1) * 2
    }

    pub fn create_new_population(
        &self,
        pop: &Population,
        problem: Arc<Problem>,
        generate_random: Option<NeuralNetworkConfig>,
    ) -> Population {
        let mut new_agents: Vec<NeuralNetwork> = Vec::new();

        match generate_random {
            Some(cfg) => {
                let initial_nns = (0..self.population_size()).for_each(|_| {
                    let cfg = cfg.clone();

                    let initial_distr = Normal::new(0.0_f32, self.initial_sigma).unwrap();

                    let params = (0..cfg.parameters_count())
                        .map(|_| rng().sample(initial_distr))
                        .collect::<Vec<_>>();
                    new_agents.push(NeuralNetwork {
                        config: cfg,
                        parameters: params,
                    });
                });
            }
            None => {
                let mut tmp = Vec::new();

                for (a, b, c) in pop.agents.iter() {
                    tmp.push((a, *c))
                }
                tmp.sort_by(|(_, a), (_, b)| b.partial_cmp(a).unwrap());

                tmp.iter().take(self.elitism as usize).for_each(|(a, c)| {
                    new_agents.push((*a).clone());
                });
                _ = tmp.drain(tmp.len().saturating_sub(self.elimination_count() as usize)..);

                tmp.shuffle(&mut rng());

                let mutation_distr = Normal::new(0.0_f32, self.mutation_sigma as f32).unwrap();

                let mutants_iter = tmp.drain(tmp.len().saturating_sub(self.mutants as usize)..);

                mutants_iter.for_each(|(mutant, _)| {
                    for i in 0..self.mutation_rounds {
                        let mut agent = (*mutant).clone();
                        agent.parameters.iter_mut().for_each(|x| {
                            if rng().random_bool(self.mutation_rate) {
                                *x += rng().sample(mutation_distr);
                            }
                        });
                        new_agents.push(agent)
                    }
                });

                for i in 0..self.recombination_rounds {
                    tmp.shuffle(&mut rng());

                    tmp.chunks_mut(2).for_each(|chunk| {
                        let mut a1 = chunk[0].0.clone();
                        let mut a2 = chunk[1].0.clone();

                        let swaps =
                            rng().random_range(self.recombination_min..=self.recombination_max);

                        let mut swap_inexes: HashSet<usize> = HashSet::new();

                        while swap_inexes.len() < swaps as usize {
                            swap_inexes.insert(rng().random_range(0usize..a1.parameters.len()));
                        }

                        for index in swap_inexes.iter() {
                            let tmp = a1.parameters[*index];
                            a1.parameters[*index] = a2.parameters[*index];
                            a2.parameters[*index] = tmp;
                        }

                        new_agents.push(a1);
                        new_agents.push(a2);
                    });
                }
            }
        }

        let a = new_agents
            .into_iter()
            .map(|agent| {
                let problem = problem.clone();

                thread::spawn(move || {
                    let tmp = problem.calculate_target(&agent);
                    (agent, tmp.0, tmp.1)
                })
            })
            .collect::<Vec<_>>();

        let res = a.into_iter().map(|h| h.join().unwrap()).collect::<Vec<_>>();

        Population {
            generation: pop.generation + 1,
            agents: res,
        }
    }
}
