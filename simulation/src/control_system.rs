use crate::model::{ControlInputs, ControlSystem, SensorData};
use crate::neural_network::nn::NeuralNetwork;
use bevy::math::{Quat, Vec2, Vec3};

pub struct PIDControlSystem {
    pub p_gain: f32,
    pub i_gain: f32,
    pub d_gain: f32,
    lats_t: f32,
    i_integral: Vec2,
    rotation: Quat,
}

impl PIDControlSystem {
    pub fn new(p: f32, i: f32, d: f32) -> PIDControlSystem {
        PIDControlSystem {
            p_gain: p,
            i_gain: i,
            d_gain: d,
            lats_t: 0.0,
            i_integral: Vec2::default(),
            rotation: Quat::default(),
        }
    }
}
impl ControlSystem for PIDControlSystem {
    fn reset(&mut self) {
        self.i_integral = Vec2::default();
        self.lats_t = 0.0;
        self.rotation = Quat::default();
    }

    fn update(&mut self, sd: &SensorData) -> ControlInputs {
        let dt = sd.time - self.lats_t;
        self.lats_t = sd.time;

        let dq = self.rotation * Quat::from_xyzw(sd.gyr.x, sd.gyr.y, sd.gyr.z, 0.0) * (dt / 2.0);

        self.rotation = (self.rotation + dq).normalize();

        let deviation = {
            let vertical = Vec3::new(0.0, 0.0, 1.0);
            let direction = self.rotation.conjugate() * vertical;
            *(Vec2::new(direction.x, direction.y)
                .try_normalize()
                .get_or_insert(Vec2::default()))
                * direction.dot(vertical).clamp(-1.0, 1.0).acos()
        };

        let xp = -deviation.y * self.p_gain;
        let xd = -sd.gyr.x * self.d_gain;
        self.i_integral.x += -deviation.y * dt;
        let xi = self.i_integral.x * self.i_gain;
        let x = (xp + xd + xi).clamp(-1.0, 1.0);

        let yp = deviation.x * self.p_gain;
        let yd = -sd.gyr.y * self.d_gain;
        self.i_integral.y += deviation.x * dt;
        let yi = self.i_integral.y * self.i_gain;
        let y = (yp + yd + yi).clamp(-1.0, 1.0);

        ControlInputs {
            tvc: Vec2::new(x, y),
            ignition: false,
            parachute: false,
        }
    }
}

pub struct NNControlSystem<'a> {
    pub nn: &'a NeuralNetwork,

    temp_value_x: f32,
    temp_value_y: f32,
    lats_t: f32,
    rotation: Quat,
}

impl<'a> NNControlSystem<'a> {
    pub fn new(nn: &'a NeuralNetwork) -> NNControlSystem<'a> {
        NNControlSystem {
            nn,
            temp_value_x: 0.0,
            temp_value_y: 0.0,
            lats_t: 0.0,
            rotation: Default::default(),
        }
    }
}

impl<'a> ControlSystem for NNControlSystem<'a> {
    fn reset(&mut self) {
        self.temp_value_x = 0.0;
        self.temp_value_y = 0.0;
        self.lats_t = 0.0;
        self.rotation = Quat::default();
    }

    fn update(&mut self, sd: &SensorData) -> ControlInputs {
        let dt = sd.time - self.lats_t;
        self.lats_t = sd.time;

        let dq = self.rotation * Quat::from_xyzw(sd.gyr.x, sd.gyr.y, sd.gyr.z, 0.0) * (dt / 2.0);

        self.rotation = (self.rotation + dq).normalize();

        let deviation = {
            let vertical = Vec3::new(0.0, 0.0, 1.0);
            let direction = self.rotation.conjugate() * vertical;
            *(Vec2::new(direction.x, direction.y)
                .try_normalize()
                .get_or_insert(Vec2::default()))
                * direction.dot(vertical).clamp(-1.0, 1.0).acos()
        };

        let ans_x = self
            .nn
            .calculate(vec![-deviation.y, sd.gyr.x, self.temp_value_x]);
        let ans_x_2 = self
            .nn
            .calculate(vec![deviation.y, -sd.gyr.x, -self.temp_value_x]);
        let ans_y = self
            .nn
            .calculate(vec![deviation.x, sd.gyr.y, self.temp_value_y]);
        let ans_y_2 = self
            .nn
            .calculate(vec![-deviation.x, -sd.gyr.y, -self.temp_value_y]);

        let x = ans_x[0] - ans_x_2[0];
        let y = ans_y[0] - ans_y_2[0];

        self.temp_value_x = ans_x[1] - ans_x_2[1];
        self.temp_value_y = ans_y[1] - ans_y_2[1];

        ControlInputs {
            tvc: Vec2::new(x.clamp(-1.0, 1.0), y.clamp(-1.0, 1.0)),
            ignition: false,
            parachute: false,
        }
    }
}
