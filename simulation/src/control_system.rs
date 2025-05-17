use crate::model::{ControlInputs, ControlSystem, SensorData};
use bevy::math::Vec2;

struct PIDControlSystem {
    pub p_gain: f32,
    pub i_gain: f32,
    pub d_gain: f32,
    lats_t: f32,
    i_integral: f32,
}

impl PIDControlSystem {
    pub fn new(p: f32, i: f32, d: f32) -> PIDControlSystem {
        PIDControlSystem {
            p_gain: p,
            i_gain: i,
            d_gain: d,
            lats_t: 0.0,
            i_integral: 0.0,
        }
    }
}
impl ControlSystem for PIDControlSystem {
    fn reset(&mut self) {
        self.i_integral = 0.0;
        self.lats_t = 0.0;
    }

    fn update(&mut self, sd: &SensorData) -> ControlInputs {
        ControlInputs {
            tvc: Vec2::default(),
            ignition: false,
            parachute: false,
        }
    }
}
