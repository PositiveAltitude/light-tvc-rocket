//! Flight-control policies, kept independent from the simulator and UI.

use light_robot_core_simulation::{ControlInputs, ControlSystem, Quat, SensorData, Vec3};

pub struct PidControlSystem {
    pub p_gain: f32,
    pub i_gain: f32,
    pub d_gain: f32,
    last_time: f32,
    integral: [f32; 2],
    rotation: Quat,
}

impl PidControlSystem {
    pub fn new(p_gain: f32, i_gain: f32, d_gain: f32) -> Self {
        Self {
            p_gain,
            i_gain,
            d_gain,
            last_time: 0.0,
            integral: [0.0; 2],
            rotation: Quat::IDENTITY,
        }
    }

    /// Seeds the estimator for simulation/HIL runs, where the simulated
    /// vehicle attitude is known at t=0 instead of coming from a real IMU.
    pub fn set_estimated_rotation(&mut self, rotation: Quat) {
        self.rotation = rotation;
    }
}

fn deviation(rotation: Quat) -> [f32; 2] {
    let direction = rotation.conjugate().rotate(Vec3::UP);
    let length = (direction.x * direction.x + direction.y * direction.y).sqrt();
    if length < 1e-6 {
        [0.0; 2]
    } else {
        let angle = direction.z.clamp(-1.0, 1.0).acos();
        [direction.x / length * angle, direction.y / length * angle]
    }
}

impl ControlSystem for PidControlSystem {
    fn reset(&mut self) {
        self.last_time = 0.0;
        self.integral = [0.0; 2];
        self.rotation = Quat::IDENTITY;
    }

    fn update(&mut self, sensor: &SensorData) -> ControlInputs {
        let dt = (sensor.time - self.last_time).max(0.0);
        self.last_time = sensor.time;
        let q_dot = self.rotation
            * Quat {
                x: sensor.angular_velocity.x,
                y: sensor.angular_velocity.y,
                z: sensor.angular_velocity.z,
                w: 0.0,
            }
            * 0.5;
        self.rotation = (self.rotation + q_dot * dt).normalized();
        let deviation = deviation(self.rotation);
        self.integral[0] = (self.integral[0] - deviation[1] * dt).clamp(-1.0, 1.0);
        self.integral[1] = (self.integral[1] + deviation[0] * dt).clamp(-1.0, 1.0);
        ControlInputs {
            tvc: [
                (-deviation[1] * self.p_gain - sensor.angular_velocity.x * self.d_gain
                    + self.integral[0] * self.i_gain)
                    .clamp(-1.0, 1.0),
                (deviation[0] * self.p_gain - sensor.angular_velocity.y * self.d_gain
                    + self.integral[1] * self.i_gain)
                    .clamp(-1.0, 1.0),
            ],
            ignition: true,
            parachute: false,
        }
    }
}
