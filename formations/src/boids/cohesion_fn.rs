use ::glam::Vec3;

pub fn cohesion_fn(pos: Vec3, vel: Vec3, desired_pos: Vec3, kp: f32, kd: f32) -> Vec3 {
    kp * (desired_pos - pos) - kd * vel
}
