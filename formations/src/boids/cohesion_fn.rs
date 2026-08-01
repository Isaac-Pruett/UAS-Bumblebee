use ::glam::Vec3A;

pub fn cohesion_fn(pos: Vec3A, vel: Vec3A, desired_pos: Vec3A, kp: f32, kd: f32) -> Vec3A {
    kp * (desired_pos - pos) - kd * vel
}
