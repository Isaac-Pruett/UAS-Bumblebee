use ::glam::Vec3;

#[derive(PartialEq, Clone, Debug)]
pub struct Boid {
    pub pos: Vec3,
    pub velo: Vec3,
    pub acc: Vec3,
}
