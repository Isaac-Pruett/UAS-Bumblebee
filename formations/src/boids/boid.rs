use ::glam::Vec3A;

#[derive(PartialEq, Clone, Debug)]
pub struct Boid {
    pub pos: Vec3A,
    pub velo: Vec3A,
    pub acc: Vec3A,
}
