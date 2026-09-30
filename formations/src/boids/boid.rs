use ::glam::Vec3A;
use serde::{Deserialize, Serialize};

#[derive(PartialEq, Clone, Debug, Serialize, Deserialize, Default)]
pub struct Boid {
    pub pos: Vec3A,
    pub velo: Vec3A,
    pub acc: Vec3A,
}

impl Boid {
    pub const ZERO: Self = Self {
        pos: Vec3A::ZERO,
        velo: Vec3A::ZERO,
        acc: Vec3A::ZERO,
    };
}
