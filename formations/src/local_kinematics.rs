use formations::boids::boid::Boid;
use glam::Vec3;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LocalKinematics {
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
}

impl From<LocalKinematics> for Boid {
    fn from(kinematics: LocalKinematics) -> Self {
        Boid {
            pos: kinematics.position.to_vec3a(),
            velo: kinematics.velocity.to_vec3a(),
            acc: kinematics.acceleration.to_vec3a(),
        }
    }
}
