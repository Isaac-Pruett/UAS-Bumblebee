use glam::Vec3;
use serde::Serialize;

#[derive(Debug, Clone, Copy, Serialize)]
pub struct LocalKinematics {
    pub position: Vec3,
    pub velocity: Vec3,
    pub acceleration: Vec3,
}
