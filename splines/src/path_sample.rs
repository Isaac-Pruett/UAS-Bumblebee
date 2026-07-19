use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct PathSample {
    pub segment: usize,
    pub t: f32,
    pub distance: f32,
    pub position: Vec3,
    pub speed_limit: f32,
}
