use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Movement {
    pub pos: Vec3,
    pub velo: Vec3,
    pub acc: Vec3,

    pub force_en: bool,

    pub yaw: f32,
    pub yaw_rate: f32,
}
