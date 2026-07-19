use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Clone)]
pub struct FullState {
    pub pos: Vec3,
    pub velo: Vec3,
    pub orientation: Quat,
    pub angular_velocity: Vec3,
    pub yaw: f32,
    pub attitude_time_ms: u32,
    pub position_time_ms: u32,
}

impl FullState {
    pub fn timestamps_are_close(&self, tolerance_ms: u32) -> bool {
        self.position_time_ms.abs_diff(self.attitude_time_ms) <= tolerance_ms
    }
}

pub type LocalPosition = (Vec3, Vec3, u32);

pub type Attitude = (
    f32, // roll
    f32, // pitch
    f32, // yaw
    f32, // roll speed
    f32, // pitch speed
    f32, // yaw speed
    u32, // time_boot_ms
);
