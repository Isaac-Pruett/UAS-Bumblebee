use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use tokio::sync::watch;

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

    pub fn watch(&self) -> watch::Receiver<Option<FullState>> {
        watch::channel(Some(self.clone())).1
    }
}
