use crate::movement::{MotionType, Movement};
use glam::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize)]
struct MotionPacket {
    pub waypoints: Vec<Vec3>,
    pub pipe_radius_m: f32,
    pub max_speed_m_s: f32,
    pub motiontype: MotionType,
}
