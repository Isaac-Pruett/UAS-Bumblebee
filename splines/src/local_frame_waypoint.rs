use std::f32;

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LocalFrameWaypoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,

    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    pub afx: f32,
    pub afy: f32,
    pub afz: f32,

    pub force_en: bool,

    pub yaw: f32,
    pub yaw_rate: f32,
}
