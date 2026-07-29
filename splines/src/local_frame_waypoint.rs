use ::glam::Vec3;
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

impl From<&[Vec3; 3]> for LocalFrameWaypoint {
    fn from(value: &[Vec3; 3]) -> Self {
        LocalFrameWaypoint {
            x: value[0].x,
            y: value[0].y,
            z: value[0].z,
            vx: value[1].x,
            vy: value[1].y,
            vz: value[1].z,
            afx: value[2].x,
            afy: value[2].y,
            afz: value[2].z,
            force_en: false,
            yaw: 0.0,
            yaw_rate: 0.0,
        }
    }
}
