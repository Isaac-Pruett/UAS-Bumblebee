use std::f32;

use mavlink::dialects::common::{PositionTargetTypemask, SET_POSITION_TARGET_LOCAL_NED_DATA};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct Movement {
    x: f32,
    y: f32,
    z: f32,

    vx: f32,
    vy: f32,
    vz: f32,
    afx: f32,
    afy: f32,
    afz: f32,

    force_en: bool,

    yaw: f32,
    yaw_rate: f32,
}

impl Movement {
    pub fn as_packet(&self) -> Option<SET_POSITION_TARGET_LOCAL_NED_DATA> {
        let f = self.flag();
        if let Some(mask) = PositionTargetTypemask::from_bits(f) {
            Some(SET_POSITION_TARGET_LOCAL_NED_DATA {
                time_boot_ms: 0,
                target_system: 1,
                target_component: 1,
                coordinate_frame: mavlink::dialects::common::MavFrame::MAV_FRAME_LOCAL_NED,
                type_mask: mask,

                x: self.x,
                y: self.y,
                z: self.z,
                vx: self.vx,
                vy: self.vy,
                vz: self.vz,
                afx: self.afx,
                afy: self.afy,
                afz: self.afz,
                yaw: self.yaw,
                yaw_rate: self.yaw_rate,
            })
        } else {
            None
        }
    }

    fn flag(&self) -> u16 {
        (bitnan(self.x)<< 0) | (bitnan(self.y)<< 0) | (bitnan(self.z)<< 0) | // x, y, z
            (bitnan(self.vx)<< 3) | (bitnan(self.vy)<< 4) | (bitnan(self.vz)<< 5) | // vx, vy, vz
            (bitnan(self.afx)<< 6) | (bitnan(self.afy)<< 7) | (bitnan(self.afz)<< 8) | // ax, ay, az
            (((!self.force_en) as u16) << 10) |                       // force/accel flag ignored
            (bitnan(self.yaw) << 11) | (bitnan(self.yaw_rate)<< 12) // yaw, yaw_rate
    }
}

fn bitnan(i: f32) -> u16 {
    match i.is_nan() {
        true => 0,
        false => 1,
    }
}
