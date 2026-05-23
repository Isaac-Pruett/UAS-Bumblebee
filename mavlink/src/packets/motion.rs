use std::f32;

use mavlink::dialects::common::{PositionTargetTypemask, SET_POSITION_TARGET_LOCAL_NED_DATA};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Movement {
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

#[inline]
pub fn ned_all(_: &mut Movement) {}

#[inline]
pub fn enu_all(m: &mut Movement) {
    let t = m.clone();

    m.x = t.y;
    m.y = t.x;
    m.z = -t.z;

    m.vx = t.vy;
    m.vy = t.vx;
    m.vz = -t.vz;

    m.afx = t.afy;
    m.afy = t.afx;
    m.afz = -t.afz;
}

#[inline]
pub fn nan_vel(m: &mut Movement) {
    m.vx = f32::NAN;
    m.vy = f32::NAN;
    m.vz = f32::NAN;
}

#[inline]
pub fn nan_acc(m: &mut Movement) {
    m.afx = f32::NAN;
    m.afy = f32::NAN;
    m.afz = f32::NAN;
}

#[inline]
pub fn nan_pos(m: &mut Movement) {
    m.x = f32::NAN;
    m.y = f32::NAN;
    m.z = f32::NAN;
}

#[inline]
pub fn enu_pos(m: &mut Movement) {
    enu_all(m);
    nan_vel(m);
    nan_acc(m);
}
