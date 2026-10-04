use std::f32;

use glam::Vec3;
use mavlink::dialects::common::{PositionTargetTypemask, SET_POSITION_TARGET_LOCAL_NED_DATA};
use serde::{Deserialize, Serialize};

use crate::connection::SYSTEM_ID;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Movement {
    pub pos: Vec3,
    pub velo: Vec3,
    pub acc: Vec3,

    pub force_en: bool,

    pub yaw: f32,
    pub yaw_rate: f32,
}

impl Movement {
    pub fn as_packet(&self) -> Option<SET_POSITION_TARGET_LOCAL_NED_DATA> {
        let f = self.flag();
        // println!("MASK: {}", format!("0x{:04X}", f));

        if let Some(mask) = PositionTargetTypemask::from_bits(f) {
            Some(SET_POSITION_TARGET_LOCAL_NED_DATA {
                time_boot_ms: 0,
                target_system: *SYSTEM_ID.get().unwrap_or(&0),
                target_component: 1,
                coordinate_frame: mavlink::dialects::common::MavFrame::MAV_FRAME_LOCAL_NED, // WILL BE OVERRIDDEN
                type_mask: mask,

                x: clean(self.pos.x),
                y: clean(self.pos.y),
                z: clean(self.pos.z),
                vx: clean(self.velo.x),
                vy: clean(self.velo.y),
                vz: clean(self.velo.z),
                afx: clean(self.acc.x),
                afy: clean(self.acc.y),
                afz: clean(self.acc.z),
                yaw: clean(self.yaw),
                yaw_rate: clean(self.yaw_rate),
            })
        } else {
            None
        }
    }

    fn flag(&self) -> u16 {
        (bitnan(self.pos.x) << 0)
            | (bitnan(self.pos.y) << 1)
            | (bitnan(self.pos.z) << 2)
            | (bitnan(self.velo.x) << 3)
            | (bitnan(self.velo.y) << 4)
            | (bitnan(self.velo.z) << 5)
            | (bitnan(self.acc.x) << 6)
            | (bitnan(self.acc.y) << 7)
            | (bitnan(self.acc.z) << 8)
            | (((self.force_en) as u16) << 9)
            | (bitnan(self.yaw) << 10)
            | (bitnan(self.yaw_rate) << 11)
    }
}

fn bitnan(i: f32) -> u16 {
    match i.is_nan() {
        true => 1,
        false => 0,
    }
}

fn clean(i: f32) -> f32 {
    if i.is_nan() { 0.0 } else { i }
}

#[inline]
pub fn ned_to_enu(v: Vec3) -> Vec3 {
    Vec3::new(v.y, v.x, -v.z)
}

#[inline]
pub fn flu_to_frd(v: Vec3) -> Vec3 {
    Vec3::new(v.x, -v.y, -v.z)
}

#[inline]
pub fn frame_swap<F>(m: &mut Movement, tx: F)
where
    F: Fn(Vec3) -> Vec3,
{
    m.pos = tx(m.pos);
    m.velo = tx(m.velo);
    m.acc = tx(m.acc);
}

#[inline]
pub fn nan_vel(m: &mut Movement) {
    m.velo = Vec3::NAN;
    m.yaw_rate = f32::NAN;
}

#[inline]
pub fn nan_acc(m: &mut Movement) {
    m.acc = Vec3::NAN;
}

#[inline]
pub fn nan_pos(m: &mut Movement) {
    m.pos = Vec3::NAN;
    m.yaw = f32::NAN;
}

#[inline]
pub fn enu_pos(m: &mut Movement) {
    frame_swap(m, ned_to_enu);
    nan_vel(m);
    nan_acc(m);
}

#[inline]
pub fn nan_yaw(m: &mut Movement) {
    m.yaw = f32::NAN;
    m.yaw_rate = f32::NAN;
}
