use crate::{
    connection::{MavlinkAckChannel, MavlinkHandle},
    telem::autopilot_mode,
};

pub mod land;
pub mod launch;

use std::time::Duration;

use glam::Vec3;

use serde::{Deserialize, Serialize};

use crate::connection::SYSTEM_ID;

const ACK_TIMEOUT: Duration = Duration::from_secs(2);
const TARGET_COMPONENT: u8 = 1;

#[derive(Debug, Deserialize, Serialize)]
pub struct LaunchCommand {
    pub height_m: f32,
    pub yaw: f32,
    pub lat: Option<f32>,
    pub lon: Option<f32>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct LandCommand {
    pub descend_speed_m_s: f32,
    pub yaw: f32,
    pub acceptance_rad_m: f32,
    pub coords: Vec3,
}

#[derive(Debug, Serialize, Deserialize)]
#[repr(u8)]
pub enum LaunchErrors {
    DeserializeError,
    CantFlyError,
    AckRejected,
    AckTimeout,
}

fn can_fly() -> bool {
    autopilot_mode().is_some_and(|mode| mode.can_fly())
}

fn system_id() -> u8 {
    *SYSTEM_ID.get().unwrap_or(&0)
}

async fn reply_error(query: zenoh::query::Query, error: LaunchErrors) {
    let Ok(payload) = rmp_serde::to_vec(&error) else {
        return;
    };

    let _ = query.reply_err(payload).await;
}
pub async fn register_launch_land(
    handle: &MavlinkHandle,
    launch_ack_router: MavlinkAckChannel,
    land_ack_router: MavlinkAckChannel,
) {
    land::register_land(handle, land_ack_router).await;
    launch::register_launch(handle, launch_ack_router).await;
}
