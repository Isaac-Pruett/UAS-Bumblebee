use std::{sync::Arc, time::Duration};

use glam::Vec3;
use mavlink::dialects::common::{MavCmd, MavMessage, MavResult};
use serde::{Deserialize, Serialize};

use tokio::sync::Mutex;
use zenoh::Session;

use crate::{
    connection::{MavlinkAckRouter, MavlinkHandle},
    telem::AUTOPILOT_MODE,
};

#[derive(Deserialize, Serialize, Debug)]
pub struct LaunchCommand {
    pub height_m: f32,
    pub yaw: f32,
    pub lat: Option<f32>,
    pub lon: Option<f32>,
}

// #[doc = "| 1 (Pitch) | Minimum pitch (if airspeed sensor present), desired pitch without sensor| rad |"]
// #[doc = "| 2         | Empty       |  |"]
// #[doc = "| 3 (Ascend Rate)| Takeoff ascend rate| m/s |"]
// #[doc = "| 4 (Yaw)   | Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these| rad |"]
// #[doc = "| 5 (Y Position)| Y-axis position| m |"]
// #[doc = "| 6 (X Position)| X-axis position| m |"]
// #[doc = "| 7 (Z Position)| Z-axis position| m |"] MAV_CMD_NAV_TAKEOFF_LOCAL = 24

#[derive(Serialize, Deserialize)]
#[repr(u8)]
pub enum LaunchErrors {
    DeserializeError,
    CantFlyError,
    AckRejected,
    AckTimeout,
}

pub async fn register_launch(
    session: &Session,
    handle: &MavlinkHandle,
    ack_router: MavlinkAckRouter,
) {
    let handle = handle.clone();
    let ack_router = Arc::new(Mutex::new(ack_router));
    session
        .declare_queryable("control/move/launch")
        .callback(move |query| {
            let handle = handle.clone();
            if let Some(payload) = query.payload() {
                let launch = rmp_serde::from_slice::<LaunchCommand>(&payload.to_bytes()).ok();
                let ack_router = ack_router.clone();
                tokio::spawn(async move {
                    if let Some(launch) = launch {
                        if AUTOPILOT_MODE
                            .read()
                            .await
                            .as_ref()
                            .is_some_and(|mode| mode.can_fly())
                        {
                            handle
                                .send(MavMessage::COMMAND_LONG(
                                    mavlink::dialects::common::COMMAND_LONG_DATA {
                                        target_system: 1,
                                        target_component: 1,
                                        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
                                        confirmation: 0,
                                        param1: 0.0, // pitch
                                        param2: 0.0,
                                        param3: 0.0,
                                        param4: launch.yaw,                // yaw
                                        param5: launch.lat.unwrap_or(0.0), // latitude; 0 may mean current depending autopilot
                                        param6: launch.lon.unwrap_or(0.0), // longitude
                                        param7: launch.height_m,           // altitude
                                    },
                                ))
                                .await
                                .ok();

                            let ack_result = {
                                let mut ack_router = ack_router.lock().await;
                                ack_router.wait_ack(Duration::from_millis(2000)).await
                            };

                            match ack_result {
                                Ok(MavResult::MAV_RESULT_ACCEPTED) => {
                                    query.reply("control/move/launch/result", "Ok").await.ok();
                                }
                                Ok(_) => {
                                    let mut buf = Vec::new();
                                    let res = rmp_serde::encode::write(
                                        &mut buf,
                                        &LaunchErrors::AckRejected,
                                    );
                                    if res.is_ok() {
                                        query.reply_err(buf).await.ok();
                                    }
                                }
                                Err(_) => {
                                    let mut buf = Vec::new();
                                    let res = rmp_serde::encode::write(
                                        &mut buf,
                                        &LaunchErrors::AckTimeout,
                                    );
                                    if res.is_ok() {
                                        query.reply_err(buf).await.ok();
                                    }
                                }
                            }
                        } else {
                            let mut buf = Vec::new();
                            let res =
                                rmp_serde::encode::write(&mut buf, &LaunchErrors::CantFlyError);
                            if res.is_ok() {
                                query.reply_err(buf).await.ok();
                            }
                        }
                    } else {
                        let mut buf = Vec::new();
                        let res =
                            rmp_serde::encode::write(&mut buf, &LaunchErrors::DeserializeError);
                        if res.is_ok() {
                            query.reply_err(buf).await.ok();
                        }
                    }
                });
            }
        })
        .background()
        .await
        .expect("Failed to initialize launch subscriber");
}

#[derive(Deserialize, Serialize)]
pub struct LandCommand {
    pub descend_speed_m_s: f32,
    pub yaw: f32,
    pub acceptance_rad_m: f32,
    pub coords: Vec3,
}

async fn register_land(session: &Session, handle: &MavlinkHandle, ack_router: MavlinkAckRouter) {
    let handle = handle.clone();
    session
        .declare_queryable("control/move/land")
        .callback(move |query| {
            let payload = query.payload();
            if let Some(payload) = payload{
                let handle = handle.clone();
                let land_cmd = rmp_serde::from_slice::<LandCommand>(&payload.to_bytes()).ok();
                // Maximum accepted offset from desired landing position
                // - computed magnitude from spherical coordinates:
                // d = sqrt(x^2 + y^2 + z^2), which gives the
                // maximum accepted distance between the desired
                // landing position and the position where the
                // vehicle is about to land
                tokio::spawn(async move {
                if let Some(land_cmd) = land_cmd {
                    if  AUTOPILOT_MODE
                        .read()
                        .await
                        .as_ref()
                        .is_some_and(|mode| mode.can_fly()) {
                            handle.send(MavMessage::COMMAND_INT(
                                mavlink::dialects::common::COMMAND_INT_DATA {
                                    param1: 0.0,
                                    param2: land_cmd.acceptance_rad_m,
                                    param3: land_cmd.descend_speed_m_s,
                                    param4: land_cmd.yaw,
                                    x: land_cmd.coords.x as i32,
                                    y: land_cmd.coords.y as i32,
                                    z: land_cmd.coords.z,
                                    command: MavCmd::MAV_CMD_NAV_LAND_LOCAL,
                                    target_system: 1,
                                    target_component: 1,
                                    frame:
                                        mavlink::dialects::common::MavFrame::MAV_FRAME_LOCAL_OFFSET_NED,
                                    current: 0,
                                    autocontinue: 0,
                                },
                            ))
                            .await
                            .ok();}
                        query.reply("control/move/launch/result", "Ok").await.ok();
                }

                    });
                }
        })
        .background()
        .await
        .expect("Failed to initialize landing subscriber");
}

pub async fn register_launch_land(
    session: &Session,
    handle: &MavlinkHandle,
    launch_ack_channel: MavlinkAckRouter,
    land_ack_channel: MavlinkAckRouter,
) {
    register_land(session, handle, land_ack_channel).await;
    register_launch(session, handle, launch_ack_channel).await;
}
