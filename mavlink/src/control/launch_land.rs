use glam::Vec3;
use mavlink::dialects::common::{MavCmd, MavMessage};
use serde::{Deserialize, Serialize};
use tokio::runtime::Handle;
use zenoh::Session;

use crate::{
    connection::{MavlinkHandle, MavlinkRouter},
    control::launch_land,
    packets::coords::LinearDistance,
};

#[derive(Deserialize, Serialize)]
pub struct LaunchCommand {
    pub height_m: f32,
    pub ascend_speed: f32, // m/s
    pub yaw: f32,
}

// #[doc = "| 1 (Pitch) | Minimum pitch (if airspeed sensor present), desired pitch without sensor| rad |"]
// #[doc = "| 2         | Empty       |  |"]
// #[doc = "| 3 (Ascend Rate)| Takeoff ascend rate| m/s |"]
// #[doc = "| 4 (Yaw)   | Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these| rad |"]
// #[doc = "| 5 (Y Position)| Y-axis position| m |"]
// #[doc = "| 6 (X Position)| X-axis position| m |"]
// #[doc = "| 7 (Z Position)| Z-axis position| m |"] MAV_CMD_NAV_TAKEOFF_LOCAL = 24

pub async fn register_launch(session: &Session, handle: &MavlinkHandle) {
    let handle = handle.clone();
    let sub =
        session
            .declare_subscriber("control/move/launch")
            .callback(move |sample| {
                let handle = handle.clone();
                let launch =
                    rmp_serde::from_slice::<LaunchCommand>(&sample.payload().to_bytes()).ok();
                tokio::spawn(async move {
                    if let Some(launch) = launch {
                        handle.send(MavMessage::COMMAND_INT(
                        mavlink::dialects::common::COMMAND_INT_DATA {
                            param1: 0.0,
                            param2: 0.0,
                            param3: launch.ascend_speed,
                            param4: launch.yaw,
                            x: 0,
                            y: 0,
                            z: launch.height_m,
                            command: MavCmd::MAV_CMD_NAV_TAKEOFF_LOCAL,
                            target_system: 1,
                            target_component: 1,
                            frame: mavlink::dialects::common::MavFrame::MAV_FRAME_LOCAL_OFFSET_NED,
                            current: 0,
                            autocontinue: 0,
                        },
                    )).await.ok();
                    }
                });
            })
            .background()
            .await
            .expect("Failed to initialize launch subscriber");
}

#[derive(Deserialize, Serialize)]
pub struct LandCommand {
    pub local_pos: Vec3,
    pub descend_speed_m_s: f32,
    pub yaw: f32,
    pub acceptance_rad_m: f32,
    pub coords: Vec3,
}

async fn register_land(session: &Session, handle: &MavlinkHandle) {
    let handle = handle.clone();
    session
        .declare_subscriber("control/move/land")
        .callback(move |sample| {
            let handle = handle.clone();
            let land_cmd = rmp_serde::from_slice::<LandCommand>(&sample.payload().to_bytes()).ok();
            // Maximum accepted offset from desired landing position
            // - computed magnitude from spherical coordinates:
            // d = sqrt(x^2 + y^2 + z^2), which gives the
            // maximum accepted distance between the desired
            // landing position and the position where the
            // vehicle is about to land
            if let Some(land_cmd) = land_cmd {
                tokio::spawn(async move {
                    handle
                        .send(MavMessage::COMMAND_INT(
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
                        .ok()
                });
            }
        })
        .background()
        .await
        .expect("Failed to initialize landing subscriber");
}

pub async fn register_launch_land(session: &Session, handle: &MavlinkHandle) {
    register_land(session, handle).await;
    register_launch(session, handle).await;
}
