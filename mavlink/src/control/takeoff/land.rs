use std::sync::Arc;

use mavlink::dialects::common::{MavCmd, MavMessage, MavResult};
use tokio::sync::Mutex;

use crate::{
    connection::{MavlinkAckChannel, MavlinkHandle},
    control::takeoff::{
        ACK_TIMEOUT, LandCommand, LaunchErrors, TARGET_COMPONENT, can_fly, reply_error, system_id,
    },
    get_session,
};

pub async fn register_land(handle: &MavlinkHandle, ack_router: MavlinkAckChannel) {
    let handle = handle.clone();
    let ack_router = Arc::new(Mutex::new(ack_router));

    get_session()
        .declare_queryable("control/move/land")
        .callback(move |query| {
            let handle = handle.clone();
            let ack_router = Arc::clone(&ack_router);

            tokio::spawn(async move {
                let Some(payload) = query.payload() else {
                    reply_error(query, LaunchErrors::DeserializeError).await;
                    return;
                };

                let Ok(command) = rmp_serde::from_slice::<LandCommand>(&payload.to_bytes()) else {
                    reply_error(query, LaunchErrors::DeserializeError).await;
                    return;
                };

                if !can_fly() {
                    reply_error(query, LaunchErrors::CantFlyError).await;
                    return;
                }

                let message =
                    MavMessage::COMMAND_INT(mavlink::dialects::common::COMMAND_INT_DATA {
                        param1: 0.0,
                        param2: command.acceptance_rad_m,
                        param3: command.descend_speed_m_s,
                        param4: command.yaw,

                        x: command.coords.x as i32,
                        y: command.coords.y as i32,
                        z: command.coords.z,

                        command: MavCmd::MAV_CMD_NAV_LAND_LOCAL,
                        target_system: system_id(),
                        target_component: TARGET_COMPONENT,

                        frame: mavlink::dialects::common::MavFrame::MAV_FRAME_LOCAL_OFFSET_NED,
                        current: 0,
                        autocontinue: 0,
                    });

                if handle.send(message).await.is_err() {
                    reply_error(query, LaunchErrors::AckTimeout).await;
                    return;
                }

                let ack = {
                    let mut router = ack_router.lock().await;
                    router.wait_ack(ACK_TIMEOUT).await
                };

                match ack {
                    Ok(MavResult::MAV_RESULT_ACCEPTED) => {
                        let _ = query.reply("control/move/land/result", "Ok").await;
                    }

                    Ok(_) => {
                        reply_error(query, LaunchErrors::AckRejected).await;
                    }

                    Err(_) => {
                        reply_error(query, LaunchErrors::AckTimeout).await;
                    }
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize landing queryable");
}
