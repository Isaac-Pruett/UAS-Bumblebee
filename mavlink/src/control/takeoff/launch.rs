use std::sync::Arc;

use mavlink::{
    MavFrame,
    dialects::common::{MavCmd, MavMessage, MavResult},
};
use tokio::sync::Mutex;

use crate::{
    connection::{MavlinkAckChannel, MavlinkHandle},
    control::takeoff::{
        ACK_TIMEOUT, LaunchCommand, LaunchErrors, TARGET_COMPONENT, can_fly, reply_error, system_id,
    },
    get_session,
};

pub async fn register_launch(handle: &MavlinkHandle, ack_router: MavlinkAckChannel) {
    let handle = handle.clone();
    let ack_router = Arc::new(Mutex::new(ack_router));

    get_session()
        .declare_queryable("control/move/launch")
        .callback(move |query| {
            let handle = handle.clone();
            let ack_router = Arc::clone(&ack_router);

            tokio::spawn(async move {
                let Some(payload) = query.payload() else {
                    reply_error(query, LaunchErrors::DeserializeError).await;
                    return;
                };

                let Ok(command) = rmp_serde::from_slice::<LaunchCommand>(&payload.to_bytes())
                else {
                    reply_error(query, LaunchErrors::DeserializeError).await;
                    return;
                };

                if !can_fly() {
                    reply_error(query, LaunchErrors::CantFlyError).await;
                    return;
                }

                let message =
                    MavMessage::COMMAND_LONG(mavlink::dialects::common::COMMAND_LONG_DATA {
                        target_system: system_id(),
                        target_component: TARGET_COMPONENT,
                        command: MavCmd::MAV_CMD_NAV_TAKEOFF,
                        confirmation: 0,

                        param1: 0.0,
                        param2: 0.0,
                        param3: 0.0,
                        param4: command.yaw,
                        param5: command.lat.unwrap_or(0.0),
                        param6: command.lon.unwrap_or(0.0),
                        param7: command.height_m,
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
                        let _ = query.reply("control/move/launch/result", "Ok").await;
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
        .expect("failed to initialize launch queryable");
}
