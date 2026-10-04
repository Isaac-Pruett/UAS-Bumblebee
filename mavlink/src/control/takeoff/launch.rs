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
    telem::{AUTOPILOT, autopilot_mode},
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
                println!("attempting to launch");
                let Some(payload) = query.payload() else {
                    reply_error(query, LaunchErrors::DeserializeError).await;
                    println!("failed to launch: deserialize");
                    return;
                };

                let Ok(command) = rmp_serde::from_slice::<LaunchCommand>(&payload.to_bytes())
                else {
                    reply_error(query, LaunchErrors::DeserializeError).await;
                    println!("failed to launch: deserialize");
                    return;
                };

                if !autopilot_mode().is_some_and(|mode| mode.armed) {
                    reply_error(query, LaunchErrors::CantFlyError).await;
                    println!("failed to launch: not armed");
                    return;
                }

                // PX4 treats finite zero coordinates as a real destination.
                let unspecified_coordinate = if AUTOPILOT.get()
                    == Some(&mavlink::dialects::common::MavAutopilot::MAV_AUTOPILOT_PX4)
                {
                    f32::NAN
                } else {
                    0.0
                };

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
                        param5: command.lat.unwrap_or(unspecified_coordinate),
                        param6: command.lon.unwrap_or(unspecified_coordinate),
                        param7: command.height_m,
                    });

                if handle.send(message).await.is_err() {
                    reply_error(query, LaunchErrors::AckTimeout).await;
                    println!("failed to launch: ack timeout");
                    return;
                }

                let ack = {
                    let mut router = ack_router.lock().await;
                    router.wait_ack(ACK_TIMEOUT).await
                };

                match ack {
                    Ok(MavResult::MAV_RESULT_ACCEPTED) => {
                        println!("takeoff command accepted by MAVLink system {}", system_id());
                        if let Err(err) = query.reply("control/move/launch", "Ok").await {
                            eprintln!("failed to reply to launch query: {err}");
                        }
                    }

                    Ok(result) => {
                        eprintln!(
                            "takeoff ACK from MAVLink system {}: {result:?}",
                            system_id()
                        );
                        reply_error(query, LaunchErrors::AckRejected).await;
                    }

                    Err(err) => {
                        eprintln!(
                            "takeoff ACK failed for MAVLink system {}: {err:?}",
                            system_id()
                        );
                        reply_error(query, LaunchErrors::AckTimeout).await;
                    }
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize launch queryable");
}
