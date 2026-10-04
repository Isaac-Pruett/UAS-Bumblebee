use std::sync::Arc;

use mavlink::dialects::common::{MavCmd, MavMessage, MavResult};
use serde::Serialize;
use tokio::sync::Mutex;

use crate::{
    connection::{MavlinkAckChannel, MavlinkHandle, SYSTEM_ID},
    get_session,
};

const ACK_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(2);
const TARGET_COMPONENT: u8 = 1;

#[derive(Debug, Serialize)]
#[repr(u8)]
enum ArmError {
    AckRejected,
    AckTimeout,
}

async fn reply_error(query: zenoh::query::Query, error: ArmError) {
    let Ok(payload) = rmp_serde::to_vec(&error) else {
        return;
    };

    let _ = query.reply_err(payload).await;
}

pub async fn register_arm(handle: &MavlinkHandle, ack_router: MavlinkAckChannel) {
    let handle = handle.clone();
    let ack_router = Arc::new(Mutex::new(ack_router));

    get_session()
        .declare_queryable("control/arm")
        .callback(move |query| {
            let handle = handle.clone();
            let ack_router = Arc::clone(&ack_router);

            tokio::spawn(async move {
                let target_system = *SYSTEM_ID.get().unwrap_or(&0);
                println!("received arm request for MAVLink system {target_system}");

                let message =
                    MavMessage::COMMAND_LONG(mavlink::dialects::common::COMMAND_LONG_DATA {
                        target_system,
                        target_component: TARGET_COMPONENT,
                        command: MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                        confirmation: 0,
                        param1: 1.0,
                        param2: 0.0,
                        param3: 0.0,
                        param4: 0.0,
                        param5: 0.0,
                        param6: 0.0,
                        param7: 0.0,
                    });

                if handle.send(message).await.is_err() {
                    eprintln!("failed to queue arm command for MAVLink system {target_system}");
                    reply_error(query, ArmError::AckTimeout).await;
                    return;
                }

                let ack = {
                    let mut router = ack_router.lock().await;
                    router.wait_ack(ACK_TIMEOUT).await
                };

                match ack {
                    Ok(MavResult::MAV_RESULT_ACCEPTED) => {
                        println!("arm command accepted by MAVLink system {target_system}");
                        if let Err(err) = query.reply("control/arm", "Ok").await {
                            eprintln!("failed to reply to arm query: {err}");
                        }
                    }
                    Ok(result) => {
                        eprintln!(
                            "arm command rejected by MAVLink system {target_system}: {result:?}"
                        );
                        reply_error(query, ArmError::AckRejected).await;
                    }
                    Err(err) => {
                        eprintln!(
                            "arm command timed out for MAVLink system {target_system}: {err:?}"
                        );
                        reply_error(query, ArmError::AckTimeout).await;
                    }
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize arm queryable");
}
