use glam::Vec3;
use mavlink::{
    MavFrame,
    dialects::common::{
        COMMAND_INT_DATA, MavMessage, PositionTargetTypemask, SET_POSITION_TARGET_LOCAL_NED_DATA,
    },
};

use crate::{
    connection::MavlinkHandle,
    packets::{coords::Coords, motion::Movement},
    topic_utils::topic,
};

pub async fn create_movement_input_sub(
    session: &zenoh::Session,
    endpoint: &str,
    mavconn: MavlinkHandle,
) {
    session
        .declare_subscriber(topic(endpoint))
        .callback(move |sample| {
            let mavconn = mavconn.clone();
            tokio::spawn(async move {
                let movement =
                    rmp_serde::from_slice::<Movement>(&*sample.payload().to_bytes()).ok();
                if let Some(m) = movement {
                    if let Some(m) = m.as_packet() {
                        let _res = mavconn
                            .send(MavMessage::SET_POSITION_TARGET_LOCAL_NED(m))
                            .await
                            .ok();
                    }
                }
            });
        })
        .background()
        .await
        .ok();
}
