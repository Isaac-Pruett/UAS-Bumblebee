use std::sync::Arc;

use mavlink::dialects::common::MavMessage;

use crate::{
    connection::MavlinkHandle,
    packets::motion::{self, Movement},
    topic_utils::topic,
};

pub async fn declare_movement_input_sub(
    session: &zenoh::Session,
    endpoint: &str,
    mavconn: MavlinkHandle,
    to_valid_packet: MovementFn,
) {
    session
        .declare_subscriber(topic(endpoint))
        .callback(move |sample| {
            let mavconn = mavconn.clone();
            let to_valid_packet = to_valid_packet.clone();

            tokio::spawn(async move {
                let movement =
                    rmp_serde::from_slice::<Movement>(&*sample.payload().to_bytes()).ok();

                if let Some(mut m) = movement {
                    to_valid_packet(&mut m);

                    if let Some(packet) = m.as_packet() {
                        let _res = mavconn
                            .send(MavMessage::SET_POSITION_TARGET_LOCAL_NED(packet))
                            .await
                            .ok();
                    }
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize movement input: {endpoint:?}");
}

type MovementFn = Arc<dyn Fn(&mut Movement) + Send + Sync>;
pub async fn declare_all(session: &zenoh::Session, mavconn: MavlinkHandle) {
    let pairs = [
        (
            "control/move/ned/raw",
            Arc::new(|m: &mut Movement| {
                // dindu nuffin
                motion::ned_all(m);
            }) as MovementFn,
        ),
        (
            "control/move/ned/pos",
            Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
        ),
        (
            "control/move/enu/raw",
            Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
            }) as MovementFn,
        ),
        (
            "control/move/enu/pos",
            Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
        ),
        (
            "control/move/enu/velo",
            Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
        ),
        (
            "control/move/ned/velo",
            Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
        ),
    ];

    for (k, f) in pairs {
        declare_movement_input_sub(&session, k, mavconn.clone(), f.clone()).await;
    }
}
