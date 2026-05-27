use std::sync::Arc;

use mavlink::dialects::common::{
    MavFrame::{
        self, MAV_FRAME_BODY_FRD, MAV_FRAME_BODY_OFFSET_NED, MAV_FRAME_LOCAL_NED,
        MAV_FRAME_LOCAL_OFFSET_NED,
    },
    MavMessage,
};

use crate::{
    connection::MavlinkHandle,
    packets::motion::{self, Movement},
    topic_utils::topic,
};

pub async fn declare_movement_input_sub(
    session: &zenoh::Session,
    endpoint: &str,
    mavconn: MavlinkHandle,
    frame_conversion: MovementFn,
    is_body_frame: bool,
) {
    session
        .declare_subscriber(topic(endpoint))
        .callback(move |sample| {
            let mavconn = mavconn.clone();
            let frame_conversion = frame_conversion.clone();

            tokio::spawn(async move {
                let bytes = &*sample.payload().to_bytes();
                let movement = rmp_serde::from_slice::<Movement>(bytes).ok();

                if let Some(mut m) = movement {
                    frame_conversion(&mut m);

                    if let Some(mut packet) = m.as_packet() {
                        packet.coordinate_frame = match is_body_frame {
                            false => MAV_FRAME_LOCAL_NED,
                            true => MAV_FRAME_BODY_OFFSET_NED,
                        };
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
            false,
        ),
        (
            "control/move/ned/pos",
            Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
            false,
        ),
        (
            "control/move/enu/raw",
            Arc::new(|m: &mut Movement| {
                // println!("MOVED: {m:?}");
                motion::enu_all(m);
            }) as MovementFn,
            false,
        ),
        (
            "control/move/enu/pos",
            Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
            false,
        ),
        (
            "control/move/enu/velo",
            Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
            false,
        ),
        (
            "control/move/ned/velo",
            Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
            false,
        ),
        /////////////////////////////////////////////
        (
            "control/move/frd/raw",
            Arc::new(|m: &mut Movement| {
                // dindu nuffin
                motion::ned_all(m);
            }) as MovementFn,
            true,
        ),
        (
            "control/move/frd/pos",
            Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
            true,
        ),
        (
            "control/move/flu/raw",
            Arc::new(|m: &mut Movement| {
                // println!("MOVED: {m:?}");
                motion::enu_all(m);
            }) as MovementFn,
            true,
        ),
        (
            "control/move/flu/pos",
            Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
            true,
        ),
        (
            "control/move/flu/velo",
            Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
            true,
        ),
        (
            "control/move/frd/velo",
            Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
            true,
        ),
    ];

    for (k, f, b) in pairs {
        declare_movement_input_sub(&session, k, mavconn.clone(), f.clone(), b).await;
    }
}
