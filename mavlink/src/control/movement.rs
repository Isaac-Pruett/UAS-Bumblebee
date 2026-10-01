use std::sync::Arc;

use mavlink::dialects::common::{
    MavAutopilot::{MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_AUTOPILOT_PX4},
    MavFrame, MavMessage,
};

use crate::{
    connection::MavlinkHandle,
    get_session,
    packets::motion::{self, Movement},
    telem::{AUTOPILOT, autopilot_mode},
};

pub async fn declare_movement_input_sub(
    session: &zenoh::Session,
    endpoint: &str,
    mavconn: MavlinkHandle,
    frame_conversion: MovementFn,
    is_body_frame: bool,
) {
    session
        .declare_subscriber(endpoint)
        .callback(move |sample| {
            let mavconn = mavconn.clone();
            let frame_conversion = frame_conversion.clone();

            tokio::spawn(async move {
                let bytes = &*sample.payload().to_bytes();
                let movement = rmp_serde::from_slice::<Movement>(bytes).ok();

                if let Some(mut m) = movement {
                    frame_conversion(&mut m);

                    if let Some(mut packet) = m.as_packet() {
                        packet.coordinate_frame = if is_body_frame {
                            match AUTOPILOT.get() {
                                Some(MAV_AUTOPILOT_PX4) => MavFrame::MAV_FRAME_BODY_NED,
                                Some(MAV_AUTOPILOT_ARDUPILOTMEGA) => {
                                    MavFrame::MAV_FRAME_BODY_OFFSET_NED
                                }
                                _ => MavFrame::MAV_FRAME_BODY_NED,
                            }
                        } else {
                            MavFrame::MAV_FRAME_LOCAL_NED
                        };

                        if let Some(autopilot_mode) = autopilot_mode() {
                            if autopilot_mode.can_fly() {
                                let _ = mavconn
                                    .send(MavMessage::SET_POSITION_TARGET_LOCAL_NED(packet))
                                    .await;
                            }
                        }
                    }
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize movement input: {endpoint:?}");
}

struct MovementSubParams {
    pub keyexpr: String,
    pub movement_transform: MovementFn,
    pub is_body_frame: bool,
}

type MovementFn = Arc<dyn Fn(&mut Movement) + Send + Sync>;
pub async fn declare_all(mavconn: MavlinkHandle) {
    let tuples = [
        MovementSubParams {
            keyexpr: "control/move/ned/raw".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                // dindu nuffin
                motion::ned_all(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/ned/pos".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/enu/raw".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                // println!("MOVED: {m:?}");
                motion::enu_all(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/enu/pos".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/enu/velo".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/ned/velo".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        /////////////////////////////////////////////
        MovementSubParams {
            keyexpr: "control/move/frd/raw".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                // dindu nuffin
                motion::ned_all(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/frd/pos".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/flu/raw".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                // println!("MOVED: {m:?}");
                motion::enu_all(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/flu/pos".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_vel(m);
                motion::nan_acc(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/flu/velo".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                motion::enu_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/frd/velo".into(),
            movement_transform: Arc::new(|m: &mut Movement| {
                motion::ned_all(m);
                motion::nan_pos(m);
                motion::nan_acc(m);
            }) as MovementFn,
            is_body_frame: true,
        },
    ];

    for params in tuples {
        declare_movement_input_sub(
            &get_session(),
            params.keyexpr.as_str(),
            mavconn.clone(),
            params.movement_transform.clone(),
            params.is_body_frame,
        )
        .await;
    }
}
