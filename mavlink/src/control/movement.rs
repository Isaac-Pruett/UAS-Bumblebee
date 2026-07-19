use std::sync::Arc;

use mavlink::dialects::common::{
    MavAutopilot::{MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_AUTOPILOT_PX4},
    MavFrame, MavMessage,
};

use crate::{
    connection::MavlinkHandle,
    packets::local_frame_waypoint::{self, LocalFrameWaypoint},
    telem::{AUTOPILOT, AUTOPILOT_MODE, FlightMode},
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
                let movement = rmp_serde::from_slice::<LocalFrameWaypoint>(bytes).ok();

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

                        if AUTOPILOT_MODE
                            .read()
                            .ok()
                            .and_then(|guard| *guard)
                            .as_ref()
                            .is_some_and(|mode| mode.can_fly())
                        {
                            let _res = mavconn
                                .send(MavMessage::SET_POSITION_TARGET_LOCAL_NED(packet))
                                .await
                                .ok();
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
    pub movement_tranform: MovementFn,
    pub is_body_frame: bool,
}

type MovementFn = Arc<dyn Fn(&mut LocalFrameWaypoint) + Send + Sync>;
pub async fn declare_all(session: &zenoh::Session, mavconn: MavlinkHandle) {
    let tuples = [
        MovementSubParams {
            keyexpr: "control/move/ned/raw".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                // dindu nuffin
                local_frame_waypoint::ned_all(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/ned/pos".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                local_frame_waypoint::ned_all(m);
                local_frame_waypoint::nan_vel(m);
                local_frame_waypoint::nan_acc(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/enu/raw".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                // println!("MOVED: {m:?}");
                local_frame_waypoint::enu_all(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/enu/pos".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                local_frame_waypoint::enu_all(m);
                local_frame_waypoint::nan_vel(m);
                local_frame_waypoint::nan_acc(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/enu/velo".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                local_frame_waypoint::enu_all(m);
                local_frame_waypoint::nan_pos(m);
                local_frame_waypoint::nan_acc(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        MovementSubParams {
            keyexpr: "control/move/ned/velo".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                local_frame_waypoint::ned_all(m);
                local_frame_waypoint::nan_pos(m);
                local_frame_waypoint::nan_acc(m);
            }) as MovementFn,
            is_body_frame: false,
        },
        /////////////////////////////////////////////
        MovementSubParams {
            keyexpr: "control/move/frd/raw".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                // dindu nuffin
                local_frame_waypoint::ned_all(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/frd/pos".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                local_frame_waypoint::ned_all(m);
                local_frame_waypoint::nan_vel(m);
                local_frame_waypoint::nan_acc(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/rfu/raw".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                // println!("MOVED: {m:?}");
                local_frame_waypoint::rfu_all(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/rfu/pos".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                local_frame_waypoint::rfu_all(m);
                local_frame_waypoint::nan_vel(m);
                local_frame_waypoint::nan_acc(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/rfu/velo".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                local_frame_waypoint::rfu_all(m);
                local_frame_waypoint::nan_pos(m);
                local_frame_waypoint::nan_acc(m);
            }) as MovementFn,
            is_body_frame: true,
        },
        MovementSubParams {
            keyexpr: "control/move/frd/velo".into(),
            movement_tranform: Arc::new(|m: &mut LocalFrameWaypoint| {
                local_frame_waypoint::ned_all(m);
                local_frame_waypoint::nan_pos(m);
                local_frame_waypoint::nan_acc(m);
            }) as MovementFn,
            is_body_frame: true,
        },
    ];

    for params in tuples {
        declare_movement_input_sub(
            &session,
            params.keyexpr.as_str(),
            mavconn.clone(),
            params.movement_tranform.clone(),
            params.is_body_frame,
        )
        .await;
    }
}
