use mavlink::dialects::common::{COMMAND_INT_DATA, MavCmd, MavFrame::MAV_FRAME_GLOBAL, MavMessage};
use zenoh::sample::Sample;

use crate::{
    connection::{MavlinkHandle, SYSTEM_ID},
    get_session,
    packets::waypoint::Waypoint,
};

pub async fn declare_goto_waypoint_sub(mavconn: MavlinkHandle) {
    get_session()
        .declare_subscriber("control/move/global/waypoint")
        .callback(move |sample| {
            let bytes = sample.payload().to_bytes();
            let res_decode = rmp_serde::from_slice::<Waypoint>(&bytes);
            let mavconn = mavconn.clone();
            match res_decode {
                Ok(waypt) => {
                    tokio::spawn(async move {
                        let res = mavconn
                            .send(MavMessage::COMMAND_INT(
                                mavlink::dialects::common::COMMAND_INT_DATA {
                                    param1: waypt.speed,
                                    param2: 0 as u32 as f32,
                                    param3: waypt.acceptance_rad.as_meters(),
                                    param4: waypt.heading,
                                    x: waypt.coord.lat_deg_e7,
                                    y: waypt.coord.lon_deg_e7,
                                    z: waypt.coord.alt_mm as f32,
                                    command: MavCmd::MAV_CMD_DO_REPOSITION,
                                    target_system: *SYSTEM_ID.get().unwrap_or(&0), // Fixed from hardcoded 1
                                    target_component: 0, // Fixed from hardcoded 0
                                    frame: MAV_FRAME_GLOBAL,
                                    current: 0,
                                    autocontinue: 0,
                                },
                            ))
                            .await;
                    });
                }
                Err(e) => {
                    println!("{:?}", e);
                }
            };
        })
        .await
        .expect("failed to initialize goto subscriber");
}
