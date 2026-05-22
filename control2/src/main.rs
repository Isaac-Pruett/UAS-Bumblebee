use mavlink::dialects::common::MavMessage;

use std::time::Instant;

use zenoh::{self};

use mavlink;

use crate::connection::mav_connect;
use crate::packets::coords::Coords;
use crate::packets::heartbeat::Heartbeat;
use crate::telem::create_telem_topic;

mod connection;
mod packets;
mod telem;
mod topic_utils;

use crate::connection::setup_mavlink_task;
use crate::topic_utils::topic;

#[tokio::main]
async fn main() {
    let mavlink_conn_str =
        std::env::var("MAVLINK_CONN").unwrap_or_else(|_| "udpin:0.0.0.0:14551".to_string());
    let mavconn = mav_connect(&mavlink_conn_str).await;
    let (mav_handle, message_rx, message_handle) = setup_mavlink_task(mavconn);

    let session = zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default()))
        .await
        .expect("control2: failed to initialize zenoh");
    let hbh = create_telem_topic(
        &session,
        "heartbeat",
        message_rx.resubscribe(),
        move |msg| match msg {
            MavMessage::HEARTBEAT(data) => {
                // println!("BEAT!");
                Some(Heartbeat {
                    elapsed_us: Instant::now().elapsed().as_micros(),
                })
            }
            _ => None,
        },
    )
    .await;

    let pos_h = create_telem_topic(
        &session,
        "heartbeat",
        message_rx.resubscribe(),
        move |msg| match msg {
            MavMessage::LOCAL_POSITION_NED(data) => {
                let s = Some(Coords {
                    lat_deg_e7: (data.x * 1e7) as i32,
                    lon_deg_e7: (data.y * 1e7) as i32,
                    alt_mm: (data.z * 1e3) as i32,
                });
                println!("S: {s:?}\n {}, {}, {},", data.x, data.y, data.z);
                s
            }
            _ => {
                println!("{msg:?}\n");
                None
            }
        },
    )
    .await;

    // let origin_queryable = session
    //     .declare_queryable(topic("origin/set"))
    //     .callback(|query| {
    //         tokio::spawn(async move {
    //             let payload = rmp_serde::to_vec(&[true]).unwrap_or(vec![]);
    //             query.reply(topic("origin/ack"), payload).await.ok();
    //         });
    //     })
    //     .await
    //     .expect("failed to initialize origin queryable");

    pos_h.await.ok();
    hbh.await.ok();

    message_handle.await.ok();
}
