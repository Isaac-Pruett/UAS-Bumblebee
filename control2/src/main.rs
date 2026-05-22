use glam::Vec3;
use mavlink::dialects::common::MavMessage;
use tokio::task::JoinHandle;
use tokio::time::sleep;

use std::time::{Duration, Instant};

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

    let start = Instant::now();
    let hbh = create_telem_topic(
        &session,
        "heartbeat",
        message_rx.resubscribe(),
        move |msg| match msg {
            MavMessage::HEARTBEAT(data) => {
                // println!("BEAT!");
                Some(Heartbeat {
                    elapsed_us: start.elapsed().as_micros(),
                })
            }
            _ => None,
        },
    )
    .await;

    let pos_lcl_ned_h = create_telem_topic(
        &session,
        "pos/local/ned",
        message_rx.resubscribe(),
        move |msg| match msg {
            MavMessage::LOCAL_POSITION_NED(data) => {
                let s = Some(Vec3::new(data.x, data.y, data.z));
                // println!("NED: {s:?}\n ");
                s
            }
            _ => None,
        },
    )
    .await;

    let pos_lcl_enu_h = create_telem_topic(
        &session,
        "pos/local/enu",
        message_rx.resubscribe(),
        move |msg| match msg {
            MavMessage::LOCAL_POSITION_NED(data) => {
                let s = Some(Vec3::new(data.y, data.x, -data.z));
                // println!("ENU: {s:?}\n ");
                s
            }
            _ => None,
        },
    )
    .await;

    let pos_gl_int_h = create_telem_topic(
        &session,
        "pos/global/int",
        message_rx.resubscribe(),
        move |msg| match msg {
            MavMessage::GLOBAL_POSITION_INT(data) => {
                let s = Some(Coords {
                    lat_deg_e7: data.lat,
                    lon_deg_e7: data.lon,
                    alt_mm: data.alt,
                });
                // println!("GL: {s:?}\n ");
                s
            }
            _ => None,
        },
    )
    .await;

    let origin_queryable = session
        .declare_queryable(topic("origin/set"))
        .callback(|query| {
            tokio::spawn(async move {
                let payload = rmp_serde::to_vec(&[true]).unwrap_or(vec![]);
                query.reply(topic("origin/ack"), payload).await.ok();
            });
        })
        .await
        .expect("failed to initialize origin queryable");

    profiler().await.ok();

    pos_gl_int_h.await.ok();
    pos_lcl_enu_h.await.ok();
    pos_lcl_ned_h.await.ok();
    hbh.await.ok();

    message_handle.await.ok();
}

const LOOP_RATE: Duration = Duration::from_millis(10);

pub fn profiler() -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut loop_count = 0u64;
        let mut total_delay = Duration::ZERO;
        let mut max_delay = Duration::ZERO;

        loop {
            let sleep_start = Instant::now();
            sleep(LOOP_RATE).await;
            let actual_sleep = sleep_start.elapsed();

            // Delay is how much longer we slept than intended
            let delay = actual_sleep.saturating_sub(LOOP_RATE);
            total_delay += delay;
            loop_count += 1;

            if delay > max_delay {
                max_delay = delay;
            }

            if loop_count % 100 == 0 {
                let avg_delay_ms = total_delay.as_secs_f64() * 1000.0 / loop_count as f64;
                println!(
                    "delay — inst: {:.2}ms | avg: {:.2}ms | max: {:.2}ms",
                    delay.as_secs_f64() * 1000.0,
                    avg_delay_ms,
                    max_delay.as_secs_f64() * 1000.0,
                );
            }
        }
    })
}
