use glam::Vec3;
use mavlink::dialects::common::MavMessage;
use tokio::task::JoinHandle;
use tokio::time::sleep;

use std::time::{Duration, Instant};

use zenoh::{self};

use mavlink;

use crate::connection::{MavlinkRouter, mav_connect, setup_mavlink_task};
use crate::packets::coords::Coords;
use crate::packets::heartbeat::Heartbeat;
use crate::telem::create_telem_topic;

mod connection;
mod packets;
mod telem;
mod topic_utils;

use crate::topic_utils::topic;

#[tokio::main(flavor = "multi_thread", worker_threads = 4)]
async fn main() {
    let mavlink_conn_str =
        std::env::var("MAVLINK_CONN").unwrap_or_else(|_| "udpin:0.0.0.0:14551".to_string());
    let mavconn = mav_connect(&mavlink_conn_str).await;

    // Register all subscriptions before the task starts.
    // Each call pattern-matches once at the router level; telem topics
    // receive already-extracted, already-typed values.
    let mut router = MavlinkRouter::new();

    let start = Instant::now();
    let heartbeat_rx = router.subscribe(move |msg| match msg {
        MavMessage::HEARTBEAT(_) => Some(Heartbeat {
            elapsed_us: start.elapsed().as_micros(),
        }),
        _ => None,
    });

    let ned_rx = router.subscribe(|msg| match msg {
        MavMessage::LOCAL_POSITION_NED(data) => Some(Vec3::new(data.x, data.y, data.z)),
        _ => None,
    });

    let enu_rx = router.subscribe(|msg| match msg {
        MavMessage::LOCAL_POSITION_NED(data) => Some(Vec3::new(data.y, data.x, -data.z)),
        _ => None,
    });

    let global_rx = router.subscribe(|msg| match msg {
        MavMessage::GLOBAL_POSITION_INT(data) => Some(Coords {
            lat_deg_e7: data.lat,
            lon_deg_e7: data.lon,
            alt_mm: data.alt,
        }),
        _ => None,
    });

    let (_mav_handle, message_handle) = setup_mavlink_task(mavconn, router);

    let session = zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default()))
        .await
        .expect("control2: failed to initialize zenoh");

    let hbh = create_telem_topic(&session, "heartbeat", heartbeat_rx).await;
    let pos_lcl_ned_h = create_telem_topic(&session, "pos/local/ned", ned_rx).await;
    let pos_lcl_enu_h = create_telem_topic(&session, "pos/local/enu", enu_rx).await;
    let pos_gl_int_h = create_telem_topic(&session, "pos/global/int", global_rx).await;

    let _origin_queryable = session
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
