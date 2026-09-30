use tokio::task::JoinHandle;
use tokio::time::sleep;

use std::{
    sync::OnceLock,
    time::{Duration, Instant},
};

use zenoh::{self, Session};

use crate::connection::{mav_connect, setup_mavlink_task};

mod connection;

mod control;
mod packets;
mod telem;

pub static ZENOH_SESSION: OnceLock<Session> = OnceLock::new();

pub fn get_session() -> &'static Session {
    ZENOH_SESSION.get().expect("ZENOH_SESSION not set")
}

#[tokio::main]
async fn main() {
    let config_path = std::env::var("ZENOH_CONFIG").expect("ZENOH_CONFIG not set");
    let mut cfg = zenoh::Config::from_file(&config_path).expect("failed to load ZENOH_CONFIG file");

    let pfx = cfg
        .get_json("namespace")
        .expect("bad configuration, no namespace")
        .trim_matches('"')
        .trim_matches('/')
        .to_string();

    let namespace = format!("{pfx}/mavlink");

    cfg.insert_json5("namespace", format!(r#""{}""#, namespace).as_str())
        .expect("mavlink: failed to set namespace");

    let session = zenoh::open(cfg)
        .await
        .expect("mavlink: failed to initialize zenoh");

    ZENOH_SESSION.set(session);

    let mavlink_conn_str =
        std::env::var("MAVLINK_CONN").unwrap_or_else(|_| "udpin:0.0.0.0:14551".to_string());
    let mavconn = mav_connect(&mavlink_conn_str).await;
    println!("{namespace} connected to mavlink on '{mavlink_conn_str}'");

    let telem_handles = telem::declare_all().await;
    let pwm_state = control::pwm::register_pwm_watchers().await;
    let launch_ack_channel =
        connection::MavlinkAckChannel::new(mavlink::dialects::common::MavCmd::MAV_CMD_NAV_TAKEOFF);
    let land_ack_channel =
        connection::MavlinkAckChannel::new(mavlink::dialects::common::MavCmd::MAV_CMD_NAV_LAND);

    let (mav_handle, message_handle) = setup_mavlink_task(mavconn);

    control::origin::declare_origin_sub(mav_handle.clone()).await;
    control::movement::declare_all(mav_handle.clone()).await;
    control::reposition::declare_goto_waypoint_sub(mav_handle.clone()).await;

    control::takeoff::register_launch_land(
        &mav_handle.clone(),
        launch_ack_channel,
        land_ack_channel,
    )
    .await;

    control::pwm::register_all_pwm_subs(mav_handle.clone(), pwm_state).await;
    // profiler().await.ok();

    for h in telem_handles {
        h.await.ok();
    }

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
