use tokio::task::JoinHandle;
use tokio::time::sleep;

use std::time::{Duration, Instant};

use zenoh::{self};

use crate::connection::{MavlinkRouter, mav_connect, setup_mavlink_task};

mod connection;

mod control;
mod packets;
mod telem;

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

    let mavlink_conn_str =
        std::env::var("MAVLINK_CONN").unwrap_or_else(|_| "udpin:0.0.0.0:14551".to_string());
    let mavconn = mav_connect(&mavlink_conn_str).await;

    let mut router = MavlinkRouter::new();
    let telem_handles = telem::declare_all(&session, &mut router).await;
    let pwm_state = control::pwm::register_pwm_watchers(&mut router).await;

    let (mav_handle, message_handle) = setup_mavlink_task(mavconn, router);

    control::origin::declare_origin_sub(&session, mav_handle.clone()).await;
    control::movement::declare_all(&session, mav_handle.clone()).await;
    control::reposition::declare_goto_waypoint_sub(&session, mav_handle.clone()).await;

    control::launch_land::register_launch(&session, &mav_handle.clone()).await;

    control::pwm::register_all_pwm_subs(&session, mav_handle.clone(), pwm_state).await;
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
