use crate::connection::MavlinkRouter;
use crate::packets::coords::Coords;
use crate::packets::heartbeat::Heartbeat;
use crate::topic_utils::topic;
use glam::Vec3;
use mavlink::dialects::common::MavMessage;
use serde::Serialize;
use std::time::Instant;
use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;
use zenoh::Session;
use zenoh::qos::Priority;

/// Spawn a Zenoh publisher that drains a typed mpsc channel.
/// Use for topics where every message matters (e.g. heartbeat).
pub async fn create_telem_topic_mpsc<T>(
    session: &Session,
    sensor_name: &str,
    mut rx: mpsc::Receiver<T>,
    is_express: bool,
    priority: Priority,
) -> JoinHandle<()>
where
    T: Serialize + Send + 'static,
{
    let topic_name = topic(format!("telemetry/{sensor_name}").as_str());

    let publ = session
        .declare_publisher(topic_name.clone())
        .express(is_express)
        .priority(priority)
        .await
        .expect("failed to declare publisher");

    let mut buf = Vec::with_capacity(512);
    tokio::spawn(async move {
        while let Some(packet) = rx.recv().await {
            buf.clear();
            match rmp_serde::encode::write(&mut buf, &packet) {
                Ok(()) => {
                    if let Err(e) = publ.put(buf.as_slice()).await {
                        eprintln!("Failed to publish telemetry on '{topic_name}': {e}");
                    }
                }
                Err(e) => {
                    eprintln!("Failed to serialize telemetry on '{topic_name}': {e}");
                }
            }
        }
        eprintln!("Telem topic '{topic_name}' channel closed, stopping.");
    })
}

/// Spawn a Zenoh publisher backed by a watch channel.
/// Only the latest value is published per iteration — stale intermediate
/// values are discarded. Use for high-rate sensor streams (position, etc.)
/// where only the current state matters.
pub async fn create_telem_topic_watch<T>(
    session: &Session,
    sensor_name: &str,
    mut rx: watch::Receiver<Option<T>>,
    is_express: bool,
    priority: Priority,
) -> JoinHandle<()>
where
    T: Serialize + Send + Sync + Clone + 'static,
{
    let topic_name = topic(format!("telemetry/{sensor_name}").as_str());

    let publ = session
        .declare_publisher(topic_name.clone())
        .express(is_express)
        .priority(priority)
        .await
        .expect("failed to declare publisher");

    let mut buf = Vec::with_capacity(512);
    tokio::spawn(async move {
        loop {
            if rx.changed().await.is_err() {
                eprintln!("Telem topic '{topic_name}' watch closed, stopping.");
                break;
            }

            let packet = rx.borrow_and_update().clone();
            let Some(packet) = packet else { continue };

            buf.clear();
            match rmp_serde::encode::write(&mut buf, &packet) {
                Ok(()) => {
                    if let Err(e) = publ.put(buf.as_slice()).await {
                        eprintln!("Failed to publish telemetry on '{topic_name}': {e}");
                    }
                }
                Err(e) => {
                    eprintln!("Failed to serialize telemetry on '{topic_name}': {e}");
                }
            }
        }
    })
}

/// Register all telemetry subscriptions and spawn their publisher tasks.
/// Must be called before `setup_mavlink_task` consumes the router.
/// Returns all JoinHandles so the caller can await them for shutdown.
pub async fn declare_all(session: &Session, router: &mut MavlinkRouter) -> Vec<JoinHandle<()>> {
    let start = Instant::now();

    // mpsc: every heartbeat event matters
    let heartbeat_rx = router.subscribe(move |msg| match msg {
        MavMessage::HEARTBEAT(_) => Some(Heartbeat {
            elapsed_us: start.elapsed().as_micros(),
        }),
        _ => None,
    });

    // watch: only the latest position matters
    let ned_rx = router.subscribe_watch(|msg| match msg {
        MavMessage::LOCAL_POSITION_NED(data) => Some(Vec3::new(data.x, data.y, data.z)),
        _ => None,
    });

    let enu_rx = router.subscribe_watch(|msg| match msg {
        MavMessage::LOCAL_POSITION_NED(data) => Some(Vec3::new(data.y, data.x, -data.z)),
        _ => None,
    });

    let global_rx = router.subscribe_watch(|msg| match msg {
        MavMessage::GLOBAL_POSITION_INT(data) => Some(Coords {
            lat_deg_e7: data.lat,
            lon_deg_e7: data.lon,
            alt_mm: data.alt,
        }),
        _ => None,
    });

    vec![
        create_telem_topic_mpsc(
            session,
            "heartbeat",
            heartbeat_rx,
            false,
            Priority::Background,
        )
        .await,
        create_telem_topic_watch(session, "pos/local/ned", ned_rx, true, Priority::RealTime).await,
        create_telem_topic_watch(session, "pos/local/enu", enu_rx, true, Priority::RealTime).await,
        create_telem_topic_watch(
            session,
            "pos/global/int",
            global_rx,
            true,
            Priority::RealTime,
        )
        .await,
    ]
}
