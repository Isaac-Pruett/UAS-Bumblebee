use crate::connection::MavlinkRouter;
use crate::packets::coords::Coords;
use crate::packets::heartbeat::Heartbeat;
use crate::topic_utils::topic;
use glam::Vec3;
use mavlink::dialects::common::MavMessage;
use serde::Serialize;
use std::time::Instant;
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use zenoh::Session;
use zenoh::qos::Priority;

/// Spawn a Zenoh publisher that drains a typed mpsc channel.
pub async fn create_telem_topic<T>(
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

/// Register all telemetry subscriptions and spawn their publisher tasks.
/// Must be called before `setup_mavlink_task` consumes the router.
/// Returns all JoinHandles so the caller can await them for shutdown.
pub async fn declare_all(session: &Session, router: &mut MavlinkRouter) -> Vec<JoinHandle<()>> {
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

    vec![
        create_telem_topic(
            session,
            "heartbeat",
            heartbeat_rx,
            false,
            Priority::Background,
        )
        .await,
        create_telem_topic(session, "pos/local/ned", ned_rx, true, Priority::RealTime).await,
        create_telem_topic(session, "pos/local/enu", enu_rx, true, Priority::RealTime).await,
        create_telem_topic(
            session,
            "pos/global/int",
            global_rx,
            true,
            Priority::RealTime,
        )
        .await,
    ]
}
