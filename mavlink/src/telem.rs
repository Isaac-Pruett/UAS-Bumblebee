use crate::connection::MavlinkRouter;
use crate::control::movement;
use crate::packets::coords::Coords;
use crate::packets::heartbeat::Heartbeat;

use glam::Vec3;
use mavlink::dialects::common::{MavAutopilot, MavMessage, MavModeFlag, MavType};
use serde::Serialize;
use std::sync::{LazyLock, OnceLock};
use std::time::Instant;
use tokio::sync::{RwLock, mpsc, watch};
use tokio::task::JoinHandle;
use zenoh::Session;
use zenoh::qos::Priority;

pub static AUTOPILOT: OnceLock<MavAutopilot> = OnceLock::new();

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
    let topic_name = String::new() + "telemetry/" + sensor_name;

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
    let topic_name = String::new() + "telemetry/" + sensor_name;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutopilotMode {
    pub armed: bool,
    pub mode: FlightMode,
}

impl AutopilotMode {
    #[inline]
    pub fn can_fly(&self) -> bool {
        self.armed && matches!(self.mode, FlightMode::Guided | FlightMode::Offboard)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    Manual,
    Stabilize,
    Guided,   // ArduPilot
    Offboard, // PX4
    Auto,
    Unknown,
}

pub static AUTOPILOT_MODE: LazyLock<RwLock<Option<AutopilotMode>>> =
    LazyLock::new(|| RwLock::new(None));
/// Register all telemetry subscriptions and spawn their publisher tasks.
/// Must be called before `setup_mavlink_task` consumes the router.
/// Returns all JoinHandles so the caller can await them for shutdown.
pub async fn declare_all(session: &Session, router: &mut MavlinkRouter) -> Vec<JoinHandle<()>> {
    let start = Instant::now();

    // mpsc: every heartbeat event matters
    let heartbeat_rx = router.subscribe(move |msg| match msg {
        MavMessage::HEARTBEAT(data) => {
            let res = AUTOPILOT.set(data.autopilot);
            if res.is_ok() {
                println!("mavlink: SET AUTOPILOT TO {:?}", data.autopilot)
            }

            let base_mode = data.base_mode;
            let custom_mode = data.custom_mode;
            let autopilot = data.autopilot;

            tokio::spawn(async move {
                let mut mode = AUTOPILOT_MODE.write().await;

                *mode = Some(AutopilotMode {
                    armed: base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),

                    mode: match autopilot {
                        MavAutopilot::MAV_AUTOPILOT_PX4 => {
                            let main_mode = ((custom_mode >> 16) & 0xFF) as u8;

                            match main_mode {
                                1 => FlightMode::Manual,
                                7 => FlightMode::Stabilize,
                                6 => FlightMode::Offboard,
                                4 => FlightMode::Auto,
                                _ => FlightMode::Unknown,
                            }
                        }

                        MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA => {
                            if base_mode.contains(MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED) {
                                FlightMode::Guided // order matters specifically here
                            } else if base_mode
                                .contains(MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED)
                            {
                                FlightMode::Stabilize
                            } else if base_mode
                                .contains(MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                            {
                                FlightMode::Manual
                            } else if base_mode.contains(MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED) {
                                FlightMode::Auto // order matters specifically here
                            } else {
                                FlightMode::Unknown
                            }
                        }

                        _ => FlightMode::Unknown,
                    },
                });
            });

            Some(Heartbeat {
                elapsed_us: start.elapsed().as_micros(),
            })
        }
        _ => None,
    });

    // watch: only the latest position matters
    let ned_rx = router.subscribe_watch(|msg| match msg {
        MavMessage::LOCAL_POSITION_NED(data) => Some((
            Vec3::new(data.x, data.y, data.z),
            Vec3::new(data.vx, data.vy, data.vz),
            data.time_boot_ms,
        )),
        _ => None,
    });

    let enu_rx = router.subscribe_watch(|msg| match msg {
        MavMessage::LOCAL_POSITION_NED(data) => Some((
            Vec3::new(data.y, data.x, -data.z),
            Vec3::new(data.vy, data.vx, -data.vz),
            data.time_boot_ms,
        )),
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
