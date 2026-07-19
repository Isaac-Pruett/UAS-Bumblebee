use crate::connection::MavlinkRouter;

use crate::packets::coords::Coords;
use crate::packets::full_state::{Attitude, FullState, LocalPosition};
use crate::packets::heartbeat::Heartbeat;

use glam::{Quat, Vec3};
use mavlink::dialects::common::{MavAutopilot, MavMessage, MavModeFlag};
use serde::{Deserialize, Serialize};

use std::sync::{LazyLock, OnceLock};
use std::time::Instant;
use tokio::sync::{mpsc, watch};
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
) -> anyhow::Result<JoinHandle<()>>
where
    T: Serialize + Send + 'static,
{
    let topic_name = String::new() + "telemetry/" + sensor_name;

    let publ = session
        .declare_publisher(topic_name.clone())
        .express(is_express)
        .priority(priority)
        .await
        .map_err(|e| anyhow::anyhow!("failed to declare publisher for '{topic_name}': {e}"))?;

    let mut buf = Vec::with_capacity(512);
    Ok(tokio::spawn(async move {
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
    }))
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
) -> anyhow::Result<JoinHandle<()>>
where
    T: Serialize + Send + Sync + Clone + 'static,
{
    let topic_name = String::new() + "telemetry/" + sensor_name;

    let publ = session
        .declare_publisher(topic_name.clone())
        .express(is_express)
        .priority(priority)
        .await
        .map_err(|e| anyhow::anyhow!("failed to declare publisher for '{topic_name}': {e}"))?;

    let mut buf = Vec::with_capacity(512);
    Ok(tokio::spawn(async move {
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
    }))
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

fn compute_flight_mode(
    autopilot: MavAutopilot,
    base_mode: MavModeFlag,
    custom_mode: u32,
) -> FlightMode {
    // PX4_CUSTOM_MAIN_MODE_* from PX4 firmware's mavlink/v2.0 custom_mode encoding.
    const PX4_CUSTOM_MAIN_MODE_MANUAL: u8 = 1;
    const PX4_CUSTOM_MAIN_MODE_AUTO: u8 = 4;
    const PX4_CUSTOM_MAIN_MODE_STABILIZED: u8 = 7;
    const PX4_CUSTOM_MAIN_MODE_OFFBOARD: u8 = 6;

    match autopilot {
        MavAutopilot::MAV_AUTOPILOT_PX4 => {
            let main_mode = ((custom_mode >> 16) & 0xFF) as u8;
            match main_mode {
                PX4_CUSTOM_MAIN_MODE_MANUAL => FlightMode::Manual,
                PX4_CUSTOM_MAIN_MODE_STABILIZED => FlightMode::Stabilize,
                PX4_CUSTOM_MAIN_MODE_OFFBOARD => FlightMode::Offboard,
                PX4_CUSTOM_MAIN_MODE_AUTO => FlightMode::Auto,
                _ => FlightMode::Unknown,
            }
        }

        // ArduPilot encodes mode as overlapping bitflags in base_mode, so the
        // check order below encodes priority, not just presence. GUIDED must
        // be checked before AUTO because ArduPilot sets both flags when a
        // guided-within-auto submode is active, and GUIDED should win since
        // it reflects the more specific/authoritative control state.
        MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA => {
            if base_mode.contains(MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED) {
                FlightMode::Guided
            } else if base_mode.contains(MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED) {
                FlightMode::Stabilize
            } else if base_mode.contains(MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) {
                FlightMode::Manual
            } else if base_mode.contains(MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED) {
                FlightMode::Auto
            } else {
                FlightMode::Unknown
            }
        }

        _ => FlightMode::Unknown,
    }
}

use std::sync::RwLock;

pub static AUTOPILOT_MODE: LazyLock<RwLock<Option<AutopilotMode>>> =
    LazyLock::new(|| RwLock::new(None));
/// Register all telemetry subscriptions and spawn their publisher tasks.
/// Must be called before `setup_mavlink_task` consumes the router.
/// Returns all JoinHandles so the caller can await them for shutdown.
pub async fn declare_all(
    session: &Session,
    router: &mut MavlinkRouter,
) -> anyhow::Result<Vec<JoinHandle<()>>> {
    let start = Instant::now();

    // mpsc: every heartbeat event matters
    let heartbeat_rx = router.subscribe(move |msg| match msg {
        MavMessage::HEARTBEAT(data) => {
            let res = AUTOPILOT.set(data.autopilot);
            if res.is_ok() {
                println!("mavlink: SET AUTOPILOT TO {:?}", data.autopilot)
            } else if AUTOPILOT.get() != Some(&data.autopilot) {
                eprintln!(
                    "mavlink: autopilot type changed after initial set (was {:?}, now {:?}), ignoring",
                    AUTOPILOT.get(),
                    data.autopilot
                );
            }

            let base_mode = data.base_mode;
            let custom_mode = data.custom_mode;
            let autopilot = data.autopilot;

            let new_mode = AutopilotMode {
                armed: base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),
                mode: compute_flight_mode(autopilot, base_mode, custom_mode),
            };

            // Sync lock, tiny critical section, no await — safe to do inline.
            *AUTOPILOT_MODE.write().unwrap() = Some(new_mode);

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

    let attitude_rx = router.subscribe_watch(|msg| match msg {
        MavMessage::ATTITUDE(data) => Some((
            data.roll,
            data.pitch,
            data.yaw,
            data.rollspeed,
            data.pitchspeed,
            data.yawspeed,
            data.time_boot_ms,
        )),
        _ => None,
    });

    let (ned_state_rx, ned_state_handle) =
        collect_full_state(ned_rx.clone(), attitude_rx.clone(), 100);

    let (enu_state_rx, enu_state_handle) =
        collect_full_state(enu_rx.clone(), attitude_rx.clone(), 100);

    Ok(vec![
        ned_state_handle,
        enu_state_handle,
        create_telem_topic_watch(
            session,
            "state/local/ned",
            ned_state_rx,
            true,
            Priority::RealTime,
        )
        .await?,
        create_telem_topic_watch(
            session,
            "state/local/enu",
            enu_state_rx,
            true,
            Priority::RealTime,
        )
        .await?,
        create_telem_topic_mpsc(
            session,
            "heartbeat",
            heartbeat_rx,
            false,
            Priority::Background,
        )
        .await?,
        create_telem_topic_watch(session, "attitude", attitude_rx, true, Priority::RealTime)
            .await?,
        create_telem_topic_watch(session, "pos/local/ned", ned_rx, true, Priority::RealTime)
            .await?,
        create_telem_topic_watch(session, "pos/local/enu", enu_rx, true, Priority::RealTime)
            .await?,
        create_telem_topic_watch(
            session,
            "pos/global/int",
            global_rx,
            true,
            Priority::RealTime,
        )
        .await?,
    ])
}

fn collect_full_state(
    mut position_rx: watch::Receiver<Option<LocalPosition>>,
    mut attitude_rx: watch::Receiver<Option<Attitude>>,
    timestamp_tolerance_ms: u32,
) -> (watch::Receiver<Option<FullState>>, JoinHandle<()>) {
    let (full_state_tx, full_state_rx) = watch::channel::<Option<FullState>>(None);

    let handle = tokio::spawn(async move {
        loop {
            tokio::select! {
                result = position_rx.changed() => {
                    if result.is_err() {
                        break;
                    }
                }

                result = attitude_rx.changed() => {
                    if result.is_err() {
                        break;
                    }
                }
            }

            let position = position_rx.borrow().clone();
            let attitude = attitude_rx.borrow().clone();

            let (
                Some((pos, velo, position_time_ms)),
                Some((roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, attitude_time_ms)),
            ) = (position, attitude)
            else {
                continue;
            };

            let state = FullState {
                pos,
                velo,
                orientation: Quat::from_euler(glam::EulerRot::XYZ, roll, pitch, yaw),
                angular_velocity: Vec3::new(rollspeed, pitchspeed, yawspeed),
                yaw,
                attitude_time_ms,
                position_time_ms,
            };

            if !state.timestamps_are_close(timestamp_tolerance_ms) {
                continue;
            }

            let _ = full_state_tx.send(Some(state));
        }
    });

    (full_state_rx, handle)
}
