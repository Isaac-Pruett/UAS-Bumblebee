use crate::{
    connection::{MavlinkHandle, SYSTEM_ID, with_mavlink_router},
    get_session,
    packets::{coords::Coords, heartbeat::Heartbeat, local_kinematics::LocalKinematics},
};

use glam::Vec3;
use mavlink::dialects::common::{COMMAND_LONG_DATA, MavAutopilot, MavCmd, MavMessage, MavModeFlag};
use serde::Serialize;
use std::{
    sync::{LazyLock, OnceLock},
    time::{Duration, Instant},
};
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};
use zenoh::{Session, qos::Priority};

pub static AUTOPILOT: OnceLock<MavAutopilot> = OnceLock::new();

pub static AUTOPILOT_MODE: LazyLock<(
    watch::Sender<Option<AutopilotMode>>,
    watch::Receiver<Option<AutopilotMode>>,
)> = LazyLock::new(|| watch::channel(None));

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutopilotMode {
    pub armed: bool,
    pub mode: FlightMode,
}

impl AutopilotMode {
    pub fn can_fly(self) -> bool {
        self.armed && matches!(self.mode, FlightMode::Guided | FlightMode::Offboard)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    Manual,
    Stabilize,
    Guided,
    Offboard,
    Auto,
    Unknown,
}

pub fn autopilot_mode() -> Option<AutopilotMode> {
    *AUTOPILOT_MODE.1.borrow()
}

pub fn subscribe_autopilot_mode() -> watch::Receiver<Option<AutopilotMode>> {
    AUTOPILOT_MODE.0.subscribe()
}

fn decode_flight_mode(
    autopilot: MavAutopilot,
    base_mode: MavModeFlag,
    custom_mode: u32,
) -> FlightMode {
    match autopilot {
        MavAutopilot::MAV_AUTOPILOT_PX4 => decode_px4_mode(custom_mode),

        MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA => decode_ardupilot_mode(base_mode),

        _ => FlightMode::Unknown,
    }
}

fn decode_px4_mode(custom_mode: u32) -> FlightMode {
    let main_mode = ((custom_mode >> 16) & 0xff) as u8;

    match main_mode {
        1 => FlightMode::Manual,
        4 => FlightMode::Auto,
        6 => FlightMode::Offboard,
        7 => FlightMode::Stabilize,
        _ => FlightMode::Unknown,
    }
}

fn decode_ardupilot_mode(base_mode: MavModeFlag) -> FlightMode {
    use MavModeFlag as Flag;

    if base_mode.contains(Flag::MAV_MODE_FLAG_GUIDED_ENABLED) {
        FlightMode::Guided
    } else if base_mode.contains(Flag::MAV_MODE_FLAG_STABILIZE_ENABLED) {
        FlightMode::Stabilize
    } else if base_mode.contains(Flag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) {
        FlightMode::Manual
    } else if base_mode.contains(Flag::MAV_MODE_FLAG_AUTO_ENABLED) {
        FlightMode::Auto
    } else {
        FlightMode::Unknown
    }
}

fn update_autopilot_state(autopilot: MavAutopilot, base_mode: MavModeFlag, custom_mode: u32) {
    if AUTOPILOT.set(autopilot).is_ok() {
        println!("mavlink: autopilot = {autopilot:?}");
    }

    let state = AutopilotMode {
        armed: base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED),
        mode: decode_flight_mode(autopilot, base_mode, custom_mode),
    };

    if autopilot_mode() != Some(state) {
        println!(
            "mavlink: flight state armed={} mode={:?} custom_mode={custom_mode:#010x}",
            state.armed, state.mode,
        );
    }
    let _ = AUTOPILOT_MODE.0.send(Some(state));
}

async fn publish<T>(
    publisher: &zenoh::pubsub::Publisher<'_>,
    topic: &str,
    buffer: &mut Vec<u8>,
    packet: &T,
) where
    T: Serialize,
{
    buffer.clear();

    if let Err(err) = rmp_serde::encode::write(&mut *buffer, packet) {
        eprintln!("failed to serialize telemetry on '{topic}': {err}");
        return;
    }

    if let Err(err) = publisher.put(buffer.as_slice()).await {
        eprintln!("failed to publish telemetry on '{topic}': {err}");
    }
}

pub fn create_telem_topic_mpsc<T>(
    session: &Session,
    sensor_name: &str,
    mut rx: mpsc::Receiver<T>,
    express: bool,
    priority: Priority,
) -> JoinHandle<()>
where
    T: Serialize + Send + 'static,
{
    let session = session.clone();
    let topic = format!("telemetry/{sensor_name}");

    tokio::spawn(async move {
        let publisher = session
            .declare_publisher(&topic)
            .express(express)
            .priority(priority)
            .await
            .expect("failed to declare telemetry publisher");

        let mut buffer = Vec::with_capacity(512);

        while let Some(packet) = rx.recv().await {
            buffer.clear();

            if let Err(err) = rmp_serde::encode::write(&mut buffer, &packet) {
                eprintln!("failed to serialize telemetry on '{topic}': {err}");
                continue;
            }

            if let Err(err) = publisher.put(buffer.as_slice()).await {
                eprintln!("failed to publish telemetry on '{topic}': {err}");
            }
        }

        eprintln!("telemetry topic '{topic}' closed");
    })
}

pub fn create_telem_topic_watch<T>(
    session: &Session,
    sensor_name: &str,
    mut rx: watch::Receiver<Option<T>>,
    express: bool,
    priority: Priority,
) -> JoinHandle<()>
where
    T: Serialize + Send + Sync + Clone + 'static,
{
    let session = session.clone();
    let topic = format!("telemetry/{sensor_name}");

    tokio::spawn(async move {
        let publisher = session
            .declare_publisher(&topic)
            .express(express)
            .priority(priority)
            .await
            .expect("failed to declare telemetry publisher");

        let mut buffer = Vec::with_capacity(512);

        while rx.changed().await.is_ok() {
            let packet = {
                let borrowed = rx.borrow_and_update();
                borrowed.clone()
            };

            let Some(packet) = packet else {
                continue;
            };

            buffer.clear();

            if let Err(err) = rmp_serde::encode::write(&mut buffer, &packet) {
                eprintln!("failed to serialize telemetry on '{topic}': {err}");
                continue;
            }

            if let Err(err) = publisher.put(buffer.as_slice()).await {
                eprintln!("failed to publish telemetry on '{topic}': {err}");
            }
        }

        eprintln!("telemetry topic '{topic}' closed");
    })
}

/// Request PX4 stream `LOCAL_POSITION_NED_COV` (message ID 64) at 50 Hz.
///
/// The command is sent only after receiving a vehicle heartbeat, so it targets
/// the discovered vehicle and its autopilot component rather than relying on a
/// broadcast command.
pub fn request_local_kinematics_stream(mavlink: MavlinkHandle) -> JoinHandle<()> {
    tokio::spawn(async move {
        let system_id = loop {
            if let Some(&system_id) = SYSTEM_ID.get() {
                break system_id;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        };

        let request = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
            target_system: system_id,
            target_component: 1,
            command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
            param1: 64.0,
            param2: 20_000.0,
            ..Default::default()
        });

        if mavlink.send(request).await.is_err() {
            eprintln!("MAVLink task stopped before LOCAL_POSITION_NED_COV could be requested");
        }
    })
}

pub async fn declare_all() -> Vec<JoinHandle<()>> {
    let session = get_session();
    let start = Instant::now();

    let (heartbeat_rx, ned_rx, enu_rx, global_rx) = with_mavlink_router(|router| {
        let heartbeat_rx = router.subscribe(move |msg| {
            let MavMessage::HEARTBEAT(data) = msg else {
                return None;
            };

            update_autopilot_state(data.autopilot, data.base_mode, data.custom_mode);

            Some(Heartbeat {
                elapsed_us: start.elapsed().as_micros(),
            })
        });

        // LOCAL_POSITION_NED_COV is the local-state message that contains
        // position, velocity, and acceleration together.
        let ned_rx = router.subscribe_watch(|msg| match msg {
            MavMessage::LOCAL_POSITION_NED(data) => Some(LocalKinematics {
                position: Vec3::new(data.x, data.y, data.z),
                velocity: Vec3::new(data.vx, data.vy, data.vz),
                acceleration: Vec3::new(f32::NAN, f32::NAN, f32::NAN),
            }),
            MavMessage::LOCAL_POSITION_NED_COV(data) => Some(LocalKinematics {
                position: Vec3::new(data.x, data.y, data.z),
                velocity: Vec3::new(data.vx, data.vy, data.vz),
                acceleration: Vec3::new(data.ax, data.ay, data.az),
            }),
            _ => None,
        });

        let enu_rx = router.subscribe_watch(|msg| match msg {
            MavMessage::LOCAL_POSITION_NED(data) => Some(LocalKinematics {
                position: Vec3::new(data.y, data.x, -data.z),
                velocity: Vec3::new(data.vy, data.vx, -data.vz),
                acceleration: Vec3::new(f32::NAN, f32::NAN, f32::NAN),
            }),
            MavMessage::LOCAL_POSITION_NED_COV(data) => Some(LocalKinematics {
                position: Vec3::new(data.y, data.x, -data.z),
                velocity: Vec3::new(data.vy, data.vx, -data.vz),
                acceleration: Vec3::new(data.ay, data.ax, -data.az),
            }),
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

        (heartbeat_rx, ned_rx, enu_rx, global_rx)
    });

    vec![
        create_telem_topic_mpsc(
            session,
            "heartbeat",
            heartbeat_rx,
            false,
            Priority::Background,
        ),
        create_telem_topic_watch(session, "state/local/ned", ned_rx, true, Priority::RealTime),
        create_telem_topic_watch(session, "state/local/enu", enu_rx, true, Priority::RealTime),
        create_telem_topic_watch(
            session,
            "pos/global/int",
            global_rx,
            true,
            Priority::RealTime,
        ),
    ]
}
