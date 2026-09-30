use std::time::Duration;

use mavlink::{
    AsyncMavConnection, MavlinkVersion,
    dialects::common::{MavCmd, MavMessage},
};
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};

type AsyncMavConn = Box<dyn AsyncMavConnection<MavMessage> + Send + Sync>;

// A filter+mapper registered with the router.
// Boxes the closure so subscribers of different output types can coexist in one Vec.
type ForwardFn = Box<dyn Fn(&MavMessage) + Send + Sync>;

struct Subscription {
    forward: ForwardFn,
}

static MAV_ROUTER: std::sync::OnceLock<std::sync::Mutex<MavlinkRouter>> =
    std::sync::OnceLock::new();

pub fn get_mavlink_router() -> &'static std::sync::Mutex<MavlinkRouter> {
    MAV_ROUTER.get_or_init(|| std::sync::Mutex::new(MavlinkRouter::new()))
}

pub fn with_mavlink_router<R>(f: impl FnOnce(&mut MavlinkRouter) -> R) -> R {
    let mut router = get_mavlink_router().lock().unwrap();
    f(&mut router)
}

// -------------------------------------------------------------------
// MavlinkRouter
//
// Call `.subscribe()` before starting the task (i.e. before calling
// `setup_mavlink_task`). Each subscription returns a typed mpsc::Receiver.
// The router task fans out every received MavMessage to whichever
// subscriptions match it, cloning only when necessary.
// -------------------------------------------------------------------
pub struct MavlinkRouter {
    subscriptions: Vec<Subscription>,
}

impl MavlinkRouter {
    pub fn new() -> Self {
        Self {
            subscriptions: Vec::new(),
        }
    }

    /// Register a typed subscription. `extract` is called once per message;
    /// if it returns `Some(T)` the value is forwarded to the returned Receiver.
    /// Slow receivers get `try_send` — lagged messages are dropped rather than
    /// blocking the router.
    pub fn subscribe<T, F>(&mut self, extract: F) -> mpsc::Receiver<T>
    where
        T: Send + 'static,
        F: Fn(&MavMessage) -> Option<T> + Send + Sync + 'static,
    {
        let (tx, rx) = mpsc::channel::<T>(256);

        // We need two separate closures that share `extract`.
        // Wrap in Arc so both closures can own it.

        self.subscriptions.push(Subscription {
            forward: Box::new(move |msg| {
                if let Some(val) = extract(&msg) {
                    if tx.try_send(val).is_err() {
                        // eprintln!("MavlinkRouter: subscriber channel full, dropping message");
                    }
                }
            }),
        });

        rx
    }

    /// Like `subscribe`, but backed by a `watch` channel. The router always
    /// writes the latest value; the receiver only ever sees the most recent one.
    /// Use for high-rate sensor streams where stale intermediate values can be
    /// discarded (position, velocity, etc.).
    pub fn subscribe_watch<T, F>(&mut self, extract: F) -> watch::Receiver<Option<T>>
    where
        T: Send + Sync + 'static,
        F: Fn(&MavMessage) -> Option<T> + Send + Sync + 'static,
    {
        let (tx, rx) = watch::channel::<Option<T>>(None);

        self.subscriptions.push(Subscription {
            forward: Box::new(move |msg| {
                if let Some(val) = extract(msg) {
                    // watch::Sender::send only errors if all receivers are
                    // dropped — safe to ignore.
                    let _ = tx.send(Some(val));
                }
            }),
        });

        rx
    }
}

// -------------------------------------------------------------------
// MavlinkHandle — send arbitrary MavMessages back to the drone
// -------------------------------------------------------------------

pub type MavlinkHandle = mpsc::Sender<MavMessage>;

// -------------------------------------------------------------------
// Connect
// -------------------------------------------------------------------
pub async fn mav_connect(mavlink_conn_str: &str) -> AsyncMavConn {
    let mut mavconn = mavlink::connect_async::<MavMessage>(mavlink_conn_str)
        .await
        .expect("Failed to connect to MAVLink");

    mavconn.set_protocol_version(MavlinkVersion::V2);
    mavconn.set_allow_recv_any_version(true);

    mavconn
}

use std::sync::OnceLock;

use crate::control::takeoff::LaunchErrors;

pub static SYSTEM_ID: OnceLock<u8> = OnceLock::new();

// -------------------------------------------------------------------
// Task setup
//
// Consumes the router (all subscriptions must be registered first),
// spawns the MAVLink I/O loop on its own blocking thread with a
// dedicated single-threaded runtime so it never competes with the
// main Tokio pool.
// -------------------------------------------------------------------
pub fn setup_mavlink_task(conn: AsyncMavConn) -> (MavlinkHandle, JoinHandle<()>) {
    println!("mavlink: INIT MAVLINK TASK");

    let (handle, mut send_rx) = mpsc::channel::<MavMessage>(128);

    let subscriptions = {
        let mut router = get_mavlink_router().lock().unwrap();
        std::mem::take(&mut router.subscriptions)
    };

    println!(
        "mavlink: starting with {} subscriptions",
        subscriptions.len()
    );

    let task = tokio::task::spawn_blocking(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();

        rt.block_on(async move {
            let mut heartbeat_interval =
                tokio::time::interval(Duration::from_secs(1));

            loop {
                tokio::select! {
                    biased;

                    _ = heartbeat_interval.tick() => {
                        let heartbeat = MavMessage::HEARTBEAT(
                            mavlink::dialects::common::HEARTBEAT_DATA {
                                custom_mode: 0,
                                mavtype: mavlink::dialects::common::MavType::MAV_TYPE_ONBOARD_CONTROLLER,
                                autopilot: mavlink::dialects::common::MavAutopilot::MAV_AUTOPILOT_INVALID,
                                base_mode: mavlink::dialects::common::MavModeFlag::empty(),
                                system_status: mavlink::dialects::common::MavState::MAV_STATE_ACTIVE,
                                mavlink_version: 3,
                            }
                        );

                        if let Err(e) = conn.send_default(&heartbeat).await {
                            eprintln!("MAVLink heartbeat send error: {e}");
                        }
                    }

                    Some(msg) = send_rx.recv() => {
                        if let Err(e) = conn.send_default(&msg).await {
                            eprintln!("MAVLink send error: {e}");
                        }
                    }

                    result = conn.recv() => {
                        match result {
                            Ok((header, msg)) => {
                                if matches!(msg, MavMessage::HEARTBEAT(_)) {
                                    SYSTEM_ID.set(header.system_id).ok();
                                }

                                for sub in &subscriptions {
                                    (sub.forward)(&msg);
                                }
                            }

                            Err(e) => {
                                eprintln!("MAVLink recv error: {e}");
                                tokio::time::sleep(
                                    Duration::from_millis(50)
                                ).await;
                            }
                        }
                    }
                }
            }
        });
    });

    (handle, task)
}

pub struct MavlinkAckChannel {
    cmd_type: MavCmd,
    rx: mpsc::Receiver<MavMessage>,
}

impl MavlinkAckChannel {
    pub fn new(cmd: MavCmd) -> Self {
        with_mavlink_router(|router| {
            let rx = router.subscribe(move |msg| match msg {
                MavMessage::COMMAND_ACK(data) if data.command == cmd => Some(msg.clone()),
                _ => None,
            });
            MavlinkAckChannel {
                cmd_type: cmd,
                rx: rx,
            }
        })
    }
    pub async fn wait_ack(
        &mut self,
        timeout: Duration,
    ) -> Result<mavlink::dialects::common::MavResult, LaunchErrors> {
        match tokio::time::timeout(timeout, self.rx.recv()).await {
            Ok(Some(MavMessage::COMMAND_ACK(data))) => Ok(data.result),
            Ok(Some(_)) => unreachable!("ACK router received non-ACK message"),
            Ok(None) => Err(LaunchErrors::AckRejected),
            Err(_) => Err(LaunchErrors::AckTimeout),
        }
    }
}
