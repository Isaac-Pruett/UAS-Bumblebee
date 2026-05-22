use std::time::Duration;

use mavlink::{AsyncMavConnection, MAX_FRAME_SIZE, MavlinkVersion, dialects::common::MavMessage};
use tokio::{
    sync::{broadcast, mpsc},
    task::JoinHandle,
};

#[derive(Clone)]
pub struct MavlinkHandle {
    tx: mpsc::Sender<MavMessage>,
}

impl MavlinkHandle {
    pub async fn send(&self, msg: MavMessage) -> Result<(), mpsc::error::SendError<MavMessage>> {
        self.tx.send(msg).await
    }
}

pub async fn mav_connect(mavlink_conn_str: &str) -> AsyncMavConn {
    let mut mavconn = mavlink::connect_async::<MavMessage>(mavlink_conn_str)
        .await
        .expect("Failed to connect to MAVLink");

    mavconn.set_protocol_version(MavlinkVersion::V2);
    mavconn.set_allow_recv_any_version(true);

    mavconn
}

pub fn setup_mavlink_task(
    conn: AsyncMavConn,
) -> (
    MavlinkHandle,
    broadcast::Receiver<MavMessage>,
    JoinHandle<()>,
) {
    println!("control2: INIT MAVLINK TASK");

    let (msg_tx, msg_rx) = broadcast::channel::<MavMessage>(1024);
    let (send_tx, mut send_rx) = mpsc::channel::<MavMessage>(128);

    let handle = MavlinkHandle { tx: send_tx };

    let task = tokio::spawn(async move {
        let mut heartbeat_interval = tokio::time::interval(Duration::from_secs(1));

        loop {
            tokio::select! {
                biased;

                _ = heartbeat_interval.tick() => {
                    let heartbeat = MavMessage::HEARTBEAT(mavlink::dialects::common::HEARTBEAT_DATA {
                        custom_mode: 0,
                        mavtype: mavlink::dialects::common::MavType::MAV_TYPE_ONBOARD_CONTROLLER,
                        autopilot: mavlink::dialects::common::MavAutopilot::MAV_AUTOPILOT_INVALID,
                        base_mode: mavlink::dialects::common::MavModeFlag::empty(),
                        system_status: mavlink::dialects::common::MavState::MAV_STATE_ACTIVE,
                        mavlink_version: 3,
                    });

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
                        Ok((_header, msg)) => {
                            msg_tx.send(msg).ok();
                        }
                        Err(e) => {
                            eprintln!("MAVLink recv error: {e}");
                            tokio::time::sleep(Duration::from_millis(50)).await;
                        }
                    }
                }
            }
        }
    });

    (handle, msg_rx, task)
}

type AsyncMavConn = Box<dyn AsyncMavConnection<MavMessage> + Send + Sync>;
