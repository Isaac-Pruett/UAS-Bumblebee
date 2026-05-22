use crate::topic_utils::topic;
use mavlink::dialects::common::MavMessage;
use tokio::task::JoinHandle;
use zenoh::Session;

use serde::{Deserialize, Serialize};

pub trait MavMessageFilter {
    fn matches(msg: &MavMessage) -> bool;
}

pub async fn create_telem_topic<T, F>(
    session: &Session,
    sensor_name: &str,
    mut rx: tokio::sync::broadcast::Receiver<MavMessage>,
    packet_from_msg: F,
) -> JoinHandle<()>
where
    T: Serialize + for<'de> Deserialize<'de> + Send + 'static,
    F: Fn(&MavMessage) -> Option<T> + Send + Sync + 'static,
{
    let topic_name = topic(format!("telemetry/{sensor_name}").as_str());

    let publ = session
        .declare_publisher(topic_name.clone())
        .await
        .expect("failed to declare publisher");

    tokio::spawn(async move {
        loop {
            match rx.recv().await {
                Ok(m) => {
                    if let Some(packet) = packet_from_msg(&m) {
                        match rmp_serde::to_vec(&packet) {
                            Ok(bytes) => {
                                if let Err(e) = publ.put(bytes).await {
                                    eprintln!("Failed to publish telemetry: {e}");
                                }
                            }
                            Err(e) => {
                                eprintln!("Failed to serialize telemetry: {e}");
                            }
                        }
                    }
                }

                Err(tokio::sync::broadcast::error::RecvError::Lagged(n)) => {
                    eprintln!("Receiver lagged, skipped {n} messages");
                }

                Err(tokio::sync::broadcast::error::RecvError::Closed) => {
                    eprintln!("Broadcast channel closed, stopping '{topic_name}'");
                    break;
                }
            }
        }
    })
}
