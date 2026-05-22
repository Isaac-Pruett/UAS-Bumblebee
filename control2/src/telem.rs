use crate::topic_utils::topic;
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use zenoh::Session;

use serde::Serialize;

/// Spawn a Zenoh publisher that drains a typed mpsc channel.
///
/// The channel is produced by `MavlinkRouter::subscribe()` — no MavMessage
/// pattern matching happens here; that was already done at subscription time.
pub async fn create_telem_topic<T>(
    session: &Session,
    sensor_name: &str,
    mut rx: mpsc::Receiver<T>,
) -> JoinHandle<()>
where
    T: Serialize + Send + 'static,
{
    let topic_name = topic(format!("telemetry/{sensor_name}").as_str());

    let publ = session
        .declare_publisher(topic_name.clone())
        .await
        .expect("failed to declare publisher");

    tokio::spawn(async move {
        let mut buf = Vec::with_capacity(512);

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
