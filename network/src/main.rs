use std::time::Duration;

use batman_robin::{self, MeshSelector};

mod serializations;

use serde::Serialize;
use serializations::{SerializableGateway, SerializableNeighbor};

async fn publish<T>(objs: &[T], pub_key: &str, publisher: &zenoh::pubsub::Publisher<'_>)
where
    T: Serialize,
{
    for obj in objs {
        match rmp_serde::to_vec(obj) {
            Ok(msg) => {
                if let Err(err) = publisher.put(msg).await {
                    log::error!("Failed to publish {pub_key}: {err}");
                }
            }
            Err(err) => {
                log::error!("Failed to serialize {pub_key}: {err}");
            }
        }
    }
}

#[cfg(target_os = "linux")]
#[tokio::main]
async fn main() -> zenoh::Result<()> {
    let mut config = zenoh::Config::from_env().unwrap_or_else(|_| zenoh::Config::default());

    let gl_namespace = config.get_json("namespace")?;

    config.insert_json5("namespace", &format!("{}/network", gl_namespace))?;

    let session = zenoh::open(config).await?;

    let neighbors_pub = session.declare_publisher("neighbors").await?;
    let gateways_pub = session.declare_publisher("gateways").await?;

    let client = batman_robin::Client::new();

    let mut interval = tokio::time::interval(Duration::from_millis(200));

    loop {
        interval.tick().await;

        let gateways = client
            .gateways(Some(MeshSelector::with_name("bat0")))
            .await?;

        let neighbors = client.neighbors(MeshSelector::with_name("bat0")).await?;

        let serializable_gateways: Vec<_> = gateways.iter().map(SerializableGateway).collect();

        let serializable_neighbors: Vec<_> = neighbors.iter().map(SerializableNeighbor).collect();

        tokio::join!(
            publish(&serializable_gateways, "gateway", &gateways_pub),
            publish(&serializable_neighbors, "neighbor", &neighbors_pub),
        );
    }
}

#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("network requires Linux with BATMAN-adv");
}
