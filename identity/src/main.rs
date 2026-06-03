use std::{sync::OnceLock, time::Duration};
use zenoh;

pub static DRONE_NAME: OnceLock<String> = OnceLock::new();

#[tokio::main]
async fn main() {
    let config_path = std::env::var("ZENOH_CONFIG").expect("ZENOH_CONFIG not set");
    let mut cfg = zenoh::Config::from_file(&config_path).expect("failed to load ZENOH_CONFIG file");
    let namespace = cfg
        .get_json("namespace")
        .expect("IDENTITY: no namespace was set!");
    let updated_namespace = namespace + "/identity";
    cfg.insert_json5("namespace", updated_namespace.as_str())
        .expect("IDENTITY: failed to set session namespace!");

    let session = zenoh::open(cfg).await.expect("failed to initialize zenoh");

    DRONE_NAME.get_or_init(|| {
        let name = std::env::var("DRONE_NAME").expect("!!!   THIS DRONE WAS NOT NAMED   !!!");
        println!("Set drone name to: {name}!");
        name
    });

    session
        .declare_queryable("whoami")
        .callback(|query| {
            tokio::spawn(async move {
                if let Some(n) = DRONE_NAME.get() {
                    let b = n.as_bytes();
                    if let Some(n) = rmp_serde::from_slice::<&[u8]>(b).ok() {
                        query.reply(query.key_expr().clone(), n).await.ok();
                    }
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize whoami queryable");
    // Wait for subscribers to be ready
    tokio::time::sleep(Duration::from_millis(500)).await;

    let session2 = session.clone();
    tokio::task::spawn(async move {
        let subs = session2
            .declare_subscriber("**")
            .await
            .expect("failed to init subs");
        loop {
            let s = subs.recv().ok();
            if let Some(s) = s {
                // println!("key: {}", s.key_expr());
            }
        }
    });

    // execute when cancelled
    tokio::signal::ctrl_c().await.ok();

    session.close().await.expect("failed to close session");
}
