use std::{sync::OnceLock, time::Duration};
use zenoh;

pub static DRONE_NAME: OnceLock<String> = OnceLock::new();

#[tokio::main]
async fn main() {
    let config_path = std::env::var("ZENOH_CONFIG").expect("ZENOH_CONFIG not set");
    let mut cfg = zenoh::Config::from_file(&config_path).expect("failed to load ZENOH_CONFIG file");

    let namespace = cfg
        .get_json("namespace")
        .expect("IDENTITY: no namespace was set!")
        .trim_matches('"')
        .trim_matches('/')
        .to_string();

    let updated_namespace = format!("{namespace}/identity");

    DRONE_NAME
        .set(namespace)
        .expect("drone name should only be initialized once");

    cfg.insert_json5("namespace", &format!("{updated_namespace:?}"))
        .expect("IDENTITY: failed to set session namespace!");

    let session = zenoh::open(cfg).await.expect("failed to initialize zenoh");

    session
        .declare_queryable("whoami")
        .callback(|query| {
            tokio::spawn(async move {
                if let Some(name) = DRONE_NAME.get() {
                    query
                        .reply(query.key_expr().clone(), name.as_str())
                        .await
                        .ok();
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize whoami queryable");

    println!(
        "Set drone name to: {}!",
        DRONE_NAME.get().expect("drone name was initialized")
    );

    // Wait for subscribers to be ready
    tokio::time::sleep(Duration::from_millis(500)).await;

    // let session2 = session.clone();
    // tokio::task::spawn(async move {
    //     let subs = session2
    //         .declare_subscriber("**")
    //         .await
    //         .expect("failed to init subs");
    //     loop {
    //         let s = subs.recv().ok();
    //         if let Some(s) = s {
    //             // println!("key: {}", s.key_expr());
    //         }
    //     }
    // });

    // execute when cancelled
    tokio::signal::ctrl_c().await.ok();

    session.close().await.expect("failed to close session");
}
