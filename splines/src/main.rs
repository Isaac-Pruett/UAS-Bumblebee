use std::time::Duration;
use tokio::signal::ctrl_c;
use tokio::time::sleep;
use zenoh;

#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default()))
        .await
        .expect("splines error: failed to initialize session");

    // Wait for subscribers to be ready
    sleep(Duration::from_millis(500)).await;

    ctrl_c()
        .await
        .expect("splines error: failed to wait for ctrl-c");

    session
        .close()
        .await
        .expect("splines error: failed to close session");
}
