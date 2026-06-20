use std::time::Duration;
use tokio::signal::ctrl_c;
use zenoh;
use zenoh::Wait;

mod motion_task;
mod proxy;
mod splines;

use splines::build_spline;

use tokio::time::sleep;

#[tokio::main]
async fn main() {
    let session = zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default()))
        .await
        .expect("motion_controller error: failed to initialize session");

    // Wait for subscribers to be ready
    sleep(Duration::from_millis(500)).await;

    ctrl_c()
        .await
        .expect("motion_controller error: failed to wait for ctrl-c");

    session
        .close()
        .await
        .expect("motion_controller error: failed to close session");
}
