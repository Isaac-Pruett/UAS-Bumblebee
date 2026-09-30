use formations::boids::boid::Boid;
use glam::Vec3;
use std::sync::Mutex;
use std::{thread::sleep, time::Duration};
use zenoh;

use crate::local_kinematics::LocalKinematics;

mod local_kinematics;

static MYSELF_STATE: Mutex<Boid> = Mutex::new(Boid::ZERO);

#[tokio::main]
async fn main() -> zenoh::Result<()> {
    let session =
        zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default())).await?;

    // Wait for subscribers to be ready
    sleep(Duration::from_millis(500));
    let replies = session.get("identity/whoami").await?;

    let mut my_name = None;

    while let Ok(reply) = replies.recv_async().await {
        let name: Option<String> = reply
            .result()
            .ok()
            .and_then(|sample| sample.payload().try_to_string().ok())
            .map(|value| value.to_string()); // owned; no longer borrows `reply`

        if let Some(name) = name {
            println!("Found identity: {name}");
            my_name = Some(name);
        }
    }

    let name = my_name.expect("Name never set");

    session
        .declare_subscriber(format!("{name}/mavlink/telemetry/state/local/enu"))
        .callback(|sample| {
            let state = rmp_serde::from_slice::<LocalKinematics>(&sample.payload().to_bytes());
            if let Some(state) = state.ok() {
                let mut my_state = MYSELF_STATE.lock().unwrap();
                my_state.pos = state.position.into();
                my_state.velo = state.velocity.into();
                my_state.acc = state.acceleration.into();
            }
        })
        .await?;

    session.close().await?;
    Ok(())
}
