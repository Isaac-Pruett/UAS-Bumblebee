use formations::boids::boid::Boid;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use zenoh;

use crate::local_kinematics::LocalKinematics;

mod local_kinematics;

struct FormationState {
    own_name: String,
    own_boid: Mutex<Boid>,
    others: Mutex<HashMap<String, Boid>>,
}

#[tokio::main]
async fn main() -> zenoh::Result<()> {
    let identity_session =
        zenoh::open(zenoh::Config::from_env().unwrap_or(zenoh::Config::default())).await?;

    let replies = identity_session.get("identity/whoami").await?;
    let mut own_name = None;

    while let Ok(reply) = replies.recv_async().await {
        let name = reply
            .result()
            .ok()
            .and_then(|sample| sample.payload().try_to_string().ok())
            .map(|value| value.to_string());

        if let Some(name) = name {
            println!("Found identity: {name}");
            own_name = Some(name);
        }
    }

    let state = Arc::new(FormationState {
        own_name: own_name.expect("Name never set"),
        own_boid: Mutex::new(Boid::ZERO),
        others: Mutex::new(HashMap::new()),
    });

    identity_session.close().await?;

    let telemetry_session = zenoh::open(
        zenoh::Config::from_file(
            std::env::var("NULL_ZENOH_CONFIG").expect("No null-namespaced config was available."),
        )
        .unwrap_or(zenoh::Config::default()),
    )
    .await?;

    let own_state = Arc::clone(&state);
    let own_keyexpr = format!("{}/mavlink/telemetry/state/local/enu", state.own_name);
    let own_state_subscriber = telemetry_session
        .declare_subscriber(own_keyexpr)
        .callback(move |sample| {
            if let Ok(kinematics) =
                rmp_serde::from_slice::<LocalKinematics>(&sample.payload().to_bytes())
            {
                let mut own_boid = own_state
                    .own_boid
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
                own_boid.pos = kinematics.position.into();
                own_boid.velo = kinematics.velocity.into();
                own_boid.acc = kinematics.acceleration.into();
            }
        })
        .await?;

    let others_state = Arc::clone(&state);
    let others_subscriber = telemetry_session
        .declare_subscriber("**/mavlink/telemetry/state/local/enu")
        .callback(move |sample| {
            let keyexpr = sample.key_expr();
            let Some(other_name) = keyexpr.split('/').next() else {
                return;
            };

            if other_name == others_state.own_name {
                return;
            }

            if let Ok(kinematics) =
                rmp_serde::from_slice::<LocalKinematics>(&sample.payload().to_bytes())
            {
                let mut others = others_state
                    .others
                    .lock()
                    .unwrap_or_else(|poisoned| poisoned.into_inner());
                others.insert(other_name.to_string(), Boid::from(kinematics));
            }
        })
        .await?;

    tokio::signal::ctrl_c()
        .await
        .expect("Failed to listen for Ctrl-C");

    drop((own_state_subscriber, others_subscriber));
    telemetry_session.close().await?;
    Ok(())
}
