use formations::boids::boid::Boid;
use formations::boids::cohesion_fn::cohesion_fn;
use formations::boids::repulsion_fn::repulsion_fn;
use glam::Vec3;
use std::collections::HashMap;

use std::sync::{Arc, LazyLock, Mutex, OnceLock};
use std::time::Duration;
use zenoh;

use crate::local_kinematics::LocalKinematics;
use crate::movement::Movement;

mod local_kinematics;
mod movement;

struct FormationState {
    own_name: String,
    own_boid: Mutex<Boid>,
    others: Mutex<HashMap<String, Boid>>,
}

static MAX_ACCEL_M_S2: OnceLock<f32> = OnceLock::new();

// Do not command the origin until a formation coordinator has supplied a center.
static CENTER: LazyLock<Mutex<Option<Vec3>>> = LazyLock::new(|| Mutex::new(None));

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
        others: Mutex::new(HashMap::default()),
    });

    identity_session.close().await?;

    let telemetry_session = zenoh::open(
        zenoh::Config::from_file(
            std::env::var("NULL_ZENOH_CONFIG").expect("No null-namespaced config was available."),
        )
        .unwrap_or(zenoh::Config::default()),
    )
    .await?;

    let formation_state = Arc::clone(&state);
    let all_states_subs = telemetry_session
        .declare_subscriber("**/mavlink/telemetry/state/local/enu")
        .callback(move |sample| {
            let keyexpr = sample.key_expr();
            let Some(other_name) = keyexpr.split('/').next() else {
                return;
            };
            if let Ok(kinematics) =
                rmp_serde::from_slice::<LocalKinematics>(&sample.payload().to_bytes())
            {
                if other_name == formation_state.own_name {
                    let mut own_boid = formation_state
                        .own_boid
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner());
                    *own_boid = Boid::from(kinematics);
                } else {
                    let mut others = formation_state
                        .others
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner());
                    others.insert(other_name.to_string(), Boid::from(kinematics));
                }
            }
        })
        .await?;

    let controller = telemetry_session
        .declare_publisher(format!("{}/mavlink/control/move/enu/raw", state.own_name))
        .await?;

    let _center_subscription = telemetry_session
        .declare_subscriber("formations/center")
        .callback(|sample| {
            let mut ctr_lck = CENTER
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());

            if let Ok(center) = rmp_serde::decode::from_slice::<Vec3>(&sample.payload().to_bytes())
            {
                *ctr_lck = Some(center);
            }
        })
        .await?;

    let mainloop_state = Arc::clone(&state);
    let _mainloop = tokio::spawn(async move {
        loop {
            let (my_state_copy, others_copy) = {
                (
                    mainloop_state
                        .own_boid
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner())
                        .clone(),
                    mainloop_state
                        .others
                        .lock()
                        .unwrap_or_else(|poisoned| poisoned.into_inner())
                        .clone(),
                )
            };

            let mut all_boids = others_copy.into_values().collect::<Vec<_>>();
            all_boids.push(my_state_copy.clone());

            let all_pos = all_boids.iter().map(|b| b.pos).collect::<Vec<_>>();

            let center = *CENTER
                .lock()
                .unwrap_or_else(|poisoned| poisoned.into_inner());

            let m = if let Some(center) = center {
                let repulsion = repulsion_fn(all_boids.len() - 1, &all_pos, 2.0, 12.0);
                let cohesion = cohesion_fn(
                    my_state_copy.pos,
                    my_state_copy.velo,
                    center.into(),
                    2.0,
                    2.0,
                );
                let max_accel = MAX_ACCEL_M_S2.get().copied().unwrap_or(1.0);
                let acc = (repulsion + cohesion).clamp_length_max(max_accel);

                Movement {
                    pos: Vec3::NAN,
                    velo: Vec3::NAN,
                    acc: acc.to_vec3(),
                    yaw: f32::NAN,
                    yaw_rate: f32::NAN,
                    force_en: false,
                }
            } else {
                // Keep feeding an offboard/GUIDED setpoint without treating the
                // local-frame origin as the formation target.
                Movement {
                    pos: my_state_copy.pos.to_vec3(),
                    velo: Vec3::NAN,
                    acc: Vec3::NAN,
                    yaw: f32::NAN,
                    yaw_rate: f32::NAN,
                    force_en: false,
                }
            };

            let payload = rmp_serde::encode::to_vec(&m).expect("failed to create payload");

            // println!("Trying to move: {m:?}");
            controller.put(payload).await.ok();
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
    });

    tokio::signal::ctrl_c()
        .await
        .expect("Failed to listen for Ctrl-C");

    drop(all_states_subs);
    telemetry_session.close().await?;
    Ok(())
}
