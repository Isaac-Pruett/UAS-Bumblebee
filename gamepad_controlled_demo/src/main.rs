mod orbit;

use glam::Vec3;
use orbit::Orbit;
use serde::{Deserialize, Serialize};
use std::sync::Mutex;
use std::{
    sync::LazyLock,
    time::{Duration, Instant},
};

#[derive(Debug, Serialize, Deserialize, Default, Clone, Copy)]
struct GamepadState {
    x: bool,
    a: bool,
    b: bool,
    y: bool,
    left_bumper: bool,
    right_bumper: bool,
    select: bool,
    start: bool,
    mode: bool,
    left_stick_button: bool,
    right_stick_button: bool,
    dpad_up: bool,
    dpad_down: bool,
    dpad_left: bool,
    dpad_right: bool,
    left_stick_x: f32,
    left_stick_y: f32,
    right_stick_x: f32,
    right_stick_y: f32,
    left_trigger: f32,
    right_trigger: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct LaunchCommand {
    pub height_m: f32,
    pub yaw: f32,
    pub lat: Option<f32>,
    pub lon: Option<f32>,
}

impl LaunchCommand {
    fn local_height_m(&self, origin: &Coords) -> f32 {
        // MAV_CMD_NAV_TAKEOFF uses AMSL altitude; formation centers use local ENU.
        self.height_m - origin.alt_mm as f32 / 1000.0
    }
}

#[derive(Serialize)]
struct Coords {
    lat_deg_e7: i32,
    lon_deg_e7: i32,
    alt_mm: i32,
}

static GP_STATE: LazyLock<Mutex<GamepadState>> =
    LazyLock::new(|| Mutex::new(GamepadState::default()));

const DRONES: &[&str] = &["luke", "akshay", "reagan", "2luke", "2akshay", "2reagan"];

async fn request_arm(session: zenoh::Session, drone: &'static str) {
    println!("arming {drone}...");

    match session.get(format!("{drone}/mavlink/control/arm")).await {
        Ok(replies) => {
            let mut received_reply = false;

            while let Ok(reply) = replies.recv_async().await {
                received_reply = true;
                match reply.result() {
                    Ok(sample) => match sample.payload().try_to_string() {
                        Ok(result) => println!("{drone}: arm result: {result}"),
                        Err(err) => eprintln!("{drone}: arm returned a non-text reply: {err}"),
                    },
                    Err(err) => eprintln!("{drone}: arm rejected: {err:?}"),
                }
            }

            if !received_reply {
                eprintln!(
                    "{drone}: arm query completed without a reply (check bridge reply errors and query routing)"
                );
            }
        }
        Err(err) => eprintln!("{drone}: failed to send arm query: {err}"),
    }
}

fn rising_edge(current: bool, previous: bool) -> bool {
    current && !previous
}

async fn request_offboard(session: zenoh::Session, drone: &'static str) {
    println!("requesting offboard for {drone}...");

    match session
        .get(format!("{drone}/mavlink/control/offboard"))
        .timeout(Duration::from_secs(10))
        .await
    {
        Ok(replies) => {
            let mut received_reply = false;

            while let Ok(reply) = replies.recv_async().await {
                received_reply = true;
                match reply.result() {
                    Ok(sample) => match sample.payload().try_to_string() {
                        Ok(result) => println!("{drone}: offboard result: {result}"),
                        Err(err) => eprintln!("{drone}: offboard returned a non-text reply: {err}"),
                    },
                    Err(err) => match err.payload().try_to_string() {
                        Ok(result) => eprintln!("{drone}: offboard rejected: {result}"),
                        Err(_) => eprintln!("{drone}: offboard rejected: {err:?}"),
                    },
                }
            }

            if !received_reply {
                eprintln!(
                    "{drone}: offboard query completed without a reply (check bridge reply errors and query routing)"
                );
            }
        }
        Err(err) => eprintln!("{drone}: failed to send offboard query: {err}"),
    }
}

async fn request_launch(session: zenoh::Session, drone: &'static str, payload: Vec<u8>) {
    println!("launching {drone}...");

    match session
        .get(format!("{drone}/mavlink/control/move/launch"))
        .payload(payload)
        .await
    {
        Ok(replies) => {
            let mut received_reply = false;

            while let Ok(reply) = replies.recv_async().await {
                received_reply = true;
                match reply.result() {
                    Ok(sample) => match sample.payload().try_to_string() {
                        Ok(result) => println!("{drone}: launch result: {result}"),
                        Err(err) => eprintln!("{drone}: launch returned a non-text reply: {err}"),
                    },
                    Err(err) => eprintln!("{drone}: launch rejected: {err:?}"),
                }
            }

            if !received_reply {
                eprintln!(
                    "{drone}: launch query completed without a reply (check bridge reply errors and query routing)"
                );
            }
        }
        Err(err) => eprintln!("{drone}: failed to send launch query: {err}"),
    }
}

#[tokio::main]
async fn main() {
    let config_path = std::env::var("ZENOH_CONFIG").expect("ZENOH_CONFIG not set");
    let cfg = zenoh::Config::from_file(&config_path).expect("failed to load ZENOH_CONFIG file");

    let session = zenoh::open(cfg).await.expect("failed to initialize zenoh");
    let topic = std::env::var("GAMEPAD_TOPIC").unwrap_or_else(|_| "control/gamepad".into());
    let _gp_subscriber = session
        .declare_subscriber(&topic)
        .callback(|sample| {
            let state = match rmp_serde::from_slice(sample.payload().to_bytes().as_ref()) {
                Ok(state) => state,
                Err(err) => {
                    eprintln!("failed to deserialize gamepad state: {err}");
                    return;
                }
            };

            match GP_STATE.lock() {
                Ok(mut gamepad) => *gamepad = state,
                Err(err) => eprintln!("gamepad state mutex was poisoned: {err}"),
            }
        })
        .await
        .expect("failed to declare gamepad subscriber");

    let origin = Coords {
        lat_deg_e7: 353_284_430,
        lon_deg_e7: -1_207_525_480,
        alt_mm: 300_000,
    };
    let payload = rmp_serde::to_vec(&origin).expect("failed to encode origin coordinates");
    for &drone in DRONES {
        'set_origin: loop {
            match session
                .get(format!("{drone}/mavlink/origin/set"))
                .payload(payload.clone())
                .await
            {
                Ok(replies) => {
                    let mut received_reply = false;
                    while let Ok(reply) = replies.recv_async().await {
                        received_reply = true;
                        match reply.result() {
                            Ok(_) => {
                                println!("{drone}: origin set queued");
                                break 'set_origin;
                            }
                            Err(err) => eprintln!("{drone}: origin set rejected: {err:?}"),
                        }
                    }
                    if !received_reply {
                        eprintln!("{drone}: origin set query completed without a reply");
                    }
                }
                Err(err) => eprintln!("{drone}: failed to send origin set query: {err}"),
            }
        }
    }

    println!("Demo is live!");

    tokio::task::spawn(async move {
        let mut prev_gp = GamepadState::default();

        let mut prev_ctr = Vec3::ZERO;

        let mut ctr = Vec3::ZERO;
        let mut center_initialized = false;
        let mut last_center_publish: Option<Instant> = None;
        let mut orbit = Orbit::default();
        let mut rotating = false;
        let mut last_tick = Instant::now();
        let mut last_target_publish: Option<Instant> = None;

        const CENTER_SPEED_M_S: f32 = 5.0;
        const TARGET_PUBLISH_INTERVAL: Duration = Duration::from_millis(50);
        const CENTER_REPLAY_INTERVAL: Duration = Duration::from_secs(1);

        let mut armed = false;
        let mut launched = false;

        'mainloop: loop {
            tokio::time::sleep(Duration::from_millis(10)).await; // busy wait after drop

            let now = Instant::now();
            // Avoid a large control jump after a scheduler or network stall.
            let dt_s = now.duration_since(last_tick).as_secs_f32().min(0.1);
            last_tick = now;

            let gamepad_cpy = match GP_STATE.lock() {
                Err(_) => continue 'mainloop,
                Ok(gamepad) => *gamepad,
            };

            // The mutex guard was dropped above; this is an owned state snapshot.

            if rising_edge(gamepad_cpy.a, prev_gp.a) {
                for &drone in DRONES {
                    let session = session.clone();
                    tokio::spawn(async move { request_arm(session, drone).await });
                }
                armed = true;
            }

            if rising_edge(gamepad_cpy.b, prev_gp.b) && launched {
                if center_initialized {
                    rotating = true;
                    for &drone in DRONES {
                        let session = session.clone();
                        tokio::spawn(async move { request_offboard(session, drone).await });
                    }
                } else {
                    eprintln!("Launch first before requesting offboard");
                }
            }

            if rising_edge(gamepad_cpy.left_bumper, prev_gp.left_bumper) && armed {
                println!("Launching!");
                let comm = LaunchCommand {
                    height_m: 305.0,
                    yaw: 0.0,
                    lat: None,
                    lon: None,
                };
                ctr.z = comm.local_height_m(&origin);
                center_initialized = true;
                match rmp_serde::encode::to_vec(&comm) {
                    Ok(payload) => {
                        for &drone in DRONES {
                            let session = session.clone();
                            let payload = payload.clone();
                            tokio::spawn(async move {
                                request_launch(session, drone, payload).await;
                            });
                        }
                        launched = true;
                    }
                    Err(err) => eprintln!("failed to encode launch command: {err}"),
                };
            }

            ctr.x += CENTER_SPEED_M_S * dt_s * gamepad_cpy.left_stick_x;
            ctr.y += CENTER_SPEED_M_S * dt_s * gamepad_cpy.left_stick_y;
            orbit.update(gamepad_cpy.right_trigger, dt_s, rotating);

            if center_initialized
                && last_target_publish
                    .is_none_or(|last| now.duration_since(last) >= TARGET_PUBLISH_INTERVAL)
            {
                // Stable slots and one shared phase keep the orbit synchronized,
                // independent of discovery order or formation-node startup time.
                for (index, drone) in DRONES.iter().enumerate() {
                    let target = orbit.target(ctr, index, DRONES.len());
                    let payload =
                        rmp_serde::to_vec(&target).expect("failed to encode orbit target");
                    if let Err(err) = session
                        .put(format!("{drone}/formations/target"), payload)
                        .await
                    {
                        eprintln!("failed to publish orbit target for {drone}: {err}");
                    }
                }
                last_target_publish = Some(now);
            }

            let should_publish_center = center_initialized
                && (ctr != prev_ctr
                    || last_center_publish.is_none_or(|published_at| {
                        published_at.elapsed() >= CENTER_REPLAY_INTERVAL
                    }));
            if should_publish_center {
                match rmp_serde::encode::to_vec(&ctr) {
                    Ok(payload) => match session.put("formations/target", payload).await {
                        Ok(()) => last_center_publish = Some(Instant::now()),
                        Err(err) => eprintln!("failed to publish formation center: {err}"),
                    },
                    Err(err) => eprintln!("failed to encode formation center: {err}"),
                };
            }
            // println!("Ctr: {ctr}");

            prev_gp = gamepad_cpy;
            prev_ctr = ctr;
        }
    });

    tokio::signal::ctrl_c()
        .await
        .expect("Failed to listen for Ctrl-C");

    // println!("Hello, world!");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rising_edge_only_triggers_on_press() {
        assert!(!rising_edge(false, false));
        assert!(rising_edge(true, false));
        assert!(!rising_edge(true, true));
        assert!(!rising_edge(false, true));
    }

    #[test]
    fn formation_center_uses_height_above_origin() {
        let origin = Coords {
            lat_deg_e7: 353_284_430,
            lon_deg_e7: -1_207_525_480,
            alt_mm: 300_000,
        };
        let launch = LaunchCommand {
            height_m: 305.0,
            yaw: 0.0,
            lat: None,
            lon: None,
        };
        assert_eq!(launch.local_height_m(&origin), 5.0);
    }

    #[test]
    fn formation_center_wire_format_is_xyz_array() {
        let payload = rmp_serde::to_vec(&Vec3::new(1.0, 2.0, 5.0)).unwrap();
        let xyz: [f32; 3] = rmp_serde::from_slice(&payload).unwrap();
        assert_eq!(xyz, [1.0, 2.0, 5.0]);
        // Also consumed by telemetry/tests/test_formation_center.py.
        assert_eq!(
            payload,
            [
                0x93, 0xca, 0x3f, 0x80, 0, 0, 0xca, 0x40, 0, 0, 0, 0xca, 0x40, 0xa0, 0, 0
            ]
        );
    }
}
