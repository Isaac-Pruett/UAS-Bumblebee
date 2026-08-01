use crate::{
    motion_task::{MotionProfile, MotionType, Movement},
    splines::BezierN,
};
use glam::Vec3A;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};
use zenoh::{Session, pubsub::Publisher};

pub struct SplineGeneratorProxy {
    current_movement: Option<Movement>,
    publ: Publisher<'static>,
}

#[derive(Deserialize, Serialize)]
struct MotionPacket {
    waypoints: Vec<Vec3A>,
    profile: MotionProfile,
}

impl SplineGeneratorProxy {
    pub async fn new(session: Session, frame: &str) -> Arc<Mutex<Self>> {
        let publ = session
            .declare_publisher(format!("mavlink/control/move/{frame}/raw"))
            .await
            .expect("Failed to create publisher for spline generator"); // drop the borrow on `session`, get Publisher<'static>

        let state = Arc::new(Mutex::new(SplineGeneratorProxy {
            current_movement: None,
            publ,
        }));

        fn handle_packet(state: &Arc<Mutex<SplineGeneratorProxy>>, bytes: &[u8]) {
            match rmp_serde::from_slice::<MotionPacket>(bytes) {
                Ok(m) => {
                    let movement = Movement::new(m.waypoints, m.profile);
                    state.lock().unwrap().current_movement = Some(movement);
                }
                Err(_) => {}
            }
        }

        let state_arc = state.clone();
        session
            .declare_subscriber(format!("motion/{frame}"))
            .callback(move |sample| {
                handle_packet(&state_arc, &sample.payload().to_bytes());
                let mut proxy = state_arc.lock().unwrap();
                let mut m = proxy.current_movement.as_mut();
                if let Some(m) = m {}
            })
            .background()
            .await
            .expect("Failed to create subscriber for motion");

        state
    }
}
