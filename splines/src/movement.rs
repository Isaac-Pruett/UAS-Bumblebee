use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::error::Error;
use tokio::task::JoinHandle;
use zenoh::{
    Session,
    handlers::{RingChannel, RingChannelHandler},
    pubsub::{Publisher, Subscriber},
    sample::Sample,
};

use crate::{
    movement::{
        MotionFrame::{EAST_NORTH_UP, FORWARD_LEFT_UP, FORWARD_RIGHT_DOWN, NORTH_EAST_DOWN},
        MotionType::{CONTINUOUS, FINE},
    },
    splines::{self},
    state::State,
};

#[derive(Debug, Deserialize, Serialize)]
pub enum MotionType {
    FINE,
    CONTINUOUS {
        resolution: u32,
        energy_to_velocity: f32,
    },
}

#[derive(Debug)]
pub enum MotionFrame {
    NORTH_EAST_DOWN,
    EAST_NORTH_UP,
    FORWARD_LEFT_UP,
    FORWARD_RIGHT_DOWN,
}

const STATE_RING_SIZE: usize = 8; // tune to how much staleness you can tolerate

pub struct Movement {
    pub points: Vec<Vec3>,
    pub speed_limits: Vec<f32>,
    pub pipe_radius_m: f32,
    pub max_speed_m_s: f32,
    pub frame: MotionFrame,
}

impl Movement {
    pub async fn new<'a>(
        targets: Vec<Vec3>,
        pipe_radius_m: f32,
        max_speed_m_s: f32,
        motion_type: MotionType,
        frame: MotionFrame,
    ) -> Self {
        let (points, speed_limits) = match motion_type {
            FINE => {
                let n = targets.len();
                (targets, vec![max_speed_m_s; n])
            }
            CONTINUOUS {
                resolution,
                energy_to_velocity,
            } => {
                let spline = splines::Spline::new(targets, energy_to_velocity, resolution);
                let limits = spline.speed_limits(pipe_radius_m, max_speed_m_s);
                (spline.points, limits)
            }
        };

        Self {
            points,
            speed_limits,
            pipe_radius_m,
            max_speed_m_s,
            frame,
        }
    }

    pub async fn start(
        &mut self,
        session: &Session,
    ) -> Result<JoinHandle<()>, Box<dyn Error + Send + Sync>> {
        let points = self.points.clone();

        let pub_frame_str = Movement::grab_frame_str(&self.frame);

        let publisher = session
            .declare_publisher(format!("control/move/{}", pub_frame_str))
            .await?;

        let sub_frame_str = match self.frame {
            NORTH_EAST_DOWN => "pos/local/ned",    // local
            EAST_NORTH_UP => "pos/local/enu",      // body
            FORWARD_RIGHT_DOWN => "pos/local/ned", // local
            FORWARD_LEFT_UP => "pos/local/enu",    // body
        };

        let subscriber = session
            .declare_subscriber(format!("control/telem/{}", sub_frame_str))
            .with(RingChannel::new(STATE_RING_SIZE))
            .await?;

        Ok(tokio::spawn(async move {
            for point in points {
                let sample = subscriber.recv_async().await.ok();
                let _current_state: Option<(Vec3, Vec3, u32)> =
                    sample.and_then(|s| rmp_serde::from_slice(&s.payload().to_bytes()).ok());

                if let Some((x_m, v_m_s, dt_ms)) = _current_state {}
                {
                    // todo while not in arad do the thing
                    let bytes = rmp_serde::to_vec(&point);
                    if let Some(bytes) = bytes.ok() {
                        let _ = publisher.put(bytes).await;
                    }
                }
                // TODO: publish `point`, e.g.:
                // let bytes = rmp_serde::to_vec(&point).unwrap();
                // let _ = publisher.put(bytes).await;
            }
        }))
    }

    fn grab_frame_str(motion_type: &MotionFrame) -> &'static str {
        match motion_type {
            FORWARD_RIGHT_DOWN => "frd/raw",
            FORWARD_LEFT_UP => "flu/raw",
            EAST_NORTH_UP => "enu/raw",
            NORTH_EAST_DOWN => "ned/raw",
        }
    }
}
