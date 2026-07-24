use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::{error::Error, ops::Add};
use tokio::{
    sync::{mpsc, watch},
    task::JoinHandle,
};

use zenoh::{
    Session,
    handlers::{RingChannel, RingChannelHandler},
    pubsub::{Publisher, Subscriber},
    sample::Sample,
};

use crate::{local_frame_waypoint::LocalFrameWaypoint, splines, state::FullState};

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub enum MotionType {
    FINE,
    CONTINUOUS { resolution: u32, smoothness: f32 },
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy, PartialEq, Eq)]
pub enum MotionFrame {
    NorthEastDown,
    EastNorthUp,
    ForwardLeftUp,
    ForwardRightDown,
}

const STATE_RING_SIZE: usize = 8;

#[derive(Debug)]
pub struct Movement {
    pub points: Vec<Vec3>,
    pub pipe_radius_m: f32,
    pub frame: MotionFrame,
    pub task: Option<JoinHandle<Result<(), Box<dyn Error + Send + Sync>>>>,
}

impl Movement {
    pub fn new(
        targets: Vec<Vec3>,
        pipe_radius_m: f32,
        motion_type: MotionType,
        frame: MotionFrame,
    ) -> Self {
        let points = match motion_type {
            MotionType::FINE => targets,
            MotionType::CONTINUOUS {
                resolution,
                smoothness: energy_to_velocity,
            } => {
                let spline = splines::Spline::new(targets, energy_to_velocity, resolution);
                spline.points
            }
        };

        Self {
            points,
            pipe_radius_m,
            frame,

            task: None,
        }
    }

    pub async fn execute(
        &mut self,
        mut state_rx: watch::Receiver<Option<FullState>>,
        waypoint_tx: mpsc::Sender<LocalFrameWaypoint>,
    ) -> Result<(), Box<dyn Error + Send + Sync>> {
        for (index, point) in self.points.iter().enumerate() {
            'control: loop {
                let current_state = state_rx.borrow_and_update().clone();

                if let Some(state) = &current_state {
                    let reached_by_proximity = (*point - state.pos).length() <= self.pipe_radius_m;

                    if reached_by_proximity {
                        break 'control;
                    }
                }

                let command = LocalFrameWaypoint {
                    x: point.x,
                    y: point.y,
                    z: point.z,
                    vx: f32::NAN,
                    vy: f32::NAN,
                    vz: f32::NAN,
                    afx: f32::NAN,
                    afy: f32::NAN,
                    afz: f32::NAN,
                    force_en: false,
                    yaw: f32::NAN,
                    yaw_rate: f32::NAN,
                };

                match waypoint_tx.try_send(command) {
                    Ok(()) | Err(mpsc::error::TrySendError::Full(_)) => {}
                    Err(mpsc::error::TrySendError::Closed(_)) => return Ok(()),
                }

                if state_rx.changed().await.is_err() {
                    return Ok(());
                }
            }
        }

        Ok(())
    }

    #[allow(dead_code)]
    async fn get_state(
        subscriber: &Subscriber<RingChannelHandler<Sample>>,
    ) -> Result<FullState, Box<dyn Error + Send + Sync>> {
        subscriber.recv_async().await.and_then(|sample| {
            let bytes = sample.payload();
            Ok(rmp_serde::from_slice::<FullState>(&bytes.to_bytes())?)
        })
    }

    #[allow(dead_code)]
    async fn get_pub_sub(
        &self,
        session: &Session,
    ) -> Result<(Publisher, Subscriber<RingChannelHandler<Sample>>), Box<dyn Error + Send + Sync>>
    {
        let frame = self.frame;
        let pub_frame_str = match frame {
            MotionFrame::ForwardRightDown => "frd/raw",
            MotionFrame::ForwardLeftUp => "flu/raw",
            MotionFrame::EastNorthUp => "enu/raw",
            MotionFrame::NorthEastDown => "ned/raw",
        };

        let publisher = session
            .declare_publisher(format!("control/move/{pub_frame_str}"))
            .await?;

        let sub_frame_str = match frame {
            MotionFrame::NorthEastDown | MotionFrame::ForwardRightDown => "ned",
            MotionFrame::EastNorthUp | MotionFrame::ForwardLeftUp => "enu",
        };

        let subscriber = session
            .declare_subscriber(format!("control/telem/state/local/{sub_frame_str}"))
            .with(RingChannel::new(STATE_RING_SIZE))
            .await?;

        Ok((publisher, subscriber))
    }

    pub fn run(
        &mut self,
        runtime: &tokio::runtime::Handle,
        state_rx: watch::Receiver<Option<FullState>>,
        waypoint_tx: mpsc::Sender<LocalFrameWaypoint>,
    ) {
        self.cancel();

        let mut movement = self.clone();

        self.task =
            Some(runtime.spawn(async move { movement.execute(state_rx, waypoint_tx).await }));
    }

    pub fn cancel(&mut self) {
        if let Some(task) = self.task.take() {
            task.abort();
        }
    }
}

impl Clone for Movement {
    fn clone(&self) -> Self {
        Self {
            points: self.points.clone(),
            pipe_radius_m: self.pipe_radius_m,
            frame: self.frame,
            task: None,
        }
    }
}
