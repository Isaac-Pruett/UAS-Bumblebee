use glam::Vec3;
use serde::{Deserialize, Serialize};
use std::error::Error;
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
    CONTINUOUS {
        resolution: u32,
        energy_to_velocity: f32,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionFrame {
    NORTH_EAST_DOWN,
    EAST_NORTH_UP,
    FORWARD_LEFT_UP,
    FORWARD_RIGHT_DOWN,
}

const STATE_RING_SIZE: usize = 8;

#[derive(Debug)]
pub struct Movement {
    pub points: Vec<Vec3>,
    pub speed_limits: Vec<f32>,
    pub pipe_radius_m: f32,
    pub max_speed_m_s: f32,
    pub frame: MotionFrame,
    pub cumulative_lengths: Vec<f32>,
    pub total_length: f32,
    pub task: Option<JoinHandle<Result<(), Box<dyn Error + Send + Sync>>>>,
}

impl Movement {
    pub fn new(
        targets: Vec<Vec3>,
        pipe_radius_m: f32,
        max_speed_m_s: f32,
        longitudinal_accel_m_s2: f32,
        motion_type: MotionType,
        frame: MotionFrame,
    ) -> Self {
        let (points, speed_limits) = match motion_type {
            MotionType::FINE => {
                let n = targets.len();
                (targets, vec![max_speed_m_s; n])
            }
            MotionType::CONTINUOUS {
                resolution,
                energy_to_velocity,
            } => {
                let spline = splines::Spline::new(targets, energy_to_velocity, resolution);
                let limits =
                    spline.speed_limits(pipe_radius_m, max_speed_m_s, longitudinal_accel_m_s2);
                (spline.points, limits)
            }
        };

        let (cumulative, total) = Self::build_arc_lengths(&points);

        Self {
            points,
            speed_limits,
            pipe_radius_m,
            max_speed_m_s,
            frame,
            cumulative_lengths: cumulative,
            total_length: total,
            task: None,
        }
    }

    fn build_arc_lengths(points: &[Vec3]) -> (Vec<f32>, f32) {
        let mut cumulative = Vec::with_capacity(points.len());

        if points.is_empty() {
            return (cumulative, 0.0);
        }

        cumulative.push(0.0);

        for pair in points.windows(2) {
            let next = cumulative.last().copied().unwrap_or(0.0) + pair[0].distance(pair[1]);

            cumulative.push(next);
        }

        let total = cumulative.last().copied().unwrap_or(0.0);
        (cumulative, total)
    }

    /// Build the command associated with a planned point. The velocity follows
    /// the local path tangent and is capped by the curvature-derived speed limit.
    pub fn waypoint_at(&self, index: usize) -> Option<LocalFrameWaypoint> {
        let point = *self.points.get(index)?;
        let speed = self
            .speed_limits
            .get(index)
            .copied()
            .unwrap_or(self.max_speed_m_s)
            .clamp(0.0, self.max_speed_m_s);

        let tangent = if self.points.len() < 2 {
            Vec3::ZERO
        } else if index + 1 < self.points.len() {
            self.points[index + 1] - point
        } else {
            point - self.points[index.saturating_sub(1)]
        }
        .normalize_or_zero();

        let velocity = tangent * speed;
        let yaw = tangent.y.atan2(tangent.x);

        Some(LocalFrameWaypoint {
            x: point.x,
            y: point.y,
            z: point.z,
            vx: velocity.x,
            vy: velocity.y,
            vz: velocity.z,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            force_en: false,
            yaw,
            yaw_rate: 0.0,
        })
    }

    pub async fn execute(
        &mut self,
        mut state_rx: watch::Receiver<Option<FullState>>,
        waypoint_tx: mpsc::Sender<LocalFrameWaypoint>,
    ) -> Result<(), Box<dyn Error + Send + Sync>> {
        for (index, point) in self.points.iter().enumerate() {
            'control: loop {
                // observe
                state_rx.changed().await?;
                let state = state_rx.borrow_and_update().clone();

                let Some(state) = state else {
                    continue 'control;
                };

                if (*point - state.pos).length() <= self.pipe_radius_m {
                    break 'control;
                }

                // command

                if let Some(command) = self.waypoint_at(index) {
                    waypoint_tx.send(command).await?;
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

    /// Collect a publisher and subscriber for the selected motion frame.
    #[allow(dead_code)]
    async fn get_pub_sub(
        frame: MotionFrame,
        session: &Session,
    ) -> Result<(Publisher, Subscriber<RingChannelHandler<Sample>>), Box<dyn Error + Send + Sync>>
    {
        let pub_frame_str = match frame {
            MotionFrame::FORWARD_RIGHT_DOWN => "frd/raw",
            MotionFrame::FORWARD_LEFT_UP => "flu/raw",
            MotionFrame::EAST_NORTH_UP => "enu/raw",
            MotionFrame::NORTH_EAST_DOWN => "ned/raw",
        };

        let publisher = session
            .declare_publisher(format!("control/move/{pub_frame_str}"))
            .await?;

        let sub_frame_str = match frame {
            MotionFrame::NORTH_EAST_DOWN | MotionFrame::FORWARD_RIGHT_DOWN => {
                "telem/state/local/ned"
            }
            MotionFrame::EAST_NORTH_UP | MotionFrame::FORWARD_LEFT_UP => "telem/state/local/enu",
        };

        let subscriber = session
            .declare_subscriber(format!("control/telem/{sub_frame_str}"))
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
            speed_limits: self.speed_limits.clone(),
            pipe_radius_m: self.pipe_radius_m,
            max_speed_m_s: self.max_speed_m_s,
            frame: self.frame,
            cumulative_lengths: self.cumulative_lengths.clone(),
            total_length: self.total_length,
            task: None,
        }
    }
}
