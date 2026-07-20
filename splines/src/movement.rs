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
    CONTINUOUS { resolution: u32, curvature: f32 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionFrame {
    NORTH_EAST_DOWN,
    EAST_NORTH_UP,
    FORWARD_LEFT_UP,
    FORWARD_RIGHT_DOWN,
}

const STATE_RING_SIZE: usize = 8;

/// Tuning for the CBF safety filter applied to outgoing velocity commands.
/// Higher gain = intervenes later / more sharply near the boundary.
/// Lower gain = intervenes earlier / more gently.
#[derive(Debug, Clone, Copy)]
pub struct BarrierMargins {
    /// Meters of lateral margin remaining before the pipe wall (can go negative if outside).
    pub pipe_margin_m: f32,
    /// Meters of along-track lead the carrot has on the vehicle (can go negative if overtaken).
    pub carrot_lead_m: f32,
}

impl Default for BarrierMargins {
    fn default() -> Self {
        Self {
            pipe_margin_m: 2.0,
            carrot_lead_m: 2.0,
        }
    }
}

#[derive(Debug)]
pub struct Movement {
    pub points: Vec<Vec3>,
    pub speed_limits: Vec<f32>,
    pub pipe_radius_m: f32,
    pub max_speed_m_s: f32,
    pub frame: MotionFrame,
    pub cumulative_lengths: Vec<f32>,
    pub total_length: f32,
    pub barrier_margins: BarrierMargins,
    pub task: Option<JoinHandle<Result<(), Box<dyn Error + Send + Sync>>>>,
}

impl Movement {
    pub fn new(
        targets: Vec<Vec3>,
        pipe_radius_m: f32,
        max_speed_m_s: f32,
        longitudinal_accel_m_s2: f32,
        barrier_margins: BarrierMargins,
        motion_type: MotionType,
        frame: MotionFrame,
    ) -> Self {
        let (points, speed_limits) = match motion_type {
            MotionType::FINE => {
                let resolution = 1;
                let energy_to_velocity = 0.0;
                let spline = splines::Spline::new(targets, energy_to_velocity, resolution);
                let limits =
                    spline.speed_limits(pipe_radius_m, max_speed_m_s, longitudinal_accel_m_s2);
                (spline.points, limits)
            }
            MotionType::CONTINUOUS {
                resolution,
                curvature: energy_to_velocity,
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
            barrier_margins,
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

    /// Projects a point onto the piecewise-linear path and returns its
    /// arc-length position. Finds the closest segment, projects onto it,
    /// and adds the segment's cumulative length up to that projection.
    fn project_arc_length(&self, pos: Vec3) -> f32 {
        if self.points.len() < 2 {
            return 0.0;
        }

        let mut best_arc_length = 0.0_f32;
        let mut best_dist_sq = f32::MAX;

        for (i, pair) in self.points.windows(2).enumerate() {
            let a = pair[0];
            let b = pair[1];
            let seg = b - a;
            let seg_len_sq = seg.length_squared();

            let t = if seg_len_sq < f32::EPSILON {
                0.0
            } else {
                ((pos - a).dot(seg) / seg_len_sq).clamp(0.0, 1.0)
            };

            let closest = a + seg * t;
            let dist_sq = (pos - closest).length_squared();

            if dist_sq < best_dist_sq {
                best_dist_sq = dist_sq;
                let seg_start_arc = self.cumulative_lengths.get(i).copied().unwrap_or(0.0);
                best_arc_length = seg_start_arc + seg.length() * t;
            }
        }

        best_arc_length
    }

    /// Returns the segment (a, b) nearest to `pos`, used as the local
    /// pipe-wall reference frame for the CBF pipe constraint.
    fn nearest_segment(&self, pos: Vec3) -> Option<(Vec3, Vec3)> {
        self.points
            .windows(2)
            .map(|seg| (seg[0], seg[1]))
            .min_by(|(a1, b1), (a2, b2)| {
                let d1 = Self::point_segment_dist_sq(pos, *a1, *b1);
                let d2 = Self::point_segment_dist_sq(pos, *a2, *b2);
                d1.partial_cmp(&d2).unwrap()
            })
    }

    fn point_segment_dist_sq(p: Vec3, a: Vec3, b: Vec3) -> f32 {
        let ab = b - a;
        let len_sq = ab.length_squared();
        if len_sq < f32::EPSILON {
            return (p - a).length_squared();
        }
        let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
        (p - (a + ab * t)).length_squared()
    }

    fn filter_velocity(&self, nominal_velocity: Vec3, carrot: Vec3, vehicle_pos: Vec3) -> Vec3 {
        let Some((seg_a, seg_b)) = self.nearest_segment(vehicle_pos) else {
            return nominal_velocity;
        };

        let seg_dir = (seg_b - seg_a).normalize_or_zero();
        if seg_dir == Vec3::ZERO {
            return nominal_velocity;
        }
        let normal = Vec3::new(-seg_dir.y, seg_dir.x, 0.0);

        let along = seg_a + seg_dir * (vehicle_pos - seg_a).dot(seg_dir);
        let lateral_vec = vehicle_pos - along;
        let lateral_signed = lateral_vec.dot(normal);

        // --- Centering term: pulls the vehicle back toward the centerline. ---
        // Strength scales with how far off-center the vehicle is, relative to
        // the pipe radius, so it gets stronger the closer you are to the wall
        // rather than being a fixed gain that's either too weak near center
        // or overpowering everywhere.
        let center_gain = self.max_speed_m_s * 3.0; // tune: "how strong" the lateral pull is
        let normalized_offset =
            (lateral_signed / self.pipe_radius_m.max(f32::EPSILON)).clamp(-1.0, 1.0);
        let centering_velocity = -normal * normalized_offset * center_gain;

        let mut velocity = nominal_velocity + centering_velocity;

        // --- Barrier 1: pipe (lateral distance from centerline), hard backstop ---
        let h_pipe = self.pipe_radius_m * self.pipe_radius_m - lateral_signed * lateral_signed;
        let grad_h_pipe = normal * (-2.0 * lateral_signed);

        let lhs_pipe = grad_h_pipe.dot(velocity) + self.barrier_margins.pipe_margin_m * h_pipe;
        if lhs_pipe < 0.0 {
            let grad_norm_sq = grad_h_pipe.length_squared().max(f32::EPSILON);
            let required = -self.barrier_margins.pipe_margin_m * h_pipe;
            let deficit = required - grad_h_pipe.dot(velocity);
            velocity += grad_h_pipe * (deficit / grad_norm_sq);
        }

        // --- Barrier 2: carrot-lead (never overtake the carrot along-track) ---
        let carrot_s = (carrot - seg_a).dot(seg_dir);
        let vehicle_s = (vehicle_pos - seg_a).dot(seg_dir);
        let h_carrot = carrot_s - vehicle_s;
        let grad_h_carrot = -seg_dir;

        let lhs_carrot =
            grad_h_carrot.dot(velocity) + self.barrier_margins.carrot_lead_m * h_carrot;
        if lhs_carrot < 0.0 {
            let grad_norm_sq = grad_h_carrot.length_squared().max(f32::EPSILON);
            let required = -self.barrier_margins.carrot_lead_m * h_carrot;
            let deficit = required - grad_h_carrot.dot(velocity);
            velocity += grad_h_carrot * (deficit / grad_norm_sq);
        }

        // Cap to the commanded speed's original magnitude scale.
        let cap = nominal_velocity
            .length()
            .max(velocity.length())
            .min(self.max_speed_m_s);
        if velocity.length() > cap {
            velocity = velocity.normalize_or_zero() * cap;
        }

        velocity
    }

    pub fn barrier_margins(&self, vehicle_pos: Vec3, carrot: Vec3) -> Option<BarrierMargins> {
        let (seg_a, seg_b) = self.nearest_segment(vehicle_pos)?;
        let seg_dir = (seg_b - seg_a).normalize_or_zero();
        if seg_dir == Vec3::ZERO {
            return None;
        }
        let normal = Vec3::new(-seg_dir.y, seg_dir.x, 0.0);

        let along = seg_a + seg_dir * (vehicle_pos - seg_a).dot(seg_dir);
        let lateral_signed = (vehicle_pos - along).dot(normal);
        let pipe_margin_m = self.pipe_radius_m - lateral_signed.abs();

        let carrot_s = (carrot - seg_a).dot(seg_dir);
        let vehicle_s = (vehicle_pos - seg_a).dot(seg_dir);
        let carrot_lead_m = carrot_s - vehicle_s;

        Some(BarrierMargins {
            pipe_margin_m,
            carrot_lead_m,
        })
    }

    /// Build the command associated with a planned point. The velocity follows
    /// the local path tangent and is capped by the curvature-derived speed limit,
    /// then CBF-filtered against the vehicle's current telemetry state so the
    /// published command can't ask the vehicle to leave the pipe or overtake
    /// the carrot.
    pub fn waypoint_at(
        &self,
        index: usize,
        vehicle_state: Option<&FullState>,
    ) -> Option<LocalFrameWaypoint> {
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

        let nominal_velocity = tangent * speed;

        let velocity = match vehicle_state {
            Some(state) => self.filter_velocity(nominal_velocity, point, state.pos),
            None => nominal_velocity,
        };

        let yaw = if velocity.length_squared() > f32::EPSILON {
            velocity.y.atan2(velocity.x)
        } else {
            tangent.y.atan2(tangent.x)
        };

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
            let target_arc_length = self
                .cumulative_lengths
                .get(index)
                .copied()
                .unwrap_or(self.total_length);

            'control: loop {
                let current_state = state_rx.borrow_and_update().clone();

                if let Some(state) = &current_state {
                    let vehicle_arc_length = self.project_arc_length(state.pos);

                    let reached_by_progress = vehicle_arc_length >= target_arc_length;
                    let reached_by_proximity = (*point - state.pos).length() <= self.pipe_radius_m;

                    if reached_by_progress || reached_by_proximity {
                        break 'control;
                    }
                }

                let Some(command) = self.waypoint_at(index, current_state.as_ref()) else {
                    continue;
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
            barrier_margins: self.barrier_margins,
            task: None,
        }
    }
}
