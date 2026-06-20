use glam::Vec3;
use serde::{Deserialize, Serialize};
use zenoh::pubsub::Publisher;

use crate::{
    motion_task::MotionType::{CONTINUOUS, FINE},
    splines::{self, BezierCostMode},
};

#[derive(Debug, Deserialize, Serialize)]
pub enum MotionType {
    FINE,
    CONTINUOUS { resolution: u32, tension: f32 },
}

pub struct Movement {
    task: Option<tokio::task::JoinHandle<()>>,
    points: Vec<Vec3>,
    pipe_radius_m: f32,
    max_speed_m_s: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct MotionProfile {
    pub motion_type: MotionType,
    pub pipe_radius_m: f32,
    pub max_speed_m_s: f32,
}

impl Movement {
    pub fn new(targets: Vec<Vec3>, profile: MotionProfile) -> Self {
        let points = match &profile.motion_type {
            FINE => targets,
            CONTINUOUS {
                resolution,
                tension,
            } => {
                let bezs =
                    splines::build_spline(targets.as_slice(), BezierCostMode::TENSIONED(*tension));
                let mut v = Vec::<Vec3>::new();
                for c in &bezs {
                    v.extend(c.interp(*resolution));
                }
                v
            }
        };

        Self {
            task: None,
            points,
            pipe_radius_m: profile.pipe_radius_m,
            max_speed_m_s: profile.max_speed_m_s,
        }
    }

    pub fn append(&mut self, other: Self) -> Self {
        self.points.extend(other.points);
        Self {
            task: None,
            points: self.points.clone(),
            pipe_radius_m: self.pipe_radius_m.max(other.pipe_radius_m),
            max_speed_m_s: self.max_speed_m_s.max(other.max_speed_m_s),
        }
    }

    pub fn broadcast(self, p: Publisher) {}
}
