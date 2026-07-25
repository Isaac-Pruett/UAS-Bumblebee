use ::glam::Vec3;
use serde::{Deserialize, Serialize};

use crate::{
    movement::{MotionFrame, Movement},
    splines::{BezierN, Spline},
};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MovementRequest {
    pub path: PathRequest,
    pub points_per_m: u32,
    pub pipe_radius_m: f32,
    pub frame: MotionFrame,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum PathRequest {
    Waypoints { points: Vec<Vec3>, smoothing: f32 },

    CubicBezier { segments: Vec<CubicBezierSegment> },
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CubicBezierSegment {
    pub start: Vec3,
    pub control_1: Vec3,
    pub control_2: Vec3,
    pub end: Vec3,
}

impl From<CubicBezierSegment> for BezierN {
    fn from(segment: CubicBezierSegment) -> Self {
        BezierN::new(
            segment.start,
            vec![segment.control_1, segment.control_2],
            segment.end,
        )
    }
}

impl Into<Movement> for MovementRequest {
    fn into(self) -> Movement {
        match self.path {
            PathRequest::Waypoints { points, smoothing } => Movement::new(
                points,
                self.pipe_radius_m,
                crate::movement::MotionType::CONTINUOUS {
                    points_per_m: self.points_per_m,
                    smoothness: smoothing,
                },
                self.frame,
            ),
            PathRequest::CubicBezier { segments } => {
                let curves = segments.into_iter().map(BezierN::from).collect::<Vec<_>>();

                let spline = Spline::from_curves_per_meter(curves, self.points_per_m);

                Movement {
                    points: spline.points,
                    pipe_radius_m: self.pipe_radius_m,
                    frame: self.frame,
                    task: None,
                }
            }
        }
    }
}
