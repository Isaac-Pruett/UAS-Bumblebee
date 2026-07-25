use glam::Vec3;

#[derive(Debug, Clone)]
pub struct CubicSegment {
    pub start: Vec3,
    pub control1: Vec3,
    pub control2: Vec3,
    pub end: Vec3,
}

impl CubicSegment {
    pub fn new(start: Vec3, control1: Vec3, control2: Vec3, end: Vec3) -> Self {
        Self {
            start,
            control1,
            control2,
            end,
        }
    }

    pub fn estimated_length(&self) -> f32 {
        self.start.distance(self.control1)
            + self.control1.distance(self.control2)
            + self.control2.distance(self.end)
    }

    pub fn into_bezier(self) -> BezierN {
        BezierN::new(self.start, vec![self.control1, self.control2], self.end)
    }
}

#[derive(Debug, Clone)]
pub struct BezierN {
    pub start: Vec3,
    pub controls: Vec<Vec3>,
    pub end: Vec3,
}

impl BezierN {
    pub fn new(start: Vec3, controls: Vec<Vec3>, end: Vec3) -> Self {
        Self {
            start,
            controls,
            end,
        }
    }

    pub fn interp(&self, resolution: u32) -> Vec<Vec3> {
        if resolution == 0 {
            return vec![self.start];
        }

        let points = self.points();

        (0..=resolution)
            .map(|i| {
                let t = i as f32 / resolution as f32;
                bezier(&points, t)
            })
            .collect()
    }

    pub fn points(&self) -> Vec<Vec3> {
        std::iter::once(self.start)
            .chain(self.controls.iter().copied())
            .chain(std::iter::once(self.end))
            .collect()
    }

    pub fn estimated_length(&self) -> f32 {
        self.points()
            .windows(2)
            .map(|pair| pair[0].distance(pair[1]))
            .sum()
    }

    pub fn concat(&self, next_controls: Vec<Vec3>, next_end: Vec3) -> Self {
        Self::new(self.end, next_controls, next_end)
    }

    pub fn concat_to(&self, next: &BezierN) -> Vec<Vec3> {
        self.points()
            .into_iter()
            .chain(next.points().into_iter().skip(1))
            .collect()
    }

    pub fn smooth_next(&self, mut controls: Vec<Vec3>, end: Vec3) -> Self {
        if let (Some(previous), Some(first)) = (self.controls.last(), controls.first_mut()) {
            *first = self.end + (self.end - *previous);
        }

        Self::new(self.end, controls, end)
    }
}

fn bezier(points: &[Vec3], t: f32) -> Vec3 {
    match points {
        [] => Vec3::ZERO,
        [point] => *point,
        _ => {
            let next: Vec<Vec3> = points
                .windows(2)
                .map(|pair| pair[0].lerp(pair[1], t))
                .collect();

            bezier(&next, t)
        }
    }
}

#[derive(Debug, Clone)]
pub struct Spline {
    pub curves: Vec<BezierN>,
    pub points: Vec<Vec3>,
}

impl Spline {
    pub fn new(waypoints: Vec<Vec3>, energy_to_velocity: f32, resolution: u32) -> Self {
        let curves = Self::build_bcurve(&waypoints, BezierCostMode::Tensioned(energy_to_velocity));

        Self::from_curves_per_meter(curves, resolution)
    }

    pub fn from_curves(curves: Vec<BezierN>, resolution: u32) -> Self {
        let points = Self::interpolate_curves(&curves, resolution);

        Self { curves, points }
    }

    pub fn from_curves_per_meter(curves: Vec<BezierN>, points_per_m: u32) -> Self {
        let points = Self::interpolate_curves_per_meter(&curves, points_per_m);

        Self { curves, points }
    }

    pub fn from_cubic_segments(segments: Vec<CubicSegment>, resolution: u32) -> Self {
        let curves = segments
            .into_iter()
            .map(CubicSegment::into_bezier)
            .collect();

        Self::from_curves(curves, resolution)
    }

    pub fn from_cubic_segments_per_meter(segments: Vec<CubicSegment>, points_per_m: u32) -> Self {
        let curves = segments
            .into_iter()
            .map(CubicSegment::into_bezier)
            .collect();

        Self::from_curves_per_meter(curves, points_per_m)
    }

    fn interpolate_curves(curves: &[BezierN], resolution: u32) -> Vec<Vec3> {
        let mut points = Vec::new();

        for (curve_index, curve) in curves.iter().enumerate() {
            let curve_points = curve.interp(resolution);

            if curve_index == 0 {
                points.extend(curve_points);
            } else {
                points.extend(curve_points.into_iter().skip(1));
            }
        }

        points
    }

    fn interpolate_curves_per_meter(curves: &[BezierN], points_per_m: u32) -> Vec<Vec3> {
        let mut points = Vec::new();

        for (curve_index, curve) in curves.iter().enumerate() {
            let resolution = if points_per_m == 0 {
                1
            } else {
                (curve.estimated_length() * points_per_m as f32)
                    .ceil()
                    .max(1.0) as u32
            };

            let curve_points = curve.interp(resolution);

            if curve_index == 0 {
                points.extend(curve_points);
            } else {
                points.extend(curve_points.into_iter().skip(1));
            }
        }

        points
    }

    #[inline]
    fn vec_from_cost_at_idx(points: &[Vec3], index: usize, cost_mode: BezierCostMode) -> Vec3 {
        assert!(points.len() >= 2);

        match cost_mode {
            BezierCostMode::Uniform => Self::tangent_at_energy(points, index),
            BezierCostMode::Velocity => Self::tangent_at_velocity(points, index),
            BezierCostMode::Tensioned(tension) => Self::tangent_at_tension(points, index, tension),
        }
    }

    #[inline]
    fn tangent_at_energy(points: &[Vec3], index: usize) -> Vec3 {
        if index == 0 {
            points[1] - points[0]
        } else if index == points.len() - 1 {
            points[index] - points[index - 1]
        } else {
            (points[index + 1] - points[index - 1]) * 0.5
        }
    }

    #[inline]
    fn tangent_at_velocity(points: &[Vec3], index: usize) -> Vec3 {
        if index == 0 {
            points[1] - points[0]
        } else if index == points.len() - 1 {
            points[index] - points[index - 1]
        } else {
            let previous = points[index] - points[index - 1];
            let next = points[index + 1] - points[index];

            let direction =
                (previous.normalize_or_zero() + next.normalize_or_zero()).normalize_or_zero();

            let magnitude = (previous.length() + next.length()) * 0.5;

            direction * magnitude
        }
    }

    #[inline]
    fn tangent_at_tension(points: &[Vec3], index: usize, tension: f32) -> Vec3 {
        let energy = Self::tangent_at_energy(points, index);
        let velocity = Self::tangent_at_velocity(points, index);

        energy.lerp(velocity, tension.clamp(0.0, 1.0))
    }

    fn cubic_from_tangents(
        start: Vec3,
        end: Vec3,
        start_tangent: Vec3,
        end_tangent: Vec3,
    ) -> BezierN {
        BezierN::new(
            start,
            vec![start + start_tangent / 3.0, end - end_tangent / 3.0],
            end,
        )
    }

    pub fn build_bcurve(points: &[Vec3], cost_mode: BezierCostMode) -> Vec<BezierN> {
        if points.len() < 2 {
            return Vec::new();
        }

        (0..points.len() - 1)
            .map(|index| {
                let start_tangent = Self::vec_from_cost_at_idx(points, index, cost_mode);

                let end_tangent = Self::vec_from_cost_at_idx(points, index + 1, cost_mode);

                Self::cubic_from_tangents(
                    points[index],
                    points[index + 1],
                    start_tangent,
                    end_tangent,
                )
            })
            .collect()
    }
}

#[derive(Debug, Clone, Copy)]
pub enum BezierCostMode {
    Uniform,
    Velocity,
    Tensioned(f32),
}
