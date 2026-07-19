use glam::Vec3;

pub struct BezierN {
    pub start: Vec3,
    pub controls: Vec<Vec3>,
    pub end: Vec3,
}

impl BezierN {
    pub fn new(start: Vec3, controls: Vec<Vec3>, end: Vec3) -> Self {
        BezierN {
            start,
            controls,
            end,
        }
    }
    pub fn interp(&self, resolution: u32) -> Vec<Vec3> {
        let ctrls = self.points();

        (0..=resolution)
            .map(|i| {
                let t = i as f32 / resolution as f32;
                bezier(&ctrls, t)
            })
            .collect()
    }

    pub fn points(&self) -> Vec<Vec3> {
        std::iter::once(self.start)
            .chain(self.controls.iter().copied())
            .chain(std::iter::once(self.end))
            .collect()
    }

    pub fn concat(&self, next_controls: Vec<Vec3>, next_end: Vec3) -> BezierN {
        BezierN::new(self.end, next_controls, next_end)
    }

    pub fn concat_to(&self, next: &BezierN) -> Vec<Vec3> {
        self.points()
            .into_iter()
            .chain(next.points().into_iter().skip(1))
            .collect()
    }

    pub fn smooth_next(&self, controls: Vec<Vec3>, end: Vec3) -> BezierN {
        let mut controls = controls;

        if let Some(last_control) = self.controls.last() {
            controls[0] = self.end + (self.end - *last_control);
        }

        BezierN::new(self.end, controls, end)
    }
}

fn bezier(points: &[Vec3], t: f32) -> Vec3 {
    if points.len() == 1 {
        return points[0];
    }

    let next: Vec<Vec3> = points.windows(2).map(|w| w[0].lerp(w[1], t)).collect();

    bezier(&next, t)
}

pub struct Spline {
    pub curves: Vec<BezierN>,
    passthrus: Vec<Vec3>,
    pub points: Vec<Vec3>,
}

impl Spline {
    pub fn new(waypoints: Vec<Vec3>, energy_to_velocity: f32, resolution: u32) -> Self {
        let curves =
            Spline::build_spline(&waypoints, BezierCostMode::TENSIONED(energy_to_velocity));

        let mut interpolated = Vec::new();
        for curve in &curves {
            let p = curve.interp(resolution);
            interpolated.extend(p);
        }

        Spline {
            curves,
            passthrus: waypoints,
            points: interpolated,
        }
    }

    /// Curvature (1/radius, in 1/meters) at each point in `self.points`,
    /// via discrete circumcircle approximation from three consecutive points.
    /// Endpoints get curvature 0 (straight-line assumption at the ends).
    pub fn curvatures(&self) -> Vec<f32> {
        let pts = &self.points;
        let n = pts.len();
        if n < 3 {
            return vec![0.0; n];
        }

        let mut out = vec![0.0; n];
        for i in 1..n - 1 {
            out[i] = Spline::curvature_at(pts[i - 1], pts[i], pts[i + 1]);
        }
        out
    }

    /// Menger curvature: 4 * triangle_area / (|ab| * |bc| * |ca|)
    #[inline]
    fn curvature_at(a: Vec3, b: Vec3, c: Vec3) -> f32 {
        let ab = (b - a).length();
        let bc = (c - b).length();
        let ca = (a - c).length();

        if ab < 1e-6 || bc < 1e-6 || ca < 1e-6 {
            return 0.0;
        }

        let area = 0.5 * (b - a).cross(c - a).length();
        (4.0 * area) / (ab * bc * ca)
    }

    /// Per-point speed cap derived from curvature and a lateral acceleration
    /// limit, itself derived from pipe_radius_m and max_speed_m_s: tighter
    /// pipes imply gentler lateral kicks are wanted through turns.
    pub fn speed_limits(&self, pipe_radius_m: f32, max_speed_m_s: f32) -> Vec<f32> {
        // Heuristic: allow up to ~1 "pipe radius worth" of lateral accel per
        // second^2 at max speed, scaled down for smaller pipes. Tune the
        // constant below once you have real hardware numbers.
        let lateral_accel_max = (pipe_radius_m * max_speed_m_s).max(0.1);

        self.curvatures()
            .into_iter()
            .map(|k| {
                if k < 1e-6 {
                    max_speed_m_s
                } else {
                    (lateral_accel_max / k).sqrt().min(max_speed_m_s)
                }
            })
            .collect()
    }

    // get the cost from tangent vector at point i in points
    #[inline]
    fn vec_from_cost_at_idx(points: &[Vec3], i: usize, cost_mode: BezierCostMode) -> Vec3 {
        assert!(points.len() >= 2);
        match cost_mode {
            BezierCostMode::UNIFORM => Spline::tangent_at_energy(points, i),
            BezierCostMode::VELOCITY => Spline::tangent_at_velo(points, i),
            BezierCostMode::TENSIONED(t) => Spline::tangent_at_tension(points, i, t),
        }
    }

    //energy
    #[inline]
    fn tangent_at_energy(points: &[Vec3], i: usize) -> Vec3 {
        if i == 0 {
            points[1] - points[0]
        } else if i == points.len() - 1 {
            points[i] - points[i - 1]
        } else {
            (points[i + 1] - points[i - 1]) * 0.5
        }
    }

    //velo
    #[inline]
    fn tangent_at_velo(points: &[Vec3], i: usize) -> Vec3 {
        if i == 0 {
            points[1] - points[0] // 2nd - 1st
        } else if i == points.len() - 1 {
            points[i] - points[i - 1] // last - 2ndlast
        } else {
            let prev = points[i] - points[i - 1];
            let next = points[i + 1] - points[i];

            let dir = (prev.normalize_or_zero() + next.normalize_or_zero()).normalize_or_zero();
            let speed = (prev.length() + next.length()) * 0.5;

            dir * speed
        }
    }

    #[inline]
    fn tangent_at_tension(points: &[Vec3], i: usize, tension: f32) -> Vec3 {
        let energy = Spline::tangent_at_energy(points, i);
        let velo = Spline::tangent_at_velo(points, i);

        energy.lerp(velo, tension.clamp(0.0, 1.0))
    }

    fn cubic_from_tangents(p0: Vec3, p1: Vec3, v0: Vec3, v1: Vec3) -> BezierN {
        BezierN::new(p0, vec![p0 + v0 / 3.0, p1 - v1 / 3.0], p1)
    }

    pub fn build_spline(points: &[Vec3], cost_mode: BezierCostMode) -> Vec<BezierN> {
        if points.len() < 2 {
            return Vec::new();
        }

        let mut curves = Vec::new();

        for i in 0..points.len() - 1 {
            let v0 = Spline::vec_from_cost_at_idx(points, i, cost_mode);
            let v1 = Spline::vec_from_cost_at_idx(points, i + 1, cost_mode);

            curves.push(Spline::cubic_from_tangents(
                points[i],
                points[i + 1],
                v0,
                v1,
            ));
        }

        curves
    }
}

#[derive(Debug, Clone, Copy)]
pub enum BezierCostMode {
    UNIFORM,        // good default, least energy
    VELOCITY,       // most flow
    TENSIONED(f32), // variable intensity
}
