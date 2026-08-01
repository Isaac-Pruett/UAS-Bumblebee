use glam::Vec3A;

pub struct BezierN {
    pub start: Vec3A,
    pub controls: Vec<Vec3A>,
    pub end: Vec3A,
}

impl BezierN {
    pub fn new(start: Vec3A, controls: Vec<Vec3A>, end: Vec3A) -> Self {
        BezierN {
            start,
            controls,
            end,
        }
    }
    pub fn interp(&self, resolution: u32) -> Vec<Vec3A> {
        let ctrls = self.points();

        (0..=resolution)
            .map(|i| {
                let t = i as f32 / resolution as f32;
                bezier(&ctrls, t)
            })
            .collect()
    }

    pub fn points(&self) -> Vec<Vec3A> {
        std::iter::once(self.start)
            .chain(self.controls.iter().copied())
            .chain(std::iter::once(self.end))
            .collect()
    }

    pub fn concat(&self, next_controls: Vec<Vec3A>, next_end: Vec3A) -> BezierN {
        BezierN::new(self.end, next_controls, next_end)
    }

    pub fn concat_to(&self, next: &BezierN) -> Vec<Vec3A> {
        self.points()
            .into_iter()
            .chain(next.points().into_iter().skip(1))
            .collect()
    }

    pub fn smooth_next(&self, controls: Vec<Vec3A>, end: Vec3A) -> BezierN {
        let mut controls = controls;

        if let Some(last_control) = self.controls.last() {
            controls[0] = self.end + (self.end - *last_control);
        }

        BezierN::new(self.end, controls, end)
    }
}

fn bezier(points: &[Vec3A], t: f32) -> Vec3A {
    if points.len() == 1 {
        return points[0];
    }

    let next: Vec<Vec3A> = points.windows(2).map(|w| w[0].lerp(w[1], t)).collect();

    bezier(&next, t)
}

pub struct Spline {
    curves: Vec<BezierN>,
    passthrus: Vec<Vec3A>,
    pub points: Vec<Vec3A>,
}

impl Spline {
    pub fn new(waypoints: Vec<Vec3A>, energy_to_speed: f32, resolution: u32) -> Self {
        let curves = Spline::build_spline(&waypoints, BezierCostMode::TENSIONED(energy_to_speed));

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

    // get the cost from tangent vector at point i in points
    #[inline]
    fn vec_from_cost_at_idx(points: &[Vec3A], i: usize, cost_mode: BezierCostMode) -> Vec3A {
        assert!(points.len() >= 2);
        match cost_mode {
            BezierCostMode::UNIFORM => Spline::tangent_at_energy(points, i),
            BezierCostMode::VELOCITY => Spline::tangent_at_velo(points, i),

            BezierCostMode::TENSIONED(t) => Spline::tangent_at_tension(points, i, t),
        }
    }

    //energy
    #[inline]
    fn tangent_at_energy(points: &[Vec3A], i: usize) -> Vec3A {
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
    fn tangent_at_velo(points: &[Vec3A], i: usize) -> Vec3A {
        if i == 0 {
            points[1] - points[0]
        } else if i == points.len() - 1 {
            points[i] - points[i - 1]
        } else {
            let prev = points[i] - points[i - 1];
            let next = points[i + 1] - points[i];

            let dir = (prev.normalize_or_zero() + next.normalize_or_zero()).normalize_or_zero();
            let speed = (prev.length() + next.length()) * 0.5;

            dir * speed
        }
    }

    #[inline]
    fn tangent_at_tension(points: &[Vec3A], i: usize, tension: f32) -> Vec3A {
        let energy = Spline::tangent_at_energy(points, i);
        let velo = Spline::tangent_at_velo(points, i);

        energy.lerp(velo, tension.clamp(0.0, 1.0))
    }

    fn cubic_from_tangents(p0: Vec3A, p1: Vec3A, v0: Vec3A, v1: Vec3A) -> BezierN {
        BezierN::new(p0, vec![p0 + v0 / 3.0, p1 - v1 / 3.0], p1)
    }

    pub fn build_spline(points: &[Vec3A], cost_mode: BezierCostMode) -> Vec<BezierN> {
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
