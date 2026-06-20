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

// get the cost from tangent vector at point i in points
#[inline]
fn vec_from_cost_at_idx(points: &[Vec3], i: usize, cost_mode: BezierCostMode) -> Vec3 {
    assert!(points.len() >= 2);
    match cost_mode {
        BezierCostMode::UNIFORM => tangent_at_energy(points, i),
        BezierCostMode::VELOCITY => tangent_at_velo(points, i),

        BezierCostMode::TENSIONED(t) => tangent_at_tension(points, i, t),
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
fn tangent_at_tension(points: &[Vec3], i: usize, tension: f32) -> Vec3 {
    let energy = tangent_at_energy(points, i);
    let velo = tangent_at_velo(points, i);

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
        let v0 = vec_from_cost_at_idx(points, i, cost_mode);
        let v1 = vec_from_cost_at_idx(points, i + 1, cost_mode);

        curves.push(cubic_from_tangents(points[i], points[i + 1], v0, v1));
    }

    curves
}

#[derive(Debug, Clone, Copy)]
pub enum BezierCostMode {
    UNIFORM,        // good default, least energy
    VELOCITY,       // most flow
    TENSIONED(f32), // variable intensity
}
