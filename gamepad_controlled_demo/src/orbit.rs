use glam::Vec3;
use std::f32::consts::TAU;

pub const INITIAL_RADIUS_M: f32 = 3.0;
pub const PERIOD_S: f32 = 40.0;
pub const RADIUS_SPEED_M_S: f32 = 1.0;
pub const MIN_RADIUS_M: f32 = 1.0;
pub const MAX_RADIUS_M: f32 = 10.0;
const STICK_DEADZONE: f32 = 0.01;

pub struct Orbit {
    pub radius_m: f32,
    phase: f32,
}

impl Default for Orbit {
    fn default() -> Self {
        Self {
            radius_m: INITIAL_RADIUS_M,
            phase: 0.0,
        }
    }
}

impl Orbit {
    pub fn update(&mut self, axis: f32, dt_s: f32, rotating: bool) {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return;
        }
        if axis.is_finite() && axis.abs() > STICK_DEADZONE {
            let axis = (axis * 2.0) - 1.0;
            self.radius_m = (self.radius_m + axis.clamp(-1.0, 1.0) * RADIUS_SPEED_M_S * dt_s)
                .clamp(MIN_RADIUS_M, MAX_RADIUS_M);
        }
        if rotating {
            self.phase = (self.phase + TAU * dt_s / PERIOD_S).rem_euclid(TAU);
        }
    }

    pub fn target(&self, center: Vec3, index: usize, count: usize) -> Vec3 {
        assert!(count > 0 && index < count);
        let angle = self.phase + TAU * index as f32 / count as f32;
        center
            + Vec3::new(
                self.radius_m * angle.cos(),
                self.radius_m * angle.sin(),
                0.0,
            )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(actual: Vec3, expected: Vec3) {
        assert!(
            (actual - expected).length() < 0.0001,
            "{actual:?} != {expected:?}"
        );
    }

    #[test]
    fn targets_are_evenly_spaced_at_center_altitude() {
        let orbit = Orbit::default();
        let center = Vec3::new(4.0, -2.0, 5.0);
        for i in 0..6 {
            let target = orbit.target(center, i, 6);
            assert!(((target - center).length() - INITIAL_RADIUS_M).abs() < 0.0001);
            assert_eq!(target.z, center.z);
            let neighbor = orbit.target(center, (i + 1) % 6, 6);
            assert!(((target - neighbor).length() - INITIAL_RADIUS_M).abs() < 0.0001);
        }
    }

    #[test]
    fn rotation_has_configured_period_and_waits_for_activation() {
        let mut orbit = Orbit::default();
        let initial = orbit.target(Vec3::ZERO, 0, 6);
        orbit.update(0.0, PERIOD_S / 4.0, false);
        close(orbit.target(Vec3::ZERO, 0, 6), initial);
        orbit.update(0.0, PERIOD_S / 4.0, true);
        close(
            orbit.target(Vec3::ZERO, 0, 6),
            Vec3::new(0.0, INITIAL_RADIUS_M, 0.0),
        );
        orbit.update(0.0, 3.0 * PERIOD_S / 4.0, true);
        close(orbit.target(Vec3::ZERO, 0, 6), initial);
    }

    #[test]
    fn radius_control_is_time_based_bounded_and_ignores_noise() {
        let mut orbit = Orbit::default();
        orbit.update(1.0, 1.0, false);
        assert_eq!(orbit.radius_m, 4.0);
        orbit.update(STICK_DEADZONE / 2.0, 1.0, false);
        orbit.update(f32::NAN, 1.0, false);
        assert_eq!(orbit.radius_m, 4.0);
        orbit.update(0.25, 100.0, false);
        assert_eq!(orbit.radius_m, MIN_RADIUS_M);
        orbit.update(1.0, 100.0, false);
        assert_eq!(orbit.radius_m, MAX_RADIUS_M);
        let mut stepped = Orbit::default();
        let mut single = Orbit::default();
        for _ in 0..10 {
            stepped.update(1.0, 0.1, true);
        }
        single.update(1.0, 1.0, true);
        close(
            stepped.target(Vec3::ZERO, 0, 6),
            single.target(Vec3::ZERO, 0, 6),
        );
    }

    #[test]
    fn trigger_input_controls_radius_in_both_directions() {
        let mut orbit = Orbit::default();
        for input in [0.0, STICK_DEADZONE, 0.5] {
            orbit.update(input, 1.0, false);
            assert_eq!(orbit.radius_m, INITIAL_RADIUS_M);
        }
        orbit.update(0.25, 1.0, false);
        assert_eq!(orbit.radius_m, INITIAL_RADIUS_M - 0.5 * RADIUS_SPEED_M_S);
        orbit.update(0.75, 1.0, false);
        assert_eq!(orbit.radius_m, INITIAL_RADIUS_M);
        orbit.update(1.0, 1.0, false);
        assert_eq!(orbit.radius_m, INITIAL_RADIUS_M + RADIUS_SPEED_M_S);
    }

    #[test]
    fn moving_center_translates_all_targets_without_changing_offsets() {
        let orbit = Orbit::default();
        let center = Vec3::new(2.0, 3.0, 5.0);
        for i in 0..6 {
            close(
                orbit.target(center, i, 6),
                orbit.target(Vec3::ZERO, i, 6) + center,
            );
        }
    }
}
