use glam::Vec3A;

/// Soft separation steering, active inside `influence_radius`.
/// `desired_clearance` sets the force scale; it is not a hard minimum distance.
pub fn repulsion_fn(
    index: usize,
    positions: &[Vec3A],
    desired_clearance: f32,
    influence_radius: f32,
) -> Vec3A {
    let position = positions[index];
    let mut repulsion = Vec3A::ZERO;
    let influence_radius_sq = influence_radius * influence_radius;

    assert!(desired_clearance.is_finite() && desired_clearance > 0.0);
    assert!(influence_radius.is_finite() && influence_radius > desired_clearance);

    // Preserve the close-range force while tapering only the outer avoidance zone.
    let softening_sq = (desired_clearance * 0.1).powi(2);

    for (i, &other) in positions.iter().enumerate() {
        if i == index {
            continue;
        }
        let delta = position - other;
        let distance_sq = delta.length_squared();

        if distance_sq >= influence_radius_sq {
            continue;
        }

        let distance = distance_sq.sqrt();
        let taper_fraction = ((influence_radius - distance)
            / (influence_radius - desired_clearance))
            .clamp(0.0, 1.0);
        let taper = taper_fraction * taper_fraction * (3.0 - 2.0 * taper_fraction);
        let strength = desired_clearance * desired_clearance / (distance_sq + softening_sq) * taper;

        // Coincident positions have no geometric escape direction. Do not use
        // index-based fallbacks: each drone builds a differently ordered snapshot.
        if distance > f32::EPSILON {
            repulsion += (delta / distance) * strength;
        }
    }

    repulsion
}

#[cfg(test)]
mod tests {
    use super::*;

    fn force(distance: f32) -> Vec3A {
        repulsion_fn(0, &[Vec3A::ZERO, Vec3A::X * distance], 2.5, 5.0)
    }

    #[test]
    fn repels_before_clearance_is_violated() {
        let result = force(4.0);
        assert!(result.x < 0.0);
        assert_eq!(result.y, 0.0);
        assert_eq!(result.z, 0.0);
    }

    #[test]
    fn preserves_close_range_strength() {
        let expected = 2.5_f32.powi(2) / (2.0_f32.powi(2) + 0.25_f32.powi(2));
        assert!((force(2.0).x + expected).abs() < 1e-6);
    }

    #[test]
    fn continuous_at_clearance_and_influence_boundary() {
        assert!((force(2.5 - 1e-4) - force(2.5 + 1e-4)).length() < 1e-3);
        assert!(force(5.0 - 1e-4).length() < 1e-6);
        assert_eq!(force(5.0), Vec3A::ZERO);
        assert_eq!(force(6.0), Vec3A::ZERO);
    }

    #[test]
    fn grows_as_neighbor_approaches() {
        let mut previous = 0.0;
        for distance in [4.9, 4.0, 3.0, 2.5, 2.0, 1.0, 0.1] {
            let magnitude = force(distance).length();
            assert!(magnitude > previous);
            previous = magnitude;
        }
    }

    #[test]
    fn pair_forces_are_opposite_and_self_is_ignored() {
        let positions = [Vec3A::ZERO, Vec3A::X * 3.0];
        assert_eq!(
            repulsion_fn(0, &positions, 2.5, 5.0),
            -repulsion_fn(1, &positions, 2.5, 5.0),
        );
        assert_eq!(repulsion_fn(0, &[Vec3A::ZERO], 2.5, 5.0), Vec3A::ZERO);
    }

    #[test]
    fn coincident_positions_remain_finite() {
        assert!(force(0.0).is_finite());
    }
}
