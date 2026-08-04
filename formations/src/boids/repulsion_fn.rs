use glam::Vec3A;

pub fn repulsion_fn(
    index: usize,
    positions: &[Vec3A],
    desired_clearance: f32,
    influence_radius: f32,
) -> Vec3A {
    let position = positions[index];
    let mut repulsion = Vec3A::ZERO;
    let influence_radius_sq = influence_radius * influence_radius;

    // Softening bounds peak force as distance -> 0 instead of a hard
    // divide-by-near-zero. Tune relative to desired_clearance, not as
    // an absolute constant — see note below.
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
        if distance >= desired_clearance {
            continue;
        }

        let strength = desired_clearance * desired_clearance / (distance_sq + softening_sq);

        // direction is delta/distance; guard distance==0 case (fully
        // coincident positions) by falling back to zero direction
        // rather than NaN.
        if distance > f32::EPSILON {
            repulsion += (delta / distance) * strength;
        }
    }

    repulsion
}
