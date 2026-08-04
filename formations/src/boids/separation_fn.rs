use glam::Vec3A;

pub fn repulsion_fn(
    position: Vec3A,
    desired_clearance: f32,
    influence_radius: f32,
    obstacles: &[Vec3A],
) -> Vec3A {
    let mut repulsion = Vec3A::ZERO;
    let influence_radius_sq = influence_radius * influence_radius;

    for obstacle in obstacles {
        let delta = position - *obstacle;
        let distance_sq = delta.length_squared();

        if distance_sq <= f32::EPSILON || distance_sq >= influence_radius_sq {
            continue;
        }

        let distance = distance_sq.sqrt();
        let penetration = desired_clearance - distance;

        if penetration > 0.0 {
            repulsion += delta / distance * penetration;
        }
    }

    repulsion
}
