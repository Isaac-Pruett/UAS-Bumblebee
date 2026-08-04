use glam::Vec3A;

pub fn enforce_hard_separation(
    proposed: Vec3A,
    others: &[Vec3A],
    min_separation: f32,
    iterations: usize,
    margin: f32,
) -> Vec3A {
    let mut corrected = proposed;

    for _ in 0..iterations {
        let mut correction = Vec3A::ZERO;
        let mut any_violation = false;

        for &other in others {
            let delta = corrected - other;
            let dist = delta.length();

            if dist < min_separation {
                any_violation = true;
                let dir = if dist > f32::EPSILON {
                    delta / dist
                } else {
                    Vec3A::X // deterministic fallback for exact overlap
                };
                let needed = min_separation - dist;
                correction += dir * (needed + margin);
            }
        }

        if !any_violation {
            break;
        }
        corrected += correction;
    }

    corrected
}
