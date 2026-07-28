use ::glam::Vec3;

pub fn separation_fn(
    x_particular: Vec3,
    radial_offset: f32,
    max_sep_rad: f32,
    x_others: &[Vec3],
) -> Vec3 {
    let mut separation = Vec3::ZERO;

    for other in x_others {
        let delta = x_particular - *other;
        let distance = delta.length();

        if distance > 0.0 && distance < max_sep_rad {
            let error = radial_offset - distance;

            if error > 0.0 {
                separation += delta.normalize() * error;
            }
        }
    }

    separation
}
