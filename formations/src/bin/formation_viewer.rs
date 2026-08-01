use ::glam::Vec3A;
use formations::boids::boid::Boid;
use formations::boids::cohesion_fn::cohesion_fn;
use formations::boids::separation_fn::separation_fn;
use macroquad::prelude::*;

const NUM_BOIDS: usize = 20;

fn draw_boid(b: &Boid) {
    draw_circle(b.pos.x, b.pos.y, 10.0, RED);

    let dir = b.velo.normalize_or_zero() * 3.0;

    draw_circle(b.pos.x + dir.x, b.pos.y + dir.y, 5.0, WHITE);
}

#[macroquad::main("Boids Viewer")]
async fn main() {
    let mut ctr = Vec3A::new(screen_width() / 2.0, screen_height() / 2.0, 0.0);

    let spacing = 40.0;
    let center_index = (NUM_BOIDS as f32 - 1.0) / 2.0;

    let mut boids: Vec<Boid> = (0..NUM_BOIDS)
        .map(|i| Boid {
            pos: ctr + Vec3A::new((i as f32 - center_index) * spacing, 0.0, 0.0),
            velo: Vec3A::ZERO,
            acc: Vec3A::ZERO,
        })
        .collect();

    loop {
        clear_background(BLACK);
        let dt = get_frame_time();

        if is_key_down(KeyCode::W) {
            ctr.y -= 2.0;
        }
        if is_key_down(KeyCode::S) {
            ctr.y += 2.0;
        }
        if is_key_down(KeyCode::A) {
            ctr.x -= 2.0;
        }
        if is_key_down(KeyCode::D) {
            ctr.x += 2.0;
        }

        let snapshot = boids.clone();
        let center_index = (boids.len() as f32 - 1.0) / 2.0;

        for (i, boid) in boids.iter_mut().enumerate() {
            let other_positions: Vec<Vec3A> = snapshot
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, other)| other.pos)
                .collect();

            let cols = 5;
            let spacing = 60.0;

            let row = i / cols;
            let col = i % cols;

            let rows = NUM_BOIDS.div_ceil(cols);

            let width = (cols - 1) as f32 * spacing;
            let height = (rows - 1) as f32 * spacing;

            let desired_position = ctr
                + Vec3A::new(
                    col as f32 * spacing - width / 2.0,
                    row as f32 * spacing - height / 2.0,
                    0.0,
                );

            let separation = 80.0 * separation_fn(boid.pos, 40.0, 80.0, &other_positions);

            let cohesion = cohesion_fn(boid.pos, boid.velo, desired_position, 2.2, 1.8);

            let too_close = snapshot
                .iter()
                .enumerate()
                .any(|(j, other)| j != i && (boid.pos - other.pos).length() < 20.0);

            if too_close {
                draw_circle(boid.pos.x, boid.pos.y, 20.0, YELLOW);
            }

            boid.acc = separation + cohesion;

            boid.velo += boid.acc * dt;
            boid.pos += boid.velo * dt;
        }

        for boid in &boids {
            draw_boid(boid);
        }

        draw_circle(ctr.x, ctr.y, 5.0, WHITE);

        next_frame().await;
    }
}
