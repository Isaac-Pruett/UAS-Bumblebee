use std::time::Instant;

use ::glam::Vec3A;
use formations::boids::boid::Boid;
use formations::boids::cohesion_fn::cohesion_fn;
use formations::boids::corrections::enforce_hard_separation;
use formations::boids::repulsion_fn::repulsion_fn;
use kiss3d::prelude::*;

const NUM_BOIDS: usize = 5;

const BOID_RADIUS: f32 = 10.0;
const MAX_SPEED: f32 = 200.0;
const MAX_ACCEL: f32 = 500.0;
const SEPARATION_RADIUS: f32 = 50.0;

const HARD_SEPARATION_RADIUS: f32 = 20.0; // hard  — e.g. 2-3x BOID_RADIUS

const COLLISION_ITERATIONS: usize = 3;
const COLLISION_MARGIN: f32 = 2.0;
const COLLISION_EPSILON: f32 = 0.5;

/// Radius of the ring boids sit on, equidistant from `ctr`. Keep this
/// comfortably above SEPARATION_RADIUS or neighboring boids on the ring
/// will sit inside each other's soft repulsion zone at rest — same
/// "hard limit radius vs soft radius" trap discussed earlier, just now
/// governing ring spacing instead of grid spacing.
const RING_RADIUS: f32 = 120.0;

/// Converts the simulation's Vec3A into kiss3d's Vec3.
fn render_vec(v: Vec3A) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

fn create_rendering_boid(scene: &mut SceneNode3d, boid: &Boid) -> (SceneNode3d, SceneNode3d) {
    let mut body = scene.add_sphere(BOID_RADIUS);
    body.set_color(RED);
    body.set_position(render_vec(boid.pos));

    let mut direction = scene.add_sphere(BOID_RADIUS / 2.0);
    direction.set_color(WHITE);
    direction.set_position(render_vec(boid.pos));
    (body, direction)
}

/// Desired position for boid `index` of `count`, equidistant from
/// `center` on a circle of `radius` in the XY plane. Evenly spaced by
/// angle, so consecutive indices are also evenly spaced by arc length —
/// this only holds true in the plane; if you need a 3D equidistant
/// arrangement (a sphere) this needs a different formula (e.g. a Fibonacci
/// sphere), since points evenly spaced by angle on a circle don't
/// generalize to "evenly spaced on a sphere" the same way.
fn sphere_offset(index: usize, count: usize, radius: f32) -> Vec3A {
    const GOLDEN_ANGLE: f32 = std::f32::consts::PI * (3.0 - 2.236_068);

    let y = 1.0 - (index as f32 / (count.max(1) - 1).max(1) as f32) * 2.0;
    let radius_at_y = (1.0 - y * y).max(0.0).sqrt();
    let theta = GOLDEN_ANGLE * index as f32;

    Vec3A::new(
        radius * theta.cos() * radius_at_y,
        radius * y,
        radius * theta.sin() * radius_at_y,
    )
}

#[kiss3d::main]
async fn main() {
    let mut window = Window::new("Boids Viewer").await;
    window.set_background_color(BLACK);
    window.set_ambient(0.55);

    // Looking straight down the Z axis makes the scene behave like the
    // original 2D Macroquad simulation while retaining a 3D scene.
    let mut camera = OrbitCamera3d::new(Vec3::new(0.0, 0.0, 600.0), Vec3::ZERO);

    let mut scene = SceneNode3d::empty();

    scene
        .add_light(Light::point(2_500.0))
        .set_position(Vec3::new(0.0, 0.0, 600.0));

    let mut ctr = Vec3A::ZERO;

    let mut boids: Vec<Boid> = (0..NUM_BOIDS)
        .map(|i| Boid {
            pos: ctr + sphere_offset(i, NUM_BOIDS, RING_RADIUS),
            velo: Vec3A::ZERO,
            acc: Vec3A::ZERO,
        })
        .collect();

    // One red body and one white direction marker for every boid.
    let mut boid_nodes = Vec::with_capacity(NUM_BOIDS);
    let mut direction_nodes = Vec::with_capacity(NUM_BOIDS);

    for boid in &boids {
        let (body, direction) = create_rendering_boid(&mut scene, boid);

        boid_nodes.push(body);
        direction_nodes.push(direction);
    }

    let mut center_node = scene.add_sphere(5.0);
    center_node.set_color(WHITE);

    let control_speed = 250.0;
    let mut previous_time = Instant::now();

    while window.render_3d(&mut scene, &mut camera).await {
        let now = Instant::now();
        let dt = (now - previous_time).as_secs_f32().min(1.0 / 15.0);
        previous_time = now;

        // In world coordinates, positive Y is visually upward.
        if window.get_key(Key::W) == Action::Press {
            ctr.y += control_speed * dt;
        }
        if window.get_key(Key::S) == Action::Press {
            ctr.y -= control_speed * dt;
        }
        if window.get_key(Key::A) == Action::Press {
            ctr.x -= control_speed * dt;
        }
        if window.get_key(Key::D) == Action::Press {
            ctr.x += control_speed * dt;
        }

        let snapshot = boids.clone();
        let snapshot_positions: Vec<Vec3A> = snapshot.iter().map(|b| b.pos).collect();

        for (i, boid) in boids.iter_mut().enumerate() {
            let desired_position = ctr + sphere_offset(i, NUM_BOIDS, RING_RADIUS);

            let separation = 160.0
                * repulsion_fn(
                    i,
                    &snapshot_positions,
                    SEPARATION_RADIUS,
                    SEPARATION_RADIUS * 2.0,
                );
            let cohesion = cohesion_fn(boid.pos, boid.velo, desired_position, 2.2, 1.8);

            boid.acc = (cohesion + separation).clamp_length_max(MAX_ACCEL); // fresh, not accumulated

            boid.velo += boid.acc * dt;
            boid.velo = boid.velo.clamp_length_max(MAX_SPEED);
            boid.velo *= 0.995_f32.powf(dt * 60.0);

            let proposed_pos = boid.pos + boid.velo * dt;

            let others: Vec<Vec3A> = snapshot_positions
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .map(|(_, p)| *p)
                .collect();

            let corrected_pos = enforce_hard_separation(
                proposed_pos,
                &others,
                HARD_SEPARATION_RADIUS, // much tighter than SEPARATION_RADIUS
                COLLISION_ITERATIONS,
                COLLISION_MARGIN,
            );

            // If the backstop moved us, velocity should reflect where we actually
            // ended up — otherwise next frame's integration carries momentum
            // toward a place we just decided not to go. Same fix as in
            // horizon_planner.rs's sep_hit handling.
            if corrected_pos != proposed_pos {
                boid.velo = ((corrected_pos - boid.pos) / dt).clamp_length_max(MAX_SPEED);
            }

            boid.pos = corrected_pos;
        }

        // Transient lines are submitted every frame.
        // draw_collisions(&mut window, &boids);

        // Synchronize the simulation state with the scene nodes.
        for ((boid, body), direction) in boids
            .iter()
            .zip(boid_nodes.iter_mut())
            .zip(direction_nodes.iter_mut())
        {
            body.set_position(render_vec(boid.pos));

            let heading = boid.velo.normalize_or_zero() * BOID_RADIUS;
            direction.set_position(render_vec(boid.pos + heading));
        }

        center_node.set_position(render_vec(ctr));
    }
}
