use std::time::Instant;

use ::glam::Vec3A;
use formations::boids::boid::Boid;
use formations::boids::cohesion_fn::cohesion_fn;
use formations::boids::separation_fn::repulsion_fn;
use kiss3d::prelude::*;

const NUM_BOIDS: usize = 20;

const BOID_RADIUS: f32 = 10.0;
const MAX_SPEED: f32 = 200.0;
const MAX_ACCEL: f32 = 500.0;
const SEPARATION_RADIUS: f32 = 80.0;

const COLLISION_ITERATIONS: usize = 3;
const COLLISION_MARGIN: f32 = 2.0;
const COLLISION_EPSILON: f32 = 0.5;

/// Converts the simulation's Vec3A into kiss3d's Vec3.
fn render_vec(v: Vec3A) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

#[kiss3d::main]
async fn main() {
    let mut window = Window::new("Boids Viewer").await;
    window.set_background_color(BLACK);
    window.set_ambient(0.55);

    // Looking straight down the Z axis makes the scene behave like the
    // original 2D Macroquad simulation while retaining a 3D scene.
    let mut camera = OrbitCamera3d::new(Vec3::new(0.0, 0.0, 1_200.0), Vec3::ZERO);

    let mut scene = SceneNode3d::empty();

    scene
        .add_light(Light::point(2_500.0))
        .set_position(Vec3::new(0.0, 0.0, 600.0));

    let mut ctr = Vec3A::ZERO;

    let initial_spacing = 40.0;
    let center_index = (NUM_BOIDS as f32 - 1.0) / 2.0;

    let mut boids: Vec<Boid> = (0..NUM_BOIDS)
        .map(|i| Boid {
            pos: ctr + Vec3A::new((i as f32 - center_index) * initial_spacing, 0.0, 0.0),
            velo: Vec3A::ZERO,
            acc: Vec3A::ZERO,
        })
        .collect();

    // One red body and one white direction marker for every boid.
    let mut boid_nodes = Vec::with_capacity(NUM_BOIDS);
    let mut direction_nodes = Vec::with_capacity(NUM_BOIDS);

    for boid in &boids {
        let mut body = scene.add_sphere(BOID_RADIUS);
        body.set_color(RED);
        body.set_position(render_vec(boid.pos));
        boid_nodes.push(body);

        let mut direction = scene.add_sphere(BOID_RADIUS / 2.0);
        direction.set_color(WHITE);
        direction.set_position(render_vec(boid.pos));
        direction_nodes.push(direction);
    }

    // Coordinates are centered around the world origin instead of being
    // expressed in screen pixels.
    let obstacles = vec![
        Vec3A::new(-100.0, 0.0, 0.0),
        Vec3A::new(100.0, 50.0, 0.0),
        Vec3A::new(200.0, -150.0, 0.0),
    ];

    let mut obstacle_nodes = Vec::with_capacity(obstacles.len());

    for obstacle in &obstacles {
        let mut node = scene.add_sphere(15.0);
        node.set_color(BLUE);
        node.set_position(render_vec(*obstacle));
        obstacle_nodes.push(node);
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

        for (i, boid) in boids.iter_mut().enumerate() {
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
                    height / 2.0 - row as f32 * spacing,
                    0.0,
                );

            let neighbors: Vec<Vec3A> = snapshot
                .iter()
                .enumerate()
                .filter_map(|(j, other)| if j != i { Some(other.pos) } else { None })
                .collect();

            let separation =
                180.0 * repulsion_fn(boid.pos, BOID_RADIUS * 2.0, SEPARATION_RADIUS, &neighbors);

            let obstacle_avoidance = 220.0 * repulsion_fn(boid.pos, 30.0, 140.0, &obstacles);

            let cohesion = cohesion_fn(boid.pos, boid.velo, desired_position, 2.2, 1.8);

            boid.acc = cohesion + separation + obstacle_avoidance;
            boid.acc = boid.acc.clamp_length_max(MAX_ACCEL);

            boid.velo += boid.acc * dt;
            boid.velo = boid.velo.clamp_length_max(MAX_SPEED);
            boid.velo *= 0.995_f32.powf(dt * 60.0);

            boid.pos += boid.velo * dt;
        }

        resolve_collisions(&mut boids);

        // Transient lines are submitted every frame.
        draw_collisions(&mut window, &boids);

        // Synchronize the simulation state with the scene nodes.
        for ((boid, body), direction) in boids
            .iter()
            .zip(boid_nodes.iter_mut())
            .zip(direction_nodes.iter_mut())
        {
            body.set_position(render_vec(boid.pos));

            let heading = boid.velo.normalize_or_zero() * (BOID_RADIUS * 1.5);
            direction.set_position(render_vec(boid.pos + heading));
        }

        center_node.set_position(render_vec(ctr));

        // Keep these handles alive for the life of the scene.
        let _ = &obstacle_nodes;
    }
}

fn resolve_collisions(boids: &mut [Boid]) {
    let minimum_distance = BOID_RADIUS * 2.0 + COLLISION_MARGIN;
    let minimum_distance_squared = minimum_distance * minimum_distance;

    for _ in 0..COLLISION_ITERATIONS {
        let mut corrected_any = false;

        for i in 0..boids.len() {
            for j in (i + 1)..boids.len() {
                let (left, right) = boids.split_at_mut(j);
                let boid_a = &mut left[i];
                let boid_b = &mut right[0];

                let delta = boid_b.pos - boid_a.pos;
                let distance_squared = delta.length_squared();

                if distance_squared >= minimum_distance_squared {
                    continue;
                }

                let distance = distance_squared.sqrt();

                let direction = if distance > 0.0001 {
                    delta / distance
                } else {
                    // Deterministic fallback for coincident boids.
                    let angle = (i as f32 * 12.9898 + j as f32 * 78.233).sin();

                    Vec3A::new(angle.cos(), angle.sin(), 0.0)
                };

                let penetration = minimum_distance - distance;

                if penetration > COLLISION_EPSILON {
                    corrected_any = true;

                    // Slight overcorrection prevents residual overlap caused
                    // by floating-point rounding.
                    let correction = direction * penetration * 0.51;

                    boid_a.pos -= correction;
                    boid_b.pos += correction;
                }

                let relative_velocity = boid_b.velo - boid_a.velo;
                let closing_speed = relative_velocity.dot(direction);

                if closing_speed < 0.0 {
                    let impulse = direction * closing_speed * 0.5;

                    boid_a.velo += impulse;
                    boid_b.velo -= impulse;
                }
            }
        }

        if !corrected_any {
            break;
        }
    }
}

fn draw_collisions(window: &mut Window, boids: &[Boid]) {
    let minimum_distance = BOID_RADIUS * 2.0;

    for i in 0..boids.len() {
        for j in (i + 1)..boids.len() {
            let delta = boids[j].pos - boids[i].pos;

            if delta.length() < minimum_distance {
                let midpoint = (boids[i].pos + boids[j].pos) * 0.5;

                window.draw_line(
                    render_vec(boids[i].pos),
                    render_vec(boids[j].pos),
                    YELLOW,
                    3.0,   // line width
                    false, // screen-space width; not perspective-scaled
                );

                window.draw_point(
                    render_vec(midpoint),
                    YELLOW,
                    10.0, // point size
                );
            }
        }
    }
}
