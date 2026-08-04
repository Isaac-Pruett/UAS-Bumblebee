use std::time::Instant;

use ::glam::Vec3A;
use formations::boids::boid::Boid;
use formations::boids::cohesion_fn::cohesion_fn;
use formations::boids::corrections::enforce_hard_separation;
use formations::boids::repulsion_fn::repulsion_fn;
use kiss3d::prelude::*;

const BOIDS_PER_FLOCK: usize = 20;
const NUM_FLOCKS: usize = 2;
const NUM_BOIDS: usize = BOIDS_PER_FLOCK * NUM_FLOCKS;

const BOID_RADIUS: f32 = 10.0;
const MAX_SPEED: f32 = 200.0;
const MAX_ACCEL: f32 = 500.0;
const SEPARATION_RADIUS: f32 = 50.0;

const HARD_SEPARATION_RADIUS: f32 = 20.0; // hard backstop — e.g. 2-3x BOID_RADIUS

const COLLISION_ITERATIONS: usize = 3;
const COLLISION_MARGIN: f32 = 2.0;
const COLLISION_EPSILON: f32 = 0.5;

const GRID_COLS: usize = 5;
const GRID_SPACING: f32 = 60.0;

/// Which flock a boid belongs to. Used only for desired-position
/// calculation and rendering color — the physics (repulsion, hard
/// separation) always runs over ALL boids regardless of flock, so
/// flocks never visually overlap/clip even while separate.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum FlockId {
    A,
    B,
}

/// Per-flock steerable center. Two of these exist independently; when
/// linked, both flocks compute their grid target from a single shared
/// point instead of their own `ctr`, which is what actually merges them
/// into one formation without changing anything about the physics loop.
struct FlockControl {
    ctr: Vec3A,
    color: Color,
}

/// Converts the simulation's Vec3A into kiss3d's Vec3.
fn render_vec(v: Vec3A) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

fn create_rendering_boid(
    scene: &mut SceneNode3d,
    boid: &Boid,
    color: Color,
) -> (SceneNode3d, SceneNode3d) {
    let mut body = scene.add_sphere(BOID_RADIUS);
    body.set_color(color);
    body.set_position(render_vec(boid.pos));

    let mut direction = scene.add_sphere(BOID_RADIUS / 2.0);
    direction.set_color(WHITE);
    direction.set_position(render_vec(boid.pos));
    (body, direction)
}

/// Grid slot offset for boid `local_index` (0..BOIDS_PER_FLOCK) within
/// its own flock's grid, centered on that flock's own origin.
fn grid_offset(local_index: usize, count: usize) -> Vec3A {
    let row = local_index / GRID_COLS;
    let col = local_index % GRID_COLS;
    let rows = count.div_ceil(GRID_COLS);

    let width = (GRID_COLS - 1) as f32 * GRID_SPACING;
    let height = (rows - 1) as f32 * GRID_SPACING;

    Vec3A::new(
        col as f32 * GRID_SPACING - width / 2.0,
        height / 2.0 - row as f32 * GRID_SPACING,
        0.0,
    )
}

#[kiss3d::main]
async fn main() {
    let mut window = Window::new("Boids Viewer").await;
    window.set_background_color(BLACK);
    window.set_ambient(0.55);

    let mut camera = OrbitCamera3d::new(Vec3::new(0.0, 0.0, 900.0), Vec3::ZERO);

    let mut scene = SceneNode3d::empty();

    scene
        .add_light(Light::point(2_500.0))
        .set_position(Vec3::new(0.0, 0.0, 600.0));

    let mut flock_a = FlockControl {
        ctr: Vec3A::new(-200.0, 0.0, 0.0),
        color: RED,
    };
    let mut flock_b = FlockControl {
        ctr: Vec3A::new(200.0, 0.0, 0.0),
        color: BLUE,
    };

    // Space toggles linked/unlinked. Debounced with a "was pressed last
    // frame" flag since kiss3d gives us raw key state, not edge events.
    let mut linked = false;
    let mut space_was_down = false;

    let initial_spacing = 40.0;
    let center_index = (BOIDS_PER_FLOCK as f32 - 1.0) / 2.0;

    // flock_of[i] tells us which FlockControl to pull `ctr`/color from
    // for boid i. Boids themselves don't carry flock identity beyond
    // this parallel array — keeps `Boid` itself unchanged from before.
    let mut flock_of: Vec<FlockId> = Vec::with_capacity(NUM_BOIDS);
    let mut boids: Vec<Boid> = Vec::with_capacity(NUM_BOIDS);

    for flock_id in [FlockId::A, FlockId::B] {
        let base = match flock_id {
            FlockId::A => flock_a.ctr,
            FlockId::B => flock_b.ctr,
        };
        for i in 0..BOIDS_PER_FLOCK {
            boids.push(Boid {
                pos: base + Vec3A::new((i as f32 - center_index) * initial_spacing, 0.0, 0.0),
                velo: Vec3A::ZERO,
                acc: Vec3A::ZERO,
            });
            flock_of.push(flock_id);
        }
    }

    // One red/blue body and one white direction marker per boid, colored
    // by initial flock membership. Colors don't change when linked —
    // the merge is positional, not a re-identification of who's who.
    let mut boid_nodes = Vec::with_capacity(NUM_BOIDS);
    let mut direction_nodes = Vec::with_capacity(NUM_BOIDS);

    for (boid, &flock_id) in boids.iter().zip(flock_of.iter()) {
        let color = match flock_id {
            FlockId::A => flock_a.color,
            FlockId::B => flock_b.color,
        };
        let (body, direction) = create_rendering_boid(&mut scene, boid, color);
        boid_nodes.push(body);
        direction_nodes.push(direction);
    }

    let mut center_node_a = scene.add_sphere(5.0);
    center_node_a.set_color(flock_a.color);
    let mut center_node_b = scene.add_sphere(5.0);
    center_node_b.set_color(flock_b.color);

    let control_speed = 250.0;
    let mut previous_time = Instant::now();

    while window.render_3d(&mut scene, &mut camera).await {
        let now = Instant::now();
        let dt = (now - previous_time).as_secs_f32().min(1.0 / 15.0);
        previous_time = now;

        // Flock A: WASD.
        if window.get_key(Key::W) == Action::Press {
            flock_a.ctr.y += control_speed * dt;
        }
        if window.get_key(Key::S) == Action::Press {
            flock_a.ctr.y -= control_speed * dt;
        }
        if window.get_key(Key::A) == Action::Press {
            flock_a.ctr.x -= control_speed * dt;
        }
        if window.get_key(Key::D) == Action::Press {
            flock_a.ctr.x += control_speed * dt;
        }

        // Flock B: arrow keys.
        if window.get_key(Key::Up) == Action::Press {
            flock_b.ctr.y += control_speed * dt;
        }
        if window.get_key(Key::Down) == Action::Press {
            flock_b.ctr.y -= control_speed * dt;
        }
        if window.get_key(Key::Left) == Action::Press {
            flock_b.ctr.x -= control_speed * dt;
        }
        if window.get_key(Key::Right) == Action::Press {
            flock_b.ctr.x += control_speed * dt;
        }

        // Space toggles link state on press (edge-triggered, not held).
        let space_down = window.get_key(Key::Space) == Action::Press;
        if space_down && !space_was_down {
            linked = !linked;
        }
        space_was_down = space_down;

        let snapshot = boids.clone();
        let snapshot_positions: Vec<Vec3A> = snapshot.iter().map(|b| b.pos).collect();

        for (i, boid) in boids.iter_mut().enumerate() {
            let desired_position = if linked {
                // Merged: one 40-boid grid centered on the midpoint of
                // both controllers, indexed by GLOBAL position `i` —
                // this is what actually produces one grid instead of
                // two overlapping ones, since every boid now reads from
                // the same base point and the same continuous index
                // space rather than a per-flock local index.
                let merged_ctr = (flock_a.ctr + flock_b.ctr) * 0.5;
                merged_ctr + grid_offset(i, NUM_BOIDS)
            } else {
                // Separate: each boid computes its slot from its own
                // flock's ctr and its LOCAL index within that flock
                // (0..BOIDS_PER_FLOCK), so both flocks form independent
                // grids regardless of global index i.
                let local_index = i % BOIDS_PER_FLOCK;
                let base = match flock_of[i] {
                    FlockId::A => flock_a.ctr,
                    FlockId::B => flock_b.ctr,
                };
                base + grid_offset(local_index, BOIDS_PER_FLOCK)
            };

            // Repulsion and the hard backstop always run over ALL boids
            // globally, linked or not — this is what keeps the two
            // flocks from flying through each other while separate, and
            // is also just correct physics once merged.
            let separation = 160.0
                * repulsion_fn(
                    i,
                    &snapshot_positions,
                    SEPARATION_RADIUS,
                    SEPARATION_RADIUS * 2.0,
                );
            let cohesion = cohesion_fn(boid.pos, boid.velo, desired_position, 2.2, 1.8);

            boid.acc = (cohesion + separation).clamp_length_max(MAX_ACCEL);

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
                HARD_SEPARATION_RADIUS,
                COLLISION_ITERATIONS,
                COLLISION_MARGIN,
            );

            if corrected_pos != proposed_pos {
                boid.velo = ((corrected_pos - boid.pos) / dt).clamp_length_max(MAX_SPEED);
            }

            boid.pos = corrected_pos;
        }

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

        center_node_a.set_position(render_vec(flock_a.ctr));
        center_node_b.set_position(render_vec(flock_b.ctr));
    }
}
