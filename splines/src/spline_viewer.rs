use ::glam::{Quat, Vec3};
use macroquad::prelude::*;
use tokio::{
    runtime::Builder,
    sync::{mpsc, watch},
};

use splines::{
    local_frame_waypoint::LocalFrameWaypoint,
    movement::{BarrierMargins, MotionFrame, MotionType, Movement},
    state::FullState,
};

fn build_movement(
    targets: &[Vec3],
    pipe_radius_m: f32,
    max_speed_m_s: f32,
    longitudinal_accel_m_s2: f32,
    resolution: u32,
    curvature: f32,
    barrier_margins: BarrierMargins,
) -> Movement {
    Movement::new(
        targets.to_vec(),
        pipe_radius_m,
        max_speed_m_s,
        longitudinal_accel_m_s2,
        barrier_margins,
        MotionType::CONTINUOUS {
            resolution,
            curvature,
        },
        MotionFrame::EAST_NORTH_UP,
    )
}

fn fresh_state(position: Vec3) -> FullState {
    FullState {
        pos: position,
        velo: Vec3::ZERO,
        orientation: Quat::IDENTITY,
        angular_velocity: Vec3::ZERO,
        yaw: 0.0,
        attitude_time_ms: 0,
        position_time_ms: 0,
    }
}

/// Color-codes a margin value: green when safe, sliding to red as it
/// approaches zero, and a saturated alarm red if it has gone negative
/// (i.e. the constraint has actually been violated).
fn margin_color(margin_m: f32, warn_at_m: f32) -> Color {
    if margin_m < 0.0 {
        return Color::new(1.0, 0.0, 0.0, 1.0);
    }

    let t = (margin_m / warn_at_m.max(f32::EPSILON)).clamp(0.0, 1.0);
    // t=0 (at the wall) -> red, t=1 (full margin) -> green
    Color::new(1.0 - t, t, 0.15, 1.0)
}

#[macroquad::main("Movement Simulation")]
async fn main() {
    let runtime = Builder::new_multi_thread()
        .enable_all()
        .build()
        .expect("failed to create Tokio runtime");

    let handle = runtime.handle().clone();

    let mut targets = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 3.0, 0.0),
        Vec3::new(5.0, 1.0, 0.0),
        Vec3::new(7.0, 4.0, 0.0),
    ];

    let mut pipe_radius_m = 2.0;
    let mut max_speed_m_s = 1.0;
    let mut longitudinal_accel_m_s2 = 3.0;
    let mut resolution = 20_u32;
    let mut curvature = 0.5;
    let mut barrier_margins = BarrierMargins::default();

    let mut movement = build_movement(
        &targets,
        pipe_radius_m,
        max_speed_m_s,
        longitudinal_accel_m_s2,
        resolution,
        curvature,
        barrier_margins,
    );

    let start = movement.points.first().copied().unwrap_or(Vec3::ZERO);

    let mut state = fresh_state(start);

    let (state_tx, state_rx) = watch::channel(Some(state.clone()));
    let (waypoint_tx, mut waypoint_rx) = mpsc::channel::<LocalFrameWaypoint>(8);

    movement.run(&handle, state_rx, waypoint_tx.clone());

    let mut active_command: Option<LocalFrameWaypoint> = None;
    let mut paused = false;
    let mut elapsed_ms = 0_u32;

    let mut dragging: Option<usize> = None;
    let drag_pick_radius_px = 12.0;

    let scale = 70.0;
    let offset = vec2(90.0, 500.0);
    let to_screen = |p: Vec3| vec2(p.x, -p.y) * scale + offset;
    let to_world = |p: Vec2| Vec3::new((p.x - offset.x) / scale, -(p.y - offset.y) / scale, 0.0);

    loop {
        let dt = get_frame_time().min(0.05);
        let mut edited = false;

        if is_key_pressed(KeyCode::Space) {
            paused = !paused;
        }

        if is_key_down(KeyCode::A) {
            pipe_radius_m = (pipe_radius_m - 0.05).max(0.1);
            edited = true;
        }
        if is_key_down(KeyCode::D) {
            pipe_radius_m += 0.05;
            edited = true;
        }

        if is_key_down(KeyCode::W) {
            max_speed_m_s += 0.1;
            edited = true;
        }
        if is_key_down(KeyCode::S) {
            max_speed_m_s = (max_speed_m_s - 0.1).max(0.1);
            edited = true;
        }

        if is_key_down(KeyCode::Q) {
            longitudinal_accel_m_s2 = (longitudinal_accel_m_s2 - 0.1).max(0.1);
            edited = true;
        }
        if is_key_down(KeyCode::E) {
            longitudinal_accel_m_s2 += 0.1;
            edited = true;
        }

        if is_key_down(KeyCode::Z) {
            resolution = resolution.saturating_sub(1).max(1);
            edited = true;
        }
        if is_key_down(KeyCode::X) {
            resolution = resolution.saturating_add(1);
            edited = true;
        }

        if is_key_down(KeyCode::C) {
            curvature = (curvature - 0.05_f32).max(0.0_f32);
            edited = true;
        }
        if is_key_down(KeyCode::V) {
            curvature = (curvature + 0.05_f32).min(1.0_f32);
            edited = true;
        }

        // G / H: pipe barrier gain
        if is_key_down(KeyCode::G) {
            barrier_margins.pipe_margin_m = (barrier_margins.pipe_margin_m - 0.05_f32).max(0.0_f32);
            edited = true;
        }
        if is_key_down(KeyCode::H) {
            barrier_margins.pipe_margin_m = (barrier_margins.pipe_margin_m + 0.05_f32).min(4.0_f32);
            edited = true;
        }

        // O / P: carrot-lead barrier gain
        if is_key_down(KeyCode::O) {
            barrier_margins.carrot_lead_m = (barrier_margins.carrot_lead_m - 0.05_f32).max(0.0_f32);
            edited = true;
        }
        if is_key_down(KeyCode::P) {
            barrier_margins.carrot_lead_m = (barrier_margins.carrot_lead_m + 0.05_f32).min(4.0_f32);
            edited = true;
        }

        // --- Mouse: pick up / drag / drop a waypoint ---
        let mouse_screen = Vec2::from(mouse_position());

        if is_mouse_button_pressed(MouseButton::Left) {
            let mut closest: Option<(usize, f32)> = None;

            for (i, t) in targets.iter().enumerate() {
                let p = to_screen(*t);
                let dist = p.distance(mouse_screen);

                if dist <= drag_pick_radius_px {
                    if closest.map_or(true, |(_, best)| dist < best) {
                        closest = Some((i, dist));
                    }
                }
            }

            if let Some((i, _)) = closest {
                dragging = Some(i);
            }
        }

        if is_mouse_button_released(MouseButton::Left) {
            dragging = None;
        }

        if let Some(i) = dragging {
            let world = to_world(mouse_screen);
            let new_pos = Vec3::new(world.x, world.y, targets[i].z);

            if new_pos != targets[i] {
                targets[i] = new_pos;
                edited = true;
            }
        }

        if is_key_pressed(KeyCode::R) || edited {
            movement.cancel();

            movement = build_movement(
                &targets,
                pipe_radius_m,
                max_speed_m_s,
                longitudinal_accel_m_s2,
                resolution,
                curvature,
                barrier_margins,
            );

            let reset_position = movement.points.first().copied().unwrap_or(start);

            state = fresh_state(reset_position);
            active_command = None;
            elapsed_ms = 0;

            while waypoint_rx.try_recv().is_ok() {}

            let _ = state_tx.send(Some(state.clone()));

            movement.run(&handle, state_tx.subscribe(), waypoint_tx.clone());
        }

        while let Ok(command) = waypoint_rx.try_recv() {
            active_command = Some(command);
        }

        if !paused {
            if let Some(command) = &active_command {
                let desired_velocity = Vec3::new(command.vx, command.vy, command.vz);

                let vehicle_max_accel_m_s2 = 2.0;
                let velocity_delta = desired_velocity - state.velo;

                state.velo += velocity_delta.clamp_length_max(vehicle_max_accel_m_s2 * dt);

                state.pos += state.velo * dt;

                if state.velo.length_squared() > f32::EPSILON {
                    state.yaw = state.velo.y.atan2(state.velo.x);
                }
            }

            elapsed_ms = elapsed_ms.saturating_add((dt * 1000.0) as u32);

            state.attitude_time_ms = elapsed_ms;
            state.position_time_ms = elapsed_ms;

            let _ = state_tx.send(Some(state.clone()));
        }

        // Live barrier margins, computed against the active carrot (if any).
        let margins = active_command
            .as_ref()
            .and_then(|c| movement.barrier_margins(state.pos, Vec3::new(c.x, c.y, c.z)));

        clear_background(Color::from_rgba(18, 18, 22, 255));

        // --- Path: static speed-limit centerline (now dimmer/secondary) ---
        for (index, segment) in movement.points.windows(2).enumerate() {
            let a = segment[0];
            let b = segment[1];

            let a_screen = to_screen(a);
            let b_screen = to_screen(b);

            let fraction = movement.speed_limits.get(index).copied().unwrap_or(0.0)
                / movement.max_speed_m_s.max(f32::EPSILON);

            draw_line(
                a_screen.x,
                a_screen.y,
                b_screen.x,
                b_screen.y,
                2.0,
                Color::new(1.0 - fraction, fraction, 0.2, 0.5),
            );

            let dir = b - a;
            let len = dir.length();

            if len > f32::EPSILON {
                let n = Vec3::new(-dir.y, dir.x, 0.0) / len;
                let offset = n * pipe_radius_m;

                let left_a = to_screen(a + offset);
                let left_b = to_screen(b + offset);
                let right_a = to_screen(a - offset);
                let right_b = to_screen(b - offset);

                let pipe_color = Color::new(0.5, 0.7, 1.0, 0.35);

                draw_line(left_a.x, left_a.y, left_b.x, left_b.y, 1.5, pipe_color);
                draw_line(right_a.x, right_a.y, right_b.x, right_b.y, 1.5, pipe_color);
            }
        }

        for (i, target) in targets.iter().enumerate() {
            let p = to_screen(*target);

            let color = if dragging == Some(i) { ORANGE } else { YELLOW };
            let radius = if dragging == Some(i) { 8.0 } else { 6.0 };

            draw_circle(p.x, p.y, radius, color);

            if dragging.is_none() && p.distance(mouse_screen) <= drag_pick_radius_px {
                draw_circle_lines(
                    p.x,
                    p.y,
                    drag_pick_radius_px,
                    1.5,
                    Color::new(1.0, 1.0, 0.0, 0.5),
                );
            }
        }

        if let Some(command) = &active_command {
            let p = to_screen(Vec3::new(command.x, command.y, command.z));
            draw_circle_lines(p.x, p.y, 9.0, 2.0, ORANGE);
        }

        // --- Vehicle: color-coded live by pipe margin instead of a fixed color ---
        let vehicle = to_screen(state.pos);

        let vehicle_color = margins
            .map(|m| margin_color(m.pipe_margin_m, pipe_radius_m))
            .unwrap_or(SKYBLUE);

        draw_circle(vehicle.x, vehicle.y, 9.0, vehicle_color);

        // Ring showing exactly how much lateral margin is left, to scale.
        if let Some(m) = margins {
            let margin_px = m.pipe_margin_m.max(0.0) * scale;
            draw_circle_lines(
                vehicle.x,
                vehicle.y,
                margin_px,
                1.0,
                Color::new(1.0, 1.0, 1.0, 0.25),
            );
        }

        let heading = vec2(state.yaw.cos(), -state.yaw.sin()) * 18.0;

        draw_line(
            vehicle.x,
            vehicle.y,
            vehicle.x + heading.x,
            vehicle.y + heading.y,
            3.0,
            WHITE,
        );

        let status = if paused { "PAUSED" } else { "RUNNING" };

        draw_text(
            &format!("{status} | Space: pause | R: reset | drag yellow dots to move waypoints"),
            20.0,
            28.0,
            24.0,
            WHITE,
        );

        draw_text(
            &format!(
                "position: ({:.2}, {:.2}) | speed: {:.2} m/s",
                state.pos.x,
                state.pos.y,
                state.velo.length(),
            ),
            20.0,
            56.0,
            21.0,
            LIGHTGRAY,
        );

        // --- Live barrier telemetry ---
        if let Some(m) = margins {
            draw_text(
                &format!("pipe margin: {:.2} m", m.pipe_margin_m),
                20.0,
                80.0,
                20.0,
                margin_color(m.pipe_margin_m, pipe_radius_m),
            );

            draw_text(
                &format!("carrot lead: {:.2} m", m.carrot_lead_m),
                220.0,
                80.0,
                20.0,
                margin_color(m.carrot_lead_m, pipe_radius_m.max(0.5)),
            );
        } else {
            draw_text("pipe margin: --", 20.0, 80.0, 20.0, GRAY);
            draw_text("carrot lead: --", 220.0, 80.0, 20.0, GRAY);
        }

        draw_text(
            &format!("A/D pipe radius: {:.2} m", pipe_radius_m,),
            20.0,
            114.0,
            20.0,
            WHITE,
        );

        draw_text(
            &format!("S/W max speed: {:.2} m/s", max_speed_m_s,),
            20.0,
            140.0,
            20.0,
            WHITE,
        );

        draw_text(
            &format!("Q/E acceleration: {:.2} m/s²", longitudinal_accel_m_s2,),
            20.0,
            166.0,
            20.0,
            WHITE,
        );

        draw_text(
            &format!("Z/X resolution: {}", resolution,),
            20.0,
            192.0,
            20.0,
            WHITE,
        );

        draw_text(
            &format!("C/V curvature: {:.2}", curvature,),
            20.0,
            218.0,
            20.0,
            WHITE,
        );

        draw_text(
            &format!("G/H pipe gain: {:.2}", barrier_margins.pipe_margin_m,),
            20.0,
            244.0,
            20.0,
            WHITE,
        );

        draw_text(
            &format!("O/P carrot gain: {:.2}", barrier_margins.carrot_lead_m,),
            20.0,
            270.0,
            20.0,
            WHITE,
        );

        next_frame().await;
    }
}
