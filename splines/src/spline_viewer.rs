use ::glam::{Quat, Vec3};
use macroquad::prelude::*;
use tokio::runtime::Builder;

use tokio::sync::mpsc;

use splines::{
    local_frame_waypoint::LocalFrameWaypoint,
    movement::{MotionFrame, MotionType, Movement},
    state::FullState,
};
use tokio::task::JoinHandle;

fn frame_name(frame: MotionFrame) -> &'static str {
    match frame {
        MotionFrame::NORTH_EAST_DOWN => "NED",
        MotionFrame::EAST_NORTH_UP => "ENU",
        MotionFrame::FORWARD_LEFT_UP => "FLU",
        MotionFrame::FORWARD_RIGHT_DOWN => "FRD",
    }
}

fn next_frame_mode(frame: MotionFrame) -> MotionFrame {
    match frame {
        MotionFrame::NORTH_EAST_DOWN => MotionFrame::EAST_NORTH_UP,
        MotionFrame::EAST_NORTH_UP => MotionFrame::FORWARD_LEFT_UP,
        MotionFrame::FORWARD_LEFT_UP => MotionFrame::FORWARD_RIGHT_DOWN,
        MotionFrame::FORWARD_RIGHT_DOWN => MotionFrame::NORTH_EAST_DOWN,
    }
}

#[macroquad::main("Spline Viewer")]
async fn main() {
    let tokio_runtime = Builder::new_multi_thread()
        .enable_all()
        .build()
        .expect("failed to create Tokio runtime");

    let tokio_handle = tokio_runtime.handle().clone();

    let mut targets = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 3.0, 0.0),
        Vec3::new(5.0, 1.0, 0.0),
        Vec3::new(7.0, 4.0, 0.0),
    ];

    let mut tension = 0.5_f32;
    let mut resolution = 20_u32;
    let mut pipe_radius_m = 1.2_f32;
    let mut max_speed_m_s = 2.5_f32;
    let mut longitudinal_accel_m_s2 = 1.0_f32;

    let mut continuous = true;
    let mut frame = MotionFrame::EAST_NORTH_UP;
    let mut dragging: Option<usize> = None;
    let mut show_hud = true;

    let scale = 60.0_f32;
    let offset = vec2(100.0, 420.0);

    /*
     * Rebuild the movement so edits are displayed immediately.
     */
    let motion_type = if continuous {
        MotionType::CONTINUOUS {
            resolution,
            energy_to_velocity: tension,
        }
    } else {
        MotionType::FINE
    };

    let mut movement = Movement::new(
        targets.clone(),
        pipe_radius_m,
        max_speed_m_s,
        longitudinal_accel_m_s2,
        motion_type,
        frame,
    );

    let mut state = FullState {
        pos: movement.points.first().copied().unwrap_or(Vec3::ZERO),
        velo: Vec3::ZERO,
        orientation: Quat::IDENTITY,
        angular_velocity: Vec3::ZERO,
        yaw: 0.0,
        attitude_time_ms: 0,
        position_time_ms: 0,
    };

    let state_rx = state.watch();
    let (waypoint_tx, waypoint_rx) = mpsc::channel::<LocalFrameWaypoint>(100);

    movement.run(&tokio_handle, state_rx, waypoint_tx);

    loop {
        clear_background(Color::from_rgba(18, 18, 22, 255));

        let mut edited = false;

        /*
         * Input controls.
         */
        if is_key_pressed(KeyCode::H) {
            show_hud = !show_hud;
            edited = true;
        }

        if is_key_pressed(KeyCode::M) {
            continuous = !continuous;
            edited = true;
        }

        if is_key_pressed(KeyCode::F) {
            frame = next_frame_mode(frame);
            edited = true;
        }

        if is_key_down(KeyCode::Up) {
            tension = (tension + 0.01).min(1.0);
            edited = true;
        }

        if is_key_down(KeyCode::Down) {
            tension = (tension - 0.01).max(0.0);
            edited = true;
        }

        if is_key_down(KeyCode::Right) {
            resolution = (resolution + 1).min(200);
            edited = true;
        }

        if is_key_down(KeyCode::Left) {
            resolution = resolution.saturating_sub(1).max(2);
            edited = true;
        }

        if is_key_down(KeyCode::W) {
            pipe_radius_m = (pipe_radius_m + 0.01).min(5.0);
            edited = true;
        }

        if is_key_down(KeyCode::S) {
            pipe_radius_m = (pipe_radius_m - 0.01).max(0.1);
            edited = true;
        }

        if is_key_down(KeyCode::D) {
            max_speed_m_s = (max_speed_m_s + 0.02).min(10.0);
            edited = true;
        }

        if is_key_down(KeyCode::A) {
            max_speed_m_s = (max_speed_m_s - 0.02).max(0.1);
            edited = true;
        }

        if is_key_down(KeyCode::Q) {
            longitudinal_accel_m_s2 = (longitudinal_accel_m_s2 - 0.01).max(0.0);
            edited = true;
        }

        if is_key_down(KeyCode::E) {
            longitudinal_accel_m_s2 = (longitudinal_accel_m_s2 + 0.01).min(10.0);
            edited = true;
        }

        /*
         * Coordinate conversion.
         */
        let to_screen = |point: Vec3| vec2(point.x, -point.y) * scale + offset;

        let to_world = |screen: Vec2| {
            Vec3::new(
                (screen.x - offset.x) / scale,
                -(screen.y - offset.y) / scale,
                0.0,
            )
        };

        /*
         * Draggable target points.
         */
        let mouse = Vec2::from(mouse_position());

        if is_mouse_button_pressed(MouseButton::Left) {
            dragging = targets
                .iter()
                .position(|point| (to_screen(*point) - mouse).length() < 10.0);
        }

        if is_mouse_button_released(MouseButton::Left) {
            dragging = None;
        }

        if let Some(index) = dragging {
            targets[index] = to_world(mouse);
            edited = true;
        }

        /*
         * Draw the sampled spline, colored by speed limit.
         */
        for (index, segment) in movement.points.windows(2).enumerate() {
            let start = to_screen(segment[0]);
            let end = to_screen(segment[1]);

            let speed_limit = movement
                .speed_limits
                .get(index)
                .copied()
                .unwrap_or_default();

            let speed_fraction = if max_speed_m_s > f32::EPSILON {
                (speed_limit / max_speed_m_s).clamp(0.0, 1.0)
            } else {
                0.0
            };

            let color = Color::new(1.0 - speed_fraction, speed_fraction, 0.25, 1.0);

            draw_line(start.x, start.y, end.x, end.y, 2.0, color);
        }

        /*
         * Draw each generated spline sample.
         */
        for point in &movement.points {
            let screen = to_screen(*point);

            draw_circle(screen.x, screen.y, 2.5, Color::new(0.75, 0.75, 0.75, 0.8));
        }

        /*
         * Draw editable source targets.
         */
        for (index, point) in targets.iter().enumerate() {
            let screen = to_screen(*point);

            let color = if dragging == Some(index) { RED } else { YELLOW };

            draw_circle(screen.x, screen.y, 7.0, color);

            draw_text(
                &index.to_string(),
                screen.x + 9.0,
                screen.y - 7.0,
                18.0,
                WHITE,
            );
        }

        /*
         * The vehicle is displayed at the movement's starting point.
         */
        let vehicle_position = state.pos;
        {
            let vehicle_screen = to_screen(vehicle_position);

            draw_circle(vehicle_screen.x, vehicle_screen.y, 9.0, SKYBLUE);

            draw_circle_lines(
                vehicle_screen.x,
                vehicle_screen.y,
                pipe_radius_m * scale,
                1.0,
                GRAY,
            );
        }

        /*
         * Viewer settings.
         */
        if show_hud {
            let mode = if continuous { "CONTINUOUS" } else { "FINE" };

            let lines = [
                format!("mode: {mode} (M)    frame: {} (F)", frame_name(frame),),
                format!(
                    "tension: {tension:.2} (Up/Down)    \
                     resolution: {resolution} (Left/Right)"
                ),
                format!(
                    "pipe radius: {pipe_radius_m:.2} m (W/S)    \
                     max speed: {max_speed_m_s:.2} m/s (D/A)"
                ),
                format!(
                    "path acceleration: \
                     {longitudinal_accel_m_s2:.2} m/s² (Q/E)"
                ),
                format!(
                    "control points: {}    spline samples: {}",
                    targets.len(),
                    movement.points.len(),
                ),
                "drag yellow points | H toggles HUD | red=slow, green=fast".to_string(),
            ];

            for (index, line) in lines.iter().enumerate() {
                draw_text(line, 20.0, 24.0 + index as f32 * 23.0, 19.0, WHITE);
            }
        }

        next_frame().await;
    }
}
