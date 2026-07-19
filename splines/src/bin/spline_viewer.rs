use ::glam::Vec3;
use macroquad::prelude::*; // explicit path avoids ambiguity with macroquad's glob-imported Vec3

#[path = "../splines.rs"]
mod splines;
use splines::Spline;

fn convex_hull(points: &[Vec2]) -> Vec<Vec2> {
    let mut pts = points.to_vec();
    pts.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap()
            .then(a.y.partial_cmp(&b.y).unwrap())
    });
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }

    fn cross(o: Vec2, a: Vec2, b: Vec2) -> f32 {
        (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
    }

    let mut lower = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }

    let mut upper = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Walk the spline once, accumulating arc length and per-segment speed limit.
/// Returns (cumulative_dist_at_each_point, total_length).
fn arc_lengths(points: &[Vec3]) -> (Vec<f32>, f32) {
    let mut cum = vec![0.0; points.len()];
    for i in 1..points.len() {
        cum[i] = cum[i - 1] + (points[i] - points[i - 1]).length();
    }
    let total = *cum.last().unwrap_or(&0.0);
    (cum, total)
}

/// Given traveled distance along the arc-length table, find the interpolated
/// position and the local speed limit (from the nearer of the two bracketing points).
fn sample_at_distance(points: &[Vec3], cum: &[f32], limits: &[f32], dist: f32) -> (Vec3, f32) {
    if points.len() < 2 {
        return (points.first().copied().unwrap_or(Vec3::ZERO), 0.0);
    }
    let idx = match cum.binary_search_by(|c| c.partial_cmp(&dist).unwrap()) {
        Ok(i) => i,
        Err(i) => i.saturating_sub(1),
    }
    .min(points.len() - 2);

    let seg_len = (cum[idx + 1] - cum[idx]).max(1e-6);
    let t = ((dist - cum[idx]) / seg_len).clamp(0.0, 1.0);
    let pos = points[idx].lerp(points[idx + 1], t);
    let speed = limits[idx].max(0.01); // avoid stalling forever at a zero-speed point
    (pos, speed)
}

#[macroquad::main("Spline Viewer")]
async fn main() {
    let mut waypoints = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(2.0, 3.0, 0.0),
        Vec3::new(5.0, 1.0, 0.0),
        Vec3::new(7.0, 4.0, 0.0),
    ];

    let mut tension = 0.5_f32;
    let mut resolution = 20_u32;
    let mut show_hud = true;
    let mut show_hull = true;
    let mut dragging: Option<usize> = None;

    let mut pipe_radius_m = 1.2_f32;
    let mut max_speed_m_s = 2.5_f32;
    let mut traveled = 0.0_f32; // meters along the arc, resets and loops

    let scale = 60.0;
    let offset = vec2(100.0, 350.0);
    let to_screen = |p: Vec3| vec2(p.x, -p.y) * scale + offset;
    let to_world = |s: Vec2| Vec3::new((s.x - offset.x) / scale, -(s.y - offset.y) / scale, 0.0);

    loop {
        clear_background(Color::from_rgba(18, 18, 22, 255));

        if is_key_pressed(KeyCode::H) {
            show_hud = !show_hud;
        }
        if is_key_pressed(KeyCode::C) {
            show_hull = !show_hull;
        }

        if is_key_down(KeyCode::Up) {
            tension = (tension + 0.01).min(1.0);
        }
        if is_key_down(KeyCode::Down) {
            tension = (tension - 0.01).max(0.0);
        }
        if is_key_down(KeyCode::Right) {
            resolution = (resolution + 1).min(200);
        }
        if is_key_down(KeyCode::Left) {
            resolution = resolution.saturating_sub(1).max(2);
        }
        if is_key_down(KeyCode::W) {
            pipe_radius_m = (pipe_radius_m + 0.01).min(5.0);
        }
        if is_key_down(KeyCode::S) {
            pipe_radius_m = (pipe_radius_m - 0.01).max(0.1);
        }
        if is_key_down(KeyCode::D) {
            max_speed_m_s = (max_speed_m_s + 0.02).min(10.0);
        }
        if is_key_down(KeyCode::A) {
            max_speed_m_s = (max_speed_m_s - 0.02).max(0.1);
        }

        let mouse = Vec2::from(mouse_position());
        if is_mouse_button_pressed(MouseButton::Left) {
            for (i, wp) in waypoints.iter().enumerate() {
                let s = to_screen(*wp);
                if (s - mouse).length() < 10.0 {
                    dragging = Some(i);
                    break;
                }
            }
        }
        if is_mouse_button_released(MouseButton::Left) {
            dragging = None;
        }
        if let Some(i) = dragging {
            waypoints[i] = to_world(mouse);
        }

        let spline = Spline::new(waypoints.clone(), tension, resolution);

        let control_points: Vec<Vec3> = spline
            .curves
            .iter()
            .flat_map(|c| c.controls.iter().copied())
            .collect();

        let hull_input: Vec<Vec2> = waypoints
            .iter()
            .chain(control_points.iter())
            .map(|p| to_screen(*p))
            .collect();
        let hull = convex_hull(&hull_input);

        // curvature-derived speed limits, same formula as Spline::speed_limits
        let speed_limits = spline.speed_limits(pipe_radius_m, max_speed_m_s);
        let (cum, total_len) = arc_lengths(&spline.points);

        // advance the vehicle along the arc by its local speed, looping forever
        let (vehicle_pos, vehicle_speed) =
            sample_at_distance(&spline.points, &cum, &speed_limits, traveled);
        traveled += vehicle_speed * get_frame_time();
        if total_len > 0.0 && traveled > total_len {
            traveled = 0.0; // loop back to the start
        }

        for w in spline.points.windows(2) {
            let a = to_screen(w[0]);
            let b = to_screen(w[1]);
            draw_line(a.x, a.y, b.x, b.y, 2.0, SKYBLUE);
        }

        if show_hull && hull.len() >= 2 {
            for i in 0..hull.len() {
                let a = hull[i];
                let b = hull[(i + 1) % hull.len()];
                draw_line(a.x, a.y, b.x, b.y, 1.5, Color::from_rgba(180, 90, 220, 200));
            }
        }

        for cp in &control_points {
            let s = to_screen(*cp);
            draw_circle(s.x, s.y, 4.0, Color::from_rgba(90, 220, 140, 255));
        }

        for (i, wp) in waypoints.iter().enumerate() {
            let s = to_screen(*wp);
            let color = if dragging == Some(i) { RED } else { YELLOW };
            draw_circle(s.x, s.y, 6.0, color);
            draw_text(&format!("{i}"), s.x + 8.0, s.y - 8.0, 16.0, WHITE);
        }

        // moving vehicle dot, colored by how close it is to max speed
        let speed_frac = (vehicle_speed / max_speed_m_s.max(0.01)).clamp(0.0, 1.0);
        let vehicle_color = Color::new(1.0 - speed_frac, speed_frac, 0.2, 1.0); // red=slow, green=fast
        let vs = to_screen(vehicle_pos);
        draw_circle(vs.x, vs.y, 8.0, vehicle_color);
        draw_circle_lines(vs.x, vs.y, 10.0, 1.5, WHITE);

        if show_hud {
            draw_text(
                &format!("tension: {tension:.2}  (Up/Down, hold)"),
                20.0,
                20.0,
                20.0,
                WHITE,
            );
            draw_text(
                &format!("resolution: {resolution}  (Left/Right, hold)"),
                20.0,
                40.0,
                20.0,
                WHITE,
            );
            draw_text(
                &format!("pipe_radius_m: {pipe_radius_m:.2}  (S/W, hold)"),
                20.0,
                60.0,
                20.0,
                WHITE,
            );
            draw_text(
                &format!("max_speed_m_s: {max_speed_m_s:.2}  (A/D, hold)"),
                20.0,
                80.0,
                20.0,
                WHITE,
            );
            draw_text(
                &format!("vehicle speed: {vehicle_speed:.2} m/s"),
                20.0,
                100.0,
                20.0,
                GREEN,
            );
            draw_text(
                "H: toggle HUD  |  C: toggle hull  |  drag yellow points",
                20.0,
                125.0,
                18.0,
                GRAY,
            );

            if let Some(i) = dragging {
                let p = waypoints[i];
                draw_text(
                    &format!("dragging #{i}: ({:.2}, {:.2}, {:.2})", p.x, p.y, p.z),
                    20.0,
                    145.0,
                    18.0,
                    ORANGE,
                );
            }
        }

        next_frame().await;
    }
}
