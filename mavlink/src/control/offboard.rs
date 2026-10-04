use std::{
    sync::{Arc, LazyLock, Mutex},
    time::{Duration, Instant},
};

use mavlink::dialects::common::{
    COMMAND_LONG_DATA, MavAutopilot, MavCmd, MavMessage, MavModeFlag, MavResult,
};
use tokio::sync::mpsc;

use crate::{
    connection::{MavlinkHandle, SYSTEM_ID, with_mavlink_router},
    get_session,
    telem::{AUTOPILOT, FlightMode, autopilot_mode, subscribe_autopilot_mode},
};

const STREAM_GAP: Duration = Duration::from_millis(300);
const WARMUP: Duration = Duration::from_millis(1200);

#[derive(Default)]
struct Priming {
    active: bool,
    since: Option<Instant>,
    last: Option<Instant>,
}

impl Priming {
    fn record(&mut self, now: Instant) {
        if self
            .last
            .is_none_or(|last| now.duration_since(last) > STREAM_GAP)
        {
            self.since = Some(now);
        }
        self.last = Some(now);
    }

    fn ready(&self, now: Instant) -> bool {
        self.last
            .is_some_and(|last| now.duration_since(last) <= STREAM_GAP)
            && self
                .since
                .is_some_and(|since| now.duration_since(since) >= WARMUP)
    }
}

static PRIMING: LazyLock<Mutex<Priming>> = LazyLock::new(|| Mutex::new(Priming::default()));

pub fn is_priming() -> bool {
    PRIMING.lock().unwrap().active
}

pub fn record_setpoint(packet: &mavlink::dialects::common::SET_POSITION_TARGET_LOCAL_NED_DATA) {
    let mask = packet.type_mask.bits();
    let groups = [
        (mask & 7, [packet.x, packet.y, packet.z]),
        ((mask >> 3) & 7, [packet.vx, packet.vy, packet.vz]),
        ((mask >> 6) & 7, [packet.afx, packet.afy, packet.afz]),
    ];
    if !groups
        .iter()
        .any(|(ignored, values)| *ignored == 0 && values.iter().all(|v| v.is_finite()))
        || groups.iter().any(|(ignored, values)| {
            *ignored != 7 && (*ignored != 0 || !values.iter().all(|v| v.is_finite()))
        })
        || mask & (1 << 9) != 0
    {
        return;
    }
    let mut state = PRIMING.lock().unwrap();
    if state.active {
        state.record(Instant::now());
    }
}

struct PrimingGuard;
impl Drop for PrimingGuard {
    fn drop(&mut self) {
        *PRIMING.lock().unwrap() = Priming::default();
    }
}

pub fn register_ack_watcher() -> mpsc::Receiver<MavResult> {
    with_mavlink_router(|router| {
        router.subscribe(|msg| match msg {
            MavMessage::COMMAND_ACK(data) if data.command == MavCmd::MAV_CMD_DO_SET_MODE => {
                Some(data.result)
            }
            _ => None,
        })
    })
}

fn offboard_command(target_system: u8) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        target_system,
        target_component: 1,
        command: MavCmd::MAV_CMD_DO_SET_MODE,
        param1: MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.bits() as f32,
        param2: 6.0, // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        ..Default::default()
    })
}

async fn enter_offboard(
    handle: &MavlinkHandle,
    ack: &mut mpsc::Receiver<MavResult>,
) -> Result<(), String> {
    if AUTOPILOT.get() != Some(&MavAutopilot::MAV_AUTOPILOT_PX4) {
        return Err("Offboard requests require a detected PX4 autopilot".into());
    }
    let target = *SYSTEM_ID.get().ok_or("No vehicle heartbeat received")?;
    let mut modes = subscribe_autopilot_mode();
    modes.borrow_and_update();
    tokio::time::timeout(Duration::from_secs(2), modes.changed())
        .await
        .map_err(|_| "No fresh vehicle heartbeat".to_string())?
        .map_err(|_| "Flight mode stream closed".to_string())?;
    let mode = (*modes.borrow_and_update()).ok_or("No flight mode received")?;
    if !mode.armed {
        return Err("Arm and launch before requesting Offboard".into());
    }
    if mode.mode == FlightMode::Offboard {
        return Ok(());
    }

    // Only the explicit request opens the pre-Offboard setpoint gate. Reset it
    // on every exit, including timeout/cancellation; never auto-reenter Offboard.
    *PRIMING.lock().unwrap() = Priming {
        active: true,
        ..Default::default()
    };
    let _guard = PrimingGuard;
    let transition = async {
        loop {
            if !autopilot_mode().is_some_and(|mode| mode.armed) {
                return Err("Vehicle disarmed during Offboard preparation".into());
            }
            if PRIMING.lock().unwrap().ready(Instant::now()) {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        while ack.try_recv().is_ok() {}
        handle
            .send(offboard_command(target))
            .await
            .map_err(|_| "MAVLink connection closed".to_string())?;
        modes.borrow_and_update();
        modes
            .changed()
            .await
            .map_err(|_| "Flight mode stream closed".to_string())?;
        loop {
            let state = *modes.borrow_and_update();
            if let Some(state) = state {
                if !state.armed {
                    return Err("Vehicle disarmed during Offboard transition".into());
                }
                if state.mode == FlightMode::Offboard {
                    return Ok(());
                }
            }
            tokio::select! {
                result = ack.recv() => match result {
                    Some(MavResult::MAV_RESULT_ACCEPTED | MavResult::MAV_RESULT_IN_PROGRESS) => {},
                    Some(result) => return Err(format!("PX4 rejected Offboard: {result:?}")),
                    None => return Err("MAVLink ACK stream closed".into()),
                },
                changed = modes.changed() => {
                    changed.map_err(|_| "Flight mode stream closed".to_string())?;
                }
            }
        }
    };
    tokio::time::timeout(Duration::from_secs(7), transition).await
        .map_err(|_| "Offboard timed out: need a continuous valid setpoint stream and PX4 heartbeat confirmation".to_string())?
}

pub async fn register(handle: &MavlinkHandle, ack: mpsc::Receiver<MavResult>) {
    let handle = handle.clone();
    let ack = Arc::new(tokio::sync::Mutex::new(ack));
    get_session()
        .declare_queryable("control/offboard")
        .callback(move |query| {
            let handle = handle.clone();
            let ack = Arc::clone(&ack);
            tokio::spawn(async move {
                let result = match ack.try_lock() {
                    Ok(mut ack) => enter_offboard(&handle, &mut ack).await,
                    Err(_) => Err("An Offboard request is already in progress".into()),
                };
                match result {
                    Ok(()) => {
                        println!("Offboard confirmed by PX4 heartbeat");
                        let _ = query.reply("control/offboard", "Offboard confirmed").await;
                    }
                    Err(error) => {
                        eprintln!("Offboard request failed: {error}");
                        let _ = query.reply_err(error).await;
                    }
                }
            });
        })
        .background()
        .await
        .expect("failed to initialize Offboard queryable");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn warmup_requires_continuous_recent_setpoints() {
        let start = Instant::now();
        let mut state = Priming::default();
        assert!(!state.ready(start));
        for i in 0..=12 {
            state.record(start + Duration::from_millis(i * 100));
        }
        assert!(state.ready(start + WARMUP));
        assert!(!state.ready(start + WARMUP + STREAM_GAP + Duration::from_millis(1)));
        state.record(start + Duration::from_secs(2));
        assert!(!state.ready(start + Duration::from_secs(2)));
    }

    #[test]
    fn mode_command_targets_px4_offboard() {
        let MavMessage::COMMAND_LONG(command) = offboard_command(7) else {
            panic!()
        };
        assert_eq!(command.target_system, 7);
        assert_eq!(command.target_component, 1);
        assert_eq!(command.command, MavCmd::MAV_CMD_DO_SET_MODE);
        assert_eq!(command.param1, 1.0);
        assert_eq!(command.param2, 6.0);
    }
}
