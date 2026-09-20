use std::f32;

use mavlink::dialects::common::{COMMAND_INT_DATA, COMMAND_LONG_DATA, MavCmd, MavMessage};
use tokio::sync::{mpsc::Sender, watch::Receiver};
use zenoh::{Session, pubsub::Subscriber};

use crate::connection::{MavlinkHandle, MavlinkRouter, SYSTEM_ID};

pub struct PwmController {
    pub pin: u8,
    mavconn: Sender<MavMessage>,
    pub value: Option<f32>,
    updater: Option<Receiver<Option<PinState>>>,
}

pub struct PinState {
    enabled: bool,
    value: Option<f32>,
}

use std::sync::LazyLock;

static ACTUATOR_COUNT: LazyLock<u8> = LazyLock::new(|| {
    option_env!("ACTUATOR_COUNT")
        .unwrap_or("6")
        .parse()
        .expect("ACTUATOR_COUNT must be a valid u8")
});

impl PwmController {
    /// Pin is 0-indexed
    async fn new(pin: u8, mavconn: MavlinkHandle, session: &Session) -> Self {
        let mut pwm = PwmController {
            pin,
            mavconn,
            value: None,
            updater: None,
        };

        // MavMessage::ACTUATOR_OUTPUT_STATUS == 375
        let message = MavMessage::COMMAND_INT(COMMAND_INT_DATA {
            command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
            param1: 375.0,
            ..Default::default()
        });

        pwm.mavconn.send(message).await.ok();

        pwm.bind_subscriber(session).await;

        return pwm;
    }

    async fn bind_watcher(&mut self, rx: Receiver<Option<PinState>>) {
        self.updater = Some(rx);
    }

    async fn bind_subscriber(&mut self, session: &Session) {
        let pin = self.pin;
        let mavconn = self.mavconn.clone();

        let _ = session
            .declare_subscriber(format!("control/pwm/{pin}"))
            .callback(move |sample| {
                let mavconn = mavconn.clone();

                if let Ok(value) = rmp_serde::from_slice::<f32>(&sample.payload().to_bytes()) {
                    tokio::spawn(async move {
                        let msg = MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
                            command: MavCmd::MAV_CMD_DO_SET_ACTUATOR,
                            param1: if pin % 6 == 0 { value } else { f32::NAN },
                            param2: if pin % 6 == 1 { value } else { f32::NAN },
                            param3: if pin % 6 == 2 { value } else { f32::NAN },
                            param4: if pin % 6 == 3 { value } else { f32::NAN },
                            param5: if pin % 6 == 4 { value } else { f32::NAN },
                            param6: if pin % 6 == 5 { value } else { f32::NAN },
                            param7: ((pin * 37) >> 8) as f32,
                            target_system: *SYSTEM_ID.get().unwrap_or(&0),
                            target_component: 1,
                            confirmation: 0,
                        });

                        let _ = mavconn.send(msg).await;
                    });
                }
            })
            .background()
            .await
            .unwrap_or_else(|_| panic!("Failed to initialize pwm subscriber on {pin}"));
    }
}

pub async fn register_pwm_watchers(router: &mut MavlinkRouter) -> Vec<Receiver<Option<PinState>>> {
    let mut pwm_watchers = Vec::new();
    for pin in 0..*ACTUATOR_COUNT {
        let state_recv = router.subscribe_watch(move |msg| {
            let pin = pin.clone();
            match msg {
                MavMessage::ACTUATOR_OUTPUT_STATUS(data) => Some(PinState {
                    enabled: (data.active & (1u32 << pin)) != 0,

                    value: data.actuator.get(pin as usize).copied(),
                }),
                _ => None,
            }
        });
        pwm_watchers.push(state_recv);
    }
    pwm_watchers
}

pub async fn register_all_pwm_subs(
    session: &Session,
    handle: MavlinkHandle,
    watchers: Vec<Receiver<Option<PinState>>>,
) -> Vec<PwmController> {
    let mut v = Vec::new();

    for i in 0..*ACTUATOR_COUNT {
        let handle = handle.clone();
        let mut p = PwmController::new(i, handle, session).await;
        p.bind_watcher(watchers[i as usize].clone()).await;
        v.push(p);
    }

    v
}
