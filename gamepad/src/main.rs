use std::time::Duration;

use gilrs::{Axis, Button, Event, EventType, Gilrs};
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Default)]
struct GamepadState {
    x: bool,
    a: bool,
    b: bool,
    y: bool,
    left_bumper: bool,
    right_bumper: bool,
    select: bool,
    start: bool,
    mode: bool,
    left_stick_button: bool,
    right_stick_button: bool,
    dpad_up: bool,
    dpad_down: bool,
    dpad_left: bool,
    dpad_right: bool,
    left_stick_x: f32,
    left_stick_y: f32,
    right_stick_x: f32,
    right_stick_y: f32,
    left_trigger: f32,
    right_trigger: f32,
}

impl GamepadState {
    fn set_button(&mut self, button: Button, pressed: bool) {
        match button {
            Button::South => self.b = pressed,
            Button::East => self.a = pressed,
            Button::North => self.x = pressed,
            Button::West => self.y = pressed,
            Button::LeftTrigger => self.left_bumper = pressed,
            Button::RightTrigger => self.right_bumper = pressed,
            Button::Select => self.select = pressed,
            Button::Start => self.start = pressed,
            Button::Mode => self.mode = pressed,
            Button::LeftThumb => self.left_stick_button = pressed,
            Button::RightThumb => self.right_stick_button = pressed,
            Button::DPadUp => self.dpad_up = pressed,
            Button::DPadDown => self.dpad_down = pressed,
            Button::DPadLeft => self.dpad_left = pressed,
            Button::DPadRight => self.dpad_right = pressed,
            _ => {}
        }
    }

    fn set_axis(&mut self, input: Input, value: f32) {
        match input {
            Input::Axis(Axis::LeftStickX) => self.left_stick_x = value,
            Input::Axis(Axis::LeftStickY) => self.left_stick_y = value,
            Input::Axis(Axis::RightStickX) => self.right_stick_x = value,
            Input::Axis(Axis::RightStickY) => self.right_stick_y = value,
            Input::Button(Button::LeftTrigger2) => self.left_trigger = value,
            Input::Button(Button::RightTrigger2) => self.right_trigger = value,
            _ => {}
        }
    }
}

enum Input {
    Axis(Axis),
    Button(Button),
}

#[tokio::main]
async fn main() {
    let config_path = std::env::var("ZENOH_CONFIG").expect("ZENOH_CONFIG not set");
    let cfg = zenoh::Config::from_file(&config_path).expect("failed to load ZENOH_CONFIG file");

    let session = zenoh::open(cfg).await.expect("failed to initialize zenoh");
    let topic = std::env::var("GAMEPAD_TOPIC").unwrap_or_else(|_| "control/gamepad".into());
    let publisher = session
        .declare_publisher(&topic)
        .await
        .expect("failed to declare gamepad publisher");

    println!("publishing gamepad state on '{topic}'");

    let mut gilrs = Gilrs::new().expect("failed to initialize gamepad input");
    let mut gamepad_state = GamepadState::default();

    for (id, gamepad) in gilrs.gamepads() {
        println!("{:?}: {}", id, gamepad.name());
    }

    loop {
        while let Some(Event { event, .. }) = gilrs.next_event() {
            match event {
                EventType::ButtonPressed(button, _) => {
                    gamepad_state.set_button(button, true);
                }

                EventType::ButtonReleased(button, _) => {
                    gamepad_state.set_button(button, false);
                }

                EventType::ButtonChanged(button, value, _) => {
                    gamepad_state.set_axis(Input::Button(button), value);
                }

                EventType::AxisChanged(axis, value, _) => {
                    gamepad_state.set_axis(Input::Axis(axis), value);
                }

                _ => {}
            }

            let payload = rmp_serde::to_vec(&gamepad_state)
                .expect("failed to serialize gamepad state as MessagePack");

            if let Err(err) = publisher.put(payload).await {
                eprintln!("failed to publish gamepad state on '{topic}': {err}");
            }
        }
        // println!("GP: {:?}", gamepad_state);
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}
