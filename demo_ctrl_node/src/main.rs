use gilrs::{Axis, Button, Event, EventType, Gilrs};
use serde::Serialize;

#[derive(Debug, Serialize)]
struct GamepadState {
    buttons: [bool; 15],
    axes: [f32; 6],
}

impl GamepadState {
    fn new() -> Self {
        Self {
            buttons: [false; 15],
            axes: [0.0; 6],
        }
    }
}

fn button_index(btn: Button) -> Option<usize> {
    match btn {
        Button::South => Some(0),
        Button::East => Some(1),
        Button::North => Some(2),
        Button::West => Some(3),

        Button::LeftTrigger => Some(4),
        Button::RightTrigger => Some(5),

        Button::Select => Some(6),
        Button::Start => Some(7),
        Button::Mode => Some(8),

        Button::LeftThumb => Some(9),
        Button::RightThumb => Some(10),

        Button::DPadUp => Some(11),
        Button::DPadDown => Some(12),
        Button::DPadLeft => Some(13),
        Button::DPadRight => Some(14),

        _ => None,
    }
}

enum Input {
    Axis(Axis),
    Button(Button),
}

fn axis_index(input: Input) -> Option<usize> {
    match input {
        Input::Axis(axis) => match axis {
            Axis::LeftStickX => Some(0),
            Axis::LeftStickY => Some(1),
            Axis::RightStickX => Some(2),
            Axis::RightStickY => Some(3),
            _ => None,
        },
        Input::Button(btn) => match btn {
            Button::LeftTrigger2 => Some(4),
            Button::RightTrigger2 => Some(5),
            _ => None,
        },
    }
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
    let mut gamepad_state = GamepadState::new();

    for (id, gamepad) in gilrs.gamepads() {
        println!("{:?}: {}", id, gamepad.name());
    }

    loop {
        while let Some(Event { event, .. }) = gilrs.next_event() {
            match event {
                EventType::ButtonPressed(btn, _) => {
                    if let Some(i) = button_index(btn) {
                        gamepad_state.buttons[i] = true;
                    }
                }

                EventType::ButtonReleased(btn, _) => {
                    if let Some(i) = button_index(btn) {
                        gamepad_state.buttons[i] = false;
                    }
                }

                EventType::ButtonChanged(btn, val, _) => {
                    if let Some(i) = axis_index(Input::Button(btn)) {
                        gamepad_state.axes[i] = val;
                    }
                }

                EventType::AxisChanged(axis, val, _) => {
                    if let Some(i) = axis_index(Input::Axis(axis)) {
                        gamepad_state.axes[i] = val;
                    }
                }

                _ => {}
            }

            let payload = rmp_serde::to_vec(&gamepad_state)
                .expect("failed to serialize gamepad state as MessagePack");

            if let Err(err) = publisher.put(payload).await {
                eprintln!("failed to publish gamepad state on '{topic}': {err}");
            }
        }
    }
}
