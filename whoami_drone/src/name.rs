use std::sync::OnceLock;

pub static DRONE_NAME: OnceLock<String> = OnceLock::new();

pub fn topic(endpoint: &str) -> String {
    let drone_name = DRONE_NAME.get_or_init(|| {
        let name = std::env::var("DRONE_NAME").expect("!!!   THIS DRONE WAS NOT NAMED   !!!");
        println!("whoami_drone: set drone name to: {name}");
        name
    });

    format!("{drone_name}/{endpoint}")
}
