use std::sync::OnceLock;

static DRONE_NAME: OnceLock<String> = OnceLock::new();

pub fn topic(endpoint: &str) -> String {
    let drone_name = DRONE_NAME.get_or_init(|| {
        let name = std::env::var("DRONE_NAME")
            .unwrap_or_else(|_| "BINGUS_THE_DRONE_THAT_SHOULD_BE_RENAMED".into());
        println!("control2: set drone name to: {name}");
        name
    });

    format!("{drone_name}/control/{endpoint}")
}
