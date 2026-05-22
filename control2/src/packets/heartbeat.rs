use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct Heartbeat {
    pub elapsed_us: u128,
}
