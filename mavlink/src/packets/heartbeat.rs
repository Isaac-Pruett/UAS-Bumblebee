use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Heartbeat {
    pub elapsed_us: u128,
}
