use serde::{Deserialize, Serialize};

use crate::packets::coords::{Coords, LinearDistance};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Waypoint {
    coord: Coords,
    acceptance_rad: LinearDistance,
    speed: f32, // m/s
}

impl Waypoint {
    pub fn new(target: Coords, acceptance_radius: LinearDistance, speed: f32) -> Self {
        Waypoint {
            coord: target,
            acceptance_rad: acceptance_radius,
            speed,
        }
    }
}
