use serde::{Deserialize, Serialize};

use crate::packets::coords::{Coords, LinearDistance};

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Waypoint {
    pub coord: Coords,
    pub acceptance_rad: LinearDistance,
    pub speed: f32,   // m/s
    pub heading: f32, // rad
}

impl Waypoint {
    pub fn new(
        target: Coords,
        acceptance_radius: LinearDistance,
        speed: f32,
        heading: f32,
    ) -> Self {
        Waypoint {
            coord: target,
            acceptance_rad: acceptance_radius,
            speed,
            heading,
        }
    }
}
