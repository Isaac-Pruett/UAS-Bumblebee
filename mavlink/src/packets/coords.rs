use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct LinearDistance {
    mm: i32,
}

impl LinearDistance {
    pub fn from_mm(mm: i32) -> Self {
        Self { mm }
    }

    pub fn from_meters(m: f32) -> Self {
        Self {
            mm: (m * 1000.0).round() as i32,
        }
    }

    pub fn as_mm(&self) -> i32 {
        self.mm
    }

    pub fn as_meters(&self) -> f32 {
        self.mm as f32 / 1000.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AngularDistance {
    deg: f32,
}

impl AngularDistance {
    pub fn from_deg(deg: f32) -> Self {
        Self { deg }
    }

    pub fn from_deg_e7(v: i32) -> Self {
        Self {
            deg: v as f32 / 1e7,
        }
    }

    pub fn as_deg(&self) -> f32 {
        self.deg
    }

    pub fn as_deg_e7(&self) -> i32 {
        (self.deg * 1e7).round() as i32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Coords {
    pub lat_deg_e7: i32,
    pub lon_deg_e7: i32,
    pub alt_mm: i32,
}

impl Coords {
    const EARTH_RADIUS_M: f32 = 6_371_000.0;
    const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0;

    pub fn new(lat: i32, lon: i32, alt: i32) -> Self {
        Self {
            lat_deg_e7: lat,
            lon_deg_e7: lon,
            alt_mm: alt,
        }
    }

    pub fn lat_deg(&self) -> f32 {
        self.lat_deg_e7 as f32 / 1e7
    }

    pub fn lon_deg(&self) -> f32 {
        self.lon_deg_e7 as f32 / 1e7
    }

    pub fn alt_m(&self) -> f32 {
        self.alt_mm as f32 / 1000.0
    }

    pub fn move_north(&self, dist: LinearDistance) -> Self {
        let delta_deg = dist.as_meters() / Self::EARTH_RADIUS_M / Self::DEG_TO_RAD;

        Self {
            lat_deg_e7: self.lat_deg_e7 + AngularDistance::from_deg(delta_deg).as_deg_e7(),
            lon_deg_e7: self.lon_deg_e7,
            alt_mm: self.alt_mm,
        }
    }

    pub fn move_east(&self, dist: LinearDistance) -> Self {
        let lat_rad = self.lat_deg().to_radians();

        let delta_deg =
            dist.as_meters() / (Self::EARTH_RADIUS_M * lat_rad.cos()) / Self::DEG_TO_RAD;

        Self {
            lat_deg_e7: self.lat_deg_e7,
            lon_deg_e7: self.lon_deg_e7 + AngularDistance::from_deg(delta_deg).as_deg_e7(),
            alt_mm: self.alt_mm,
        }
    }

    pub fn move_up(&self, dist: LinearDistance) -> Self {
        Self {
            lat_deg_e7: self.lat_deg_e7,
            lon_deg_e7: self.lon_deg_e7,
            alt_mm: self.alt_mm + dist.as_mm(),
        }
    }

    pub fn offset(&self, north: LinearDistance, east: LinearDistance, up: LinearDistance) -> Self {
        self.move_north(north).move_east(east).move_up(up)
    }
}
