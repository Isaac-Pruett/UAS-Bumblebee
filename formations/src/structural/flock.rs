use glam::Vec3A;

use crate::boids::boid::Boid;

pub struct Flock {
    boid_offset_pairs: Vec<(Boid, Vec3A)>,
    cohesion_gains: (f32, f32),
    ctr: Vec3A,
}

impl Flock {
    pub fn new(boids: Vec<Boid>, kp: f32, kd: f32, offsets: Vec<Vec3A>) -> Self {
        Self {
            boid_offset_pairs: boids.into_iter().zip(offsets).collect(),
            cohesion_gains: (kp, kd),
            ctr: Vec3A::ZERO,
        }
    }
}

impl From<FlockPrimitive> for Flock {
    fn from(prim: FlockPrimitive) -> Self {
        match prim {
            FlockPrimitive::Flock(f) => f,
            FlockPrimitive::Boids(boids, offsets, gains) => Flock {
                boid_offset_pairs: boids.into_iter().zip(offsets).collect(),
                cohesion_gains: gains,
                ctr: Vec3A::ZERO,
            },
            FlockPrimitive::BoidAndOffsets(pairs, gains) => Flock {
                boid_offset_pairs: pairs,
                cohesion_gains: gains,
                ctr: Vec3A::ZERO,
            },
            FlockPrimitive::Swarm(flocks) => Swarm { flocks }.into(),
        }
    }
}

pub struct Swarm {
    flocks: Vec<Flock>,
}
impl Into<Flock> for Swarm {
    fn into(self) -> Flock {
        Flock {
            boid_offset_pairs: self
                .flocks
                .into_iter()
                .flat_map(|f| f.boid_offset_pairs)
                .collect(),
            cohesion_gains: (0.0, 0.0),
            ctr: Vec3A::ZERO,
        }
    }
}

enum FlockPrimitive {
    Flock(Flock),
    Boids(Vec<Boid>, Vec<Vec3A>, (f32, f32)),
    BoidAndOffsets(Vec<(Boid, Vec3A)>, (f32, f32)),
    Swarm(Vec<Flock>),
}
