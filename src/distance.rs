// src/distance.rs
use rhdl::prelude::*;
use crate::Point3D;

#[kernel]
pub fn distance_squared(p1: Point3D, p2: Point3D) -> Bits<32> {
    let dx = p1.x.resize::<32>() - p2.x.resize::<32>();
    let dy = p1.y.resize::<32>() - p2.y.resize::<32>();
    let dz = p1.z.resize::<32>() - p2.z.resize::<32>();

    let dx2 = dx * dx;
    let dy2 = dy * dy;
    let dz2 = dz * dz;

    dx2 + dy2 + dz2
}

#[kernel]
pub fn is_within_threshold(dist_sq: Bits<32>, threshold_sq: Bits<32>) -> bool {
    dist_sq <= threshold_sq
}