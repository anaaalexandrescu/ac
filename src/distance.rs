// src/distance.rs
use rhdl::prelude::*;

use crate::Point3D;

// Calculeaza distanta patrata intre doua puncte.
// Folosim 32 de biti pentru a evita overflow la inmultire.
#[kernel]
pub fn distance_squared(p1: Point3D, p2: Point3D) -> Bits<32> {
    let dx: Bits<32> = (p1.x.resize::<32>() - p2.x.resize::<32>()).resize();
    let dy: Bits<32> = (p1.y.resize::<32>() - p2.y.resize::<32>()).resize();
    let dz: Bits<32> = (p1.z.resize::<32>() - p2.z.resize::<32>()).resize();

    let dx2: Bits<32> = dx * dx;
    let dy2: Bits<32> = dy * dy;
    let dz2: Bits<32> = dz * dz;

    dx2 + dy2 + dz2
}

// Compara distanta cu un prag (epsilon^2 / threshold^2)
#[kernel]
pub fn is_within_threshold(dist_sq: Bits<32>, threshold_sq: Bits<32>) -> bool {
    dist_sq <= threshold_sq
}
