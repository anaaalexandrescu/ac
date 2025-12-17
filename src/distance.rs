use rhdl::prelude::*;
use crate::Point3D;

#[kernel]
pub fn distance_squared(p1: Point3D, p2: Point3D) -> Bits<32> {
    if !p1.valid || !p2.valid {
        return bits::<32>(0xFFFFFFFF);
    }

    let dx: Bits<16> = if p1.x > p2.x { p1.x - p2.x } else { p2.x - p1.x };
    let dy: Bits<16> = if p1.y > p2.y { p1.y - p2.y } else { p2.y - p1.y };
    let dz: Bits<16> = if p1.z > p2.z { p1.z - p2.z } else { p2.z - p1.z };

    let dx32: Bits<32> = bits(dx.raw() as u128);
    let dy32: Bits<32> = bits(dy.raw() as u128);
    let dz32: Bits<32> = bits(dz.raw() as u128);

    (dx32 * dx32) + (dy32 * dy32) + (dz32 * dz32)
}

#[kernel]
pub fn is_within_threshold(dist_sq: Bits<32>, eps_sq: Bits<32>) -> bool {
    dist_sq <= eps_sq
}