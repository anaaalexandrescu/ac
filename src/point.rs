use rhdl::prelude::*;

#[derive(Copy, Clone, PartialEq, Digital, Default, Debug)]  
pub struct Point3D {
    pub x: Bits<16>,
    pub y: Bits<16>,
    pub z: Bits<16>,
    pub valid: bool,
}

impl Point3D {
    pub fn new(x: Bits<16>, y: Bits<16>, z: Bits<16>) -> Self {
        Self { x, y, z, valid: true }
    }

    pub fn invalid() -> Self {
        Self {
            x: bits(0),
            y: bits(0),
            z: bits(0),
            valid: false,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Digital, Debug)]  
pub enum PointState {
    Unvisited,
    Visited,
    Clustered(Bits<8>),
    Noise,
}

impl Default for PointState {
    fn default() -> Self {
        PointState::Unvisited
    }
}