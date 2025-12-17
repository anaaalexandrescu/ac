use rhdl::prelude::*;
use crate::{Point3D, PointState};

pub const MAX_POINTS: usize = 256;

#[derive(Copy, Clone, PartialEq, Digital)]
pub enum EceState {
    Idle,
    Growing,
}

impl Default for EceState {
    fn default() -> Self {
        EceState::Idle
    }
}

#[derive(Copy, Clone, PartialEq, Digital)]
pub struct EceCore {
    pub threshold_sq: Bits<32>,
    pub current_cluster: Bits<8>,
    pub state: EceState,
    pub seed_point: Point3D,
    pub scan_idx: Bits<8>,
}

impl EceCore {
    pub fn new(threshold_sq: Bits<32>) -> Self {
        Self {
            threshold_sq,
            current_cluster: bits(0),
            state: EceState::Idle,
            seed_point: Point3D::invalid(),
            scan_idx: bits(0),
        }
    }
}

#[kernel]
pub fn ece_step(
    core: EceCore,
    input_point: Point3D,
    input_state: PointState,
) -> (EceCore, PointState) {
    if core.state == EceState::Idle {
        if input_point.valid && (input_state == PointState::Unvisited) {
            (
                EceCore {
                    threshold_sq: core.threshold_sq,
                    current_cluster: core.current_cluster,
                    state: EceState::Growing,
                    seed_point: input_point,
                    scan_idx: bits(0),
                },
                PointState::Clustered(core.current_cluster)
            )
        } else {
            (core, input_state)
        }
    } else {
        (core, input_state)
    }
}