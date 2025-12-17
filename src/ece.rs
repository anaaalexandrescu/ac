use rhdl::prelude::*;
use crate::{Point3D, PointState, distance_squared};

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
}

impl EceCore {
    pub fn new(threshold_sq: Bits<32>) -> Self {
        Self {
            threshold_sq,
            current_cluster: bits(0),
            state: EceState::Idle,
            seed_point: Point3D::invalid(),
        }
    }
}

#[kernel]
pub fn ece_step(
    core: EceCore,
    p: Point3D,
    ps: PointState,
) -> (EceCore, PointState) {

    match core.state {
        // prima dată → seed
        EceState::Idle => {
            if p.valid && ps == PointState::Unvisited {
                (
                    EceCore {
                        threshold_sq: core.threshold_sq,
                        current_cluster: core.current_cluster,
                        state: EceState::Growing,
                        seed_point: p,
                    },
                    PointState::Clustered(core.current_cluster),
                )
            } else {
                (core, ps)
            }
        }

        // punctele apropiate de seed sunt marcate în același cluster
        EceState::Growing => {
            if p.valid && ps == PointState::Unvisited {
                let dist = distance_squared(core.seed_point, p);

                if dist <= core.threshold_sq {
                    (core, PointState::Clustered(core.current_cluster))
                } else {
                    (core, ps)
                }
            } else {
                (core, ps)
            }
        }
    }
}