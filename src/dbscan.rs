use rhdl::prelude::*;
use crate::{Point3D, PointState, distance_squared};

#[derive(Copy, Clone, PartialEq, Digital)]
pub enum DbscanState {
    Idle,
    Scanning,
}

impl Default for DbscanState {
    fn default() -> Self {
        DbscanState::Idle
    }
}

#[derive(Copy, Clone, PartialEq, Digital)]
pub struct DbscanCore {
    pub epsilon_sq: Bits<32>,
    pub min_pts: Bits<8>,
    pub current_cluster: Bits<8>,
    pub state: DbscanState,
    pub seed_point: Point3D,
}

impl DbscanCore {
    pub fn new(eps: Bits<32>, min_pts: Bits<8>) -> Self {
        Self {
            epsilon_sq: eps,
            min_pts,
            current_cluster: bits(0),
            state: DbscanState::Idle,
            seed_point: Point3D::invalid(),
        }
    }
}

#[kernel]
pub fn dbscan_step(
    core: DbscanCore,
    p: Point3D,
    ps: PointState,
) -> (DbscanCore, PointState) {

    match core.state {
        // primul punct → seed
        DbscanState::Idle => {
            if ps == PointState::Unvisited && p.valid {
                (
                    DbscanCore {
                        epsilon_sq: core.epsilon_sq,
                        min_pts: core.min_pts,
                        current_cluster: core.current_cluster,
                        state: DbscanState::Scanning,
                        seed_point: p,
                    },
                    PointState::Clustered(core.current_cluster),
                )
            } else {
                (core, ps)
            }
        }

        // punctele apropiate seed-ului → intră în cluster
        DbscanState::Scanning => {
            if ps == PointState::Unvisited && p.valid {
                let d = distance_squared(core.seed_point, p);

                if d <= core.epsilon_sq {
                    (
                        core,
                        PointState::Clustered(core.current_cluster),
                    )
                } else {
                    (core, ps)
                }
            } else {
                (core, ps)
            }
        }
    }
}