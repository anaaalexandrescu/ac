// src/dbscan.rs
use rhdl::prelude::*;
use crate::{Point3D, PointState};
use crate::distance::distance_squared;

#[derive(Copy, Clone, PartialEq, Digital)]
pub enum DbscanState {
    Idle,
    CountingNeighbors,
    Expanding,
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
    pub neighbor_count: Bits<8>,
}

impl DbscanCore {
    pub fn new(eps: Bits<32>, min_pts: Bits<8>) -> Self {
        Self {
            epsilon_sq: eps,
            min_pts,
            current_cluster: bits(0),
            state: DbscanState::Idle,
            seed_point: Point3D {
                x: bits(0),
                y: bits(0),
                z: bits(0),
                valid: false,
            },
            neighbor_count: bits(0),
        }
    }
    
    pub fn next_cluster(&self) -> Self {
        Self {
            epsilon_sq: self.epsilon_sq,
            min_pts: self.min_pts,
            current_cluster: self.current_cluster + bits(1),
            state: DbscanState::Idle,
            seed_point: Point3D {
                x: bits(0),
                y: bits(0),
                z: bits(0),
                valid: false,
            },
            neighbor_count: bits(0),
        }
    }
}

#[kernel]
pub fn dbscan_step(
    core: DbscanCore,
    p: Point3D,
    ps: PointState,
) -> (DbscanCore, PointState, bool) {

    match core.state {
        DbscanState::Idle => {
            if ps == PointState::Unvisited && p.valid {
                // gasit seed - marcheaza ca vizitat si numara vecini
                (
                    DbscanCore {
                        epsilon_sq: core.epsilon_sq,
                        min_pts: core.min_pts,
                        current_cluster: core.current_cluster,
                        state: DbscanState::CountingNeighbors,
                        seed_point: p,
                        neighbor_count: bits(1), // seed se numara pe sine
                    },
                    PointState::Visited,
                    true,
                )
            } else {
                (core, ps, false)
            }
        }

        DbscanState::CountingNeighbors => {
            // numara vecinii seed-ului
            if p.valid && p != core.seed_point {
                let d = distance_squared(core.seed_point, p);
                
                if d <= core.epsilon_sq {
                    (
                        DbscanCore {
                            epsilon_sq: core.epsilon_sq,
                            min_pts: core.min_pts,
                            current_cluster: core.current_cluster,
                            state: core.state,
                            seed_point: core.seed_point,
                            neighbor_count: core.neighbor_count + bits(1),
                        },
                        ps,
                        false,
                    )
                } else {
                    (core, ps, false)
                }
            } else {
                (core, ps, false)
            }
        }

        DbscanState::Expanding => {
            (core, ps, false)
        }
    }
}

#[kernel]
pub fn dbscan_expand(
    epsilon_sq: Bits<32>,
    current_cluster: Bits<8>,
    test_point: Point3D,
    test_state: PointState,
    reference_point: Point3D,
    reference_state: PointState,
) -> (PointState, bool) {
    
    let ref_in_cluster = match reference_state {
        PointState::Clustered(id) => id == current_cluster,
        _ => false,
    };
    
    let can_expand = test_point.valid 
                     && test_state == PointState::Unvisited
                     && reference_point.valid
                     && ref_in_cluster;
    
    if can_expand {
        let dist = distance_squared(test_point, reference_point);
        
        if dist <= epsilon_sq {
            (PointState::Clustered(current_cluster), true)
        } else {
            (test_state, false)
        }
    } else {
        (test_state, false)
    }
}

#[kernel]
pub fn has_min_neighbors(core: DbscanCore) -> bool {
    core.neighbor_count >= core.min_pts
}

#[kernel]
pub fn start_expanding(core: DbscanCore) -> DbscanCore {
    DbscanCore {
        epsilon_sq: core.epsilon_sq,
        min_pts: core.min_pts,
        current_cluster: core.current_cluster,
        state: DbscanState::Expanding,
        seed_point: core.seed_point,
        neighbor_count: core.neighbor_count,
    }
}

#[kernel]
pub fn mark_seed_as_noise(core: DbscanCore) -> DbscanCore {
    DbscanCore {
        epsilon_sq: core.epsilon_sq,
        min_pts: core.min_pts,
        current_cluster: core.current_cluster,
        state: DbscanState::Idle,
        seed_point: Point3D {
            x: bits(0),
            y: bits(0),
            z: bits(0),
            valid: false,
        },
        neighbor_count: bits(0),
    }
}