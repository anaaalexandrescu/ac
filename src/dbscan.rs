// src/dbscan.rs
use rhdl::prelude::*;
use crate::{Point3D, PointState, distance_squared, is_within_threshold};

pub const MAX_POINTS: usize = 256;

#[derive(Copy, Clone, PartialEq, Digital)]
pub enum DbscanState {
    Idle,
    ScanNeighbors,
    Decide,
    Done,
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
    pub scan_idx: Bits<8>,
    pub neighbor_count: Bits<8>,
    pub seed_point: Point3D,
}

impl DbscanCore {
    pub fn new(eps_sq: Bits<32>, min_pts: Bits<8>) -> Self {
        Self {
            epsilon_sq: eps_sq,
            min_pts,
            current_cluster: bits(0),
            state: DbscanState::Idle,
            scan_idx: bits(0),
            neighbor_count: bits(0),
            seed_point: Point3D::invalid(),
        }
    }
}

#[kernel]
pub fn dbscan_step(
    core: DbscanCore,
    input_point: Point3D,
    input_state: PointState,
) -> (DbscanCore, PointState) {
    match core.state {
        DbscanState::Idle => {
            if input_point.valid && (input_state == PointState::Unvisited) {
                (
                    DbscanCore {
                        epsilon_sq: core.epsilon_sq,
                        min_pts: core.min_pts,
                        current_cluster: core.current_cluster,
                        state: DbscanState::ScanNeighbors,
                        scan_idx: bits(0),
                        neighbor_count: bits(0),
                        seed_point: input_point,
                    },
                    input_state,
                )
            } else {
                (core, input_state)
            }
        }

        DbscanState::ScanNeighbors => {
            let d = distance_squared(core.seed_point, input_point);
            let new_count = if is_within_threshold(d, core.epsilon_sq) {
                core.neighbor_count + bits(1)
            } else {
                core.neighbor_count
            };

            let next_idx = core.scan_idx + bits(1);

            if next_idx == bits((MAX_POINTS - 1) as u128) {
                (
                    DbscanCore {
                        epsilon_sq: core.epsilon_sq,
                        min_pts: core.min_pts,
                        current_cluster: core.current_cluster,
                        state: DbscanState::Decide,
                        scan_idx: next_idx,
                        neighbor_count: new_count,
                        seed_point: core.seed_point,
                    },
                    input_state,
                )
            } else {
                (
                    DbscanCore {
                        epsilon_sq: core.epsilon_sq,
                        min_pts: core.min_pts,
                        current_cluster: core.current_cluster,
                        state: DbscanState::ScanNeighbors,
                        scan_idx: next_idx,
                        neighbor_count: new_count,
                        seed_point: core.seed_point,
                    },
                    input_state,
                )
            }
        }

        DbscanState::Decide => {
            let new_state = if core.neighbor_count >= core.min_pts {
                PointState::Clustered(core.current_cluster)
            } else {
                PointState::Noise
            };

            (
                DbscanCore {
                    epsilon_sq: core.epsilon_sq,
                    min_pts: core.min_pts,
                    current_cluster: core.current_cluster + bits(1),
                    state: DbscanState::Idle,
                    scan_idx: core.scan_idx,
                    neighbor_count: core.neighbor_count,
                    seed_point: core.seed_point,
                },
                new_state,
            )
        }

        DbscanState::Done => (core, input_state),
    }
}
