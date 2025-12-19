// src/clustering_core.rs
use rhdl::prelude::*;
use crate::{Point3D, PointState};
use crate::distance::{distance_squared, is_within_threshold};

// Restul fișierului rămâne la fel

#[derive(Copy, Clone, PartialEq, Digital)]
pub struct ClusteringCore {
    pub threshold_sq: Bits<32>,
    pub current_cluster_id: Bits<8>,
    pub seed_point: Point3D,
    pub state: ClusteringState,
}

#[derive(Copy, Clone, PartialEq, Digital)]
pub enum ClusteringState {
    Idle,
    GrowingRegion,
}

impl Default for ClusteringState {
    fn default() -> Self {
        ClusteringState::Idle
    }
}

impl ClusteringCore {
    pub fn new(threshold_sq: Bits<32>) -> Self {
        Self {
            threshold_sq,
            current_cluster_id: bits(0),
            seed_point: Point3D::invalid(),
            state: ClusteringState::Idle,
        }
    }
    
    pub fn next_cluster(&self) -> Self {
        Self {
            threshold_sq: self.threshold_sq,
            current_cluster_id: self.current_cluster_id + bits(1),
            seed_point: Point3D::invalid(),
            state: ClusteringState::Idle,
        }
    }
}

#[kernel]
pub fn process_point(
    core: ClusteringCore,
    input_point: Point3D,
    point_state: PointState,
) -> (ClusteringCore, PointState, bool) {  // bool = punct modificat
    
    match core.state {
        ClusteringState::Idle => {
            if (point_state == PointState::Unvisited) && input_point.valid {
                (
                    ClusteringCore {
                        threshold_sq: core.threshold_sq,
                        current_cluster_id: core.current_cluster_id,
                        seed_point: input_point,
                        state: ClusteringState::GrowingRegion,
                    },
                    PointState::Clustered(core.current_cluster_id),
                    true,
                )
            } else {
                (core, point_state, false)
            }
        }

        ClusteringState::GrowingRegion => {
            if (point_state == PointState::Unvisited) && input_point.valid {
                let dist_sq = distance_squared(core.seed_point, input_point);
                if is_within_threshold(dist_sq, core.threshold_sq) {
                    (core, PointState::Clustered(core.current_cluster_id), true)
                } else {
                    (core, point_state, false)
                }
            } else {
                (core, point_state, false)
            }
        }
    }
}