// src/lib.rs
use rhdl::prelude::*;

pub mod point;
pub mod distance;
pub mod ece;
pub mod dbscan;
pub mod clustering_core;
pub mod segmentation;

// Re-exports, astfel incat benches/tests sa poata face:
// use lidar_clustering_hdl::*;
pub use point::{Point3D, PointState};
pub use distance::{distance_squared, is_within_threshold};
pub use ece::{EceCore, EceState, ece_step, MAX_POINTS as ECE_MAX_POINTS};
pub use dbscan::{DbscanCore, DbscanState, dbscan_step, MAX_POINTS as DBSCAN_MAX_POINTS};
pub use clustering_core::{ClusteringCore, ClusteringState, process_point};
pub use segmentation::{SegmentationCore, SegmentationMode, segmentation_step};
