// src/lib.rs
use rhdl::prelude::*;

pub mod point;
pub mod distance;
pub mod ece;
pub mod dbscan;
pub mod clustering_core;
pub mod segmentation;

pub use point::{Point3D, PointState};
pub use distance::{distance_squared, is_within_threshold};
pub use ece::{EceCore, EceState, ece_step, ece_expand};
pub use dbscan::{DbscanCore, DbscanState, dbscan_step, dbscan_expand, has_min_neighbors, start_expanding, mark_seed_as_noise};
pub use clustering_core::{ClusteringCore, ClusteringState, process_point};
pub use segmentation::{SegmentationCore, SegmentationMode, segmentation_step};