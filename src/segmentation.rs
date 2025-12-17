// src/segmentation.rs
use rhdl::prelude::*;
use crate::{
    Point3D, PointState,
    EceCore, ece_step,
    DbscanCore, dbscan_step,
};

#[derive(Copy, Clone, PartialEq, Digital)]
pub enum SegmentationMode {
    Ece,
    Dbscan,
}

impl Default for SegmentationMode {
    fn default() -> Self {
        SegmentationMode::Ece
    }
}

#[derive(Copy, Clone, PartialEq, Digital)]
pub struct SegmentationCore {
    pub mode: SegmentationMode,
    pub ece_core: EceCore,
    pub dbscan_core: DbscanCore,
}

impl SegmentationCore {
    pub fn new_ece(threshold_sq: Bits<32>, eps_sq: Bits<32>, min_pts: Bits<8>) -> Self {
        Self {
            mode: SegmentationMode::Ece,
            ece_core: EceCore::new(threshold_sq),
            dbscan_core: DbscanCore::new(eps_sq, min_pts),
        }
    }

    pub fn new_dbscan(threshold_sq: Bits<32>, eps_sq: Bits<32>, min_pts: Bits<8>) -> Self {
        Self {
            mode: SegmentationMode::Dbscan,
            ece_core: EceCore::new(threshold_sq),
            dbscan_core: DbscanCore::new(eps_sq, min_pts),
        }
    }
}

#[kernel]
pub fn segmentation_step(
    core: SegmentationCore,
    input_point: Point3D,
    input_state: PointState,
) -> (SegmentationCore, PointState) {
    match core.mode {
        SegmentationMode::Ece => {
            let (new_ece, new_state) = ece_step(core.ece_core, input_point, input_state);
            (
                SegmentationCore {
                    mode: core.mode,
                    ece_core: new_ece,
                    dbscan_core: core.dbscan_core,
                },
                new_state,
            )
        }
        SegmentationMode::Dbscan => {
            let (new_dbscan, new_state) = dbscan_step(core.dbscan_core, input_point, input_state);
            (
                SegmentationCore {
                    mode: core.mode,
                    ece_core: core.ece_core,
                    dbscan_core: new_dbscan,
                },
                new_state,
            )
        }
    }
}
