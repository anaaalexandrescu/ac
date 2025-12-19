// src/ece.rs
use rhdl::prelude::*;
use crate::{Point3D, PointState};
use crate::distance::distance_squared;

#[derive(Copy, Clone, PartialEq, Digital)]
pub enum EceState {
    Idle,           // caută seed nou
    Growing,        // crește cluster-ul curent
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
            seed_point: Point3D {
                x: bits(0),
                y: bits(0),
                z: bits(0),
                valid: false,
            },
        }
    }
    
    pub fn next_cluster(&self) -> Self {
        Self {
            threshold_sq: self.threshold_sq,
            current_cluster: self.current_cluster + bits(1),
            state: EceState::Idle,
            seed_point: Point3D {
                x: bits(0),
                y: bits(0),
                z: bits(0),
                valid: false,
            },
        }
    }
}

// Kernel principal - găsește seed
#[kernel]
pub fn ece_step(
    core: EceCore,
    p: Point3D,
    ps: PointState,
) -> (EceCore, PointState, bool) {
    match core.state {
        EceState::Idle => {
            if p.valid && ps == PointState::Unvisited {
                // Am găsit seed - treci la Growing
                (
                    EceCore {
                        threshold_sq: core.threshold_sq,
                        current_cluster: core.current_cluster,
                        state: EceState::Growing,
                        seed_point: p,
                    },
                    PointState::Clustered(core.current_cluster),
                    true,
                )
            } else {
                (core, ps, false)
            }
        }

        EceState::Growing => {
            // Nu mai face nimic aici - expansion se face cu ece_expand
            (core, ps, false)
        }
    }
}

// Kernel pentru region growing - compară un punct nevizitat cu unul deja clustered
#[kernel]
pub fn ece_expand(
    threshold_sq: Bits<32>,
    current_cluster: Bits<8>,
    test_point: Point3D,
    test_state: PointState,
    reference_point: Point3D,
    reference_state: PointState,
) -> (PointState, bool) {
    
    // Verifică dacă reference_point e în cluster-ul curent
    let ref_in_cluster = match reference_state {
        PointState::Clustered(id) => id == current_cluster,
        _ => false,
    };
    
    // Test_point trebuie să fie unvisited
    let can_expand = test_point.valid 
                     && test_state == PointState::Unvisited
                     && reference_point.valid
                     && ref_in_cluster;
    
    if can_expand {
        let dist = distance_squared(test_point, reference_point);
        
        if dist <= threshold_sq {
            (PointState::Clustered(current_cluster), true)
        } else {
            (test_state, false)
        }
    } else {
        (test_state, false)
    }
}