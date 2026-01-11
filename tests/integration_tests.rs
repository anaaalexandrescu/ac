use lidar_clustering_hdl::*;
use rhdl::prelude::*;

fn pt(x: u16, y: u16, z: u16) -> Point3D {
    Point3D::new(bits::<16>(x as u128), bits::<16>(y as u128), bits::<16>(z as u128))
}

fn tiny_cloud() -> Vec<Point3D> {
    vec![
        pt(0, 0, 0),
        pt(0, 3, 0),
        pt(4, 0, 0),
        pt(120, 120, 0),
    ]
}

fn summarize_states(states: &[PointState]) -> (usize, usize, usize, usize) {
    let mut clustered = 0;
    let mut noise = 0;
    let mut visited = 0;
    let mut unvisited = 0;

    for state in states {
        match state {
            PointState::Clustered(_) => clustered += 1,
            PointState::Noise => noise += 1,
            PointState::Visited => visited += 1,
            PointState::Unvisited => unvisited += 1,
        }
    }

    (clustered, noise, visited, unvisited)
}

fn run_ece_flow(
    points: &[Point3D],
    threshold_sq: Bits<32>,
    max_clusters: usize,
    max_passes: usize,
) -> (usize, usize, usize, usize) {
    let mut core = SegmentationCore::new_ece(threshold_sq, bits::<32>(0), bits::<8>(0));
    let mut states = vec![PointState::Unvisited; points.len()];
    let mut cluster_count = 0;
    let min_cluster_points = 2;

    while cluster_count < max_clusters {
        let mut seed_found = false;
        for i in 0..points.len() {
            let (new_core, new_state, modified) = segmentation_step(core, points[i], states[i]);
            core = new_core;

            if modified && new_state != states[i] {
                states[i] = new_state;
                seed_found = true;
                break;
            }
        }

        if !seed_found {
            break;
        }

        for _ in 0..max_passes {
            let mut modified_count = 0;

            for i in 0..points.len() {
                if !points[i].valid || states[i] != PointState::Unvisited {
                    continue;
                }

                for j in 0..points.len() {
                    if i == j || !points[j].valid {
                        continue;
                    }

                    let (new_state, modified) = ece_expand(
                        threshold_sq,
                        core.ece_core.current_cluster,
                        points[i],
                        states[i],
                        points[j],
                        states[j],
                    );

                    if modified && new_state != states[i] {
                        states[i] = new_state;
                        modified_count += 1;
                        break;
                    }
                }
            }

            if modified_count == 0 {
                let mut current_size = 0;
                for state in states.iter() {
                    if let PointState::Clustered(id) = state {
                        if *id == core.ece_core.current_cluster {
                            current_size += 1;
                        }
                    }
                }

                if current_size < min_cluster_points {
                    for state in states.iter_mut() {
                        if let PointState::Clustered(id) = state {
                            if *id == core.ece_core.current_cluster {
                                *state = PointState::Noise;
                            }
                        }
                    }
                }

                break;
            }
        }

        core.ece_core = core.ece_core.next_cluster();
        cluster_count += 1;
    }

    summarize_states(&states)
}

fn run_dbscan_flow(
    points: &[Point3D],
    eps_sq: Bits<32>,
    min_pts: Bits<8>,
    max_clusters: usize,
    max_passes: usize,
) -> (usize, usize, usize, usize) {
    let mut core = SegmentationCore::new_dbscan(bits::<32>(0), eps_sq, min_pts);
    let mut states = vec![PointState::Unvisited; points.len()];
    let mut cluster_count = 0;

    while cluster_count < max_clusters {
        let mut seed_idx = None;

        for i in 0..points.len() {
            let (new_core, new_state, modified) = segmentation_step(core, points[i], states[i]);
            core = new_core;

            if modified && new_state != states[i] {
                states[i] = new_state;
                seed_idx = Some(i);
                break;
            }
        }

        if seed_idx.is_none() {
            break;
        }

        for i in 0..points.len() {
            if i == seed_idx.unwrap() || !points[i].valid {
                continue;
            }

            let (new_core, _, _) = segmentation_step(core, points[i], states[i]);
            core = new_core;
        }

        let has_enough = has_min_neighbors(core.dbscan_core);

        if !has_enough {
            states[seed_idx.unwrap()] = PointState::Noise;
            core.dbscan_core = mark_seed_as_noise(core.dbscan_core);
            continue;
        }

        states[seed_idx.unwrap()] = PointState::Clustered(core.dbscan_core.current_cluster);
        core.dbscan_core = start_expanding(core.dbscan_core);

        for _ in 0..max_passes {
            let mut modified_count = 0;

            for i in 0..points.len() {
                if !points[i].valid || states[i] != PointState::Unvisited {
                    continue;
                }

                for j in 0..points.len() {
                    if i == j || !points[j].valid {
                        continue;
                    }

                    let (new_state, modified) = dbscan_expand(
                        eps_sq,
                        core.dbscan_core.current_cluster,
                        points[i],
                        states[i],
                        points[j],
                        states[j],
                    );

                    if modified && new_state != states[i] {
                        states[i] = new_state;
                        modified_count += 1;
                        break;
                    }
                }
            }

            if modified_count == 0 {
                break;
            }
        }

        core.dbscan_core = core.dbscan_core.next_cluster();
        cluster_count += 1;
    }

    summarize_states(&states)
}

#[test]
fn distance_squared_basic() {
    let p1 = pt(10, 20, 30);
    let p2 = pt(4, 6, 9);
    let dist = distance_squared(p1, p2);
    assert_eq!(dist.0, 673);
}

#[test]
fn ece_step_marks_seed() {
    let core = EceCore::new(bits::<32>(225));
    let (new_core, new_state, modified) = ece_step(core, pt(1, 2, 3), PointState::Unvisited);

    assert!(modified);
    assert_eq!(new_state, PointState::Clustered(bits::<8>(0)));
    assert_eq!(new_core.state, EceState::Growing);
}

#[test]
fn ece_expand_clusters_when_within_threshold() {
    let threshold_sq = bits::<32>(225);
    let current_cluster = bits::<8>(1);
    let test_point = pt(5, 5, 5);
    let ref_point = pt(8, 5, 5);
    let (new_state, modified) = ece_expand(
        threshold_sq,
        current_cluster,
        test_point,
        PointState::Unvisited,
        ref_point,
        PointState::Clustered(current_cluster),
    );

    assert!(modified);
    assert_eq!(new_state, PointState::Clustered(current_cluster));
}

#[test]
fn dbscan_step_counts_neighbors() {
    let eps_sq = bits::<32>(225);
    let min_pts = bits::<8>(3);
    let core = DbscanCore::new(eps_sq, min_pts);

    let (core, state, modified) = dbscan_step(core, pt(0, 0, 0), PointState::Unvisited);
    assert!(modified);
    assert_eq!(state, PointState::Visited);
    assert_eq!(core.neighbor_count.0, 1);
    assert_eq!(core.state, DbscanState::CountingNeighbors);

    let (core, _, _) = dbscan_step(core, pt(0, 3, 0), PointState::Unvisited);
    let (core, _, _) = dbscan_step(core, pt(4, 0, 0), PointState::Unvisited);
    assert_eq!(core.neighbor_count.0, 3);
    assert!(has_min_neighbors(core));
}

#[test]
fn dbscan_expand_clusters_when_within_threshold() {
    let eps_sq = bits::<32>(225);
    let current_cluster = bits::<8>(2);
    let test_point = pt(2, 2, 2);
    let ref_point = pt(4, 2, 2);
    let (new_state, modified) = dbscan_expand(
        eps_sq,
        current_cluster,
        test_point,
        PointState::Unvisited,
        ref_point,
        PointState::Clustered(current_cluster),
    );

    assert!(modified);
    assert_eq!(new_state, PointState::Clustered(current_cluster));
}

#[test]
fn segmentation_step_respects_mode() {
    let threshold_sq = bits::<32>(225);
    let eps_sq = bits::<32>(225);
    let min_pts = bits::<8>(3);
    let point = pt(1, 1, 1);

    let core = SegmentationCore::new_ece(threshold_sq, eps_sq, min_pts);
    let (_, state, modified) = segmentation_step(core, point, PointState::Unvisited);
    assert!(modified);
    assert!(matches!(state, PointState::Clustered(_)));

    let core = SegmentationCore::new_dbscan(threshold_sq, eps_sq, min_pts);
    let (_, state, modified) = segmentation_step(core, point, PointState::Unvisited);
    assert!(modified);
    assert_eq!(state, PointState::Visited);
}

#[test]
fn ece_flow_clusters_tiny_cloud() {
    let points = tiny_cloud();
    let summary = run_ece_flow(&points, bits::<32>(225), 8, 8);
    assert_eq!(summary, (3, 1, 0, 0));
}

#[test]
fn dbscan_flow_clusters_tiny_cloud() {
    let points = tiny_cloud();
    let summary = run_dbscan_flow(&points, bits::<32>(225), bits::<8>(3), 8, 8);
    assert_eq!(summary, (3, 1, 0, 0));
}
