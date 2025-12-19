use criterion::{black_box, criterion_group, criterion_main, Criterion};
use lidar_clustering_hdl::*;
use rhdl::prelude::*;
use std::time::Duration;

fn bench_ece_full(c: &mut Criterion) {
    let small = tiled_cloud(1);
    let medium = tiled_cloud(8);
    let large = tiled_cloud(22); // ~1k points
    let noisy = noisy_cloud(512);
    let threshold_sq = bits::<32>(225);

    c.bench_function("ece_full_48_points", |b| {
        b.iter(|| {
            let result = run_ece_flow(black_box(&small), black_box(threshold_sq), 16, 20);
            black_box(result);
        });
    });

    c.bench_function("ece_full_384_points", |b| {
        b.iter(|| {
            let result = run_ece_flow(black_box(&medium), black_box(threshold_sq), 64, 20);
            black_box(result);
        });
    });

    c.bench_function("ece_full_~1050_points", |b| {
        b.iter(|| {
            let result = run_ece_flow(black_box(&large), black_box(threshold_sq), 128, 20);
            black_box(result);
        });
    });

    c.bench_function("ece_full_noisy_512_points", |b| {
        b.iter(|| {
            let result = run_ece_flow(black_box(&noisy), black_box(threshold_sq), 64, 20);
            black_box(result);
        });
    });
}

fn bench_dbscan_full(c: &mut Criterion) {
    let small = tiled_cloud(1);
    let medium = tiled_cloud(8);
    let large = tiled_cloud(22); // ~1k points
    let noisy = noisy_cloud(512);
    let eps_sq = bits::<32>(225); // epsilon = 15
    let min_pts = bits::<8>(3);

    c.bench_function("dbscan_full_48_points", |b| {
        b.iter(|| {
            let result = run_dbscan_flow(black_box(&small), black_box(eps_sq), black_box(min_pts), 16, 20);
            black_box(result);
        });
    });

    c.bench_function("dbscan_full_384_points", |b| {
        b.iter(|| {
            let result = run_dbscan_flow(black_box(&medium), black_box(eps_sq), black_box(min_pts), 64, 20);
            black_box(result);
        });
    });

    c.bench_function("dbscan_full_~1050_points", |b| {
        b.iter(|| {
            let result = run_dbscan_flow(black_box(&large), black_box(eps_sq), black_box(min_pts), 128, 20);
            black_box(result);
        });
    });

    c.bench_function("dbscan_full_noisy_512_points", |b| {
        b.iter(|| {
            let result = run_dbscan_flow(black_box(&noisy), black_box(eps_sq), black_box(min_pts), 64, 20);
            black_box(result);
        });
    });
}

fn test_distance_calculation(c: &mut Criterion) {
    c.bench_function("calculate_distance", |b| {
        let point1 = Point3D::new(bits::<16>(100), bits::<16>(200), bits::<16>(50));
        let point2 = Point3D::new(bits::<16>(150), bits::<16>(250), bits::<16>(75));

        b.iter(|| {
            distance_squared(black_box(point1), black_box(point2))
        });
    });
}

fn run_ece_flow(points: &[Point3D], threshold_sq: Bits<32>, max_clusters: usize, max_passes: usize) -> (usize, usize, usize, usize) {
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

fn run_dbscan_flow(points: &[Point3D], eps_sq: Bits<32>, min_pts: Bits<8>, max_clusters: usize, max_passes: usize) -> (usize, usize, usize, usize) {
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

fn base_cloud() -> Vec<Point3D> {
    let mut points = Vec::new();

    for i in 0..15 {
        let x = 20 + (i % 5) * 4;
        let y = 20 + (i / 5) * 4;
        points.push(Point3D::new(bits::<16>(x as u128), bits::<16>(y as u128), bits::<16>(20)));
    }

    for i in 0..18 {
        let x = 100 + (i % 6) * 4;
        let y = 100 + (i / 6) * 4;
        points.push(Point3D::new(bits::<16>(x as u128), bits::<16>(y as u128), bits::<16>(50)));
    }

    for i in 0..12 {
        let x = 200 + (i % 4) * 3;
        let y = 50 + (i / 4) * 3;
        points.push(Point3D::new(bits::<16>(x as u128), bits::<16>(y as u128), bits::<16>(100)));
    }

    points.push(Point3D::new(bits(500), bits(500), bits(500)));
    points.push(Point3D::new(bits(50), bits(300), bits(200)));
    points.push(Point3D::new(bits(400), bits(20), bits(300)));

    points
}

fn tiled_cloud(tiles: usize) -> Vec<Point3D> {
    let base = base_cloud();
    let mut points = Vec::with_capacity(base.len() * tiles);

    for t in 0..tiles {
        let offset = (t as u16) * 300;
        for p in base.iter() {
            let x = p.x.0 as u32 + offset as u32;
            let y = p.y.0 as u32 + offset as u32;
            points.push(Point3D::new(bits::<16>(x as u128), bits::<16>(y as u128), p.z));
        }
    }

    points
}

fn noisy_cloud(size: usize) -> Vec<Point3D> {
    let mut pts = Vec::with_capacity(size);
    let mut seed: u64 = 0x1234_5678;
    let mut next = |s: &mut u64| {
        *s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
        *s
    };

    for i in 0..size {
        let r1 = next(&mut seed);
        let r2 = next(&mut seed);
        let r3 = next(&mut seed);

        let x = ((r1 >> 16) & 0x3FF) as u128; 
        let y = ((r2 >> 16) & 0x3FF) as u128;
        let z = ((r3 >> 16) & 0x1FF) as u128; 

        if i % 10 == 0 {
            let bx = 50 + (i % 5) * 4;
            let by = 80 + (i % 7) * 3;
            pts.push(Point3D::new(bits::<16>(bx as u128), bits::<16>(by as u128), bits::<16>(30)));
        } else {
            pts.push(Point3D::new(bits::<16>(x), bits::<16>(y), bits::<16>(z)));
        }
    }

    pts
}

criterion_group!(
    name = benches;
    config = Criterion::default()
        .sample_size(40)
        .measurement_time(Duration::from_secs(10))
        .warm_up_time(Duration::from_secs(3));
    targets = bench_ece_full, bench_dbscan_full, test_distance_calculation
);
criterion_main!(benches);
