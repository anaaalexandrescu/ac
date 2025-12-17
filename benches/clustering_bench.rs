use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use lidar_clustering_hdl::*;
use rhdl::prelude::*;

fn benchmark_ece(c: &mut Criterion) {
    let mut group = c.benchmark_group("ECE Point Cloud Segmentation");
    
    for size in [100, 500, 1000, 2000].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let points = generate_points(size);
            let threshold_sq = bits(200);
            
            b.iter(|| {
                let mut ece_core = EceCore::new(threshold_sq);
                let mut states = vec![PointState::Unvisited; points.len()];
                
                for (i, point) in points.iter().enumerate() {
                    let (new_core, new_state) = ece_step(
                        black_box(ece_core), 
                        black_box(*point), 
                        black_box(states[i])
                    );
                    ece_core = new_core;
                    states[i] = new_state;
                }
                states
            });
        });
    }
    
    group.finish();
}

fn benchmark_dbscan(c: &mut Criterion) {
    let mut group = c.benchmark_group("DBSCAN Point Cloud Segmentation");
    
    for size in [100, 500, 1000, 2000].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let points = generate_points(size);
            let epsilon_sq = bits(200);
            let min_pts = bits(3);
            
            b.iter(|| {
                let mut dbscan_core = DbscanCore::new(epsilon_sq, min_pts);
                let mut states = vec![PointState::Unvisited; points.len()];
                
                for (i, point) in points.iter().enumerate() {
                    let (new_core, new_state) = dbscan_step(
                        black_box(dbscan_core), 
                        black_box(*point), 
                        black_box(states[i])
                    );
                    dbscan_core = new_core;
                    states[i] = new_state;
                }
                states
            });
        });
    }
    
    group.finish();
}

fn benchmark_distance(c: &mut Criterion) {
    c.bench_function("distance_squared", |b| {
        let p1 = Point3D::new(bits(100), bits(200), bits(50));
        let p2 = Point3D::new(bits(150), bits(250), bits(75));
        
        b.iter(|| {
            distance_squared(black_box(p1), black_box(p2))
        });
    });
}

fn benchmark_clustering_core(c: &mut Criterion) {
    let mut group = c.benchmark_group("Clustering Core");
    
    for size in [100, 500, 1000].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let points = generate_points(size);
            let threshold_sq = bits(200);
            
            b.iter(|| {
                let mut core = ClusteringCore::new(threshold_sq);
                let mut states = vec![PointState::Unvisited; points.len()];
                
                for (i, point) in points.iter().enumerate() {
                    let (new_core, new_state, _) = process_point(
                        black_box(core), 
                        black_box(*point), 
                        black_box(states[i])
                    );
                    core = new_core;
                    states[i] = new_state;
                }
                states
            });
        });
    }
    
    group.finish();
}

fn generate_points(n: usize) -> Vec<Point3D> {
    (0..n).map(|i| {
        Point3D::new(
            bits((i * 7 % 1000) as u128),
            bits((i * 13 % 1000) as u128),
            bits((i * 3 % 500) as u128),
        )
    }).collect()
}

criterion_group!(benches, benchmark_ece, benchmark_dbscan, benchmark_distance, benchmark_clustering_core);
criterion_main!(benches);