use criterion::{black_box, criterion_group, criterion_main, Criterion};
use lidar_clustering_hdl::*;
use rhdl::prelude::*;

fn test_ece_performance(c: &mut Criterion) {
    c.bench_function("ece_100_points", |b| {
        let points = make_test_points(100);
        let threshold = bits(225);

        b.iter(|| {
            let mut core = EceCore::new(threshold);
            let mut states = vec![PointState::Unvisited; 100];

            for i in 0..points.len() {
                let (new_core, new_state, _) = ece_step(core, points[i], states[i]);
                core = new_core;
                states[i] = new_state;
            }
        });
    });
}

fn test_dbscan_performance(c: &mut Criterion) {
    c.bench_function("dbscan_100_points", |b| {
        let points = make_test_points(100);
        let epsilon = bits(225);
        let min_pts = bits(3);

        b.iter(|| {
            let mut core = DbscanCore::new(epsilon, min_pts);
            let mut states = vec![PointState::Unvisited; 100];

            for i in 0..points.len() {
                let (new_core, new_state, _) = dbscan_step(core, points[i], states[i]);
                core = new_core;
                states[i] = new_state;
            }
        });
    });
}

fn test_distance_calculation(c: &mut Criterion) {
    c.bench_function("calculate_distance", |b| {
        let point1 = Point3D::new(bits(100), bits(200), bits(50));
        let point2 = Point3D::new(bits(150), bits(250), bits(75));

        b.iter(|| {
            distance_squared(black_box(point1), black_box(point2))
        });
    });
}

fn make_test_points(n: usize) -> Vec<Point3D> {
    let mut points = Vec::new();
    for i in 0..n {
        let x = (i * 7 % 1000) as u128;
        let y = (i * 13 % 1000) as u128;
        let z = (i * 3 % 500) as u128;
        points.push(Point3D::new(bits(x), bits(y), bits(z)));
    }
    points
}

criterion_group!(
    benches,
    test_ece_performance,
    test_dbscan_performance,
    test_distance_calculation
);
criterion_main!(benches);