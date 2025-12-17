use rhdl::prelude::*;
use lidar_clustering_hdl::*;

fn main() {
    println!("=== RHDL POINT CLOUD SEGMENTATION ===");
    println!("1. ECE Clustering");
    println!("2. DBSCAN Clustering");

    let mut input = String::new();
    std::io::stdin().read_line(&mut input).unwrap();
    let choice = input.trim();

    let points = generate_points(30);
    let mut states = vec![PointState::Unvisited; 30];

    match choice {
        "1" => run_ece(points, states),
        "2" => run_dbscan(points, states),
        _ => println!("Invalid option"),
    }
}

fn generate_points(n: usize) -> Vec<Point3D> {
    (0..n)
        .map(|i| {
            Point3D::new(
                bits((i * 7 % 200) as u128),
                bits((i * 5 % 200) as u128),
                bits((i * 3 % 200) as u128),
            )
        })
        .collect()
}

fn run_ece(points: Vec<Point3D>, mut states: Vec<PointState>) {
    let mut core = EceCore::new(bits(30_000));

    for i in 0..points.len() {
        let (new_core, new_state) = ece_step(core, points[i], states[i]);
        core = new_core;
        states[i] = new_state;
    }

    println!("ECE RESULTS:");
    for (i, st) in states.iter().enumerate() {
        println!("Point {} → {:?}", i, st);
    }
}

fn run_dbscan(points: Vec<Point3D>, mut states: Vec<PointState>) {
    let mut core = DbscanCore::new(bits(30_000), bits(3));

    for i in 0..points.len() {
        let (new_core, new_state) = dbscan_step(core, points[i], states[i]);
        core = new_core;
        states[i] = new_state;
    }

    println!("DBSCAN RESULTS:");
    for (i, st) in states.iter().enumerate() {
        println!("Point {} → {:?}", i, st);
    }
}