// src/main.rs
use rhdl::prelude::*;

mod point;
mod distance;
mod ece;
mod dbscan;
mod clustering_core;
mod segmentation;

use point::{Point3D, PointState};
use segmentation::{SegmentationCore, segmentation_step};

fn main() {
    println!("=== Test ECE Clustering ===\n");
    test_ece_clustering();
    
    println!("\n=== Test DBSCAN Clustering ===\n");
    test_dbscan_clustering();
}

fn test_ece_clustering() {
    let points = generate_test_data();
    let mut states = vec![PointState::Unvisited; points.len()];
    
    println!("Total puncte: {}", points.len());
    
    let threshold_sq = bits::<32>(225);
    
    let mut core = SegmentationCore::new_ece(
        threshold_sq,
        bits::<32>(0),
        bits::<8>(0),
    );
    
    let mut cluster_count = 0;
    let max_clusters = 10;
    
    while cluster_count < max_clusters {
        // Găsește seed pentru cluster nou
        let mut seed_found = false;
        for i in 0..points.len() {
            let (new_core, new_state, modified) = 
                segmentation_step(core, points[i], states[i]);
            
            core = new_core;
            
            if modified && new_state != states[i] {
                states[i] = new_state;
                seed_found = true;
                println!("→ Seed găsit pentru cluster {}: punct {}", cluster_count, i);
                break;
            }
        }
        
        if !seed_found {
            println!("✓ Nu mai sunt puncte nevizitate!");
            break;
        }
        
        // Region growing: repetă până nu mai sunt modificări
        let max_expansion_passes = 20;
        for pass in 0..max_expansion_passes {
            let mut modified_count = 0;
            
            // Pentru fiecare punct nevizitat
            for i in 0..points.len() {
                if !points[i].valid || states[i] != PointState::Unvisited {
                    continue;
                }
                
                // Compară cu TOATE punctele deja clusterizate
                for j in 0..points.len() {
                    if i == j || !points[j].valid {
                        continue;
                    }
                    
                    let (new_state, modified) = ece::ece_expand(
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
                        break; // Punct adăugat, treci la următorul
                    }
                }
            }
            
            println!("  Pass {}: {} puncte adăugate", pass, modified_count);
            
            if modified_count == 0 {
                println!("  ✓ Cluster {} complet", cluster_count);
                break;
            }
        }
        
        // Următorul cluster
        core.ece_core = core.ece_core.next_cluster();
        cluster_count += 1;
    }
    
    print_results(&points, &states);
}

fn test_dbscan_clustering() {
    let points = generate_test_data();
    let mut states = vec![PointState::Unvisited; points.len()];
    
    println!("Total puncte: {}", points.len());
    
    let eps_sq = bits::<32>(225);  // epsilon = 15
    let min_pts = bits::<8>(3);
    
    let mut core = SegmentationCore::new_dbscan(
        bits::<32>(0),
        eps_sq,
        min_pts,
    );
    
    let mut cluster_count = 0;
    let max_clusters = 10;
    
    while cluster_count < max_clusters {
        // Găsește seed
        let mut seed_idx = None;
        for i in 0..points.len() {
            let (new_core, new_state, modified) = 
                segmentation_step(core, points[i], states[i]);
            
            core = new_core;
            
            if modified && new_state != states[i] {
                states[i] = new_state;
                seed_idx = Some(i);
                break;
            }
        }
        
        if seed_idx.is_none() {
            println!("✓ Nu mai sunt puncte nevizitate!");
            break;
        }
        
        // Numără vecinii seed-ului
        for i in 0..points.len() {
            if i == seed_idx.unwrap() || !points[i].valid {
                continue;
            }
            
            let (new_core, _, _) = 
                segmentation_step(core, points[i], states[i]);
            core = new_core;
        }
        
        // Verifică dacă seed are suficienți vecini
        let has_enough = dbscan::has_min_neighbors(core.dbscan_core);
        
        if !has_enough {
            // Seed e noise
            println!("→ Seed {} e noise (doar {} vecini)", 
                seed_idx.unwrap(), 
                core.dbscan_core.neighbor_count.0
            );
            
            states[seed_idx.unwrap()] = PointState::Noise;
            core.dbscan_core = dbscan::mark_seed_as_noise(core.dbscan_core);
            continue;
        }
        
        // Seed e core point - marchează ca clustered și expandează
        println!("→ Seed {} pentru cluster {} (cu {} vecini)", 
            seed_idx.unwrap(),
            cluster_count,
            core.dbscan_core.neighbor_count.0
        );
        
        states[seed_idx.unwrap()] = PointState::Clustered(core.dbscan_core.current_cluster);
        core.dbscan_core = dbscan::start_expanding(core.dbscan_core);
        
        // Region growing
        let max_expansion_passes = 20;
        for pass in 0..max_expansion_passes {
            let mut modified_count = 0;
            
            // Pentru fiecare punct nevizitat
            for i in 0..points.len() {
                if !points[i].valid || states[i] != PointState::Unvisited {
                    continue;
                }
                
                // Compară cu TOATE punctele din cluster
                for j in 0..points.len() {
                    if i == j || !points[j].valid {
                        continue;
                    }
                    
                    let (new_state, modified) = dbscan::dbscan_expand(
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
            
            println!("  Pass {}: {} puncte adăugate", pass, modified_count);
            
            if modified_count == 0 {
                println!("  ✓ Cluster {} complet", cluster_count);
                break;
            }
        }
        
        // Următorul cluster
        core.dbscan_core = core.dbscan_core.next_cluster();
        cluster_count += 1;
    }
    
    print_results(&points, &states);
}

fn generate_test_data() -> Vec<Point3D> {
    let mut points = Vec::new();
    
    // Cluster 1: centrat la (20, 20, 20)
    for i in 0..15 {
        let x = 20 + (i % 5) * 4;
        let y = 20 + (i / 5) * 4;
        points.push(Point3D::new(
            bits::<16>(x as u128),
            bits::<16>(y as u128),
            bits::<16>(20),
        ));
    }
    
    // Cluster 2: centrat la (100, 100, 50)
    for i in 0..18 {
        let x = 100 + (i % 6) * 4;
        let y = 100 + (i / 6) * 4;
        points.push(Point3D::new(
            bits::<16>(x as u128),
            bits::<16>(y as u128),
            bits::<16>(50),
        ));
    }
    
    // Cluster 3: centrat la (200, 50, 100)
    for i in 0..12 {
        let x = 200 + (i % 4) * 3;
        let y = 50 + (i / 4) * 3;
        points.push(Point3D::new(
            bits::<16>(x as u128),
            bits::<16>(y as u128),
            bits::<16>(100),
        ));
    }
    
    // Zgomot
    points.push(Point3D::new(bits(500), bits(500), bits(500)));
    points.push(Point3D::new(bits(50), bits(300), bits(200)));
    points.push(Point3D::new(bits(400), bits(20), bits(300)));
    
    points
}

fn print_results(points: &[Point3D], states: &[PointState]) {
    println!("\n=== Rezultate ===");
    
    let mut clusters = std::collections::HashMap::new();
    let mut noise = 0;
    let mut unvisited = 0;
    let mut visited = 0;
    
    for (i, &state) in states.iter().enumerate() {
        if !points[i].valid {
            continue;
        }
        
        match state {
            PointState::Clustered(id) => {
                *clusters.entry(id.0 as u64).or_insert(0) += 1;
            }
            PointState::Noise => noise += 1,
            PointState::Unvisited => unvisited += 1,
            PointState::Visited => visited += 1,
        }
    }
    
    println!("\nClustere: {}", clusters.len());
    let mut cluster_ids: Vec<_> = clusters.keys().collect();
    cluster_ids.sort();
    
    for id in cluster_ids {
        println!("  Cluster {}: {} puncte", id, clusters[id]);
    }
    
    println!("  Noise: {}", noise);
    println!("  Unvisited: {}", unvisited);
    println!("  Visited: {}", visited);
    
    println!("\nPrimele 20 puncte:");
    for i in 0..20.min(points.len()) {
        let p = &points[i];
        if !p.valid {
            println!("[{:2}] INVALID", i);
            continue;
        }
        
        let state_str = match states[i] {
            PointState::Clustered(id) => format!("C{}", id.0),
            PointState::Noise => "Noise".to_string(),
            PointState::Unvisited => "Unvis".to_string(),
            PointState::Visited => "Visit".to_string(),
        };
        
        println!("[{:2}] ({:3}, {:3}, {:3}) → {:6}", 
            i,
            p.x.0,
            p.y.0,
            p.z.0,
            state_str
        );
    }
}