# LiDAR Clustering HDL

We are building LiDAR point cloud clustering logic in Rust and targeting hardware with rhdl. The code is kept small and straight so it can map to RTL without dynamic memory or floating point.

## Goals
- Show two simple clustering ideas that could be pushed to hardware: Euclidean Cluster Extraction (ECE) and a reduced DBSCAN.
- Keep all math on fixed width integers so the kernels stay synthesizable.
- Provide a tiny driver that feeds points and prints the cluster growth for debugging.

## Algorithms in code
- ECE: pick a seed, grow the region while distance squared to cluster points is under `threshold_sq`, then bump the cluster id. Kernels live in `src/ece.rs` and are called through `SegmentationCore::new_ece` in `src/segmentation.rs`.
- DBSCAN (trimmed): pick a seed, count neighbors within `eps_sq`, check `min_pts`, then expand by comparing unvisited points with clustered ones. Kernels are in `src/dbscan.rs` and are wired by `SegmentationCore::new_dbscan`.
- Distance math is shared in `src/distance.rs` and uses integer squared distance to avoid roots.

## Data layout
- Each point is `Point3D` with `Bits<16>` coords and a `valid` flag. `PointState` tracks `Unvisited`, `Visited`, `Noise`, or `Clustered(id)`.
- Thresholds: `threshold_sq` and `eps_sq` are `Bits<32>`. `min_pts` and cluster ids are `Bits<8>`.
- No heap allocations inside kernels; the driver owns the vectors and feeds the kernels one point at a time.

## Driver and flow
- `src/main.rs` builds a fake point cloud in `generate_test_data`, then runs ECE and DBSCAN separately. It logs seed detection, passes of expansion, and final cluster stats.
- `print_results` dumps cluster sizes and the first 20 points with their state so you can sanity check grouping.
- The segmentation wrapper in `src/segmentation.rs` holds the mode (ECE vs DBSCAN) and routes each point to the right kernel.

## Hardware angle
- Kernels are marked with `#[kernel]` and use `Bits` and `Digital` types from rhdl so they can become HDL. There is no floating point, recursion, or dynamic memory inside them.
- Cluster ids and counters wrap naturally because they are fixed width; adjust the bit sizes if you need more range.
- The project stops at the algorithmic model; no Verilog is emitted here, but the code is structured to make that step straightforward.

## Project layout
- `src/`: point types, distance math, ECE and DBSCAN kernels, segmentation wrapper, and the demo driver.
- `benches/clustering_bench.rs`: Criterion benchmark to time the CPU model of the kernels.
- `visualize_hdl_points.py`: quick script to plot sample points if you want a picture.

## How to run
1. Install Rust and cargo.
2. `cargo run` to execute both clustering flows and print the logs.
3. `cargo bench` to run the Criterion benchmark.

## Tuning and notes
- Change the fake cloud in `generate_test_data` to match whatever scenario you want to check (dense, sparse, more noise).
- Adjust `threshold_sq`, `eps_sq`, and `min_pts` in `main.rs` to see how sensitive the kernels are.
- There is no file I/O, sensor interface, or graphics in the driver. Everything is in memory and printed to stdout.
