# Evaluation Phase: Point Cloud Clustering (Subject 3)

This document summarizes the FPGA synthesis + performance results for the
clustering kernel in this repository.

## Setup
- HDL source: `segmentation_step` kernel exported from RHDL
- Export tool: `cargo run --bin export_vlog` (writes `synth/segmentation_step_top.v`)
- FPGA flow: Yosys + nextpnr-ecp5
- Target board/device: ULX3S (Lattice ECP5 LFE5U-45F, CABGA381, default speed grade)
- Clock constraint: 100 MHz (nextpnr `--freq 100`)
- Placement mode: out-of-context (IO buffers disabled to avoid IO pin limits)

## Synthesis Results (nextpnr report)
From `synth/nextpnr_report.json`:
- LUTs (TRELLIS_COMB): 623 / 43,848
- FFs (TRELLIS_FF): 466 / 43,848
- DSP blocks (MULT18X18D): 9 / 72
- BRAM (DP16KD): 0 / 108
- RAMW (TRELLIS_RAMW): 0 / 5,481
- Fmax: 37.77 MHz (constraint 100 MHz, timing not met)

Notes:
- Out-of-context mode disables IO placement; only core logic is timed.
- This kernel is heavy on multiply/accumulate operations, hence DSP usage.

## Latency + Throughput
The synthesized top wraps the combinational kernel with input/output registers:
- Latency: 1 cycle (register → combinational → register)
- Throughput: 1 output per cycle (when streaming inputs each clock)

## CPU Baseline (software)
Criterion baseline (short run with reduced sample sizes):
- ECE 48 points: ~88–95 us
- ECE 384 points: ~54–59 ms
- ECE ~1050 points: ~1.23–1.58 s
- ECE noisy 512: ~160–170 ms
- DBSCAN 48 points: ~103–111 us
- DBSCAN 384 points: ~44–48 ms
- DBSCAN ~1050 points: ~0.95–1.14 s
- DBSCAN noisy 512: ~28.2–28.8 ms
- Distance kernel: ~20.9–21.6 ns

Command used:
`cargo bench --bench clustering_bench -- --sample-size 10 --measurement-time 2 --warm-up-time 1`

## Power
Power estimation not available in the open-source flow used here.

## Reproducibility
Run `synth/run_synth.sh` to regenerate HDL + reports.
The script writes:
- `synth/segmentation_step.v` (kernel function)
- `synth/segmentation_step_top.v` (wrapped top module)
- `synth/segmentation_step.json` (yosys netlist)
- `synth/yosys.log`
- `synth/nextpnr_report.json`
