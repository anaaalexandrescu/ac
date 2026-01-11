#!/usr/bin/env bash
set -euo pipefail

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$root_dir"

cargo run --bin export_vlog

yosys -p "read_verilog -sv synth/segmentation_step_top.v; synth_ecp5 -top segmentation_step_top -json synth/segmentation_step.json; stat" \
  > synth/yosys.log

nextpnr-ecp5 \
  --45k \
  --package CABGA381 \
  --json synth/segmentation_step.json \
  --top segmentation_step_top \
  --freq 100 \
  --timing-allow-fail \
  --out-of-context \
  --report synth/nextpnr_report.json \
  > synth/nextpnr.log

echo "Reports written to synth/yosys.log and synth/nextpnr_report.json"
