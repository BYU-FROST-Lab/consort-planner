#!/usr/bin/env bash
# Plan the behind-wall mission that was flown in simulation (Sec. VI).
# About 2 minutes and ~7 GB RAM. Fly it with src/sim/ (see src/sim/README.md).
set -u; source "$(dirname "$0")/env.sh"
OUTPUT_DIR=results/new/behind_wall "$JULIA" experiments/generate_plan.jl experiments/configs/sim_mission > results/new/behind_wall.log 2>&1 \
  || { echo "planner failed, see results/new/behind_wall.log"; exit 1; }
R=results/new/behind_wall/hexspline_cl/results.yaml
check "primary length (m)"         1081.6 "$(num $R primary_length 1)"
check "primary terminal sigma (m)" 1.796  "$(num $R primary_unc 3)"
check "identical to the flown plan" same "$(cmp -s $R results/paper/montecarlo/hexspline_cl/results.yaml && echo same || echo different)"
finish
