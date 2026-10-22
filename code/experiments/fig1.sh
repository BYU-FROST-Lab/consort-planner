#!/usr/bin/env bash
# Fig. 1: plan the escort scenario (about 20 s), then draw it.
set -u; source "$(dirname "$0")/env.sh"
OUTPUT_DIR=results/new/fig1 "$JULIA" experiments/generate_plan.jl results/paper/fig1/config > results/new/fig1.log 2>&1 \
  || { echo "planner failed, see results/new/fig1.log"; exit 1; }
FIG1_PYTHON="$PY" "$JULIA" figures/scripts/fig1_overview.jl results/new/fig1 results/new/fig1 100 > results/new/fig1_draw.log 2>&1
cp results/new/fig1/fig1_continuous_ellipses_every100m.pdf figures/new/fig1.pdf
R=results/new/fig1/hexspline_cl/results.yaml
check "primary length (m)"         1100.0 "$(num $R primary_length 1)"
check "primary terminal sigma (m)" 1.799  "$(num $R primary_unc 3)"
echo "figure: figures/new/fig1.pdf"; finish
