#!/usr/bin/env bash
# Fig. 2: walk the ladder scenario from 100 % to 30 % of the reference bound
# (8 plans, about 4 minutes, up to ~6 GB RAM), then draw the four paper panels.
# DRAW_ONLY=1 draws from the shipped results/paper/ladder instead of re-planning.
set -u; source "$(dirname "$0")/env.sh"
if [ -z "${DRAW_ONLY:-}" ]; then
  "$JULIA" experiments/run_constraint_sweep.jl --sweep experiments/configs/benchmark/sweep_fig2.yaml --tag ladder > results/new/fig2.log 2>&1
  SW=results/new/fig2_ladder/ladder
else
  SW=results/paper/ladder
fi
"$JULIA" figures/scripts/export_ladder_paths.jl $SW 100,70,50,30 figures/new/fig2_paths.csv hexspline_cl > /dev/null
"$PY" figures/scripts/plot_fig2_ladder_mpl.py $SW --methods=hexspline_cl \
    --paths=figures/new/fig2_paths.csv --stem=fig2 --out=figures/new > /dev/null
for pair in "100 1741.2 9.73" "070 1760.8 8.89" "050 1821.6 6.66" "030 1890.2 3.91"; do
  set -- $pair; R=$SW/hexspline_cl/s001_p$1/results.yaml
  check "${1#0}% level: length (m)" "$2" "$(num $R primary_length 1)"
  check "${1#0}% level: sigma (m)"  "$3" "$(num $R primary_unc 2)"
done
echo "figure: figures/new/fig2.pdf"; finish
