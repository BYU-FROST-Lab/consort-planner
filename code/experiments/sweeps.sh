#!/usr/bin/env bash
# Re-run the benchmark (50 scenarios x 5 planners) and the ablation (30 x 3)
# behind Figs. 3-6 and Table I, then compare every run with results/paper/.
#
#   SMOKE=1 experiments/sweeps.sh   first 2 scenarios of each, about 1 hour
#   experiments/sweeps.sh           everything, about 16 h + 4 h of planner time
#
# The sweep files run up to 4 planners at once (workers: 4); each CONSORT run
# can reach ~6 GB, so lower `workers` in experiments/configs/benchmark/sweep*.yaml on smaller
# machines. An interrupted sweep resumes when re-run.
set -u; source "$(dirname "$0")/env.sh"
for s in benchmark ablation; do
  if [ -n "${SMOKE:-}" ]; then y=experiments/configs/benchmark/sweep_smoke_$s.yaml; tag=smoke_$s
  else y=experiments/configs/benchmark/sweep$([ $s = ablation ] && echo _ablation).yaml; tag=$s; fi
  echo "=== $s ($y)"
  "$JULIA" experiments/run_constraint_sweep.jl --sweep $y --tag $tag | tail -2
  check "$s runs match results/paper/$s" yes \
    "$("$PY" experiments/compare_trials.py results/new/constraint_sweep/$tag/trials_all.csv results/paper/$s/trials_all.csv >&2 && echo yes || echo no)"
done
[ -z "${SMOKE:-}" ] && echo "plot them: CONSORT_BASELINE_SWEEP=results/new/constraint_sweep/benchmark CONSORT_ABLATION_SWEEP=results/new/constraint_sweep/ablation experiments/figs3to6.sh"
finish
