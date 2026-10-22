#!/usr/bin/env bash
# Fig. 8 and the Monte Carlo numbers of Sec. VI-C, from the 30 trials in results/paper/montecarlo (seconds).
set -u; source "$(dirname "$0")/env.sh"
"$PY" figures/scripts/fig8_montecarlo.py results/paper/montecarlo --stem=fig8 --case=behind_wall \
    --nstd=3 --dev=0 --out=figures/new > results/new/fig8.log
sed -n '/reached goal/,/empirical/p' results/new/fig8.log
L=results/new/fig8.log
check "reached goal"          30/30  "$(awk '/reached goal/{print $3; exit}' $L)"
check "collision-free"        30/30  "$(awk '/obstacle-clear/{print $2; exit}' $L)"
check "terminal bound met"    28/30  "$(awk '/terminal bound met/{print $4; exit}' $L)"
check "predicted sigma (m)"   1.7960 "$(awk '/predicted sigma_T/{print $3; exit}' $L)"
check "measured sigma (m)"    1.7025 "$(awk '/empirical sigma_T/{print $3; exit}' $L)"
echo "figure: figures/new/fig8.pdf"; finish
