#!/usr/bin/env bash
# Figs. 3-6 and Table I from the benchmark and ablation sweeps (seconds).
# Plots results/paper/ by default; set CONSORT_BASELINE_SWEEP / CONSORT_ABLATION_SWEEP
# to a re-run sweep folder (see sweeps.sh) to plot that instead.
set -u; source "$(dirname "$0")/env.sh"
"$PY" figures/scripts/make_figs_baseline.py | tee results/new/table1.txt
row() { grep "^ *$1 " results/new/table1.txt | sed 's/ *\\\\$//; s/^ *//; s/  */ /g'; }
check "Table I, CONSORT"   'Ours & 1.14$\pm$0.09 & 1.22$\pm$0.23 & 1.34$\pm$0.26 & 1.41$\pm$0.21' "$(row Ours)"
check "Table I, Formation" 'Formation & 1.22$\pm$0.12 & 1.34$\pm$0.25 & 1.33$\pm$0.16 & 1.48$\pm$0.45' "$(row Formation)"
S=${CONSORT_BASELINE_SWEEP:-results/paper/benchmark}/summary.csv
solved() { awk -F, -v m="$1" '$1==m && $2==50 {print $4"/"$3}' "$S"; }
check "CONSORT solved at 50 %"   40/50 "$(solved hexspline_cl)"
check "Formation solved at 50 %" 27/50 "$(solved formation)"
echo "figures: figures/new/fig{3a,3b,4,5a,5b,6}.pdf"; finish
