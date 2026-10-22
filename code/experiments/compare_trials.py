#!/usr/bin/env python3
"""Compare a re-run sweep's trials_all.csv with a shipped one, row by row.

    python experiments/compare_trials.py <new trials_all.csv> <shipped trials_all.csv>

Rows are matched on (method, scenario_id, pct). Wall-clock columns are
ignored, since they depend on the machine. Exits 1 if any matched row differs
or a new row has no shipped counterpart.
"""
import csv
import sys

new, ref = (list(csv.DictReader(open(p))) for p in sys.argv[1:3])
key = lambda r: (r["method"], r["scenario_id"], r["pct"])
ref = {key(r): r for r in ref}
bad = 0
for r in new:
    k = key(r)
    if k not in ref:
        print("no shipped row for", k); bad += 1; continue
    d = [c for c in r if "wall" not in c and c in ref[k] and r[c] != ref[k][c]]
    if d:
        bad += 1
        print("differs", k, [(c, r[c], ref[k][c]) for c in d[:4]])
print(f"{len(new) - bad} of {len(new)} rows identical to the shipped sweep (wall clock ignored)")
sys.exit(1 if bad else 0)
