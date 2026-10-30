# Reproducing the paper

The scripts here reproduce every figure and table in the paper. Every command
below is run from the repository root.

## Setup

The drivers need [Julia 1.12](https://julialang.org/downloads/) and Python 3.12. The
results were produced on Ubuntu with an AMD Ryzen 5 5600X and 32 GB of RAM.

```bash
./setup.sh              # pinned Julia packages (Project.toml, Manifest.toml) and .venv for the figures
tests/run_tests.sh      # about 1 minute
```

The figure scripts use the Nimbus Roman font (`fonts-urw-base35` on
Debian/Ubuntu) and fall back to another serif font without it.

## Where things are

```
experiments/configs/      sim_mission/: the simulated mission (Sec. VI); benchmark/: the benchmark (Sec. V) and Fig. 2
results/paper/            the runs behind every figure, and RESULTS.md with every table and number
results/new/              where the drivers write new runs (not tracked)
figures/scripts/          the scripts that draw the figures
figures/paper/            the published figures (PDF)
figures/new/              where the figure scripts write new figures (not tracked)
```

`results/paper/` holds:

| Folder | Content | Paper |
|---|---|---|
| `benchmark/` | `trials_all.csv`, one row per planner run over 50 scenarios and 8 constraint levels, and its `summary.csv` | Figs. 3, 4, Table I |
| `ablation/` | the same for the 30-scenario ablation | Figs. 5, 6, Table I |
| `ladder/` | the four plans of the constraint ladder | Fig. 2 |
| `fig1/` | the Fig. 1 plan and its config | Fig. 1 |
| `montecarlo/` | the plan flown in simulation and its 30 trials `mc0`..`mc29` | Fig. 8, Sec. VI-C |

## Running the drivers

Each driver writes its runs to `results/new/` and its figures to
`figures/new/`, and ends by checking its output against the paper. Compare
the figures with `figures/paper/`.

| Result | Driver | Time | Checks |
|---|---|---|---|
| Fig. 1 | `experiments/fig1.sh` | 1 min | primary length 1100.0 m, terminal σ 1.799 m |
| Fig. 2 | `experiments/fig2.sh` | 4 min | length and σ of all four panels |
| Figs. 3-6, Table I | `experiments/figs3to6.sh` | seconds | Table I rows, 40/50 vs 27/50 solved at the 50 % level |
| Fig. 8, Sec. VI-C | `experiments/fig8.sh` | seconds | 30/30 reached, 30/30 collision-free, 28/30 under the bound, σ 1.80 m predicted vs 1.70 m measured |
| The simulated mission's plan | `experiments/plan_behind_wall.sh` | 2 min | identical to the plan in `results/paper/montecarlo` |
| The benchmark and ablation runs | `SMOKE=1 experiments/sweeps.sh` | 1 h | the first 2 scenarios of each, every run identical to `results/paper/` |
| | `experiments/sweeps.sh` | about 20 h | all 80 scenarios, every run compared with `results/paper/` |

**Figs. 3-6 and Table I** are drawn from `results/paper/benchmark` and
`results/paper/ablation`. `experiments/sweeps.sh` re-runs those sweeps from
scratch. For each scenario it
first plans a single unconstrained agent (giving L_ref and U_ref), then runs
every planner with the bound at 100 % down to 30 % of U_ref. The sweep files
run up to 4 planners at once and each CONSORT run can use about 6 GB, so on a
smaller machine lower `workers:` in `experiments/configs/benchmark/sweep*.yaml`. Success rates
and lengths reproduce exactly. Wall-clock times (Figs. 4 and 6) depend on the
machine, and a much slower machine can hit the 1200 s per-run limit.

**Fig. 8** is drawn from the 30 simulated trials in `results/paper/montecarlo`.
Flying them again needs HoloOcean and a GPU; see [`src/sim/README.md`](../src/sim/README.md).

**Table II** (the estimator's noise) is derived from the planner config by
`FactorGraph.calibrate_to_planner` in `src/sim/factor_graph.py`; the values are in
`results/paper/RESULTS.md`.
