# src/sim: HoloOcean closed-loop validation

The simulator and factor-graph estimator behind Sec. VI. `monte_carlo.py`
flies a plan N times in [HoloOcean](https://github.com/byu-holoocean/HoloOcean)
with torpedo-AUV dynamics and a waypoint follower that steers on a GTSAM
factor-graph estimate, and writes one `mc<i>/` folder per trial
(`sim_report.yaml`, `run_log.npz`) inside the plan's directory.
`figures/scripts/fig8_montecarlo.py` reads those folders.

This is a snapshot of the `multiagent_base` stack, including the estimator
changes the paper's trials were flown with: world-frame poses, a relative-pose
communication factor, the support's buffered nodes seeded from its own
solution, and factor noise derived from the planner config
(`FactorGraph.calibrate_to_planner`). The original stack (factor graph and
vehicle dynamics) is by Kalliyan Velasco and Ryan Bartholomew.

## Install

Linux with a GPU, Python 3.12:

```bash
cd src/sim
python3.12 -m venv .venv
.venv/bin/pip install -r requirements-sim.txt
# then HoloOcean 2.3.0 from source and its Ocean world, see requirements-sim.txt
```

## Fly the paper's trials

```bash
cd src/sim
cp -r ../../results/paper/montecarlo ../../results/new/montecarlo       # keep the shipped trials intact
N_TRIALS=30 .venv/bin/python monte_carlo.py ../../results/new/montecarlo
cd ../.. && .venv/bin/python figures/scripts/fig8_montecarlo.py results/new/montecarlo --case=behind_wall --nstd=3 --dev=0
```

Trial i uses noise seed 1000 + i, about 1 minute each. On the machine that
produced the paper, re-flying seed 1000 gave a report identical to the shipped
`mc0`. On other GPUs and drivers the physics may differ slightly, so expect a
statistical rather than trial-for-trial match.

`main.py <plan_dir>` flies one trial; useful environment variables are `SEED`,
`OUT_TAG`, `HEADLESS`, and `USE_TRUTH=1` (steer on ground truth instead of the
estimate).
