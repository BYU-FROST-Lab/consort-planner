"""Fly the same plan N times with different noise draws, then plot the mean
flown path and a shaded cross-track std-dev band around it.

    .venv/bin/python monte_carlo.py <run_dir>
    # writes <run_dir>/monte_carlo/fig5_monte_carlo.{png,pdf}

Each trial is its own `main.py` subprocess (HEADLESS=1, uncapped -- this is
scoring only, so RECORD stays off and no close-up cameras are even attached,
FIGURES=0 since only run_log.npz gets read here, not
the per-trial figures main.py normally writes) with a distinct SEED, so the
noise draws actually differ between trials -- main.py's SEED otherwise pins
every run to the same np.random.seed(1000) and would make every trial
identical.

Trials run sequentially, not in parallel -- each one spawns its own
HoloOcean/Unreal process, and several running at once would fight over the
same CPU/GPU. Expect roughly N_TRIALS x (one full run's time), most of it the
actual simulation/estimator compute, not something this script can skip.

N_TRIALS (default 20) and OUT_TAG_PREFIX (default "mc") are still env-var
overrides, matching this repo's convention for the handful of knobs worth
varying without editing code.
"""
import os
import subprocess
import sys

import numpy as np

import viz

RUN_DIR = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("RUN_DIR", "")
N_TRIALS = int(os.environ.get("N_TRIALS", "20"))
OUT_TAG_PREFIX = os.environ.get("OUT_TAG_PREFIX", "mc")
GRID_POINTS = 100
SIGMA_MULT = 2  # band width in standard deviations; 1 was too thin to see


def run_trials():
    for i in range(N_TRIALS):
        out_tag = f"{OUT_TAG_PREFIX}{i}"
        print(f"trial {i + 1}/{N_TRIALS} (SEED={1000 + i}, OUT_TAG={out_tag})")
        env = dict(os.environ, RUN_DIR=RUN_DIR, OUT_TAG=out_tag, SEED=str(1000 + i),
                   HEADLESS="1", RECORD="0", LIVE_2D="0", FIGURES="0")
        subprocess.run([sys.executable, "-u", "main.py"], env=env, check=True,
                        cwd=os.path.dirname(os.path.abspath(__file__)))
    return [os.path.join(RUN_DIR, f"{OUT_TAG_PREFIX}{i}") for i in range(N_TRIALS)]


def aggregate(trial_dirs):
    logs = [viz.Log(d) for d in trial_dirs]
    agg = {}
    n_used = {}
    for name in logs[0].agents:
        arcs = [log.node(name, "arc") for log in logs]
        truths = [log.node(name, "truth") for log in logs]
        usable = [(a, t) for a, t in zip(arcs, truths) if len(a) > 1]
        if len(usable) < 2:
            if name == logs[0].primary:
                sys.exit("fewer than 2 trials produced usable trajectories -- can't compute a std band")
            print(f"warning: fewer than 2 usable trials for {name}, skipping its band")
            continue

        # Cap the shared grid at the SHORTEST trial's flown arc, not the
        # longest, so the band never extrapolates past a trial that stalled
        # early.
        max_arc = min(a[-1] for a, _ in usable)
        grid = np.linspace(0.0, max_arc, GRID_POINTS)

        paths = np.array([
            np.column_stack([np.interp(grid, a, t[:, 0]), np.interp(grid, a, t[:, 1])])
            for a, t in usable
        ])  # (n_trials, GRID_POINTS, 2)

        mean_xy = paths.mean(axis=0)  # (GRID_POINTS, 2)

        # Local tangent via central differences -> normal -> per-trial
        # deviation projected onto that normal. This is a "tube" around the
        # mean path, not independent x/y std bands, which would be wrong for
        # anything but a straight line.
        tangent = np.gradient(mean_xy, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
        cross = np.einsum("tgk,gk->tg", paths - mean_xy, normal)  # (n_trials, GRID_POINTS)
        std_cross = SIGMA_MULT * cross.std(axis=0)

        agg[name] = (mean_xy, std_cross, normal)
        n_used[name] = len(usable)

    return logs[0], agg, n_used


def main():
    if not RUN_DIR:
        sys.exit("usage: python monte_carlo.py <run_dir>\n"
                 "  <run_dir> is a landmark_planning results/<timestamp> directory")
    trial_dirs = run_trials()
    log, agg, n_used = aggregate(trial_dirs)
    for name, n in n_used.items():
        print(f"aggregated {n}/{N_TRIALS} trials for {name}")
    out_dir = os.path.join(RUN_DIR, "monte_carlo")
    paths = viz.fig_monte_carlo(log, agg, out_dir, nstd=SIGMA_MULT)
    for p in paths:
        print("wrote", p)


if __name__ == "__main__":
    main()
