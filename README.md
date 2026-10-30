<div align="center">

# CONSORT

### Uncertainty-Constrained Multi-Robot Planning for Cooperative Localization in GPS-Denied Environments

[Tony Zhang](https://www.linkedin.com/in/tony-hu-zhang)<sup>1,2</sup>,
[Ryan Bartholomew](https://sites.google.com/view/ryan-m-bartholomew)<sup>1</sup>,
[Kalliyan Velasco](https://frostlab.byu.edu/people/kalliyan-velasco/)<sup>1</sup>,
[Joshua Mangelson](https://frostlab.byu.edu/people/josh-mangelson/)<sup>1</sup>,
[Brady Moon](https://frostlab.byu.edu/people/brady-moon/)<sup>1</sup>

<sup>1</sup>[Brigham Young University](https://www.byu.edu/) &nbsp;&nbsp; <sup>2</sup>[University of Utah](https://www.utah.edu/)

[FROST Lab](https://frostlab.byu.edu)

[![Project page](https://img.shields.io/badge/Project-Page-0f6fb3?style=for-the-badge)](https://frostlab.byu.edu/consort-planner/)
![Paper: coming soon](https://img.shields.io/badge/Paper-Coming_Soon-lightgrey?style=for-the-badge&logo=arxiv&logoColor=white)
[![Video](https://img.shields.io/badge/Video-YouTube-ff0000?style=for-the-badge&logo=youtube&logoColor=white)](https://youtu.be/prarYmi_3y4)

<img src="website/src/assets/figures/fig1-escort-problem.webp" width="560" alt="A primary agent's path passes above an obstacle to the goal while a support agent dips below it toward a landmark, takes a reading, and relays position data to the primary near the goal.">

<sub>The support agent localizes with a landmark and relays position data, so the primary agent can take a more direct path to the goal.</sub>

</div>

---

CONSORT is a centralized planner for the escort problem. A primary agent must reach a goal in minimum time with its localization uncertainty under a terminal bound. Support agents have no goals of their own and are routed only to keep the primary's uncertainty under that bound.

CONSORT optimizes over the agents' mean trajectories and propagates covariance along them only to evaluate the constraints. It first searches a heading-aware hexagonal lattice jointly over all agents, then refines the result into B-spline trajectories. Both stages enforce the curvature bound, chance-constrained obstacle avoidance, and the terminal uncertainty bound.

## Getting started

The planner needs [Julia 1.12](https://julialang.org/downloads/). Commands are run from the repository root.

```bash
git clone https://github.com/BYU-FROST-Lab/consort-planner.git && cd consort-planner
julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

To plan the mission flown in the paper's simulation (Sec. VI), which takes about 2 minutes:

```bash
OUTPUT_DIR=results/new/my_run julia --project=. experiments/generate_plan.jl experiments/configs/sim_mission
```

This writes `results/new/my_run/hexspline_cl/results.yaml` (primary length and
terminal uncertainty), `csv/main_ctrls.csv` (the B-spline control points of
every agent) and figures. `tests/run_tests.sh` runs the planner's tests in about a minute.

## Planning your own scenario

A config is a folder of flat `key: value` files: `main.yaml` holds the scenario,
uncertainty model and constraints, and `<planner>.yaml` the planner's own
settings. Start from a copy of `experiments/configs/sim_mission`; the comments in
its `main.yaml` and `hexspline_cl.yaml` describe every key. The ones you are
most likely to change:

| Key | Meaning |
|---|---|
| `landmark_scenario` | a scenario in `src/planner/scenario_generation.jl` (`behind_wall`, `ladder_r3`, ...), `random`, or `manual` (geometry given inline) |
| `algorithms` | `hexspline_cl` (CONSORT), `greedy`, `formation`, `sequential`, `clgbt`, or the ablation arms `discrete_only`, `straight_cont` |
| `num_agents` | team size; the last agent is the primary |
| `unc_radius_threshold` | the terminal bound ū in metres |
| `astar_iteration_limit` | expansion budget of the joint search (memory grows about 1.8 GB per 300k expansions) |

## Comparing with the baselines

The paper compares CONSORT with four baselines (Sec. V-B), all in `src/planner/algorithms/`:

- **Greedy** (`greedy`). The primary takes its shortest route; the support takes whichever next step most reduces the primary's uncertainty.
- **Formation** (`formation`). The primary plans alone while the support holds a fixed position beside it.
- **Sequential** (`sequential`). Agents plan one at a time: the primary, then the support against it, then the primary once more.
- **CL-GBT** (`clgbt`). A sampling-based tree over both agents' joint state, after Theurkauf et al., instead of a lattice search.

To run several planners on the same scenario, list them in `algorithms`, separated by commas,
for example `algorithms: hexspline_cl, greedy, sequential`. Each writes to its own folder under `OUTPUT_DIR`.

## How the code maps to the paper

| Paper | Code |
|---|---|
| Covariance propagation, landmark update, covariance intersection, Eqs. (2)-(5) | `src/planner/covariance.jl` |
| Chance-constrained obstacle clearance, Eq. (6b) | `src/planner/obstacles.jl`, `src/planner/minvo.jl` |
| Heading-aware hexagonal lattice | `build_hex_graph`, `src/planner/graph.jl` |
| Joint search and B-spline refinement, Algorithm 1 | `joint_astar`, `optimize_continuous`, `src/planner/algorithms/hexspline_cl.jl` |
| Random benchmark scenarios | `generate_scenario`, `src/planner/scenario_generation.jl` |

## Repository layout

```
src/planner/    the planner: CONSORT (algorithms/hexspline_cl.jl) and the four baselines
src/sim/        HoloOcean simulator and factor-graph estimator (Sec. VI); see src/sim/README.md
experiments/    the planner's entry point, its configs, and the scripts that reproduce the paper
results/paper/  the runs behind every figure and table in the paper
figures/        the scripts that draw the figures, and the published figures
tests/          planner tests
website/        the project page; see website/README.md
```

## Reproducing the paper

Every figure and table in the paper has a script in `experiments/` that
reproduces it and checks the result against the paper, and the runs behind
them are in `results/paper/`. [`experiments/README.md`](experiments/README.md)
covers the extra setup (Python 3.12 for the figures), how long each script
takes, and what it checks.

## Citation

If you use this code, please cite the paper. The up-to-date BibTeX entry is on the [project page](https://frostlab.byu.edu/consort-planner/#bibtex); until the preprint is posted it is:

```bibtex
@article{zhang2026consort,
  title   = {{CONSORT}: Uncertainty-Constrained Multi-Robot Planning for Cooperative Localization in {GPS}-Denied Environments},
  author  = {Zhang, Tony and Bartholomew, Ryan and Velasco, Kalliyan and Mangelson, Joshua and Moon, Brady},
  journal = {arXiv preprint},
  year    = {2026}
}
```

## Acknowledgments

This work was supported by the NSF-funded BYU Robotics Pioneers REU program (Award No. 2447307) and in part by NAVSEA NEEC (Award No. N00174-23-1-0005) in coordination with NSWC PCD.
