# CONSORT

**Cooperative Navigation with Support-Optimized Routing and Trajectories for GPS-Denied Escort Missions**

Tony Zhang, Ryan Bartholomew, Kalliyan Velasco, Joshua Mangelson, Brady Moon
[FROST Lab](https://frostlab.byu.edu), Brigham Young University

[Project page](https://frostlab.byu.edu/consort-planner/) · Paper: coming soon

CONSORT is a centralized planner for the escort problem. A primary agent must reach a goal in minimum time with its localization uncertainty under a terminal bound. Support agents have no goals of their own and are routed only to keep the primary's uncertainty under that bound. CONSORT optimizes over the agents' mean trajectories and propagates covariance along them only to evaluate the constraints. It first searches a heading-aware hexagonal lattice jointly over all agents, then refines the result into B-spline trajectories. Both stages enforce the curvature bound, chance-constrained obstacle avoidance, and the terminal uncertainty bound.

## Repository layout

| Path | Contents |
|---|---|
| [`code/`](code/) | The Julia planner (CONSORT and the four baselines), the HoloOcean simulator, the experiment drivers, and the data behind every figure in the paper. See [`code/README.md`](code/README.md) for setup and how to reproduce the results. |
| [`website/`](website/) | The project page: Astro and Tailwind CSS, deployed to GitHub Pages. See [`website/README.md`](website/README.md). |

## Citation

If you use this code, please cite the paper. The BibTeX entry is on the [project page](https://frostlab.byu.edu/consort-planner/#bibtex).

## License

The code is released under the [Clear BSD License](LICENSE) (`BSD-3-Clause-Clear`). It grants no patent rights. The paper's text and figures reproduced on the project page are not covered by this license.
