# CONSORT

**Cooperative Navigation with Support-Optimized Routing and Trajectories for GPS-Denied Escort Missions**

[Project page](https://frostlab.byu.edu/consort-planner/) · Paper: coming soon

CONSORT is a centralized planner for the escort problem. A primary agent must reach a goal in minimum time with its localization uncertainty under a terminal bound. Support agents have no goals of their own and are routed only to keep the primary's uncertainty under that bound. CONSORT optimizes over the agents' mean trajectories and propagates covariance along them only to evaluate the constraints. It first searches a heading-aware hexagonal lattice jointly over all agents, then refines the result into B-spline trajectories. Both stages enforce the curvature bound, chance-constrained obstacle avoidance, and the terminal uncertainty bound.

## Repository layout

| Path | Contents |
|---|---|
| [`website/`](website/) | The project page: Astro and Tailwind CSS, deployed to GitHub Pages. See [`website/README.md`](website/README.md). |

The Julia implementation, the 50 benchmark scenarios, and the HoloOcean simulation setup will be added here.

## Citation

If you use this code, please cite the paper. The BibTeX entry is on the [project page](https://frostlab.byu.edu/consort-planner/#bibtex).
