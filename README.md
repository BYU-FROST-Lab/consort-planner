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

CONSORT is a centralized planner for the escort problem. A primary agent must reach a goal in minimum time with its localization uncertainty under a terminal bound. Support agents have no goals of their own and are routed only to keep the primary's uncertainty under that bound. CONSORT optimizes over the agents' mean trajectories and propagates covariance along them only to evaluate the constraints. It first searches a heading-aware hexagonal lattice jointly over all agents, then refines the result into B-spline trajectories. Both stages enforce the curvature bound, chance-constrained obstacle avoidance, and the terminal uncertainty bound.

## Repository layout

| Path | Contents |
|---|---|
| [`code/`](code/) | The Julia planner (CONSORT and the four baselines), the HoloOcean simulator, the experiment drivers, and the data behind every figure in the paper. See [`code/README.md`](code/README.md) for setup and how to reproduce the results. |
| [`website/`](website/) | The project page: Astro and Tailwind CSS, deployed to GitHub Pages. See [`website/README.md`](website/README.md). |

## Citation

If you use this code, please cite the paper. The BibTeX entry is on the [project page](https://frostlab.byu.edu/consort-planner/#bibtex).
