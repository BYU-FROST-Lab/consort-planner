# Results as published

The PDFs in `figures/paper/` are the figures the paper's figures were made from
(Figs. 1, 2, 3, 5 and 8 were then labeled and assembled in Illustrator; the
data is unchanged). The drivers in `experiments/` regenerate each of them
into `figures/new/`; the regenerated PDFs render pixel-identical to these.
Fig. 7 is a screenshot of the simulator and is not a result, so it is not
included.

| Paper | File | Script |
|---|---|---|
| Fig. 1 | `figures/paper/fig1.pdf` | `experiments/fig1.sh` |
| Fig. 2 | `figures/paper/fig2.pdf` | `experiments/fig2.sh` |
| Fig. 3 | `figures/paper/fig3a.pdf` (success rate), `fig3b.pdf` (path length) | `experiments/figs3to6.sh` |
| Fig. 4 | `figures/paper/fig4.pdf` | `experiments/figs3to6.sh` |
| Fig. 5 | `figures/paper/fig5a.pdf`, `fig5b.pdf` | `experiments/figs3to6.sh` |
| Fig. 6 | `figures/paper/fig6.pdf` | `experiments/figs3to6.sh` |
| Fig. 8 | `figures/paper/fig8.pdf` | `experiments/fig8.sh` |

## Fig. 2 ladder

The reference path is 1712.1 m long, with terminal uncertainty U_ref = 13.75 m.

| Panel | Level | Bound ū (m) | Primary length (m) | Terminal σ (m) |
|---|---|---|---|---|
| (a) | 100 % | 13.75 | 1741 | 9.73 |
| (b) | 70 % | 9.62 | 1761 | 8.89 |
| (c) | 50 % | 6.87 | 1822 | 6.66 |
| (d) | 30 % | 4.12 | 1890 | 3.91 |

## Success rate (Figs. 3a and 5a)

Percent of scenarios solved at each constraint level, as a fraction of U_ref.

Benchmark, 50 scenarios:

| Planner | 100 | 90 | 80 | 70 | 60 | 50 | 40 | 30 |
|---|---|---|---|---|---|---|---|---|
| CONSORT | 100 | 100 | 100 | 100 | 96 | 80 | 60 | 40 |
| Formation | 98 | 94 | 90 | 72 | 62 | 54 | 32 | 22 |
| Sequential | 100 | 100 | 100 | 94 | 74 | 52 | 24 | 16 |
| CL-GBT | 100 | 90 | 80 | 70 | 50 | 32 | 10 | 6 |
| Greedy | 90 | 26 | 20 | 14 | 8 | 4 | 4 | 4 |

Ablation, 30 scenarios:

| Arm | 100 | 90 | 80 | 70 | 60 | 50 | 40 | 30 |
|---|---|---|---|---|---|---|---|---|
| CONSORT (full) | 100 | 100 | 100 | 100 | 97 | 87 | 60 | 43 |
| Discrete only | 100 | 100 | 100 | 100 | 93 | 80 | 60 | 43 |
| Straight seed + cont. | 53 | 33 | 23 | 7 | 3 | 0 | 0 | 0 |

## Table I: primary path length / L_ref, mean ± SD over solved scenarios

| Method | 100 % | 70 % | 50 % | 30 % |
|---|---|---|---|---|
| CONSORT | 1.14 ± 0.09 | 1.22 ± 0.23 | 1.34 ± 0.26 | 1.41 ± 0.21 |
| Formation | 1.22 ± 0.12 | 1.34 ± 0.25 | 1.33 ± 0.16 | 1.48 ± 0.45 |
| Sequential | 1.18 ± 0.14 | 1.41 ± 0.92 | 1.36 ± 0.50 | 2.76 ± 1.51 |
| CL-GBT | 1.47 ± 0.33 | 1.78 ± 0.43 | 2.01 ± 0.41 | 2.07 ± 0.26 |
| Greedy | 1.24 ± 0.10 | 1.21 ± 0.06 | 1.13 ± 0.01 | 1.13 ± 0.01 |
| CONSORT (full), ablation set | 1.12 ± 0.14 | 1.17 ± 0.23 | 1.22 ± 0.22 | 1.29 ± 0.22 |
| Discrete only | 1.20 ± 0.15 | 1.23 ± 0.23 | 1.25 ± 0.22 | 1.29 ± 0.22 |
| Straight seed + cont. | 0.99 ± 0.04 | 1.00 ± 0.00 | - | - |

## Monte Carlo, Sec. VI-C (behind-wall plan, 30 trials)

| Quantity | Value |
|---|---|
| Reached the goal | 30 / 30 |
| Collision-free | 30 / 30 |
| Estimator's terminal σ under the 1.8 m bound | 28 / 30 (misses at 1.85 and 1.96 m) |
| Planner's predicted terminal σ | 1.80 m |
| Terminal σ of the true error (sample covariance) | 1.70 m |
| Estimator's mean terminal σ | 1.51 m |

## Table II: factor noise of the simulated estimator

Derived from the plan's config by `FactorGraph.calibrate_to_planner` in
`src/sim/factor_graph.py`, with one pose node per 5 m. The simulator prints these
values when it starts.

| Factor | σ |
|---|---|
| Odometry (between, per 5 m node) | 1°/axis; 0.22, 0.02, 0.01 m |
| Depth (unary) | 0.5 m |
| Compass (unary) | 0.86°/axis |
| Landmark azimuth / elevation / range | 1.05°, 1.05°, 2.09 m; r < 100 m |
| Inter-agent relative pose | 0.86°/axis; 0.5/√w m |
| Initial pose prior | 10°/axis; 1.15, 0.76, 0.5 m |
