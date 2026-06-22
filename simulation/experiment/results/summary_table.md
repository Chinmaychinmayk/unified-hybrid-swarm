# Baseline vs Robotic-fish: results

Identical EGO-Swarm formation planner, identical random-forest environment (7 agents, hexagon formation, 5 obstacle seeds). Only the dynamic-feasibility envelope differs: **baseline** max_vel=1.5 m/s, max_acc=8.0 m/s^2; **fish** max_vel=1.0 m/s, max_acc=2.5 m/s^2.

| Metric | Unit | Baseline (quadrotor) | Robotic fish |
|---|---|---|---|
| Max speed | m/s | 1.581 ± 0.014 | 1.126 ± 0.047 |
| Mean speed | m/s | 0.976 ± 0.030 | 0.726 ± 0.068 |
| Max accel | m/s^2 | 1.355 ± 0.249 | 0.854 ± 0.383 |
| Mean accel | m/s^2 | 0.218 ± 0.025 | 0.123 ± 0.031 |
| RMS jerk | m/s^3 | 0.658 ± 0.116 | 0.338 ± 0.129 |
| Path length | m | 51.420 ± 0.224 | 52.325 ± 0.372 |
| Traversal time | s | 44.919 ± 1.433 | 64.501 ± 4.327 |
| Formation RMSE (mean) | m | 0.176 ± 0.023 | 0.217 ± 0.026 |
| Formation RMSE (max) | m | 0.774 ± 0.078 | 0.799 ± 0.208 |
| Min inter-agent dist | m | 1.599 ± 0.354 | 1.705 ± 0.453 |
| Mean obstacle clearance | m | 1.116 ± 0.068 | 1.094 ± 0.077 |
| Near-miss fraction (<0.3 m) | - | 0.058 ± 0.014 | 0.066 ± 0.013 |
| Success rate | % | 100 | 100 |

_n = 5 seeds per condition. ± is std across seeds._
