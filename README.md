# A Unified Hybrid Aerial–Aquatic Swarm Architecture

Integrating Biomimetic Locomotion, Hierarchical Path Planning, and an Adaptive
Cross-Domain Communication Protocol.

**Authors:** I. Harihara Sudar, Harsh Soni, Chinmay K, Rishi Charan,
Laxmi Narayana Charan, and Mervin Joe Thomas
*(Robotics Lab, Dept. of Mechanical Engineering, National Institute of
Technology Karnataka — NITK, Surathkal, India).*
Corresponding author: Mervin Joe Thomas (`mervinthomas@nitk.edu.in`).

This repository is the complete, reproducible companion to the journal paper:
the manuscript, the swarm-formation simulation, the physical-robot CAD, and the
demo videos in one place.

---

## Abstract

Single-medium robots are fundamentally constrained: aerial drones offer rapid
mobility and wide situational awareness but cannot sense beneath the surface,
while aquatic robots offer endurance, stealth, and precise underwater
maneuvering but suffer from slow, latency-bound communication and difficult
surface transitions. This work presents a **unified hybrid aerial–aquatic swarm
architecture** that couples a school of biomimetic robotic fish with a team of
autonomous quadrotors into a single, cooperative, cross-domain system, organized
into five layers — physical modeling, hardware and actuation, control and
coordination, hybrid communication, and simulation/visualization — with three
principal contributions:

1. **Biomimetic locomotion** — undulatory propulsion grounded in Lighthill's
   elongated-body theory and validated by an ALE moving-mesh CFD study (up to a
   **10 % gain in Froude propulsive efficiency** for two fish swimming in
   antiphase at 0.4 L spacing), realized in hardware through an ESP32-driven
   central pattern generator (CPG) with PID closure.
2. **Hierarchical path planning** — a two-level global-PSO + local-MPC scheme
   feeding the CPG for the fish and a cascaded ArduPilot/optical-flow stack for
   the GNSS-denied drone, with frustum-guided precision docking and magnetic
   auto-charging. A controlled graph-theoretic formation study (7 agents,
   5 obstacle seeds) shows that contracting the dynamic envelope to fish-like
   values **roughly halves RMS jerk** while preserving formation accuracy and a
   **100 % success rate**.
3. **The ADAPTH protocol** — *Adaptive Dynamic Aerial–Aquatic Path-Tracking and
   Hybridization*: a relay-based "wait-and-verify" cross-domain communication
   scheme bridging low-latency RF and high-latency acoustic links, sustaining
   **85 % end-to-end communication success** in noisy conditions.

The complete system is validated in ROS2/Gazebo with RViz visualization and on
hardware prototypes.

---

## Repository map

| Path | Contents |
| --- | --- |
| [`papers/unified/`](papers/unified/) | The journal manuscript (`main.tex`, `main.pdf`, figures, `refs.bib`). |
| [`papers/ascend/`](papers/ascend/) | The companion ASCEND aerial-platform paper (`main.tex`, `main.pdf`, figures). |
| [`simulation/`](simulation/) | The robotic-fish swarm-formation simulation (ROS/EGO-Swarm re-skin) and the baseline-vs-fish experiment. See [`simulation/PROJECT_GUIDE.md`](simulation/PROJECT_GUIDE.md). |
| [`hardware/cad/`](hardware/cad/) | Physical-robot CAD: SolidWorks parts/assemblies, STL meshes, DXF fabrication drawings, and Parasolid exports. |
| [`media/videos/`](media/videos/) | Demo clips (GIF) for YouTube and the journal supplement. See [`media/videos/README.md`](media/videos/README.md). |

---

## The simulation

`simulation/` is a robotic-fish re-skin of the ZJU-FAST-Lab **Swarm-Formation**
distributed formation planner, plus a controlled experiment quantifying how a
fish-like dynamic envelope changes swarm behaviour versus the original
quadrotor. Same planner, same environment, 7 agents, 5 obstacle seeds — only
the motion envelope differs:

| Metric | Baseline (quadrotor) | Robotic fish |
| --- | --- | --- |
| Max speed (m/s) | 1.581 ± 0.014 | 1.126 ± 0.047 |
| RMS jerk (m/s³) | 0.658 ± 0.116 | **0.338 ± 0.129** |
| Traversal time (s) | 44.9 ± 1.4 | 64.5 ± 4.3 |
| Formation RMSE, mean (m) | 0.176 ± 0.023 | 0.217 ± 0.026 |
| Success rate | 100 % | 100 % |

The fish envelope roughly halves RMS jerk (smoother, lower-effort motion) at the
cost of ~44 % longer traversal, while formation accuracy and safety stay within
noise of the baseline. Full method, reproduction commands, and per-seed data are
in [`simulation/EXPERIMENTS.md`](simulation/EXPERIMENTS.md); the conversion
itself is documented in [`simulation/ROBOTIC_FISH.md`](simulation/ROBOTIC_FISH.md).
A focused write-up of just this study lives in
[`simulation/paper/`](simulation/paper/) ("Dynamic-Envelope Re-Tuning of a
Graph-Theoretic Swarm") — distinct from the broader unified manuscript in
`papers/unified/`.

---

## The hardware / CAD

`hardware/cad/` holds the mechanical design of the physical platform, organized
as exported from OSF:

| Folder | Files | Use |
| --- | --- | --- |
| `SOLIDWORKS FILES/` | 37 `.SLDPRT` parts + 2 `.SLDASM` assemblies | Native editable source (SolidWorks). |
| `FABRICATION FILES/` | 15 `.STL` + 9 `.DXF` | 3D-printing meshes and 2D laser/water-jet cut profiles. |
| `PARASOLID FILES/` | 2 `.x_t` | Neutral CAD exchange format (open `Assembly #1.x_t` in any CAD package). |

> To view without SolidWorks: open the `.STL` files in any free mesh viewer
> (e.g. the system 3D viewer, MeshLab, or an online STL viewer), or import the
> `PARASOLID FILES/*.x_t` into FreeCAD / Fusion / Onshape.

---

## Videos (for YouTube / journal supplement)

All demo clips are gathered in [`media/videos/`](media/videos/) as GIFs:

| File | Shows |
| --- | --- |
| `01_sim_hexagon_formation.gif` | Seven-agent hexagon school holding formation. |
| `02_sim_hexagon_forest_navigation.gif` | The school weaving through the random-forest obstacle field. |
| `03_sim_set_goal_navigation.gif` | Interactive "2D Nav Goal" target selection in RViz. |
| `04_experiment_baseline_vs_fish_playback.gif` | Side-by-side baseline-quadrotor vs robotic-fish playback (seed 1). |

For YouTube, convert each GIF to MP4 (e.g.
`ffmpeg -i in.gif -movflags +faststart -pix_fmt yuv420p out.mp4`); they are kept
as GIF here so they render inline on GitHub and in the paper.

---

## How to cite

```bibtex
@article{sudar2026unified,
  title   = {A Unified Hybrid Aerial--Aquatic Swarm Architecture Integrating
             Biomimetic Locomotion, Hierarchical Path Planning, and an Adaptive
             Cross-Domain Communication Protocol},
  author  = {Sudar, I. Harihara and Soni, Harsh and K, Chinmay and
             Charan, Rishi and Charan, Laxmi Narayana and Thomas, Mervin Joe},
  year    = {2026}
}
```

## Licensing & attribution

The simulation under `simulation/` is derived from
[ZJU-FAST-Lab/Swarm-Formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation)
and is distributed under the **GPLv3** license (see `simulation/LICENSE` and the
top-level [`LICENSE`](LICENSE)). The paper text, figures, and CAD models are the
work of the authors above; please cite the paper before reuse.
