#!/usr/bin/env python3
"""Render an animated top-down playback of the recorded swarm traversal.

Plays baseline and fish side-by-side (seed 1) from the recorded odom CSVs so the
speed difference is visible: the two swarms start together and the fish falls
behind. Writes an animated GIF via matplotlib's Pillow writer.

Run inside the robotic-fish container (needs numpy/matplotlib/PIL):
    docker exec fish_exp python3 /exp/make_video.py
"""
import os, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

RES = os.environ.get("RES", "/out")
N = 7
CONDS = ["baseline", "fish"]
LABEL = {"baseline": "Baseline (quadrotor)", "fish": "Robotic fish"}
COLS = plt.cm.tab10.colors
DT = 0.9           # seconds between frames (playback step)
TAIL = 3.0         # seconds to keep after the slower swarm arrives


def load(path):
    if not os.path.exists(path):
        return None
    a = np.loadtxt(path, delimiter=",", skiprows=1)
    return a if a.ndim == 2 else a.reshape(1, -1)


def cond_data(cond, seed="seed1"):
    d = os.path.join(RES, cond, seed)
    odo = [load(os.path.join(d, f"odom_{i}.csv")) for i in range(N)]
    obs = load(os.path.join(d, "obstacles.csv"))
    return odo, obs


def main():
    data = {c: cond_data(c) for c in CONDS}
    # stop shortly after the slower swarm reaches the goal (x ~ +26)
    def arrival(odo):
        ts = [o[np.argmax(o[:, 1] > 21.0), 0] if (o[:, 1] > 21.0).any() else o[-1, 0] for o in odo]
        return max(ts)
    tmax = min(max(arrival(data[c][0]) for c in CONDS) + TAIL,
               max(max(o[:, 0][-1] for o in data[c][0]) for c in CONDS))
    tgrid = np.arange(0, tmax + DT, DT)

    fig, axes = plt.subplots(1, 2, figsize=(11, 3.7), sharex=True, sharey=True)
    artists = {}
    for ax, c in zip(axes, CONDS):
        odo, obs = data[c]
        if obs is not None:
            ob = obs[::2]   # subsample for lighter frames
            ax.scatter(ob[:, 0], ob[:, 1], s=0.6, c="0.75", alpha=0.6, linewidths=0)
        trails = [ax.plot([], [], "-", color=COLS[i % 10], lw=1.2, alpha=0.7)[0] for i in range(N)]
        dots = [ax.plot([], [], "o", color=COLS[i % 10], ms=6)[0] for i in range(N)]
        poly, = ax.plot([], [], "-", color="0.2", lw=0.8, alpha=0.5)
        ax.set_xlim(-30, 30); ax.set_ylim(-8, 8)
        ax.set_title(LABEL[c]); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.grid(True, alpha=0.3)
        # pre-interpolate agent positions onto tgrid
        X = np.zeros((len(tgrid), N)); Y = np.zeros((len(tgrid), N))
        for i in range(N):
            o = odo[i]
            X[:, i] = np.interp(tgrid, o[:, 0], o[:, 1])
            Y[:, i] = np.interp(tgrid, o[:, 0], o[:, 2])
        artists[c] = (trails, dots, poly, X, Y)
    txt = fig.suptitle("")

    def init():
        out = []
        for c in CONDS:
            trails, dots, poly, _, _ = artists[c]
            out += trails + dots + [poly]
        return out

    def update(k):
        out = []
        for c in CONDS:
            trails, dots, poly, X, Y = artists[c]
            for i in range(N):
                trails[i].set_data(X[:k + 1, i], Y[:k + 1, i])
                dots[i].set_data([X[k, i]], [Y[k, i]])
            order = list(range(N)) + [0]   # close the formation polygon
            poly.set_data(X[k, order], Y[k, order])
            out += trails + dots + [poly]
        txt.set_text(f"Swarm formation flight through random forest   t = {tgrid[k]:4.1f} s")
        return out + [txt]

    anim = FuncAnimation(fig, update, frames=len(tgrid), init_func=init, blit=False)
    out = os.path.join(RES, "figs_rviz", "swarm_playback.gif")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    anim.save(out, writer=PillowWriter(fps=12), dpi=60)
    print("wrote", out, f"({len(tgrid)} frames, {tmax:.0f}s @ {1/DT:.0f}x sampling)")


if __name__ == "__main__":
    main()
