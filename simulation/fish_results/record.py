#!/usr/bin/env python3
"""Record robotic-fish swarm odometry headless and plot trajectories + formation.

Subscribes to /drone_<i>_visual_slam/odom for i in 0..N-1, logs positions for a
fixed duration, then writes a top-down trajectory PNG and a formation-metrics
summary to /out.
"""
import os
import time
import rospy
from nav_msgs.msg import Odometry

N = 7
DURATION = float(os.environ.get("REC_SECONDS", "45"))
OUT = "/out"

data = {i: [] for i in range(N)}  # i -> list of (t, x, y, z)
t0 = None


def make_cb(i):
    def cb(msg):
        global t0
        now = msg.header.stamp.to_sec()
        if t0 is None:
            t0 = now
        p = msg.pose.pose.position
        data[i].append((now - t0, p.x, p.y, p.z))
    return cb


def main():
    rospy.init_node("fish_recorder", anonymous=True)
    for i in range(N):
        rospy.Subscriber("/drone_%d_visual_slam/odom" % i, Odometry, make_cb(i), queue_size=50)
    rospy.loginfo("recording for %.0fs ..." % DURATION)
    end = time.time() + DURATION
    rate = rospy.Rate(50)
    while time.time() < end and not rospy.is_shutdown():
        rate.sleep()

    # --- write CSVs ---
    for i in range(N):
        with open(os.path.join(OUT, "drone_%d.csv" % i), "w") as f:
            f.write("t,x,y,z\n")
            for t, x, y, z in data[i]:
                f.write("%.3f,%.4f,%.4f,%.4f\n" % (t, x, y, z))

    counts = {i: len(data[i]) for i in range(N)}
    rospy.loginfo("samples per fish: %s" % counts)

    # --- formation metrics: nearest-neighbour spacing over time ---
    import math

    def positions_at(idx_frac):
        pts = []
        for i in range(N):
            d = data[i]
            if not d:
                return None
            k = min(int(idx_frac * (len(d) - 1)), len(d) - 1)
            pts.append(d[k][1:4])
        return pts

    def spread(pts):
        # mean pairwise distance + displacement of centroid x
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        ds = []
        for a in range(len(pts)):
            for b in range(a + 1, len(pts)):
                dx = pts[a][0] - pts[b][0]
                dy = pts[a][1] - pts[b][1]
                dz = pts[a][2] - pts[b][2]
                ds.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        return cx, cy, (sum(ds) / len(ds) if ds else 0.0), (min(ds) if ds else 0.0), (max(ds) if ds else 0.0)

    summary_lines = ["# Robotic-fish school formation summary", ""]
    for label, frac in [("start", 0.0), ("mid", 0.5), ("end", 1.0)]:
        pts = positions_at(frac)
        if pts is None:
            summary_lines.append("%-5s : no data" % label)
            continue
        cx, cy, mean_d, min_d, max_d = spread(pts)
        summary_lines.append(
            "%-5s : centroid=(%.2f, %.2f)  mean_pair_dist=%.2f  min=%.2f  max=%.2f"
            % (label, cx, cy, mean_d, min_d, max_d)
        )
    summary = "\n".join(summary_lines) + "\n"
    with open(os.path.join(OUT, "summary.txt"), "w") as f:
        f.write(summary)
    print(summary)

    # --- plot ---
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(11, 5))
        colors = plt.cm.tab10.colors
        for i in range(N):
            d = data[i]
            if not d:
                continue
            xs = [p[1] for p in d]
            ys = [p[2] for p in d]
            ax.plot(xs, ys, "-", color=colors[i % 10], lw=1.5, label="fish %d" % i)
            ax.plot(xs[0], ys[0], "o", color=colors[i % 10], ms=6)
            ax.plot(xs[-1], ys[-1], "s", color=colors[i % 10], ms=6)
        ax.set_title("Robotic-fish school: top-down trajectories (o=start, s=end)")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncol=2, fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "fish_trajectories.png"), dpi=120)
        print("wrote fish_trajectories.png")
    except Exception as e:
        print("plot failed: %s" % e)


if __name__ == "__main__":
    main()
