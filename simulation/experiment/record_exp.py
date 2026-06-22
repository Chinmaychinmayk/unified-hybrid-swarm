#!/usr/bin/env python3
"""Record one trial of the swarm experiment (baseline or fish).

Records, for N agents:
  * odometry  (/drone_i_visual_slam/odom)            -> odom_i.csv   (t,x,y,z)
  * planning command (/drone_i_planning/pos_cmd)     -> cmd_i.csv
        (t, x,y,z, vx,vy,vz, ax,ay,az)  -- velocity/accel are the commanded,
        dynamically-constrained motion produced by the planner.
One obstacle-cloud snapshot (/map_generator/global_cloud) -> obstacles.csv (x,y,z).

Output dir comes from $OUT, duration from $REC_SECONDS.
"""
import os
import time
import struct
import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2

N = int(os.environ.get("N_AGENTS", "7"))
DURATION = float(os.environ.get("REC_SECONDS", "55"))
OUT = os.environ.get("OUT", "/out")

odom = {i: [] for i in range(N)}   # i -> (t,x,y,z)
cmd = {i: [] for i in range(N)}    # i -> (t,x,y,z,vx,vy,vz,ax,ay,az)
obstacles = []
t0 = None


def odom_cb(i):
    def cb(msg):
        global t0
        now = msg.header.stamp.to_sec()
        if t0 is None:
            t0 = now
        p = msg.pose.pose.position
        odom[i].append((now - t0, p.x, p.y, p.z))
    return cb


def cmd_cb(i):
    def cb(msg):
        global t0
        now = msg.header.stamp.to_sec()
        if t0 is None:
            t0 = now
        p, v, a = msg.position, msg.velocity, msg.acceleration
        cmd[i].append((now - t0, p.x, p.y, p.z, v.x, v.y, v.z, a.x, a.y, a.z))
    return cb


def cloud_cb(msg):
    global obstacles
    # keep the most complete snapshot (map is published incrementally)
    # locate x,y,z float32 field offsets
    off = {f.name: f.offset for f in msg.fields}
    ox, oy, oz = off.get("x", 0), off.get("y", 4), off.get("z", 8)
    step = msg.point_step
    data = msg.data
    pts = []
    for base in range(0, len(data), step):
        x = struct.unpack_from("<f", data, base + ox)[0]
        y = struct.unpack_from("<f", data, base + oy)[0]
        z = struct.unpack_from("<f", data, base + oz)[0]
        pts.append((x, y, z))
    if len(pts) > len(obstacles):
        obstacles = pts
        rospy.loginfo("captured %d obstacle points" % len(pts))


def main():
    rospy.init_node("exp_recorder", anonymous=True)
    for i in range(N):
        rospy.Subscriber("/drone_%d_visual_slam/odom" % i, Odometry, odom_cb(i), queue_size=100)
        rospy.Subscriber("/drone_%d_planning/pos_cmd" % i, PositionCommand, cmd_cb(i), queue_size=100)
    rospy.Subscriber("/map_generator/global_cloud", PointCloud2, cloud_cb, queue_size=2)

    rospy.loginfo("recording for %.0fs ..." % DURATION)
    end = time.time() + DURATION
    rate = rospy.Rate(100)
    while time.time() < end and not rospy.is_shutdown():
        rate.sleep()

    for i in range(N):
        with open(os.path.join(OUT, "odom_%d.csv" % i), "w") as f:
            f.write("t,x,y,z\n")
            for r in odom[i]:
                f.write("%.3f,%.4f,%.4f,%.4f\n" % r)
        with open(os.path.join(OUT, "cmd_%d.csv" % i), "w") as f:
            f.write("t,x,y,z,vx,vy,vz,ax,ay,az\n")
            for r in cmd[i]:
                f.write("%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % r)
    with open(os.path.join(OUT, "obstacles.csv"), "w") as f:
        f.write("x,y,z\n")
        for x, y, z in obstacles:
            f.write("%.4f,%.4f,%.4f\n" % (x, y, z))

    rospy.loginfo("odom samples: %s" % {i: len(odom[i]) for i in range(N)})
    rospy.loginfo("cmd  samples: %s" % {i: len(cmd[i]) for i in range(N)})


if __name__ == "__main__":
    main()
