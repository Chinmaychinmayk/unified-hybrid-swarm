#!/bin/bash
# Headless run of the robotic-fish school inside the container.
set -e
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

LAUNCH_DIR="$(rospack find ego_planner)/launch"
# Switch to autonomous global-waypoint mode (flight_type=2) so the school
# navigates without a manual 2D Nav Goal click.
sed -i 's/name="flight_type" value="3"/name="flight_type" value="2"/' "$LAUNCH_DIR/run_in_sim.launch"
echo "flight_type now: $(grep -o 'name=\"flight_type\" value=\"[0-9]\"' "$LAUNCH_DIR/run_in_sim.launch")"

# headless roscore + sim
roscore >/out/roscore.log 2>&1 &
sleep 4
roslaunch ego_planner normal_hexagon.launch >/out/launch.log 2>&1 &
LAUNCH_PID=$!
echo "launched normal_hexagon (pid $LAUNCH_PID), warming up..."
sleep 12

REC_SECONDS=${REC_SECONDS:-45} python3 /out/record.py 2>&1 | tee /out/record.log

echo "shutting down..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
pkill -f roslaunch 2>/dev/null || true
pkill -f rosmaster 2>/dev/null || true
echo "done"
