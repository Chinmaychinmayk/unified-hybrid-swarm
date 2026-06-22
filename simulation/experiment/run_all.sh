#!/bin/bash
# Headless experiment driver: baseline (quadrotor) vs fish, across obstacle seeds.
# Runs inside the robotic-fish container. Writes to /out/<cond>/seed<k>/.
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

LDIR=/workspace/src/src/planner/plan_manage/launch
RIS=$LDIR/run_in_sim.launch
HEX=$LDIR/normal_hexagon.launch
RESULTS=/out

SEEDS=${SEEDS:-"1 2 3 4 5"}
WARMUP=${WARMUP:-6}
export REC_SECONDS=${REC_SECONDS:-60}
export N_AGENTS=7

# autonomous global-waypoint mode
sed -i 's/name="flight_type" value="[0-9]"/name="flight_type" value="2"/' "$RIS"

cleanup() {
  pkill -f normal_hexagon 2>/dev/null
  pkill -f roslaunch 2>/dev/null
  pkill -f ego_planner 2>/dev/null
  pkill -f traj_server 2>/dev/null
  pkill -f random_forest 2>/dev/null
  pkill -f swarm_bridge 2>/dev/null
  pkill -f nodelet 2>/dev/null
  pkill -f so3 2>/dev/null
  pkill -f rosmaster 2>/dev/null
  pkill -f rosout 2>/dev/null
  sleep 3
}

run_one() {
  local cond=$1 mv=$2 ma=$3 seed=$4
  local outdir=$RESULTS/$cond/seed$seed
  mkdir -p "$outdir"
  sed -i "s/name=\"max_vel\" value=\"[0-9.]*\"/name=\"max_vel\" value=\"$mv\"/" "$RIS"
  sed -i "s/name=\"max_acc\" value=\"[0-9.]*\"/name=\"max_acc\" value=\"$ma\"/" "$RIS"
  sed -i "s#name=\"ObstacleShape/seed\" value=\"[0-9]*\"#name=\"ObstacleShape/seed\" value=\"$seed\"#" "$HEX"

  echo "=== $cond seed=$seed :: $(grep -o 'name="max_vel" value="[0-9.]*"' "$RIS") $(grep -o 'name="max_acc" value="[0-9.]*"' "$RIS") $(grep -o 'ObstacleShape/seed" value="[0-9]*"' "$HEX") ==="

  roscore >"$outdir/roscore.log" 2>&1 &
  sleep 5
  roslaunch ego_planner normal_hexagon.launch >"$outdir/sim.log" 2>&1 &
  sleep "$WARMUP"
  OUT="$outdir" python3 /exp/record_exp.py >"$outdir/record.log" 2>&1
  echo "  ... recorded; cleaning up"
  cleanup
}

t_start=$(date +%s)
for seed in $SEEDS; do
  run_one baseline 1.5 8.0 "$seed"
  run_one fish     1.0 2.5 "$seed"
done
echo "ALL DONE in $(( $(date +%s) - t_start ))s"
