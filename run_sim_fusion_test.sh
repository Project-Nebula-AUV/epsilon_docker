#!/bin/bash
# Headless sim test of depth_fusion in the loop. Runs simulator_node (fusion
# mode) + depth_fusion for a given world_z_sign, drives vertical motion, and
# scores fused /sensors/depth vs /sim/true_depth.
#   run_sim_fusion_test.sh <world_z_sign> <duration> <success_prob>
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash
export SDL_VIDEODRIVER=dummy   # headless pygame

SIGN="${1:-1.0}"
DUR="${2:-45}"
PSUCC="${3:-0.10}"

for p in "[s]imulator_node" "[d]epth_fusion" "[s]im_depth_monitor" "[s]ubmarine_node"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

ros2 run robosub simulator_node --ros-args \
    -p fuse_depth:=true -p depth_success_prob:="$PSUCC" > /tmp/sim.log 2>&1 &
SIM_PID=$!
ros2 run epsilon_sensors depth_fusion --ros-args \
    -r /depth:=/sensors/depth -p world_z_sign:="$SIGN" > /tmp/simfusion.log 2>&1 &
FUS_PID=$!
sleep 6

python3 /home/robosub/robosub_ws/sim_depth_monitor.py "$DUR" "world_z_sign=$SIGN"

kill -9 "$SIM_PID" "$FUS_PID" 2>/dev/null
for p in "[s]imulator_node" "[d]epth_fusion"; do pkill -9 -f "$p" 2>/dev/null; done

echo "=== simfusion.log (tail) ==="
tail -6 /tmp/simfusion.log
echo "=== sim.log (head) ==="
head -4 /tmp/sim.log
