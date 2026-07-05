#!/bin/bash
# Headless sim test of the style roll: simulator_node + submarine_node with
# ROBOSUB_STYLE_ROLL=720, start the mission, watch /sub/status until the
# mission advances past the outbound GateTask (or timeout).
#   run_style_roll_test.sh [duration_s]
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash
export SDL_VIDEODRIVER=dummy       # headless pygame
export ROBOSUB_STYLE_ROLL="${ROBOSUB_STYLE_ROLL:-720}"

DUR="${1:-240}"

for p in "[s]imulator_node" "[s]ubmarine_node" "[s]tyle_roll_monitor"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

ros2 run robosub simulator_node > /tmp/sr_sim.log 2>&1 &
SIM_PID=$!
ros2 run robosub submarine_node > /tmp/sr_sub.log 2>&1 &
SUB_PID=$!
sleep 8

ros2 topic pub --once /sim/control std_msgs/msg/String "data: 'start'" > /dev/null

python3 /home/robosub/robosub_ws/style_roll_monitor.py "$DUR"

kill -9 "$SIM_PID" "$SUB_PID" 2>/dev/null
for p in "[s]imulator_node" "[s]ubmarine_node"; do pkill -9 -f "$p" 2>/dev/null; done

echo "=== sr_sub.log (StyleRoll lines) ==="
grep -i "styleroll\|ERROR" /tmp/sr_sub.log | tail -8
