#!/bin/bash
# Headless full-mission dry run: nav brain (submarine_node) vs the simulator
# with depth fusion in the loop. Auto-starts the mission and scores it.
#   run_sim_mission.sh [duration_s] [world_z_sign]
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash
export SDL_VIDEODRIVER=dummy    # headless pygame (no GUI needed)

DUR="${1:-60}"
SIGN="${2:--1.0}"

for p in "[s]imulator_node" "[d]epth_fusion" "[s]ubmarine_node" "[s]im_mission_monitor"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

ros2 run robosub simulator_node --ros-args -p fuse_depth:=true > /tmp/msim.log 2>&1 &
ros2 run epsilon_sensors depth_fusion --ros-args \
    -r /depth:=/sensors/depth -p world_z_sign:="$SIGN" > /tmp/mfus.log 2>&1 &
ros2 run robosub submarine_node > /tmp/msub.log 2>&1 &
sleep 6

python3 /home/robosub/robosub_ws/sim_mission_monitor.py "$DUR"

for p in "[s]imulator_node" "[d]epth_fusion" "[s]ubmarine_node"; do pkill -9 -f "$p" 2>/dev/null; done
# reparented children survive pkill-by-pattern sometimes; sweep by exe too
pkill -9 -f simulator_node 2>/dev/null
pkill -9 -f depth_fusion 2>/dev/null
pkill -9 -f submarine_node 2>/dev/null
echo "=== depth_fusion tail ==="
tail -4 /tmp/mfus.log
