#!/bin/bash
# Isolate recorder_node CPU cost: baseline = full stack + ACTIVE hold mission
# (disarmed), then the same + recorder_node at native 320x240. Tears down after.
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash

cleanup_all() {
  for pat in "[o]mni_control" "epsilon_bridge/[s]ensor_bridge" "epsilon_bridge/[t]hruster_bridge" \
             "epsilon_sensors/[c]amera" "epsilon_sensors/[i]mu" "epsilon_sensors/[d]epth_sensor" \
             "[d]epth_fusion" "[s]ubmarine_node" "[r]ecorder_node" "[r]os2 launch epsilon_bridge"; do
    pkill -9 -f "$pat" 2>/dev/null
  done
}
cpu_sample() {
  for i in 1 2 3; do
    top -bn2 -d 0.7 | grep '%Cpu(s)' | tail -1 | awk '{print 100 - $8}'
    sleep 1
  done | awk '{s+=$1} END {printf "%.1f", s/NR}'
}

cleanup_all; sleep 2
sudo chmod a+rw /dev/video0 /dev/video1 2>/dev/null

export ROBOSUB_MISSION=hold
ros2 launch epsilon_bridge prequal.launch.py > /tmp/rec2_launch.log 2>&1 &
sleep 12
ros2 topic pub --once /sim/control std_msgs/msg/String "data: 'start'" > /dev/null
sleep 5

echo "=== baseline: full stack, ACTIVE hold mission, disarmed, no recorder ==="
echo "CPU busy: $(cpu_sample)%"

ros2 run robosub recorder_node --ros-args -p scale:=1.0 > /tmp/rec2_node.log 2>&1 &
sleep 3
ros2 topic pub --once /sim/control std_msgs/msg/String "data: 'start'" > /dev/null
sleep 5

echo "=== same + recorder_node (native 320x240 @ 30 fps mp4) ==="
echo "CPU busy: $(cpu_sample)%"

sleep 10
ros2 topic pub --once /sim/control std_msgs/msg/String "data: 'quit'" > /dev/null
sleep 3
echo "=== recorder result ==="
tail -2 /tmp/rec2_node.log
ls -lh ~/robosub_recordings/ | tail -2

cleanup_all
echo "=== torn down ==="
