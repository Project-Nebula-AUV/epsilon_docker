#!/bin/bash
# Bench parity check: bring up the REAL prequal stack (new fused-depth defaults)
# DISARMED, verify every readiness topic flows — including fused /sensors/depth
# from the marginal MS5837 — then tear down. NEVER arms.
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash

for pat in "[o]mni_control" "epsilon_bridge/[s]ensor_bridge" "epsilon_bridge/[t]hruster_bridge" \
           "epsilon_sensors/[c]amera" "epsilon_sensors/[i]mu" "epsilon_sensors/[d]epth_sensor" \
           "[d]epth_fusion" "[s]ubmarine_node" "[r]os2 launch epsilon_bridge"; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 2
sudo chmod a+rw /dev/video0 /dev/video1 2>/dev/null

export ROBOSUB_MISSION=prequal
export ROBOSUB_STYLE_ROLL=720

ros2 launch epsilon_bridge prequal.launch.py > /tmp/parity_launch.log 2>&1 &
LP=$!
sleep 12

echo "=== readiness topics (new defaults: with_depth=true, synthetic_depth=-1.0) ==="
for t in /camera/image_raw /imu /sensors/heading /sensors/depth /sub/status; do
  if timeout -k 2 8 ros2 topic echo --once --qos-reliability best_effort "$t" > /tmp/topic_sample.txt 2>&1; then
    echo "OK   $t"
  else
    echo "FAIL $t"
  fi
done

echo "=== fused depth samples (should be ~0 m on the bench, from real MS5837) ==="
for i in 1 2 3; do
  timeout -k 2 6 ros2 topic echo --once --qos-reliability best_effort /sensors/depth 2>/dev/null | grep data
done

echo "=== depth_fusion status (from launch log) ==="
grep -i "depth fusion\|pressure fix\|re-anchor" /tmp/parity_launch.log | tail -4

echo "=== nav status ==="
timeout -k 2 6 ros2 topic echo --once /sub/status 2>/dev/null | grep data

echo "=== thruster_bridge armed state (must be disarmed) ==="
grep -i "thruster_bridge up" /tmp/parity_launch.log | tail -1

kill "$LP" 2>/dev/null
sleep 3
for pat in "[o]mni_control" "epsilon_bridge/[s]ensor_bridge" "epsilon_bridge/[t]hruster_bridge" \
           "epsilon_sensors/[c]amera" "epsilon_sensors/[i]mu" "epsilon_sensors/[d]epth_sensor" \
           "[d]epth_fusion" "[s]ubmarine_node" "[r]os2 launch epsilon_bridge"; do
  pkill -9 -f "$pat" 2>/dev/null
done
echo "=== torn down ==="
