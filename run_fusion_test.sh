#!/bin/bash
# Bench test: imu + depth_sensor(->/depth_raw) + depth_fusion, monitored.
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash

DUR="${1:-120}"

for p in "epsilon_sensors/[i]mu" "[d]epth_sensor" "[d]epth_fusion" "[f]usion_monitor"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

ros2 run epsilon_sensors imu > /tmp/imu.log 2>&1 &
IMU_PID=$!
ros2 run epsilon_sensors depth_sensor --ros-args -r /depth:=/depth_raw > /tmp/depth.log 2>&1 &
DEPTH_PID=$!
ros2 run epsilon_sensors depth_fusion > /tmp/fusion.log 2>&1 &
FUSION_PID=$!

sleep 8
python3 /home/robosub/robosub_ws/fusion_monitor.py "$DUR"

kill -9 "$IMU_PID" "$DEPTH_PID" "$FUSION_PID" 2>/dev/null
for p in "epsilon_sensors/[i]mu" "[d]epth_sensor" "[d]epth_fusion"; do
  pkill -9 -f "$p" 2>/dev/null
done

echo
echo "=== fusion.log ==="
cat /tmp/fusion.log
echo "=== depth.log (first 5 / last 5) ==="
head -5 /tmp/depth.log
echo "..."
tail -5 /tmp/depth.log
echo "=== imu.log ==="
head -5 /tmp/imu.log
