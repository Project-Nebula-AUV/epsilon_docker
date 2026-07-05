#!/bin/bash
# Depth-sensor-ONLY bench test: just the depth_node driver (remapped ->/depth_raw)
# + fusion_monitor. No IMU, no fusion. Verifies the tightened operating-envelope
# gate bounds published values and the driver still delivers valid reads.
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash

DUR="${1:-300}"

for p in '[d]epth_sensor' '[d]epth_fusion' '[f]usion_monitor' 'epsilon_sensors/[i]mu'; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

ros2 run epsilon_sensors depth_sensor --ros-args -r /depth:=/depth_raw > /tmp/depthonly.log 2>&1 &
DPID=$!
sleep 5
python3 /home/robosub/robosub_ws/fusion_monitor.py "$DUR"
kill -9 "$DPID" 2>/dev/null
pkill -9 -f '[d]epth_sensor' 2>/dev/null

echo
echo "=== depthonly.log: connect/reconnect events ==="
grep -nE 'connected|reconnect|PROM' /tmp/depthonly.log | head -20
echo "=== read()==False count / read-failed count ==="
echo "read()==False: $(grep -c 'read() returned False' /tmp/depthonly.log)"
echo "read failed  : $(grep -c 'read failed' /tmp/depthonly.log)"
