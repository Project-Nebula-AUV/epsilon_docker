#!/bin/bash
# Full fusion stack + hardening verification: imu + depth_sensor(->/depth_raw)
# + depth_fusion + sensor_bridge, monitored. Verifies rate/gaps, the
# envelope-on-anchor guard, the /depth_fusion/stale + /sensors/depth_ok health
# flags (startup vs after first fix), and the health state machine.
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash

DUR="${1:-480}"

for p in '[d]epth_sensor' '[d]epth_fusion' '[f]usion_monitor' 'epsilon_sensors/[i]mu' '[s]ensor_bridge'; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

ros2 run epsilon_sensors imu > /tmp/imu.log 2>&1 &
IMU=$!
ros2 run epsilon_sensors depth_sensor --ros-args -r /depth:=/depth_raw > /tmp/depth.log 2>&1 &
DEP=$!
ros2 run epsilon_sensors depth_fusion > /tmp/fusion.log 2>&1 &
FUS=$!
ros2 run epsilon_bridge sensor_bridge > /tmp/bridge.log 2>&1 &
BRG=$!

sleep 3
echo "=== health topics at STARTUP (~3s; expect stale=true, depth_ok=false pre-fix) ==="
echo -n "  /depth_fusion/stale : "; timeout 3 ros2 topic echo --once /depth_fusion/stale 2>/dev/null | tr -d '\n'; echo
echo -n "  /sensors/depth_ok   : "; timeout 3 ros2 topic echo --once /sensors/depth_ok 2>/dev/null | tr -d '\n'; echo

sleep 5
python3 /home/robosub/robosub_ws/fusion_monitor.py "$DUR"

echo "=== health topics AFTER run (expect stale=false, depth_ok=true if healthy) ==="
echo -n "  /depth_fusion/stale : "; timeout 3 ros2 topic echo --once /depth_fusion/stale 2>/dev/null | tr -d '\n'; echo
echo -n "  /sensors/depth_ok   : "; timeout 3 ros2 topic echo --once /sensors/depth_ok 2>/dev/null | tr -d '\n'; echo

kill -9 "$IMU" "$DEP" "$FUS" "$BRG" 2>/dev/null
for p in '[d]epth_sensor' '[d]epth_fusion' 'epsilon_sensors/[i]mu' '[s]ensor_bridge'; do
  pkill -9 -f "$p" 2>/dev/null
done

echo
echo "=== fusion.log: first fix + tail ==="
grep -n 'first pressure fix' /tmp/fusion.log | head -1
tail -20 /tmp/fusion.log
echo "=== out-of-envelope rejections in fusion ==="
echo "count: $(grep -c 'out of envelope\|out-of-envelope' /tmp/fusion.log)"
echo "=== re-anchor / freeze / stale events ==="
echo "re-anchors: $(grep -c 're-anchor' /tmp/fusion.log)"
echo "freezes   : $(grep -c 'freezing estimate' /tmp/fusion.log)"
echo "watchdog  : $(grep -c 'NEVER anchor' /tmp/fusion.log)  (expect 0 -- /depth_raw is flowing)"
echo "=== health-state tallies from status lines ==="
grep -oE '\[(ok|STALE|FROZEN)\]' /tmp/fusion.log | sort | uniq -c
echo "=== bridge.log tail ==="
tail -4 /tmp/bridge.log
