#!/bin/bash
# Deterministically exercise the staleness deadman: run the fusion stack with
# LOWERED thresholds (stale > 1.5s, freeze > 4s) so the bus's own natural read
# gaps trip the stale flag and the bounded-drift freeze/un-freeze paths.
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash

DUR="${1:-120}"

for p in '[d]epth_sensor' '[d]epth_fusion' 'epsilon_sensors/[i]mu' '[s]ensor_bridge'; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

ros2 run epsilon_sensors imu > /tmp/imu.log 2>&1 &
ros2 run epsilon_sensors depth_sensor --ros-args -r /depth:=/depth_raw > /tmp/depth.log 2>&1 &
ros2 run epsilon_sensors depth_fusion --ros-args \
    -p dr_max_age_s:=1.5 -p dead_reckon_hard_limit_s:=4.0 > /tmp/fusion_stale.log 2>&1 &
sleep "$DUR"
for p in '[d]epth_sensor' '[d]epth_fusion' 'epsilon_sensors/[i]mu'; do
  pkill -9 -f "$p" 2>/dev/null
done

echo "=== staleness deadman (thresholds: stale>1.5s, freeze>4s) ==="
echo "freeze events  : $(grep -c 'freezing estimate' /tmp/fusion_stale.log)"
echo "un-freeze events: $(grep -c 'un-frozen' /tmp/fusion_stale.log)"
echo "--- sample freeze/un-freeze lines ---"
grep -E 'freezing estimate|un-frozen' /tmp/fusion_stale.log | head -12
echo "--- health-state tally from status lines (want a mix of ok/STALE/FROZEN) ---"
grep -oE '\[(ok|STALE|FROZEN)\]' /tmp/fusion_stale.log | sort | uniq -c
echo "--- first fix ---"
grep 'first pressure fix' /tmp/fusion_stale.log | head -1
