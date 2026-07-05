#!/bin/bash
# Headless sim test of the FULL course (RoboSim-matched): simulator_node +
# submarine_node, default mission (full course) + style roll, start, monitor
# /sub/status until ShutdownTask (or timeout).
#   run_full_course_test.sh [duration_s]
# HW_FAITHFUL=1: rehearse under REAL vehicle sensing — emulated marginal
# MS5837 + the real depth_fusion in the loop (world_z_sign -1.0, the sim's
# verified sign), and /sensors/velocity as hardware publishes it (x/y zero,
# z = fused vertical velocity).
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash
export SDL_VIDEODRIVER=dummy
export ROBOSUB_STYLE_ROLL="${ROBOSUB_STYLE_ROLL:-720}"
unset ROBOSUB_MISSION   # default = full course

DUR="${1:-600}"
HW_FAITHFUL="${HW_FAITHFUL:-0}"

for p in "[s]imulator_node" "[s]ubmarine_node" "[d]epth_fusion" "[f]ull_course_monitor" "[s]tyle_roll_monitor"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

FUS_PID=""
if [ "$HW_FAITHFUL" = "1" ]; then
  echo "[test] HARDWARE-FAITHFUL sensing: emulated MS5837 + depth_fusion + zeroed x/y velocity"
  ros2 run robosub simulator_node --ros-args \
      -p fuse_depth:=true -p hw_velocity:=true > /tmp/fc_sim.log 2>&1 &
  SIM_PID=$!
  ros2 run epsilon_sensors depth_fusion --ros-args \
      -r /depth:=/sensors/depth -p world_z_sign:=-1.0 > /tmp/fc_fusion.log 2>&1 &
  FUS_PID=$!
else
  ros2 run robosub simulator_node > /tmp/fc_sim.log 2>&1 &
  SIM_PID=$!
fi
ros2 run robosub submarine_node > /tmp/fc_sub.log 2>&1 &
SUB_PID=$!
sleep 8

ros2 topic pub --once /sim/control std_msgs/msg/String "data: 'start'" > /dev/null

python3 /home/robosub/robosub_ws/full_course_monitor.py "$DUR"

kill -9 "$SIM_PID" "$SUB_PID" $FUS_PID 2>/dev/null
for p in "[s]imulator_node" "[s]ubmarine_node" "[d]epth_fusion"; do pkill -9 -f "$p" 2>/dev/null; done

echo "=== fc_sub.log (INFO/WARN tail) ==="
grep -E "INFO:|WARN:|ERROR" /tmp/fc_sub.log | tail -12
if [ "$HW_FAITHFUL" = "1" ]; then
  echo "=== depth_fusion status (tail) ==="
  grep -E "depth .* accepted|re-anchor|first pressure" /tmp/fc_fusion.log | tail -4
fi
