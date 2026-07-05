#!/bin/bash
# Self-contained runner for the theoretical I/O test. Starts submarine_node (hold)
# + thruster_bridge (armed, NO omni_control -> no motors), runs auton_io_test.py,
# writes auton_io_result.txt, then cleans up. Launch detached:
#   docker exec -d robosub_dev bash /home/robosub/robosub_ws/run_auton_test.sh
set -u
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash
cd /home/robosub/robosub_ws
rm -f auton_io_result.txt auton_test_DONE

for p in "[o]mni_control" "[s]ubmarine_node" "epsilon_bridge/[t]hruster_bridge" \
         "epsilon_bridge/[s]ensor_bridge" "epsilon_sensors/[c]amera" "[a]uton_io_test"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 1

ROBOSUB_MISSION=hold ros2 run robosub submarine_node > /tmp/sub.log 2>&1 &
SUB_PID=$!
ros2 run epsilon_bridge thruster_bridge --ros-args \
    -p start_armed:=true -p heave_sign:=-1.0 -p heave_bias:=0.0 > /tmp/tb.log 2>&1 &
TB_PID=$!
sleep 5

python3 auton_io_test.py > /tmp/auton_test_stdout.log 2>&1
echo "test_exit=$?" >> /tmp/auton_test_stdout.log

kill -9 "$SUB_PID" "$TB_PID" 2>/dev/null
for p in "[s]ubmarine_node" "epsilon_bridge/[t]hruster_bridge"; do pkill -9 -f "$p" 2>/dev/null; done
touch auton_test_DONE
