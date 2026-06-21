#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/robosub_ws/install/setup.bash

ros2 run epsilon_control omni_control &
ros2 run epsilon_teleop thruster_tester &

wait
