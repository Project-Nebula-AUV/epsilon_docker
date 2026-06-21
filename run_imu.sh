#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash
exec ros2 run epsilon_sensors imu
