source /opt/ros/humble/setup.bash
source /home/robosub/robosub_ws/install/setup.bash

ros2 run epsilon_sensors camera_controls \
  --params-file $(ros2 pkg prefix epsilon_sensors)/share/epsilon_sensors/config/camera_params.yam
