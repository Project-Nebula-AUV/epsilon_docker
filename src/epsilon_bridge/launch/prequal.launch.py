"""Epsilon PREQUAL bring-up: the FULL autonomous stack, started DISARMED.

  ros2 launch epsilon_bridge prequal.launch.py
Then, once the sub is in the water, arm + start the mission (motors go live):
  ros2 run epsilon_bridge arming_helper arm
(.devcontainer/motortest.sh does both: launches this, waits 10 s, then arms.)

Difference vs hardware.launch.py: this also brings up omni_control (the actuator)
and submarine_node (the nav brain), and it runs WITHOUT the depth sensor --

  * with_depth:=false        -> the dead/marginal MS5837 driver is not started.
  * synthetic_depth:=1.5      -> sensor_bridge publishes a constant /sensors/depth
                                 equal to MISSION_DEPTH, so the nav's depth_error is
                                 ~0: the depth loop commands ~zero heave (neutral
                                 hold) and StabilizeTask's depth gate is satisfied so
                                 the mission advances. START THE SUB ALREADY SUBMERGED.
  * heave_bias:=0.0           -> open-loop buoyancy trim. 0.0 is neutral (the sub
                                 slowly surfaces -- the fail-safe). Raise toward ~0.5
                                 in-water to actually hold against positive buoyancy.

The nav brain emits nothing until it has a camera frame AND a 'start' on /sim/control
(arming_helper sends that), so launching this alone fires no thrusters.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('epsilon_bridge')

    # Tunables surfaced at the prequal level so motortest.sh / the user can set them.
    args = [
        DeclareLaunchArgument('synthetic_depth', default_value='1.5'),  # = MISSION_DEPTH
        DeclareLaunchArgument('heave_bias', default_value='0.0'),       # buoyancy trim
        DeclareLaunchArgument('with_depth', default_value='false'),     # MS5837 is dead
    ]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hardware.launch.py')),
        launch_arguments={
            'with_depth': LaunchConfiguration('with_depth'),
            'synthetic_depth': LaunchConfiguration('synthetic_depth'),
            'heave_bias': LaunchConfiguration('heave_bias'),
        }.items(),
    )

    omni_control = Node(
        package='epsilon_control', executable='omni_control',
        name='omni_control', output='screen',
    )

    submarine_node = Node(
        package='robosub', executable='submarine_node',
        name='submarine_node', output='screen',
    )

    return LaunchDescription(args + [hardware, omni_control, submarine_node])
