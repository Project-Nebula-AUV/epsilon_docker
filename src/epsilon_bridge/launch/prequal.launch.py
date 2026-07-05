"""Epsilon PREQUAL bring-up: the FULL autonomous stack, started DISARMED.

  ros2 launch epsilon_bridge prequal.launch.py
Then, once the sub is in the water, arm + start the mission (motors go live):
  ros2 run epsilon_bridge arming_helper arm
(.devcontainer/motortest.sh does both: launches this, waits 10 s, then arms.)

Difference vs hardware.launch.py: this also brings up omni_control (the actuator)
and submarine_node (the nav brain).

DEPTH IS CLOSED-LOOP BY DEFAULT (2026-07-03, matching the sim code path):
  * with_depth:=true          -> MS5837 driver + depth_fusion run; /sensors/depth is
                                 the fused estimate (gyro/accel dead-reckoning between
                                 sparse pressure reads, innovation-gated). The nav's
                                 depth PID actually holds depth.
  * synthetic_depth:=-1.0     -> the constant-depth bypass is OFF (real passthrough).
  * world_z_sign:=1.0         -> depth_fusion vertical sign (bench-checked; verify in
                                 water via /depth_fusion/innovation -- see hardware.launch).
  * heave_bias:=0.0           -> now only a trim aid; the closed-loop depth PID supplies
                                 the buoyancy-holding heave via its integrator.

FALLBACK to the old open-loop mode if the depth sensor acts up in the water:
  ros2 launch epsilon_bridge prequal.launch.py with_depth:=false synthetic_depth:=1.5
  (or via motortest.sh: WITH_DEPTH=false SYNTHETIC_DEPTH=1.5)

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
        DeclareLaunchArgument('synthetic_depth', default_value='-1.0'), # real depth passthrough
        DeclareLaunchArgument('heave_bias', default_value='0.0'),       # buoyancy trim
        DeclareLaunchArgument('with_depth', default_value='true'),      # fused MS5837 depth
        DeclareLaunchArgument('world_z_sign', default_value='1.0'),     # depth_fusion sign
        DeclareLaunchArgument('gray_world', default_value='false'),     # camera tint fix
    ]

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'hardware.launch.py')),
        launch_arguments={
            'with_depth': LaunchConfiguration('with_depth'),
            'synthetic_depth': LaunchConfiguration('synthetic_depth'),
            'heave_bias': LaunchConfiguration('heave_bias'),
            'world_z_sign': LaunchConfiguration('world_z_sign'),
            'gray_world': LaunchConfiguration('gray_world'),
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
