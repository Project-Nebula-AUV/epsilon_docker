"""RoboSub simulation bring-up.

  ros2 launch robosub robosub.launch.py

Depth fusion in the loop (optional):
  ros2 launch robosub robosub.launch.py fuse_depth:=true

With fuse_depth:=true the simulator stops publishing ground-truth
/sensors/depth and instead emulates the marginal MS5837 -- a sparse,
occasionally-corrupt /depth_raw plus the raw /imu + /imu/gravity -- and the
real epsilon_sensors depth_fusion node reconstructs /sensors/depth from them
(same code path as the vehicle). Ground truth is still published on
/sim/true_depth so the fused estimate can be scored. world_z_sign selects the
gravity/accel vertical convention -- verify it against /sim/true_depth here
before trusting it in the water.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    fuse_depth = LaunchConfiguration('fuse_depth')

    args = [
        DeclareLaunchArgument('fuse_depth', default_value='false'),
        # Gravity/accel vertical sign for depth_fusion. -1.0 is correct for
        # this sim's IMU convention (gravity vector +Z=down, heave accel
        # down-positive): verified here by scoring fused depth against
        # /sim/true_depth -- -1.0 tracks to ~7 cm through a full 0..1.9 m dive,
        # +1.0 diverges to ~0.65 m mean. The right value on the real vehicle
        # depends on the BNO055 gravity-vector direction (up vs down); confirm
        # it on the bench by echoing /imu/gravity, then in the water.
        DeclareLaunchArgument('world_z_sign', default_value='-1.0'),
    ]

    simulator_node = Node(
        package='robosub', executable='simulator_node', name='simulator_node',
        output='screen',
        parameters=[{
            'fuse_depth': ParameterValue(fuse_depth, value_type=bool),
        }],
    )

    # Real fusion node, fed by the sim's emulated raw topics. /depth is
    # remapped onto /sensors/depth so submarine_node consumes the fused
    # estimate exactly as it would the sensor_bridge passthrough on hardware.
    depth_fusion = Node(
        package='epsilon_sensors', executable='depth_fusion', name='depth_fusion',
        output='screen',
        condition=IfCondition(fuse_depth),
        remappings=[('/depth', '/sensors/depth')],
        parameters=[{
            'world_z_sign': ParameterValue(LaunchConfiguration('world_z_sign'), value_type=float),
        }],
    )

    submarine_node = Node(
        package='robosub', executable='submarine_node', name='submarine_node',
        output='screen',
    )
    recorder_node = Node(
        package='robosub', executable='recorder_node', name='recorder_node',
        output='screen',
    )
    web_node = Node(
        package='robosub', executable='web_node', name='web_node',
        output='screen',
    )

    return LaunchDescription(
        args + [simulator_node, depth_fusion, submarine_node, recorder_node, web_node])
