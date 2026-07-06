"""Sysid stack: sensors + logger (+ runner) (+ motors), NEVER thruster_bridge.

sysid_runner publishes /thrust_control directly and carries its own arm gate
(~/arm, default DISARMED) -- running thruster_bridge here would put two
publishers on /thrust_control. Do not include it.

Modes:
  DRY (default, with_motors:=false): omni_control NOT started; a run produces
      cmd.csv from the runner's actual /thrust_control output -- the
      verification artifact -- with zero motor risk.
  HAND-TEST (with_runner:=false): sensors + logger only, motors impossible;
      for S1/S2 tilt-release etc. Sub must be LEVEL at launch (sensor_bridge
      captures the roll mounting offset in the first 1.5 s).
  LIVE (with_motors:=true): full chain; arm via
      ros2 run epsilon_bridge arming_helper is NOT used here -- the runner's
      own gate is armed by sysid_run.sh after its countdown:
      ros2 service call /sysid_runner/arm std_srvs/srv/SetBool "{data: true}"

Required args: run_dir; sequence_file (unless with_runner:=false).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _s(name):
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def _depth_cond(source, invert=False):
    """IfCondition: with_depth AND depth_source == source (optionally inverted)."""
    expr = ["'", LaunchConfiguration('with_depth'), "'.lower() == 'true' and '",
            LaunchConfiguration('depth_source'), "' == '", source, "'"]
    if invert:
        expr = ['not (', *expr, ')']
    return IfCondition(PythonExpression(expr))


def generate_launch_description():
    args = [
        DeclareLaunchArgument('run_dir'),
        DeclareLaunchArgument('sequence_file', default_value=''),
        DeclareLaunchArgument('with_runner', default_value='true'),
        DeclareLaunchArgument('with_motors', default_value='false'),
        DeclareLaunchArgument('with_camera', default_value='true'),
        DeclareLaunchArgument('with_depth', default_value='true'),
        # esp32 (default) = MS5837 via Xiao ESP32-C3 USB serial (filtered, no
        # fusion); i2c = legacy Pi-I2C MS5837 + depth_fusion.
        DeclareLaunchArgument('depth_source', default_value='esp32'),
        DeclareLaunchArgument('jpeg_hz', default_value='4.0'),
        DeclareLaunchArgument('gray_world', default_value='false'),
        DeclareLaunchArgument('world_z_sign', default_value='1.0'),
    ]

    imu = Node(package='epsilon_sensors', executable='imu', name='imu',
               output='screen')
    esp32_depth = Node(package='epsilon_sensors', executable='esp32_depth',
                       name='esp32_depth', output='screen',
                       condition=_depth_cond('esp32'))
    depth = Node(package='epsilon_sensors', executable='depth_sensor',
                 name='depth_sensor', output='screen',
                 remappings=[('/depth', '/depth_raw')],
                 condition=_depth_cond('i2c'))
    depth_fusion = Node(package='epsilon_sensors', executable='depth_fusion',
                        name='depth_fusion', output='screen',
                        parameters=[{'world_z_sign': _f('world_z_sign')}],
                        condition=_depth_cond('i2c'))
    camera = Node(package='epsilon_sensors', executable='camera', name='camera',
                  output='screen',
                  parameters=[{'gray_world': ParameterValue(
                      LaunchConfiguration('gray_world'), value_type=bool)}],
                  condition=IfCondition(LaunchConfiguration('with_camera')))
    # Two variants (remappings are fixed per Node): esp32 mode points the
    # bridge's fusion-era subscriptions at the esp32 driver.
    sensor_bridge = Node(package='epsilon_bridge', executable='sensor_bridge',
                         name='sensor_bridge', output='screen',
                         remappings=[('/depth_fusion/velocity_z', '/esp32_depth/velocity_z'),
                                     ('/depth_fusion/stale', '/esp32_depth/stale')],
                         condition=_depth_cond('esp32'))
    sensor_bridge_plain = Node(package='epsilon_bridge', executable='sensor_bridge',
                               name='sensor_bridge', output='screen',
                               condition=_depth_cond('esp32', invert=True))

    logger = Node(package='epsilon_bridge', executable='sysid_logger',
                  name='sysid_logger', output='screen',
                  parameters=[{'run_dir': _s('run_dir'),
                               'jpeg_hz': _f('jpeg_hz')}])

    runner = Node(package='epsilon_bridge', executable='sysid_runner',
                  name='sysid_runner', output='screen',
                  parameters=[{'sequence_file': _s('sequence_file')}],
                  condition=IfCondition(LaunchConfiguration('with_runner')))

    omni = Node(package='epsilon_control', executable='omni_control',
                name='omni_control', output='screen',
                condition=IfCondition(LaunchConfiguration('with_motors')))

    return LaunchDescription(args + [imu, esp32_depth, depth, depth_fusion, camera,
                                     sensor_bridge, sensor_bridge_plain,
                                     logger, runner, omni])
