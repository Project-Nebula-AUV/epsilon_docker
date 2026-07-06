"""Epsilon hardware bring-up: sensor drivers + bridges, thruster_bridge DISARMED.

  ros2 launch epsilon_bridge hardware.launch.py
Then arm explicitly (never from this launch):
  ros2 run epsilon_bridge arming_helper arm

Brings up: imu, depth (see below), camera, sensor_bridge, thruster_bridge(disarmed).
It does NOT start submarine_node (the nav brain) or omni_control (the actuator) --
launch those deliberately for P4.

DEPTH (2026-07-06): the MS5837 now hangs off a Xiao ESP32-C3 which streams JSON
over USB CDC (/dev/ttyACM0) -- the Pi 5's own I2C buses proved unreliable with
this sensor. depth_source:=esp32 (default) runs the esp32_depth driver (filtered
/depth at 20 Hz, no fusion needed); depth_source:=i2c is the legacy chain
(MS5837 on Pi I2C + depth_fusion dead-reckoning), kept intact as a fallback.
sensor_bridge's fusion-era subscriptions (/depth_fusion/velocity_z, /stale) are
remapped to /esp32_depth/* in esp32 mode -- no bridge code changes.

SIGN CONFIGURATION (confirmed on the bench, 2026-06-21, in-sub IMU orientation):
  heave_sign = -1.0  (CONFIRMED: +heave=descend must drive motors 0/1 negative; air test
                      + updown.log + depth law). This is the default below.
  all other signs (heading/yaw_rate/roll/roll_rate, thruster roll) = +1.0 (confirmed correct).
Override any sign via launch args, e.g.:
  ros2 launch epsilon_bridge hardware.launch.py heave_sign:=1.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _f(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _depth_cond(source, invert=False):
    """IfCondition: with_depth AND depth_source == source (optionally inverted)."""
    expr = ["'", LaunchConfiguration('with_depth'), "'.lower() == 'true' and '",
            LaunchConfiguration('depth_source'), "' == '", source, "'"]
    if invert:
        expr = ['not (', *expr, ')']
    return IfCondition(PythonExpression(expr))


def generate_launch_description():
    args = [
        DeclareLaunchArgument('with_imu', default_value='true'),
        DeclareLaunchArgument('with_depth', default_value='true'),
        DeclareLaunchArgument('with_camera', default_value='true'),
        # esp32 (default) = MS5837 via Xiao ESP32-C3 USB serial, filtered, no fusion.
        # i2c = legacy Pi-I2C MS5837 + depth_fusion (unreliable on the Pi 5).
        DeclareLaunchArgument('depth_source', default_value='esp32'),
        # camera software white balance: neutralize the OV9782 warm tint
        DeclareLaunchArgument('gray_world', default_value='false'),
        # sensor_bridge sign params -- confirmed +1 at P4 (in-sub IMU orientation)
        DeclareLaunchArgument('heading_sign', default_value='1.0'),
        DeclareLaunchArgument('yaw_rate_sign', default_value='-1.0'),
        DeclareLaunchArgument('roll_rate_sign', default_value='-1.0'),
        DeclareLaunchArgument('heading_offset_deg', default_value='0.0'),
        DeclareLaunchArgument('sensor_roll_sign', default_value='1.0'),
        # thruster_bridge sign params -- heave CONFIRMED -1 (descend = motors 0/1 negative)
        DeclareLaunchArgument('heave_sign', default_value='-1.0'),
        DeclareLaunchArgument('thruster_roll_sign', default_value='1.0'),
        # Depth-sensorless hold: -1.0 => sensor_bridge passes real /depth through.
        # Set >=0 (e.g. 1.5 = MISSION_DEPTH) to bypass the depth sensor and hold.
        DeclareLaunchArgument('synthetic_depth', default_value='-1.0'),
        # Constant descend trim for open-loop depth hold (nav-heave: +heave=descend).
        DeclareLaunchArgument('heave_bias', default_value='0.0'),
        # depth_fusion vertical sign (legacy i2c path only; see 2026-07-03 notes).
        DeclareLaunchArgument('world_z_sign', default_value='1.0'),
    ]

    imu = Node(package='epsilon_sensors', executable='imu', name='imu', output='screen',
               condition=IfCondition(LaunchConfiguration('with_imu')))

    # ESP32 path (default): filtered /depth at 20 Hz straight from the driver.
    esp32_depth = Node(package='epsilon_sensors', executable='esp32_depth',
                       name='esp32_depth', output='screen',
                       condition=_depth_cond('esp32'))

    # Legacy i2c path: raw MS5837 driver remapped off /depth; depth_fusion
    # dead-reckons across its multi-second gaps with the BNO055 and
    # republishes a continuous /depth, so downstream consumers are unchanged.
    depth = Node(package='epsilon_sensors', executable='depth_sensor', name='depth_sensor', output='screen',
                 remappings=[('/depth', '/depth_raw')],
                 condition=_depth_cond('i2c'))
    depth_fusion = Node(package='epsilon_sensors', executable='depth_fusion', name='depth_fusion', output='screen',
                        parameters=[{'world_z_sign': _f('world_z_sign')}],
                        condition=_depth_cond('i2c'))

    camera = Node(package='epsilon_sensors', executable='camera', name='camera', output='screen',
                  parameters=[{'gray_world': ParameterValue(
                      LaunchConfiguration('gray_world'), value_type=bool)}],
                  condition=IfCondition(LaunchConfiguration('with_camera')))

    bridge_params = [{
        'heading_sign': _f('heading_sign'),
        'yaw_rate_sign': _f('yaw_rate_sign'),
        'roll_rate_sign': _f('roll_rate_sign'),
        'heading_offset_deg': _f('heading_offset_deg'),
        'roll_sign': _f('sensor_roll_sign'),
        'synthetic_depth': _f('synthetic_depth'),
    }]
    # Two variants because remappings are fixed per Node: in esp32 mode the
    # bridge's fusion-era subscriptions point at the esp32 driver instead.
    sensor_bridge_esp32 = Node(
        package='epsilon_bridge', executable='sensor_bridge', name='sensor_bridge', output='screen',
        parameters=bridge_params,
        remappings=[('/depth_fusion/velocity_z', '/esp32_depth/velocity_z'),
                    ('/depth_fusion/stale', '/esp32_depth/stale')],
        condition=_depth_cond('esp32'))
    sensor_bridge_plain = Node(
        package='epsilon_bridge', executable='sensor_bridge', name='sensor_bridge', output='screen',
        parameters=bridge_params,
        condition=_depth_cond('esp32', invert=True))

    # ALWAYS disarmed: start_armed is intentionally not set (node default False).
    thruster_bridge = Node(
        package='epsilon_bridge', executable='thruster_bridge', name='thruster_bridge', output='screen',
        parameters=[{
            'heave_sign': _f('heave_sign'),
            'roll_sign': _f('thruster_roll_sign'),
            'heave_bias': _f('heave_bias'),
        }],
    )

    return LaunchDescription(args + [imu, esp32_depth, depth, depth_fusion, camera,
                                     sensor_bridge_esp32, sensor_bridge_plain,
                                     thruster_bridge])
