import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('epsilon_sensors')

    default_params = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    # Allow the params file to be overridden at launch time:
    #   ros2 launch epsilon_sensors camera.launch.py params_file:=/path/to/other.yaml
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the camera params YAML file',
    )

    camera_node = Node(
        package='epsilon_sensors',
        executable='camera',
        name='camera_input',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        camera_node,
    ])
