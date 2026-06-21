from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosub',
            executable='simulator_node',
            name='simulator_node',
            output='screen',
        ),
        Node(
            package='robosub',
            executable='submarine_node',
            name='submarine_node',
            output='screen',
        ),
        Node(
            package='robosub',
            executable='recorder_node',
            name='recorder_node',
            output='screen',
        ),
        Node(
            package='robosub',
            executable='web_node',
            name='web_node',
            output='screen',
        ),
    ])
