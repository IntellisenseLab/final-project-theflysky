from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qbot_bringup',
            executable='motion_test_node',
            name='motion_test_node',
            output='screen'
        )
    ])