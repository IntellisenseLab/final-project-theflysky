import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    qbot_description_share = get_package_share_directory('qbot_description')
    simulation_launch_path = os.path.join(
        qbot_description_share,
        'launch',
        'simulation.launch.py'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch_path)
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/qbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    motion_test = Node(
        package='qbot_bringup',
        executable='motion_test_node',
        name='motion_test_node',
        output='screen'
    )

    return LaunchDescription([
        simulation,
        bridge,
        TimerAction(period=10.0, actions=[motion_test]),
    ])