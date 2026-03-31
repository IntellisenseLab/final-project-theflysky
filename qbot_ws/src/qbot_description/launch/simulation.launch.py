import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('qbot_description')
    world_path = os.path.join(pkg_share, 'sdf', 'qbot_world.sdf')
    sdf_path = os.path.join(pkg_share, 'sdf')

    gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=sdf_path
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '-r'],
        output='screen'
    )

    motion_test = Node(
        package='qbot_bringup',
        executable='motion_test_node',
        name='motion_test_node',
        output='screen',
        remappings=[
            ('/cmd_vel', '/model/qbot/cmd_vel')
        ]
    )

    return LaunchDescription([
        gazebo_resource_path,
        gazebo,
        TimerAction(period=2.0, actions=[motion_test]),
    ])