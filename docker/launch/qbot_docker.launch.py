"""All-in-one launch for the containerised QBot.

Includes the existing ``qbot_bringup/system.launch.py`` (Kinect driver +
gesture + face + behaviour nodes) and additionally starts the
``kobuki_control`` serial bridge, which system.launch.py does not start on
its own. Launch arguments are passed straight through, e.g.:

  ros2 launch /opt/qbot/launch/qbot_docker.launch.py enable_depth:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    system_launch = os.path.join(
        get_package_share_directory("qbot_bringup"),
        "launch",
        "system.launch.py",
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch)
        ),
        Node(
            package="kobuki_control",
            executable="kobuki_control_node",
            name="kobuki_control_node",
            output="screen",
        ),
    ])
