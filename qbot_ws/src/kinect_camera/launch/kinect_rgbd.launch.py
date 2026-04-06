from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_index', default_value='0'),
        DeclareLaunchArgument('publish_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('enable_rgb', default_value='true'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        Node(
            package='kinect_camera',
            executable='kinect_rgbd_node',
            name='kinect_rgbd_node',
            output='screen',
            parameters=[{
                'device_index': LaunchConfiguration('device_index'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'enable_rgb': LaunchConfiguration('enable_rgb'),
                'enable_depth': LaunchConfiguration('enable_depth'),
            }],
        ),
    ])
