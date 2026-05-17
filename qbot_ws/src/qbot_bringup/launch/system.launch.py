"""Launches the full gesture-to-behaviour pipeline for the Kobuki QBot.

Parameters are loaded from ``qbot_bringup/config/qbot_params.yaml`` and
can be overridden via launch arguments. Common overrides:

* ``cmd_vel_topic:=/cmd_vel`` if the Kobuki driver listens there;
* ``mirror_horizontal_commands:=true`` if camera image is mirrored;
* ``publish_debug_image:=true`` to enable the annotated gesture overlay;
* ``enable_depth:=false`` to skip depth (also disables depth-aware
  come-closer and obstacle safety stop).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _params_file():
    return PathJoinSubstitution([
        FindPackageShare("qbot_bringup"),
        "config",
        "qbot_params.yaml",
    ])


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    sound_topic = LaunchConfiguration("sound_topic")
    model_path = LaunchConfiguration("gesture_model_path")
    params_file = _params_file()

    kinect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("kinect_camera"),
                "launch",
                "kinect_rgbd.launch.py",
            ])
        ),
        condition=IfCondition(LaunchConfiguration("start_kinect")),
        launch_arguments={
            "device_index": LaunchConfiguration("kinect_device_index"),
            "publish_rate_hz": LaunchConfiguration("kinect_publish_rate_hz"),
            "enable_rgb": "true",
            "enable_depth": LaunchConfiguration("enable_depth"),
        }.items(),
    )

    gesture_node = Node(
        package="gesture_node",
        executable="gesture_command_node",
        name="gesture_command_node",
        output="screen",
        parameters=[
            params_file,
            {
                "image_topic": image_topic,
                "model_path": model_path,
                "max_fps": LaunchConfiguration("gesture_max_fps"),
                "mirror_horizontal_commands": LaunchConfiguration("mirror_horizontal_commands"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            },
        ],
    )

    face_tracker = Node(
        package="vision_node",
        executable="face_tracker_node",
        name="face_tracker_node",
        output="screen",
        parameters=[
            params_file,
            {
                "image_topic": image_topic,
                "max_fps": LaunchConfiguration("face_max_fps"),
            },
        ],
    )

    behavior_node = Node(
        package="behavior_node",
        executable="pet_behavior_node",
        name="pet_behavior_node",
        output="screen",
        parameters=[
            params_file,
            {
                "cmd_vel_topic": cmd_vel_topic,
                "sound_topic": sound_topic,
                "target_turn_sign": LaunchConfiguration("target_turn_sign"),
                "use_depth_for_come_closer": LaunchConfiguration("enable_depth"),
                "use_depth_for_safety": LaunchConfiguration("enable_depth"),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "venv_site_packages",
            default_value=EnvironmentVariable(
                "QBOT_VENV_SITE_PACKAGES",
                default_value="/home/nadeesha/final-project-theflysky/.venv/lib/python3.12/site-packages",
            ),
        ),
        DeclareLaunchArgument("start_kinect", default_value="true"),
        DeclareLaunchArgument("kinect_device_index", default_value="0"),
        DeclareLaunchArgument("kinect_publish_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/kinect/rgb/image_raw"),
        DeclareLaunchArgument(
            "gesture_model_path",
            default_value=EnvironmentVariable(
                "QBOT_GESTURE_MODEL",
                default_value="/home/nadeesha/final-project-theflysky/models/gesture_recognizer.task",
            ),
        ),
        DeclareLaunchArgument("gesture_max_fps", default_value="12.0"),
        DeclareLaunchArgument("face_max_fps", default_value="8.0"),
        DeclareLaunchArgument("mirror_horizontal_commands", default_value="false"),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("sound_topic", default_value="/commands/sound"),
        DeclareLaunchArgument("target_turn_sign", default_value="-1.0"),
        SetEnvironmentVariable(
            "PYTHONPATH",
            [
                LaunchConfiguration("venv_site_packages"),
                ":",
                EnvironmentVariable("PYTHONPATH", default_value=""),
            ],
        ),
        kinect_launch,
        gesture_node,
        face_tracker,
        behavior_node,
    ])
