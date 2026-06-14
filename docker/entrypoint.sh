#!/usr/bin/env bash
# Sources ROS + the QBot workspace and wires up the paths the launch files
# expect, then execs whatever command was passed (default: the full pipeline).
set -e

source /opt/ros/jazzy/setup.bash
source /opt/qbot/qbot_ws/install/setup.bash

# libfreenect was installed to /usr/local; firmware blob baked into the image.
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
export LIBFREENECT_FIRMWARE_PATH="/opt/qbot/firmware"

# Override the laptop-specific defaults baked into system.launch.py.
export QBOT_GESTURE_MODEL="/opt/qbot/models/gesture_recognizer.task"
export QBOT_VENV_SITE_PACKAGES="/usr/local/lib/python3.12/dist-packages"

exec "$@"
