#!/usr/bin/env bash
# Launches the whole QBot pipeline in one shot:
#   Kinect driver -> gesture node -> face tracker -> behaviour node
#   -> Kobuki serial bridge, plus the live high-FPS viewer.
#
# All nodes share the shared-memory DDS profile so the 30 fps Kinect feed is
# delivered without frame loss (see fastdds_shm_local.xml).
#
# Usage:
#   ./run_qbot.sh                 # full pipeline + viewer (RGB only, 30 fps)
#   ./run_qbot.sh enable_depth:=true   # also stream depth (lower RGB fps)
#   QBOT_NO_VIEWER=1 ./run_qbot.sh     # skip the camera viewer window
#
# Press Ctrl-C once to stop everything cleanly.
set -e

REPO="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$REPO/qbot_ws/install/setup.bash"

export FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/fastdds_shm_local.xml"
export LIBFREENECT_FIRMWARE_PATH="$HOME/.libfreenect"
export QBOT_GESTURE_MODEL="$REPO/models/gesture_recognizer.task"
export PYTHONPATH="$REPO/.venv/lib/python3.12/site-packages:$PYTHONPATH"

pids=()
cleanup() {
  echo
  echo "Stopping QBot..."
  kill "${pids[@]}" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM

echo "Starting perception + behaviour pipeline..."
ros2 launch qbot_bringup system.launch.py \
  enable_depth:=false gesture_max_fps:=30.0 cmd_vel_topic:=/cmd_vel "$@" &
pids+=($!)

sleep 4
echo "Starting Kobuki serial bridge (/cmd_vel -> robot)..."
ros2 run kobuki_control kobuki_control_node &
pids+=($!)

if [ "${QBOT_NO_VIEWER:-0}" != "1" ]; then
  sleep 2
  echo "Starting live viewer (press q in the window or Ctrl-C here to quit)..."
  python3 "$REPO/Component testing/gesture_feed_view.py" &
  pids+=($!)
fi

wait
