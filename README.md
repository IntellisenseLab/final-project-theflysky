# QBot Autonomous Vision-Based Control System

## Project Overview

This project implements an autonomous mobile robot system using ROS 2. A Kobuki-based QBot uses computer vision to detect faces, recognize hand gestures, and execute motion behaviors such as approach, stop, and rotate.

The software is organized as a modular ROS 2 workspace with separate nodes for perception, decision logic, and robot control.

## Objectives

- Detect and track human faces from RGB camera input
- Recognize hand gestures as command inputs
- Control robot motion from visual signals
- Implement behavior-based decision logic
- Integrate all components into a single ROS 2 pipeline

## System Architecture

### Vision Node

- Captures camera frames
- Performs face detection with OpenCV
- Publishes perception outputs

### Gesture Node

- Runs hand landmark/gesture detection using MediaPipe and OpenCV
- Classifies supported gesture commands
- Publishes gesture outputs
- Detects stream gestures such as beckon curls, palm waves, and index-finger circles

### Behavior Node

- Subscribes to vision and gesture topics
- Resolves behavior priorities and robot actions
- Publishes velocity commands on `/cmd_vel`
- Executes Kobuki/QBot pet-like behaviors and sound cues
- Re-centers the robot toward the latest hand or face target after each action

### QBot Controller

- Interfaces with the Kobuki/QBot base
- Executes motion commands from behavior outputs

## Technologies

- ROS 2 (Jazzy)
- Python 3
- OpenCV
- MediaPipe
- Kobuki ROS drivers

## Project Structure

```text
qbot_ws/
├── src/
│   ├── vision_node/
│   ├── gesture_node/
│   ├── behavior_node/
│   ├── kinect_camera/
│   └── qbot_bringup/
├── build/
├── install/
└── log/
```

## Setup

1. Clone the repository and enter the workspace:

```bash
git clone https://github.com/IntellisenseLab/final-project-theflysky.git
cd final-project-theflysky
```

2. Install system and ROS dependencies (Ubuntu/ROS Jazzy):

```bash
# Review and run commands from requirements.txt
# (contains apt and ROS tooling commands only)
```

3. Create a Python virtual environment and install Python packages:

```bash
python3 -m venv .venv
./.venv/bin/pip install -r requirements-venv.txt
```

4. Build and source the ROS 2 workspace:

```bash
cd qbot_ws
colcon build
source install/setup.bash
```

5. Launch the integrated system:

```bash
ros2 launch qbot_bringup system.launch.py
```

For a real Kobuki/QBot base, the launch file defaults to the Kobuki-style
velocity topic `/commands/velocity`. If your driver or mux listens on `/cmd_vel`,
run:

```bash
ros2 launch qbot_bringup system.launch.py cmd_vel_topic:=/cmd_vel
```

See `KOBUKI_QBOT_GESTURE_CONTROL.md` for the full gesture table, Kobuki sound
topic, launch arguments, and tuning notes.

The launch file also points ROS 2's system Python at the repo virtual
environment so MediaPipe can be imported from `.venv`. Override
`venv_site_packages:=...` if the project is moved to a different path.

### Running Python scripts in this repo

Use the project virtual environment so Python dependencies stay isolated:

```bash
source .venv/bin/activate
python "Component testing/hand_gestures.py"
deactivate
```

## Features

- Real-time MediaPipe face tracking with Haar fallback
- MediaPipe hand gesture recognition with CLAHE preprocessing, landmark
  EMA + jump rejection, majority-vote label smoothing and per-frame
  brightness quality gating
- Continuous-signal beckon detector (does not depend on MediaPipe's
  discrete OPEN_PALM/CLOSED_FIST flip-flopping)
- Depth-aware come-closer that stops when the user is within
  `come_closer_target_m` and aborts forward motion if anything is
  within `obstacle_stop_m`
- Distinctive Kobuki sound sequences for every gesture command plus an
  opt-in idle wag so the robot looks alive between commands
- Stream-based pet gestures:
  - beckon/curl hand 2 times: robot comes closer
  - open palm held: robot stops
  - index finger circle: robot rotates once
  - index finger left/right: robot moves one foot in that direction
  - open palm wave: robot oscillates left and right like a tail wag
  - every non-stop command ends with visual re-centering toward the
    user, biased by the last known hand or face offset

## Testing Workflow

1. Verify camera device and image feed (`ros2 topic hz /kinect/rgb/image_raw`)
2. Run and validate the vision node (`ros2 topic echo /vision/target`)
3. Run and validate the gesture node (`ros2 topic echo /gesture/tracking`)
4. Confirm motion control with teleoperation and behavior outputs
5. Launch the full system and verify end-to-end behavior

Run the offline decoder unit tests (no ROS or camera required):

```bash
PYTHONPATH=qbot_ws/src/gesture_node:.venv/lib/python3.12/site-packages \
  python3 -m pytest qbot_ws/src/gesture_node/test/test_decoder.py -q
```

## Notes

- Install dependencies before building the workspace
- Keep node parameters configurable instead of hardcoded
- Performance depends on camera quality and lighting conditions

## Future Work

- Add identity-level face recognition
- Improve gesture classifier robustness
- Integrate navigation/SLAM capabilities
- Add voice command support

## Team

Team FlySky

## License

This project is intended for academic and research use.

## References

- ROS 2 installation guide: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- OpenCV documentation: https://docs.opencv.org/
- MediaPipe documentation: https://ai.google.dev/edge/mediapipe
- Gesture idetifying models: https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/


## Kinect 360 Support

A ROS 2 Kinect v1 / Xbox 360 publisher now lives in `qbot_ws/src/kinect_camera`.
It publishes `/kinect/rgb/image_raw` and `/kinect/depth/image_raw` using `libfreenect` and is designed for Ubuntu 24.04 + ROS 2 Jazzy without relying on the fragile legacy Python wrapper.

See `KINECT_360_SETUP.md` for the full step-by-step install, debugging, run, and viewer guide. See `qbot_ws/src/kinect_camera/README.md` for the package-local summary.
