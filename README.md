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

### Behavior Node

- Subscribes to vision and gesture topics
- Resolves behavior priorities and robot actions
- Publishes velocity commands on `/cmd_vel`

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
│   └── qbot_bringup/
├── build/
├── install/
└── log/
```

## Setup

1. Clone the repository and enter the workspace:

```bash
git clone https://github.com/IntellisenseLab/final-project-flysky.git
cd final-project-flysky
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

### Running Python scripts in this repo

Use the project virtual environment so Python dependencies stay isolated:

```bash
source .venv/bin/activate
python "Component testing/hand_gestures.py"
deactivate
```

## Features

- Real-time face detection
- Hand gesture recognition
- Behavior-based autonomous response
- Modular node-level architecture for maintenance and extension

## Testing Workflow

1. Verify camera device and image feed
2. Run and validate the vision node
3. Run and validate the gesture node
4. Confirm motion control with teleoperation and behavior outputs
5. Launch the full system and verify end-to-end behavior

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

