# Kobuki QBot Gesture Control

A ROS 2 (Jazzy) gesture-to-behaviour pipeline that runs on a Raspberry
Pi 5, reads from a Kinect 360, and drives a Kobuki / QBot base.

## What Runs

* `kinect_camera/kinect_rgbd_node` (C++) — libfreenect driver that
  publishes `/kinect/rgb/image_raw` and `/kinect/depth/image_raw`. Each
  frame is pushed out directly from the libfreenect callback, so there
  is no extra timer latency or duplicate-frame republishing.
* `gesture_node/gesture_command_node` — MediaPipe gesture pipeline with
  CLAHE preprocessing, landmark-jump rejection, and majority-vote label
  smoothing. Publishes:
  * `/gesture/tracking` — continuous JSON state (centre, openness,
    pointing direction) used by the behaviour node to re-centre on you.
  * `/gesture/command` — confirmed temporal commands (see table).
* `vision_node/face_tracker_node` — MediaPipe FaceDetection (with a
  Haar-cascade fallback for stripped-down images). Publishes
  `/vision/target` as a secondary re-centring signal when the hand is
  out of frame.
* `behavior_node/pet_behavior_node` — turns commands into Twist
  segments, plays distinctive Kobuki sound sequences, optionally uses
  the Kinect depth stream both to stop come-closer at the right
  distance and to abort any forward motion if something is in the way.

## Gesture Commands

| Gesture | Stream rule | Robot behaviour |
| --- | --- | --- |
| Curl/beckon hand 2 times | Continuous openness signal oscillates open↔closed twice within ~4.5 s | Drive forward; stop at `come_closer_target_m` (0.7 m by default) or after `come_closer_max_time_sec` |
| Open palm held still | Openness ≥ 0.7 for ~0.45 s with no horizontal motion | Stop in place immediately |
| Index finger circle | Pointing fingertip traces a full circle covering all 4 quadrants | Rotate 360° in place |
| Index finger held left | Pointing-LEFT held for ~0.55 s | Turn left, drive 1 ft, turn back |
| Index finger held right | Pointing-RIGHT held for ~0.55 s | Turn right, drive 1 ft, turn back |
| Open palm wave | Open palm with ≥ 3 horizontal reversals over ~2.4 s | Tail-wag oscillation for a few seconds |

After every non-stop command the behaviour node runs an alignment step:
turn back toward the latest hand (preferred) or face target, with the
search direction biased by the last known offset so the robot looks
where the user was last seen. If nothing is found inside
`align_search_max_sec` it gives up gracefully instead of spinning
forever.

When the user is visible but no command is active, an opt-in **idle
wag** fires every `idle_wag_interval_sec` so the robot looks alive.

## Why the New Beckon Detector Is More Reliable

Previously the come-closer command counted MediaPipe label flips
(OPEN_PALM ↔ CLOSED_FIST). MediaPipe's classifier often emits "Custom"
for partial curls, so a real beckon was easy to miss. The new detector
computes a continuous **openness** metric directly from the landmarks
(mean fingertip-to-wrist distance ÷ palm width) and counts open ↔
closed transitions with Schmitt-trigger hysteresis. Tested with
synthetic streams in `qbot_ws/src/gesture_node/test/test_decoder.py`.

## Frame-level Error Correction

The gesture node now applies multiple layers of robustness before any
temporal decision is made:

* **Quality gate** — frames with mean luma < 22 or > 245 are dropped so
  the model is never fed pure black or saturated frames.
* **CLAHE** on the luma channel — recovers contrast under indoor lamps.
* **Optional downscale** (`downscale_width`, default 480 px) — keeps the
  Pi 5 CPU happy without losing recognition quality.
* **Landmark EMA + jump rejection** — sudden landmark jumps (e.g. when
  the hand is briefly half-occluded) are damped rather than trusted.
* **Majority vote** on the per-frame label over a 7-frame window with a
  4-vote threshold.
* **Temporal cooldowns** so one gesture never fires twice in a row.

## Kobuki ROS 2 Topics

The behaviour node publishes `geometry_msgs/msg/Twist` to `/cmd_vel`,
which is what the in-repo `kobuki_control` bridge subscribes to. That
bridge clamps the velocity to safe limits, converts the Twist into the
Kobuki `(speed, radius)` API and pushes it over serial through the
`kobukidriver` Python package. Start the bridge in its own terminal:

```bash
ros2 run kobuki_control kobuki_control_node
```

If you use the upstream `kobuki_node` driver instead, switch the topic:

```bash
ros2 launch qbot_bringup system.launch.py cmd_vel_topic:=/commands/velocity
```

Cute Kobuki sounds are published as `kobuki_ros_interfaces/msg/Sound`
on `/commands/sound`. Each gesture command gets a distinct multi-tone
sequence so the user can tell from the audio alone which command was
recognised. Install the interface package on Jazzy:

```bash
sudo apt install ros-jazzy-kobuki-ros-interfaces
```

Start the Kobuki base driver separately if it is not already launched
by the robot image:

```bash
ros2 launch kobuki_node kobuki_node-launch.py
```

If the QBot image uses a different topic, check with:

```bash
ros2 topic list -t
ros2 topic info /commands/velocity -v
ros2 topic info /cmd_vel -v
```

## Running The System

From the repo root:

```bash
cd qbot_ws
colcon build
source install/setup.bash
ros2 launch qbot_bringup system.launch.py
```

If `libfreenect` was built from a local checkout instead of the system
package:

```bash
export LD_LIBRARY_PATH=/tmp/libfreenect/build/lib:$LD_LIBRARY_PATH
export LIBFREENECT_FIRMWARE_PATH=$HOME/.libfreenect
```

If the MediaPipe model lives elsewhere on the Pi:

```bash
ros2 launch qbot_bringup system.launch.py \
  gesture_model_path:=/path/to/models/gesture_recognizer.task
```

ROS 2 console scripts use the system Python interpreter. The launch
file prepends the repo venv's `site-packages` to `PYTHONPATH` so
MediaPipe can still come from `.venv`. Override the path if needed:

```bash
ros2 launch qbot_bringup system.launch.py \
  venv_site_packages:=/path/to/.venv/lib/python3.12/site-packages
```

To disable depth (saves USB bandwidth; turns off depth-aware come-closer
and the obstacle safety stop):

```bash
ros2 launch qbot_bringup system.launch.py enable_depth:=false
```

To flip mirrored left/right gestures:

```bash
ros2 launch qbot_bringup system.launch.py mirror_horizontal_commands:=true
```

If alignment turns away from the user instead of toward them, flip the
sign:

```bash
ros2 launch qbot_bringup system.launch.py target_turn_sign:=1.0
```

## Tuning

All tunable parameters live in
`qbot_ws/src/qbot_bringup/config/qbot_params.yaml` and can be overridden
from the launch command. Pi 5 defaults: gesture at 12 fps, face at
8 fps, both downscaled to 480 px wide. If recognition still lags, try:

```bash
ros2 launch qbot_bringup system.launch.py gesture_max_fps:=8.0 face_max_fps:=5.0
```

## Useful Debug Topics

```bash
ros2 topic echo /gesture/tracking
ros2 topic echo /gesture/command
ros2 topic echo /vision/target
ros2 topic echo /commands/velocity
ros2 topic hz /kinect/rgb/image_raw
ros2 topic hz /kinect/depth/image_raw
```

For an annotated overlay showing the per-frame label, pointing
direction and openness:

```bash
ros2 launch qbot_bringup system.launch.py publish_debug_image:=true
ros2 run rqt_image_view rqt_image_view
```

Then select `/gesture/debug_image`.

## Tests

Decoder logic is covered by unit tests that synthesise frame streams
(no ROS / camera needed):

```bash
PYTHONPATH=qbot_ws/src/gesture_node:.venv/lib/python3.12/site-packages \
  python3 -m pytest qbot_ws/src/gesture_node/test/test_decoder.py -q
```

## References

* Kobuki ROS 2 node package: https://index.ros.org/p/kobuki_node/
* Kobuki velocity topic note: https://idorobotics.com/2024/02/20/ros2-on-kobuki-turtlebot/
* Kobuki sound message: https://docs.ros.org/en/jazzy/p/kobuki_ros_interfaces/msg/Sound.html
* cmd_vel_mux behavior: https://github.com/kobuki-base/cmd_vel_mux
* MediaPipe gesture recogniser: https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/
* MediaPipe face detector: https://ai.google.dev/edge/mediapipe/solutions/vision/face_detector/
