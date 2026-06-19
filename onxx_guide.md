# ONNX Setup Guide for Hand Gesture Recognition

## Overview

This guide explains how to set up and use **ONNX Runtime** with converted **MediaPipe hand models** for real-time hand tracking and gesture recognition on QBot. The system runs on **ARM64 (Raspberry Pi 5)** where MediaPipe's native Python library has no wheel.

### Why ONNX?

| Problem | Solution |
|---------|----------|
| **MediaPipe** has no ARM64 Python wheel | **ONNX Runtime** supports ARM64 |
| Depth silhouette (earlier approach) couldn't distinguish open palm vs fist at arm's length | **21-point hand landmarks** give per-finger joint positions with 0.98+ confidence |
| Need CPU-only inference on Pi without GPU | **ONNX Runtime** CPU provider works anywhere |

---

## 1. Installation

### 1.1 Install onnxruntime (pip)

```bash
# Install onnxruntime for Python
python3 -m pip install --break-system-packages onnxruntime

# Verify installation
python3 -c "import onnxruntime; print('onnxruntime', onnxruntime.__version__, 'OK')"
```

**Note:** `--break-system-packages` is needed on Ubuntu 24.04 where pip respects PEP 668 (externally managed environments). Alternatively, use a venv.

### 1.2 Install libfreenect (for Kinect camera)

```bash
# Install the system library
sudo apt install -y libfreenect-dev freenect

# Build the Python freenect binding (not in apt for Ubuntu 24.04)
git clone --depth 1 https://github.com/OpenKinect/libfreenect /tmp/libfreenect
cd /tmp/libfreenect/wrappers/python
python3 setup.py build_ext --inplace
sudo cp freenect.cpython-*.so /usr/local/lib/python3.12/dist-packages/

# Verify
python3 -c "import freenect; print('freenect OK')"
```

### 1.3 Install ROS 2 dependencies

```bash
# Core dependencies for image handling
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-sensor-msgs \
  python3-opencv \
  python3-numpy
```

---

## 2. Download ONNX Models

The system uses **two pre-trained ONNX models** from [OpenCV Model Zoo](https://github.com/opencv/opencv_zoo):

1. **Palm Detection** (`palm_detection_mediapipe_2023feb.onnx`) — finds hands in the image
2. **Hand Pose** (`handpose_estimation_mediapipe_2023feb.onnx`) — extracts 21 landmarks per hand

### 2.1 Automatic Download (recommended)

Run the provided setup script:

```bash
bash setup_qbot_env.sh
```

This downloads models to `src/gesture_node/models/` and verifies them.

### 2.2 Manual Download

```bash
# Create models directory
mkdir -p src/gesture_node/models

# Download palm detection model
curl -sL -o src/gesture_node/models/palm_detection_mediapipe_2023feb.onnx \
  "https://huggingface.co/opencv/palm_detection_mediapipe/resolve/main/palm_detection_mediapipe_2023feb.onnx"

# Download hand pose model
curl -sL -o src/gesture_node/models/handpose_estimation_mediapipe_2023feb.onnx \
  "https://huggingface.co/opencv/handpose_estimation_mediapipe/resolve/main/handpose_estimation_mediapipe_2023feb.onnx"

# Verify files exist
ls -lh src/gesture_node/models/
# Output should show ~3 MB + ~12 MB files
```

### 2.3 Model Format

- **Format:** ONNX (Open Neural Network Exchange)
- **Framework Origin:** MediaPipe (converted by OpenCV)
- **Runtime:** ONNX Runtime (CPU inference, ~30–40ms per frame on Pi)
- **Input:** 192×192 or 224×224 RGB images (pre-processed by wrapper classes)
- **Output:** Bounding boxes, 21 hand landmarks (x, y, z per joint), confidence scores

---

## 3. Code Integration

### 3.1 Model Wrappers

The gesture_node wraps each ONNX model in a class for easy use:

**[`gesture_node/mp_models/mp_palmdet.py`](../src/gesture_node/gesture_node/mp_models/mp_palmdet.py)** — Palm Detection
```python
import onnxruntime as ort

class MPPalmDet:
    def __init__(self, modelPath, nmsThreshold=0.3, scoreThreshold=0.5):
        # Load ONNX model with CPU-only execution
        self.session = ort.InferenceSession(modelPath, 
                                           providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
    
    def infer(self, image):
        # image: BGR, HxWx3
        # Returns: detected palms (bounding boxes + 7 palm landmarks)
        output_blob = self.session.run(None, {self.input_name: preprocessed_image})
        return self._postprocess(output_blob)
```

**[`gesture_node/mp_models/mp_handpose.py`](../src/gesture_node/gesture_node/mp_models/mp_handpose.py)** — Hand Pose
```python
class MPHandPose:
    def __init__(self, modelPath, confThreshold=0.8):
        self.session = ort.InferenceSession(modelPath, 
                                           providers=['CPUExecutionProvider'])
    
    def infer(self, image, palm):
        # image: BGR RGB frame
        # palm: detected palm from palm detector
        # Returns: 21 hand landmarks (x, y, z) + confidence
        output_blob = self.session.run(None, {self.input_name: preprocessed_image})
        return self._postprocess(output_blob)
```

### 3.2 Hand Landmark Tracker (High-Level API)

**[`gesture_node/hand_tracker.py`](../src/gesture_node/gesture_node/hand_tracker.py)**

```python
from gesture_node.hand_tracker import HandLandmarkTracker, HandFeatures

# Initialize (loads both ONNX models)
tracker = HandLandmarkTracker(model_dir="src/gesture_node/models/")

# Process each frame
features = tracker.process(bgr_image)

# Output: HandFeatures dataclass
print(features.fingers)        # 0–5 (number of extended fingers)
print(features.label)          # 'POINTING', 'FIST', 'OPEN_PALM', etc.
print(features.landmarks)      # (21, 3) array in [0, 1] space
print(features.openness)       # 0.0 (fist) to 1.0 (open palm)
print(features.pointing)       # 'LEFT', 'RIGHT', 'UP', 'DOWN', 'NONE'
```

### 3.3 Pipeline

```
RGB Video Frame (640×480)
    ↓
[Palm Detection ONNX] → detect hands
    ↓
[Hand Pose ONNX] → extract 21 landmarks per hand
    ↓
[Landmark Smoother] → temporal filtering
    ↓
[Feature Extraction] → finger states, openness, pointing direction
    ↓
[Gesture Classifier] → temporal state machine
    ↓
Command Event (forward/backward/turn_left/rotate360/etc.)
```

---

## 4. Running the System

### 4.1 Build the workspace

```bash
source /opt/ros/jazzy/setup.bash
cd ~/flysky
colcon build --symlink-install

# On Pi with limited RAM, use limited parallelism
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1
source install/setup.bash
```

### 4.2 Launch the gesture node alone (test perception without robot)

```bash
ros2 launch gesture_node gesture_test.launch.py
```

**Output:** ROS topics
- `/vision/target` — tracked person (largest face)
- `/gesture/command` — detected gesture + confidence

### 4.3 Launch the full system (includes robot base)

```bash
ros2 launch behavior_node qbot.launch.py
```

### 4.4 Monitor hand tracking in real-time

```bash
# In another terminal
ros2 topic echo /gesture/command --field data
# Output: forward, backward, rotate360, turn_left, turn_right, tail_wag, stop
```

---

## 5. Hand Landmarks (MediaPipe 21-Point Format)

Each hand is represented as 21 (x, y, z) landmarks in normalized space [0, 1]:

| Index | Joint | Index | Joint | Index | Joint |
|-------|-------|-------|-------|-------|-------|
| 0 | Wrist | 7 | Index PIP | 14 | Ring PIP |
| 1 | Thumb CMC | 8 | Index TIP | 15 | Ring DIP |
| 2 | Thumb MCP | 9 | Middle MCP | 16 | Ring TIP |
| 3 | Thumb IP | 10 | Middle PIP | 17 | Pinky MCP |
| 4 | Thumb TIP | 11 | Middle DIP | 18 | Pinky PIP |
| 5 | Index MCP | 12 | Middle TIP | 19 | Pinky DIP |
| 6 | Index PIP | 13 | Ring MCP | 20 | Pinky TIP |

**Coordinate Space:**
- `x`: [0, 1] fraction of image width (0 = left, 1 = right)
- `y`: [0, 1] fraction of image height (0 = top, 1 = bottom)
- `z`: [0, 1] relative depth (0 = far, 1 = close to camera, relative to wrist)

See [signals.py](../src/gesture_node/gesture_node/signals.py) for landmark-based feature extraction.

---

## 6. Gesture Recognition Pipeline

Once hand landmarks are extracted, the gesture classifier detects commands via a **temporal state machine**:

### Gesture Vocabulary

| Gesture | Hand Motion | Robot Command | Thresholds |
|---------|-------------|---------------|-----------|
| **Stop** | Open palm held still | Halt immediately | Openness ≥ 0.70, held 0.45s, motion < 0.12 pixels |
| **Forward** | Curl hand twice (open→close→open→close) | Drive forward | 2+ curl oscillations in 4.5s window |
| **Backward** | Thumbs-down held | Drive backward | Thumb down (detected by z-coordinate), held 0.6s |
| **Rotate 360** | Index finger drawing circle | Spin full rotation | Circle radius ≥ 4% image, passes all 4 quadrants |
| **Turn Left** | Index pointing left, held | Sidestep left 90° | POINTING pose, pointing LEFT, held 0.55s |
| **Turn Right** | Index pointing right, held | Sidestep right 90° | POINTING pose, pointing RIGHT, held 0.55s |
| **Tail Wag** | Open-hand wave side-to-side | Oscillate 3× | Palm motion ≥ 8% image width, 2+ reversals |

See [gesture_classifier.py](../src/gesture_node/gesture_node/gesture_classifier.py) and [signals.py](../src/gesture_node/gesture_node/signals.py) for implementation.

---

## 7. Tuning & Calibration

### 7.1 Enable Debug Logging

```bash
ros2 launch gesture_node gesture_test.launch.py debug:=true
```

**Logs per-frame features:**
- `fingers` — finger count (0–5)
- `index_only` — true if only index is extended (POINTING pose)
- `openness` — continuous 0.0 (fist) to 1.0 (open) score
- `label` — discrete pose (FIST, OPEN_PALM, POINTING, etc.)
- `pointing` — direction (LEFT, RIGHT, UP, DOWN, NONE)

### 7.2 Tune Thresholds

**Config file:** [`src/gesture_node/config/gesture_classifier.yaml`](../src/gesture_node/config/gesture_classifier.yaml)

Example adjustments:

```yaml
# Decrease to make gestures more sensitive
stop_open: 0.70          # lower → easier to trigger
beckon_oscillations: 2   # lower → fewer curls required

# Increase to require more deliberate motion
stop_motion_tol: 0.04    # higher → allow more palm drift
rotate_min_radius: 0.04  # higher → require larger circle
```

### 7.3 Common Issues

**Problem:** Fingers miscount (wrapped thumb reads as 1 finger instead of 0)

**Solution:** Lower `slack` parameter in `extended()` check or inspect raw z-coordinates.

**Problem:** Gestures fire too easily

**Solution:** Increase `hold` thresholds (e.g., `stop_hold: 0.45` → `0.60`)

**Problem:** Rotate360 not detected

**Solution:** Ensure circle motion passes all 4 quadrants relative to circle center (debug log `quads`).

---

## 8. Troubleshooting

### 8.1 ONNX Runtime Not Found

```bash
python3 -c "import onnxruntime"
# ModuleNotFoundError: No module named 'onnxruntime'

# Fix: install with pip
python3 -m pip install onnxruntime
```

### 8.2 Model Files Not Found

```
FileNotFoundError: ONNX model not found: .../palm_detection_mediapipe_2023feb.onnx
Run setup_qbot_env.sh to download it.

# Fix: download models
bash setup_qbot_env.sh
# or manually:
mkdir -p src/gesture_node/models
# then curl as shown in Section 2.2
```

### 8.3 Kinect Camera Not Detected

```bash
# Verify freenect module
python3 -c "import freenect; print(freenect.freenect_sync_get_depth_cv2())"

# If fails, check USB permissions and rebuild freenect binding
# See QBOT_OVERVIEW.md "Kinect operational notes"
```

### 8.4 Inference Slow (>100ms per frame)

- **CPU bound:** 30–40ms is normal on Pi 5 for both models
- **Check I/O:** Is Kinect frame grab blocking? See `kinect_rgbd.py`
- **Reduce resolution:** Downscale input before inference (tradeoff: less detail)
- **Lower image quality:** Reduce Kinect frame rate from 30 Hz to 15 Hz in launch file

### 8.5 Low Confidence Detections

**Symptom:** Frequent loss of hand tracking or wrong pose classification

**Causes:**
1. **Poor lighting** — hand landmarks need contrast
2. **Hand too far** — models trained for ~0.5–1.5 m distance
3. **Occlusion** — fingers behind other objects
4. **Low contrast** — dark hand on dark background

**Fixes:**
- Increase room lighting
- Ensure hand is within ~1.5 m of camera
- Adjust `scoreThreshold` and `confThreshold` in `hand_tracker.py` (lower = more lenient)

---

## 9. Performance Metrics

### Latency Breakdown (Pi 5, 4 GB RAM, @ 30 Hz)

| Step | Time |
|------|------|
| Kinect capture | ~8 ms |
| **Palm detection (ONNX)** | **~12 ms** |
| **Hand pose (ONNX)** | **~18 ms** |
| Landmark smoothing | ~1 ms |
| Feature extraction | ~2 ms |
| Gesture classification | ~1 ms |
| **Total** | **~42 ms** (~24 Hz effective) |

### Memory Usage

- **ONNX Runtime** base: ~20 MB
- **Palm model** (in-memory): ~3 MB
- **Hand model** (in-memory): ~12 MB
- **Frame buffers** (RGB + depth): ~4 MB
- **Total per-process**: ~45 MB

Safe on 4 GB Pi alongside ROS 2 Jazzy + Kobuki driver + behavior node.

---

## 10. Extending the System

### 10.1 Add a Custom ONNX Model

```python
# Create a new wrapper class (e.g., gesture_node/mp_models/my_model.py)
import onnxruntime as ort

class MyModel:
    def __init__(self, model_path):
        self.session = ort.InferenceSession(model_path, 
                                           providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name
    
    def infer(self, image):
        input_blob = self._preprocess(image)
        output = self.session.run(None, {self.input_name: input_blob})
        return self._postprocess(output)
```

### 10.2 Add a New Gesture

1. **Extract features** from landmarks in `signals.py` (e.g., `wrist_velocity()`)
2. **Implement detector** in `gesture_classifier.py` (e.g., `_my_gesture_ready()`)
3. **Add to command list** in `update()` method
4. **Map to behavior** in `behavior_node/pet_behavior_node.py`

Example: detect "peace sign" (index + middle extended)

```python
# signals.py
def is_peace(lms):
    finger_pattern = fingers_up(lms)
    return finger_pattern == [0, 1, 1, 0, 0]  # thumb, index, middle, ring, pinky

# gesture_classifier.py
if label == 'PEACE':
    peace_track.append((t, ...))
    if len(peace_track) > 10:
        return 'peace'  # or whatever command
```

---

## 11. References

- **ONNX Runtime:** https://onnxruntime.ai/
- **OpenCV Model Zoo:** https://github.com/opencv/opencv_zoo
- **MediaPipe Handpose:** https://developers.google.com/mediapipe/solutions/vision/hand_landmarker
- **QBot Hardware:** See `QBOT_OVERVIEW.md`
- **Gesture Algorithm:** See `GESTURE_ALGORITHM_REFERENCE.md`

---

## 12. Quick Checklist

- [ ] Install onnxruntime: `pip3 install onnxruntime`
- [ ] Install libfreenect dev package and build Python binding
- [ ] Download ONNX models (or run `setup_qbot_env.sh`)
- [ ] Build workspace: `colcon build --symlink-install`
- [ ] Verify Kinect is detected: `ros2 launch kinect_camera kinect_rgbd.launch.py`
- [ ] Test gesture recognition: `ros2 launch gesture_node gesture_test.launch.py`
- [ ] Calibrate thresholds with `debug:=true`
- [ ] Launch full system: `ros2 launch behavior_node qbot.launch.py`
- [ ] Verify robot responds to gestures

---

**Questions?** Check `CLAUDE.md` for hardware status, `GESTURE_ALGORITHM_REFERENCE.md` for detector deep-dives, or raise an issue.
