# Kinect 360 / Kinect for Windows Setup Guide

Validated on April 7, 2026 on this machine with:

- Ubuntu 24.04.4 LTS
- ROS 2 Jazzy
- Python 3.12.3
- Kinect for Windows v1 style USB IDs `045e:02c2`, `045e:02ad`, `045e:02ae`

This guide documents the exact path that worked in this repository, including the issues we hit in the terminal and how we fixed them.

## What is already in this repo

This repository now includes a ROS 2 package at `qbot_ws/src/kinect_camera`.

It publishes:

- `/kinect/rgb/image_raw`
- `/kinect/depth/image_raw`

The package uses the lower-level `libfreenect` camera API instead of the old synchronous wrapper. That matters for Kinect for Windows devices, because the sync path tried to open motor and camera together and failed with motor or LED related errors before the stream came up.

## 1. Check that the Kinect is really detected

Run:

```bash
lsusb
```

For this setup, the important lines looked like this:

```text
Bus 003 Device 006: ID 045e:02c2 Microsoft Corp. Kinect for Windows NUI Motor
Bus 003 Device 008: ID 045e:02ad Microsoft Corp. Xbox NUI Audio
Bus 003 Device 009: ID 045e:02ae Microsoft Corp. Xbox NUI Camera
```

Important: in our session, `/dev/video0` and `/dev/video1` belonged to the laptop webcam, not the Kinect. Do not assume the Kinect shows up as a normal webcam device.

## 2. Make sure your user can access the Kinect

Check group membership:

```bash
groups
```

You want to see `video` and `plugdev`.

If they are missing, add them:

```bash
sudo usermod -aG video,plugdev $USER
```

Log out and back in after changing groups.

If USB permissions are still too strict, install the OpenKinect udev rules:

```bash
sudo tee /etc/udev/rules.d/51-kinect.rules >/dev/null <<'RULES'
# ATTR{product}=="Xbox NUI Motor"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
# ATTR{product}=="Xbox NUI Audio"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
# ATTR{product}=="Xbox NUI Camera"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
# Kinect for Windows
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
RULES
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 3. Clone and build `libfreenect` locally

This is the exact path we used successfully.

```bash
cd /tmp
rm -rf libfreenect
git clone --depth 1 https://github.com/OpenKinect/libfreenect

cmake -S /tmp/libfreenect -B /tmp/libfreenect/build \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_FAKENECT=OFF \
  -DBUILD_PYTHON=OFF \
  -DBUILD_PYTHON2=OFF \
  -DBUILD_PYTHON3=OFF \
  -DBUILD_CPP=OFF \
  -DBUILD_CV=OFF \
  -DBUILD_OPENNI2_DRIVER=OFF

cmake --build /tmp/libfreenect/build -j$(nproc)
```

Notes:

- This keeps the setup local and avoids needing a full system install.
- The repository package is built against this local checkout by using `LIBFREENECT_ROOT=/tmp/libfreenect`.

## 4. Fetch the Kinect audio firmware blob

This was necessary for the Kinect for Windows hardware.

```bash
mkdir -p ~/.libfreenect
cd /tmp/libfreenect/src
python3 ./fwfetcher.py ~/.libfreenect/audios.bin
```

After this step, you should have:

```bash
ls -lh ~/.libfreenect/audios.bin
```

## 5. Build the ROS 2 package in this repo

From the repository root:

```bash
cd /home/nadeesha/final-project-theflysky/qbot_ws
LIBFREENECT_ROOT=/tmp/libfreenect \
colcon build --packages-select kinect_camera \
  --cmake-args -DLIBFREENECT_ROOT=/tmp/libfreenect

source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
```

If your repository lives somewhere else, change the path accordingly.

## 6. Run the Kinect ROS 2 node

Use these environment variables so the node can find the locally built library and the downloaded firmware:

```bash
source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
export LD_LIBRARY_PATH=/tmp/libfreenect/build/lib:$LD_LIBRARY_PATH
export LIBFREENECT_FIRMWARE_PATH=$HOME/.libfreenect
ros2 run kinect_camera kinect_rgbd_node
```

If you prefer the launch file:

```bash
source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
export LD_LIBRARY_PATH=/tmp/libfreenect/build/lib:$LD_LIBRARY_PATH
export LIBFREENECT_FIRMWARE_PATH=$HOME/.libfreenect
ros2 launch kinect_camera kinect_rgbd.launch.py
```

A harmless warning may still appear on Kinect for Windows hardware:

```text
Failed to set the LED of K4W or 1473 device: LIBUSB_ERROR_IO
```

In our final working run, the node still started and the image topics published correctly.

## 7. Verify that the streams are live

List the topics:

```bash
source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
ros2 topic list
```

Expected topics:

```text
/kinect/rgb/image_raw
/kinect/depth/image_raw
```

Check RGB metadata:

```bash
source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
ros2 topic echo --once --timeout 5 /kinect/rgb/image_raw --field header.frame_id
ros2 topic echo --once --timeout 5 /kinect/rgb/image_raw --field encoding
ros2 topic echo --once --timeout 5 /kinect/rgb/image_raw --field width
ros2 topic echo --once --timeout 5 /kinect/rgb/image_raw --field height
```

Expected values:

```text
kinect_rgb_optical_frame
rgb8
640
480
```

Check depth metadata:

```bash
source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
ros2 topic echo --once --timeout 5 /kinect/depth/image_raw --field header.frame_id
ros2 topic echo --once --timeout 5 /kinect/depth/image_raw --field encoding
ros2 topic echo --once --timeout 5 /kinect/depth/image_raw --field width
ros2 topic echo --once --timeout 5 /kinect/depth/image_raw --field height
```

Expected values:

```text
kinect_depth_optical_frame
16UC1
640
480
```

If `ros2 topic echo` prints `A message was lost!!!`, that usually means the CLI subscriber is slower than the image stream. It is not fatal if you still receive the expected metadata.

## 8. View the RGB stream on the laptop

### Option A: try `rqt_image_view`

```bash
source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Then select `/kinect/rgb/image_raw` in the GUI.

### Option B: OpenCV fallback viewer

If `rqt_image_view` closes immediately on your machine, use this direct ROS 2 subscriber viewer instead:

```bash
source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
python3 - <<'PY'
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

class KinectRgbViewer(Node):
    def __init__(self):
        super().__init__('kinect_rgb_viewer')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/kinect/rgb/image_raw', self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Kinect RGB', frame)
        cv2.waitKey(1)

rclpy.init()
node = KinectRgbViewer()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
PY
```

## 9. Issues we hit and how we solved them

### Issue 1

```text
Failed to set the LED of K4W or 1473 device: LIBUSB_ERROR_IO
```

What it meant:

- The Kinect for Windows unit was unhappy with motor or LED initialization.

What fixed it:

- Downloading `~/.libfreenect/audios.bin`
- Avoiding the old sync wrapper path
- Using the repo's updated `kinect_camera` package, which opens the camera subdevice only

### Issue 2

```text
Could not open device: LIBUSB_ERROR_NO_DEVICE
Could not open device: LIBUSB_ERROR_IO
Could not find device sibling
Error: Invalid index [0]
```

What it meant:

- The original ROS node path was using the sync wrapper, which tried to open motor and camera together.
- On this hardware, that caused the whole stream setup to collapse before RGB or depth could publish.

What fixed it:

- Rewriting the node to use the lower-level `libfreenect` camera API and `freenect_select_subdevices(..., FREENECT_DEVICE_CAMERA)`

### Issue 3

`rqt_image_view` exited immediately in this session.

What fixed it:

- Using the OpenCV fallback viewer above

## 10. Optional system-wide install

If you prefer a system package instead of the local build, try:

```bash
sudo apt-get update
sudo apt-get install -y freenect
```

Even then, for Kinect for Windows hardware, you may still need the firmware blob at `~/.libfreenect/audios.bin`.

## 11. Files to look at in this repo

- `qbot_ws/src/kinect_camera/src/kinect_rgbd_node.cpp`
- `qbot_ws/src/kinect_camera/CMakeLists.txt`
- `qbot_ws/src/kinect_camera/launch/kinect_rgbd.launch.py`
- `qbot_ws/src/kinect_camera/README.md`

## 12. Upstream references

- OpenKinect `libfreenect`: https://github.com/OpenKinect/libfreenect
- Official Linux udev rules: https://raw.githubusercontent.com/OpenKinect/libfreenect/master/platform/linux/udev/51-kinect.rules
- Ubuntu Noble `libfreenect` source package: https://launchpad.net/ubuntu/noble/+source/libfreenect
- ROS 2 Jazzy `rqt_image_view`: https://docs.ros.org/en/ros2_packages/jazzy/api/rqt_image_view/
