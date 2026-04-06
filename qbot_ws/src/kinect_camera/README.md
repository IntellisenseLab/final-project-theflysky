# Kinect 360 on ROS 2 Jazzy

This package publishes Kinect v1 / Xbox 360 RGB and depth frames as ROS 2 topics using `libfreenect`.

## Why this package exists

On this machine, the active stack is:

- Ubuntu 24.04.4 LTS
- ROS 2 Jazzy
- Python 3.12.3

That combination makes the legacy `libfreenect` Python wrapper a risky capture path. Upstream still documents Python 3 support, but it also warns that wrappers are not guaranteed to stay up to date. This package avoids that problem by talking to `libfreenect` from C++ and publishing standard ROS 2 `sensor_msgs/Image` topics.

## Topics

- `/kinect/rgb/image_raw`
- `/kinect/depth/image_raw`

## Recommended install on Ubuntu 24.04

Use Ubuntu's packaged `freenect` first:

```bash
sudo apt-get update
sudo apt-get install -y freenect
sudo usermod -aG video,plugdev $USER
```

Log out and back in after changing groups.

If you still get USB permission failures, install the upstream udev rules:

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

## Fallback if K4W audio or LED initialization still fails

Some Kinect for Windows style devices need the audio firmware path that upstream documents. If the packaged build still gives errors like LED or motor I/O failures, build upstream `libfreenect` from source and let it fetch the firmware locally:

```bash
git clone https://github.com/OpenKinect/libfreenect
cd libfreenect
cmake -S . -B build \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_FAKENECT=OFF \
  -DBUILD_PYTHON=OFF \
  -DBUILD_PYTHON2=OFF \
  -DBUILD_PYTHON3=OFF \
  -DBUILD_C_SYNC=ON \
  -DBUILD_REDIST_PACKAGE=OFF
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig
```

## Build this ROS 2 package

After `libfreenect` is available:

```bash
cd /home/nadeesha/final-project-theflysky/qbot_ws
colcon build --packages-select kinect_camera
source install/setup.bash
```

If you built `libfreenect` in a local checkout instead of installing it system-wide, point the package at that source tree before building:

```bash
export LIBFREENECT_ROOT=/path/to/libfreenect
export LD_LIBRARY_PATH=$LIBFREENECT_ROOT/build/lib:$LD_LIBRARY_PATH
colcon build --packages-select kinect_camera --cmake-args -DLIBFREENECT_ROOT=$LIBFREENECT_ROOT
source install/setup.bash
```

## Run the Kinect node

```bash
ros2 launch kinect_camera kinect_rgbd.launch.py
```

## View the stream on the laptop

In a second terminal:

```bash
source /home/nadeesha/final-project-theflysky/qbot_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Then choose one of these topics inside `rqt_image_view`:

- `/kinect/rgb/image_raw`
- `/kinect/depth/image_raw`

## Useful parameter overrides

```bash
ros2 launch kinect_camera kinect_rgbd.launch.py enable_depth:=false
ros2 launch kinect_camera kinect_rgbd.launch.py device_index:=0 publish_rate_hz:=15.0
```
