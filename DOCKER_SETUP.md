# Running the QBot Pipeline in Docker on Raspberry Pi 5

This guide packages the whole gesture-control stack — Kinect 360 driver,
gesture/face/behaviour nodes, and the Kobuki serial bridge — into **one**
Docker container so you can bring it up on a fresh Raspberry Pi 5 (Ubuntu
24.04, arm64) without hand-installing ROS, libfreenect, MediaPipe, etc.

It mirrors the validated setups in [README.md](README.md),
[KINECT_360_SETUP.md](KINECT_360_SETUP.md) and
[KOBUKI_QBOT_GESTURE_CONTROL.md](KOBUKI_QBOT_GESTURE_CONTROL.md), but inside a
container.

---

## 0. What is in the box

| File | Purpose |
| --- | --- |
| [docker/Dockerfile](docker/Dockerfile) | Builds ROS 2 Jazzy + libfreenect + Python deps + the workspace |
| [docker/entrypoint.sh](docker/entrypoint.sh) | Sources ROS + workspace and sets the Kinect/model/venv paths |
| [docker/launch/qbot_docker.launch.py](docker/launch/qbot_docker.launch.py) | Starts `system.launch.py` **plus** the `kobuki_control` serial bridge |
| [docker/docker-compose.yml](docker/docker-compose.yml) | One-command build/run with USB + serial + host networking |
| [.dockerignore](.dockerignore) | Keeps the x86 `build/`/`install/` and `.venv` out of the image |

**Design decisions (and why):**

- **Build natively on the Pi 5.** The Pi is arm64; your laptop is x86_64.
  Building on the Pi avoids QEMU emulation, which is slow and flaky for
  MediaPipe and native libraries.
- **One container, host networking, privileged.** Simplest thing that works
  for a single-board robot: all nodes share one ROS graph, and the Kinect
  (USB) and Kobuki (USB-serial) are reachable without per-device plumbing.
- **Only `mediapipe`, `opencv`, `numpy` are installed.** The ROS nodes in
  `qbot_ws/src` import only these. `dlib`/`face-recognition` from
  `requirements-venv.txt` are used by the standalone scripts in
  `Component testing/` and `AccurateGesture/`, **not** the robot pipeline, so
  they are deliberately left out (compiling dlib on arm is painful).

---

## 1. Prepare the Raspberry Pi 5

On the Pi (Ubuntu 24.04 LTS, 64-bit):

```bash
# Install Docker Engine + compose plugin
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-v2 git

# Run docker without sudo (log out / back in afterwards)
sudo usermod -aG docker $USER

# USB access for the Kinect on the HOST (helps even with a privileged container)
sudo usermod -aG video,plugdev $USER
```

Log out and back in so the group changes take effect.

> **Power note:** The Kinect 360 needs its 12 V wall adapter; the Pi's USB
> cannot power it. Use a powered USB hub if you draw a lot from the Pi's ports.

Install the Kinect udev rules on the host (same rules as
[KINECT_360_SETUP.md](KINECT_360_SETUP.md) §2):

```bash
sudo tee /etc/udev/rules.d/51-kinect.rules >/dev/null <<'RULES'
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ae", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02ad", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02b0", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c2", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02be", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02bf", MODE="0666"
RULES
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 2. Get the code onto the Pi

```bash
git clone https://github.com/IntellisenseLab/final-project-theflysky.git
cd final-project-theflysky
```

(Or `scp`/`rsync` your working copy across. The `.dockerignore` keeps the
heavy `build/`, `install/`, `log/` and `.venv` directories out of the build.)

---

## 3. Build the image (one-time, on the Pi)

```bash
docker compose -f docker/docker-compose.yml build
```

This is the slow step — it compiles libfreenect, installs MediaPipe, and runs
`colcon build`. Expect **15–40+ minutes** on a Pi 5 the first time. Subsequent
builds reuse cached layers.

Equivalent without compose:

```bash
docker build -f docker/Dockerfile -t qbot:latest .
```

> If the build dies on `pip install ... mediapipe`, jump to
> [Troubleshooting → MediaPipe on arm64](#mediapipe-on-arm64). This is the
> single most likely failure point.

---

## 4. Plug in the hardware and check the host sees it

Connect the **Kinect** (with its power brick) and the **Kobuki** USB cable,
then on the Pi:

```bash
lsusb                 # expect 045e:02ae (Camera), 045e:02ad (Audio), 045e:02c2 (Motor)
ls /dev/ttyUSB*       # the Kobuki USB-serial port, e.g. /dev/ttyUSB0
```

If `/dev/ttyUSB0` is missing, the Kobuki isn't powered/connected or its
USB-serial driver didn't bind — fix that before running.

---

## 5. Run the full pipeline

```bash
docker compose -f docker/docker-compose.yml up
```

This launches, inside the container:

- `kinect_rgbd_node` → `/kinect/rgb/image_raw`, `/kinect/depth/image_raw`
- `gesture_command_node` → `/gesture/command`, `/gesture/tracking`
- `face_tracker_node` → `/vision/target`
- `pet_behavior_node` → `/cmd_vel`, `/commands/sound`
- `kobuki_control_node` → reads `/cmd_vel`, drives the Kobuki over serial

Press `Ctrl-C` to stop. To run detached: add `-d`, and follow logs with
`docker compose -f docker/docker-compose.yml logs -f`.

You should see `Kobuki connected successfully.` in the logs. If the Kobuki
isn't found you'll instead see `Kobuki not connected: ...` — the rest of the
pipeline still runs, it just won't move the base.

### Passing launch options

The container's default command is
`ros2 launch /opt/qbot/launch/qbot_docker.launch.py`. To override arguments
(see [KOBUKI_QBOT_GESTURE_CONTROL.md](KOBUKI_QBOT_GESTURE_CONTROL.md) for the
full list), run a one-off instead of `up`:

```bash
# Example: disable depth, mirror left/right, slower gesture fps
docker compose -f docker/docker-compose.yml run --rm qbot \
  ros2 launch /opt/qbot/launch/qbot_docker.launch.py \
  enable_depth:=false mirror_horizontal_commands:=true gesture_max_fps:=8.0
```

Common arguments:

| Argument | Default | Effect |
| --- | --- | --- |
| `enable_depth:=false` | `true` | Skip depth stream (saves USB bandwidth; disables depth come-closer + obstacle stop) |
| `mirror_horizontal_commands:=true` | `false` | Flip mirrored left/right gestures |
| `cmd_vel_topic:=/commands/velocity` | `/cmd_vel` | Use this if you swap to the upstream `kobuki_node` driver |
| `gesture_max_fps:=8.0` `face_max_fps:=5.0` | `12.0`/`8.0` | Lower CPU load |
| `publish_debug_image:=true` | `false` | Publish the annotated overlay on `/gesture/debug_image` |

---

## 6. Inspecting and debugging the running system

Open a shell inside the running container (it already sources ROS + the
workspace via the entrypoint):

```bash
docker exec -it qbot bash

# inside the container:
ros2 topic list
ros2 topic hz /kinect/rgb/image_raw
ros2 topic echo /gesture/command
ros2 topic echo /cmd_vel
```

Because the container uses **host networking**, you can also run ROS 2 CLI
tools from a *second* container or from a host-installed ROS 2 — they share
the same DDS graph (keep `ROS_DOMAIN_ID` consistent, default `0`).

### Seeing the camera image (optional GUI)

On the Pi's local desktop session:

```bash
xhost +local:root
```

Uncomment the `DISPLAY` and `/tmp/.X11-unix` lines in
[docker/docker-compose.yml](docker/docker-compose.yml), bring the stack up,
then in a shell into the container:

```bash
apt-get update && apt-get install -y ros-jazzy-rqt-image-view   # not baked in by default
ros2 run rqt_image_view rqt_image_view
```

(For a headless Pi, prefer `ros2 topic hz` / `echo` over GUI tools.)

---

## 7. Verifying it actually works (success checks)

1. **Camera up:** `ros2 topic hz /kinect/rgb/image_raw` shows ~30 Hz.
2. **Depth up:** `ros2 topic hz /kinect/depth/image_raw` shows frames
   (skip if you launched with `enable_depth:=false`).
3. **Gestures recognised:** wave / beckon and watch
   `ros2 topic echo /gesture/command`.
4. **Behaviour reacts:** `ros2 topic echo /cmd_vel` shows non-zero Twist
   while a command is active.
5. **Base moves:** logs show `Kobuki connected successfully.` and the robot
   responds.

Offline decoder unit tests (no camera/robot needed) can be run in the
container too:

```bash
docker exec -it qbot bash -lc \
  'PYTHONPATH=/opt/qbot/qbot_ws/src/gesture_node:/usr/local/lib/python3.12/dist-packages \
   python3 -m pytest /opt/qbot/qbot_ws/src/gesture_node/test/test_decoder.py -q'
```

---

## 8. Rebuilding after code changes

The workspace is **copied into the image at build time**, so after editing
anything in `qbot_ws/src`:

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up
```

Only the `colcon build` layer (and later) re-runs, so this is much faster than
the first build.

> Faster dev loop (optional): bind-mount the source and rebuild inside the
> container instead of rebuilding the image — add
> `- ../qbot_ws:/opt/qbot/qbot_ws` under `volumes:` in the compose file, then
> `docker exec -it qbot bash -lc 'cd /opt/qbot/qbot_ws && colcon build'`.
> Note this shadows the baked-in `install/`, so you must build at least once
> inside the container.

---

## 9. Troubleshooting

### MediaPipe on arm64

`pip install mediapipe==0.10.32` needs a `linux/aarch64` wheel. If the build
fails with "Could not find a version that satisfies the requirement
mediapipe":

1. **Try the piwheels index** (community arm wheels). In the Dockerfile, change
   the pip step to:
   ```dockerfile
   RUN pip3 install --break-system-packages --no-cache-dir \
         --extra-index-url https://www.piwheels.org/simple \
         numpy==2.4.3 opencv-python==4.13.0.92 mediapipe==0.10.32 kobukidriver
   ```
2. **Relax the version pin** — drop `==0.10.32` to let pip pick the newest
   arm64 build, then re-test gesture recognition.
3. As a last resort, install a prebuilt Pi MediaPipe wheel into the image
   (`COPY` a `.whl` and `pip install` it).

`numpy`/`opencv` clashes: ROS pulls in the apt `python3-numpy`/`python3-opencv`;
the pip versions above take precedence on `sys.path`. If you see a numpy ABI
warning, align the versions (the pins here match the laptop `.venv`).

### `libfreenect was not found` during colcon build

The Dockerfile installs libfreenect to `/usr/local`, which the package's
`CMakeLists.txt` searches. If you changed the install prefix, pass
`LIBFREENECT_ROOT` via `--cmake-args -DLIBFREENECT_ROOT=...`.

### `Could not open device: LIBUSB_ERROR_NO_DEVICE` / `Invalid index [0]`

The container can't see the Kinect. Check:
- `lsusb` on the **host** shows the Kinect (camera `045e:02ae`).
- The Kinect's **power brick** is plugged in (USB alone is not enough).
- The compose file has `privileged: true` and `- /dev:/dev`.
- Re-seat the USB cable; the Kinect re-enumerates during firmware upload.

A harmless `Failed to set the LED of K4W or 1473 device: LIBUSB_ERROR_IO` can
still appear on Kinect-for-Windows units — the streams publish anyway.

### `Kobuki not connected`

- `ls /dev/ttyUSB*` on the host shows the port.
- `privileged: true` + `- /dev:/dev` are set so the container sees it.
- The Kobuki is powered on. The driver scans serial ports automatically; no
  fixed path is hard-coded.

### Nodes can't import MediaPipe at runtime

The entrypoint sets `QBOT_VENV_SITE_PACKAGES=/usr/local/lib/python3.12/dist-packages`
(where pip installed it). If you used a different Python minor version, update
that path in [docker/entrypoint.sh](docker/entrypoint.sh).

### Two robots / shared lab network

Set a unique `ROS_DOMAIN_ID` in the compose `environment:` block so multiple
ROS 2 graphs on the same LAN don't cross-talk.

---

## 10. Quick reference

```bash
# one-time, on the Pi
docker compose -f docker/docker-compose.yml build

# run the whole robot
docker compose -f docker/docker-compose.yml up

# run detached + watch logs
docker compose -f docker/docker-compose.yml up -d
docker compose -f docker/docker-compose.yml logs -f

# shell into the running robot
docker exec -it qbot bash

# stop
docker compose -f docker/docker-compose.yml down
```
