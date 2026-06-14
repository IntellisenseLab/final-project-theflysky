# QBot gesture-control pipeline (ROS 2 Jazzy) in a single container.
# Build this NATIVELY on the Raspberry Pi 5 (arm64) so mediapipe and
# libfreenect are compiled/installed for the real target.
#
#   docker build -f docker/Dockerfile -t qbot:latest .
#
# See DOCKER_SETUP.md for the full step-by-step run guide.

FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# 1. System + ROS dependencies (mirrors requirements.txt, runtime subset only)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      git \
      libusb-1.0-0-dev \
      python3-pip \
      python3-colcon-common-extensions \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport \
      ros-jazzy-vision-opencv \
      ros-jazzy-tf2-ros \
      ros-jazzy-kobuki-ros-interfaces \
      ros-jazzy-kobuki-velocity-smoother \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. Build libfreenect from source and install to /usr/local
#    (reproduces the validated path in KINECT_360_SETUP.md)
# ---------------------------------------------------------------------------
RUN git clone --depth 1 https://github.com/OpenKinect/libfreenect /tmp/libfreenect \
 && cmake -S /tmp/libfreenect -B /tmp/libfreenect/build \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_FAKENECT=OFF \
      -DBUILD_PYTHON=OFF \
      -DBUILD_PYTHON2=OFF \
      -DBUILD_PYTHON3=OFF \
      -DBUILD_CPP=OFF \
      -DBUILD_CV=OFF \
      -DBUILD_OPENNI2_DRIVER=OFF \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
 && cmake --build /tmp/libfreenect/build -j"$(nproc)" \
 && cmake --install /tmp/libfreenect/build \
 && ldconfig

# Kinect-for-Windows audio firmware blob (needed by some K4W v1 units).
# Tolerate a download failure so the image still builds for plain Kinect 360s.
RUN mkdir -p /opt/qbot/firmware \
 && (python3 /tmp/libfreenect/src/fwfetcher.py /opt/qbot/firmware/audios.bin \
     || echo "WARN: fwfetcher failed; K4W audio firmware not baked in")

# ---------------------------------------------------------------------------
# 3. Python runtime deps for the ROS nodes.
#    Only mediapipe + opencv + numpy are imported by the pipeline; dlib /
#    face-recognition (requirements-venv.txt) are NOT used by qbot_ws/src.
#    NOTE: mediapipe needs a linux/arm64 wheel. If the plain PyPI install
#    fails on the Pi, see the troubleshooting section in DOCKER_SETUP.md.
# ---------------------------------------------------------------------------
RUN pip3 install --break-system-packages --no-cache-dir \
      numpy==2.4.3 \
      opencv-python==4.13.0.92 \
      mediapipe==0.10.32 \
      kobukidriver

# ---------------------------------------------------------------------------
# 4. Copy and build the ROS 2 workspace
# ---------------------------------------------------------------------------
WORKDIR /opt/qbot
COPY qbot_ws/src /opt/qbot/qbot_ws/src
COPY models /opt/qbot/models
COPY docker/launch /opt/qbot/launch

RUN . /opt/ros/jazzy/setup.sh \
 && cd /opt/qbot/qbot_ws \
 && colcon build --symlink-install

# ---------------------------------------------------------------------------
# 5. Entrypoint
# ---------------------------------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/opt/qbot/launch/qbot_docker.launch.py"]
