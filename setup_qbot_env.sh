#!/usr/bin/env bash
# =============================================================================
# QBot environment setup for a FRESH Raspberry Pi 5, Ubuntu 24.04 LTS (arm64).
#
# Installs ROS 2 Jazzy + project ROS packages, builds the Python virtual
# environment NATIVELY on the Pi, and colcon-builds the workspace. Designed to
# be idempotent and safe to re-run.
#
# Run as your normal user (NOT root) from the repo root:
#     ./setup_qbot_env.sh
#
# (Optional, first: ./setup_rpi.sh qbot  -> sets hostname + mDNS so the Pi is
#  reachable as qbot.local. That script is separate and unrelated to this one.)
#
# To also install the optional owner-recognition stack (dlib + face_recognition,
# slow source compile on the Pi), run:
#     WITH_FACE_RECOGNITION=1 ./setup_qbot_env.sh
# =============================================================================
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WITH_FACE_RECOGNITION="${WITH_FACE_RECOGNITION:-0}"

if [ "$(id -u)" -eq 0 ]; then
  echo "ERROR: run as your normal user (it will call sudo as needed). Running as"
  echo "root would create a root-owned .venv that the nodes cannot use." >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
. /etc/os-release
echo "==> Repo:        $REPO"
echo "==> Arch:        $ARCH"
echo "==> Ubuntu:      ${VERSION_ID:-?} (${UBUNTU_CODENAME:-?})"
[ "$ARCH" = "arm64" ] || echo "WARNING: expected arm64; continuing on $ARCH anyway."

# ---------------------------------------------------------------------------
# 1. Locale. Provide a UTF-8 locale via LANG only. We deliberately DO NOT set
#    LC_ALL system-wide: forcing LC_ALL to a locale that is ever missing or
#    corrupt makes every locale-aware binary (apt, dpkg, grep) segfault.
# ---------------------------------------------------------------------------
echo "==> [1/5] Locale (LANG=en_US.UTF-8, no system-wide LC_ALL)"
sudo apt-get update -qq
sudo apt-get install -y locales
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
sudo sed -i '/^LC_ALL=/d' /etc/default/locale 2>/dev/null || true

# ---------------------------------------------------------------------------
# 2. ROS 2 apt repository (key + source list) for this Ubuntu codename/arch.
# ---------------------------------------------------------------------------
echo "==> [2/5] ROS 2 apt repository"
sudo apt-get install -y software-properties-common curl gnupg ca-certificates
sudo add-apt-repository -y universe
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update

# ---------------------------------------------------------------------------
# 3. ROS 2 Jazzy + project packages + system libs.
#    --force-overwrite resolves the known python3-catkin-pkg vs
#    python3-catkin-pkg-modules file conflict on Ubuntu 24.04.
# ---------------------------------------------------------------------------
echo "==> [3/5] ROS 2 Jazzy base, dev tools, project + system packages"
FORCE='-o Dpkg::Options::=--force-overwrite'
sudo apt-get $FORCE install -y ros-jazzy-ros-base ros-dev-tools
sudo apt-get $FORCE install -y \
  ros-jazzy-nav-msgs \
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-vision-opencv ros-jazzy-image-tools \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
  ros-jazzy-twist-mux ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-kobuki-ros-interfaces ros-jazzy-kobuki-velocity-smoother \
  ros-jazzy-usb-cam
# Camera + Kinect (libfreenect) + Python/venv + build toolchain.
sudo apt-get install -y \
  v4l-utils ffmpeg freenect libfreenect-dev \
  python3-venv python3-dev python3-pip cmake build-essential

# ---------------------------------------------------------------------------
# 4. Native Python virtual environment at $REPO/.venv (arm64 wheels).
#    rclpy comes from ROS; the venv supplies cv2/mediapipe/numpy, which the
#    nodes pick up because run_qbot.sh prepends .venv to PYTHONPATH.
# ---------------------------------------------------------------------------
echo "==> [4/5] Python venv (native arm64)"
if [ ! -x "$REPO/.venv/bin/python" ]; then
  python3 -m venv "$REPO/.venv"
fi
PIP="$REPO/.venv/bin/pip"
"$PIP" install --upgrade pip wheel

# Core pipeline deps from requirements-venv.txt, excluding the heavy
# recognition stack and mediapipe (handled separately so a missing aarch64
# mediapipe wheel cannot abort the whole install).
grep -ivE 'dlib|face[-_]recognition|imutils|mediapipe' "$REPO/requirements-venv.txt" \
  | grep -vE '^\s*(#|$)' > /tmp/qbot-req-core.txt
"$PIP" install -r /tmp/qbot-req-core.txt
"$PIP" install pyserial

# mediapipe: best effort. If no arm64 wheel exists, the face tracker falls back
# to the OpenCV Haar cascade automatically.
MP_VER="$(grep -iE '^mediapipe' "$REPO/requirements-venv.txt" | head -1 || true)"
if "$PIP" install "${MP_VER:-mediapipe}" ; then
  echo "    mediapipe installed."
else
  echo "    WARNING: mediapipe unavailable for arm64; face tracker will use the"
  echo "    OpenCV Haar fallback (set force_haar handled in code)."
fi

if [ "$WITH_FACE_RECOGNITION" = "1" ]; then
  echo "==> [4b] Optional owner-recognition stack (dlib source compile, slow)"
  sudo apt-get install -y libboost-all-dev
  "$PIP" install dlib face-recognition face-recognition-models imutils
fi

# ---------------------------------------------------------------------------
# 5. Build the ROS workspace.
# ---------------------------------------------------------------------------
echo "==> [5/5] colcon build qbot_ws"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd "$REPO/qbot_ws"
colcon build --symlink-install

echo
echo "============================================================"
echo " QBot environment ready."
echo "   ROS:   /opt/ros/jazzy   (\$ROS_DISTRO once sourced)"
echo "   venv:  $REPO/.venv      (native $ARCH)"
echo
echo " Launch the full pipeline with:"
echo "   ./run_qbot.sh"
echo "============================================================"
