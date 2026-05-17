"""Stream-based gesture command node for the QBot on a Raspberry Pi 5.

Subscribes to the Kinect RGB stream and emits:

* ``/gesture/tracking`` - continuous JSON state of the hand (used by the
  behaviour node to re-centre on the user after each action);
* ``/gesture/command`` - confirmed temporal commands such as
  ``COME_CLOSER``, ``STOP``, ``ROTATE_ONCE``, ``MOVE_LEFT_FOOT``,
  ``MOVE_RIGHT_FOOT``, ``TAIL_WAG``.

All temporal decisions are made over a sliding window of frames so a
single misclassified frame cannot trigger or block a command. The
beckon/come-closer detector watches a continuous ``openness`` signal
instead of MediaPipe's discrete OPEN_PALM/CLOSED_FIST labels, which is
considerably more reliable when the hand is partially curled.
"""

from __future__ import annotations

import collections
import json
import math
import os
import time
import warnings
from pathlib import Path

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
os.environ.setdefault("GLOG_minloglevel", "2")
os.environ.setdefault("ABSL_MIN_LOG_LEVEL", "2")
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from gesture_node.frame_processor import (
    apply_clahe,
    assess_quality,
    decode_ros_image,
    downscale,
)
from gesture_node.signals import (
    BeckonOscillationDetector,
    LandmarkSmoother,
    MajorityLabelFilter,
    count_axis_reversals,
    hand_openness,
    label_from_finger_pattern,
)


GESTURE_MAP = {
    "Closed_Fist": "FIST",
    "Open_Palm": "OPEN_PALM",
    "Pointing_Up": "POINTING",
    "Thumb_Up": "THUMBS_UP",
    "Thumb_Down": "THUMBS_DOWN",
    "Victory": "PEACE",
    "ILoveYou": "I_LOVE_YOU",
    "None": "NONE",
}

TIP_IDS = [4, 8, 12, 16, 20]
PIP_IDS = [3, 6, 10, 14, 18]


def _now() -> float:
    return time.monotonic()


def _angle_deg(a, b, c) -> float:
    ab = np.array([a.x - b.x, a.y - b.y, a.z - b.z], dtype=np.float32)
    cb = np.array([c.x - b.x, c.y - b.y, c.z - b.z], dtype=np.float32)
    denom = float(np.linalg.norm(ab) * np.linalg.norm(cb))
    if denom < 1e-6:
        return 0.0
    cosine = float(np.dot(ab, cb) / denom)
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def fingers_up(landmarks):
    fingers = []
    thumb_mcp, thumb_ip, thumb_tip, wrist = (
        landmarks[2],
        landmarks[3],
        landmarks[4],
        landmarks[0],
    )
    thumb_angle = _angle_deg(thumb_mcp, thumb_ip, thumb_tip)
    tip_dist = math.dist((thumb_tip.x, thumb_tip.y, thumb_tip.z),
                         (wrist.x, wrist.y, wrist.z))
    ip_dist = math.dist((thumb_ip.x, thumb_ip.y, thumb_ip.z),
                        (wrist.x, wrist.y, wrist.z))
    fingers.append(1 if thumb_angle > 150.0 and tip_dist > ip_dist + 0.035 else 0)

    for finger in range(1, 5):
        tip = landmarks[TIP_IDS[finger]]
        pip = landmarks[PIP_IDS[finger]]
        fingers.append(1 if tip.y < pip.y else 0)
    return fingers


def hand_center_and_span(landmarks):
    xs = [lm.x for lm in landmarks]
    ys = [lm.y for lm in landmarks]
    center_x = sum(xs) / len(xs)
    center_y = sum(ys) / len(ys)
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    return center_x, center_y, span


def pointing_direction(landmarks, mirror_horizontal: bool = False):
    mcp = landmarks[5]
    tip = landmarks[8]
    dx = tip.x - mcp.x
    dy = tip.y - mcp.y
    dz = mcp.z - tip.z
    xy_len = math.hypot(dx, dy)
    if dz > 0.12 and xy_len < 0.10:
        return "TOWARDS_CAMERA"

    if abs(dx) > max(0.06, abs(dy) * 1.25):
        direction = "RIGHT" if dx > 0 else "LEFT"
        if mirror_horizontal:
            direction = "LEFT" if direction == "RIGHT" else "RIGHT"
        return direction

    angle = math.degrees(math.atan2(-dy, dx))
    if angle < 0.0:
        angle += 360.0
    sectors = ["RIGHT", "UP_RIGHT", "UP", "UP_LEFT", "LEFT", "DOWN_LEFT", "DOWN", "DOWN_RIGHT"]
    direction = sectors[int((angle + 22.5) // 45.0) % len(sectors)]
    if mirror_horizontal:
        direction = (
            direction.replace("LEFT", "TMP").replace("RIGHT", "LEFT").replace("TMP", "RIGHT")
        )
    return direction


def _find_default_model_path() -> str:
    env = os.environ.get("QBOT_GESTURE_MODEL")
    candidates = [Path(env)] if env else []
    cwd = Path.cwd()
    candidates.append(cwd / "models" / "gesture_recognizer.task")
    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / "models" / "gesture_recognizer.task")
        candidates.append(parent.parent / "models" / "gesture_recognizer.task")
    for candidate in candidates:
        if candidate and candidate.is_file():
            return str(candidate)
    return ""


class TemporalCommandDecoder:
    """Decodes confirmed temporal commands from per-frame observations."""

    def __init__(
        self,
        beckon_oscillations_required: int = 2,
        beckon_window_sec: float = 4.5,
        palm_wave_window_sec: float = 2.4,
        palm_wave_reversals: int = 3,
        palm_wave_sweep: float = 0.18,
        palm_wave_amp: float = 0.08,
        circle_window_sec: float = 3.2,
        point_hold_sec: float = 0.55,
        stop_hold_sec: float = 0.45,
        stop_motion_tolerance: float = 0.04,
        command_cooldown_sec: float = 1.8,
    ) -> None:
        self._beckon = BeckonOscillationDetector(
            oscillations_required=beckon_oscillations_required,
            window_sec=beckon_window_sec,
        )
        self._palm_wave_window = float(palm_wave_window_sec)
        self._palm_wave_reversals = int(palm_wave_reversals)
        self._palm_wave_sweep = float(palm_wave_sweep)
        self._palm_wave_amp = float(palm_wave_amp)
        self._circle_window = float(circle_window_sec)
        self._point_hold = float(point_hold_sec)
        self._stop_hold = float(stop_hold_sec)
        self._stop_motion_tolerance = float(stop_motion_tolerance)
        self._cooldown = float(command_cooldown_sec)

        self._palm_track: collections.deque = collections.deque()
        self._point_track: collections.deque = collections.deque()
        self._open_palm_since: float | None = None
        self._point_direction: str | None = None
        self._point_direction_since: float | None = None
        self._last_emit: dict = collections.defaultdict(lambda: -math.inf)

    def reset(self) -> None:
        self._beckon.reset()
        self._palm_track.clear()
        self._point_track.clear()
        self._open_palm_since = None
        self._point_direction = None
        self._point_direction_since = None

    def update(self, obs):
        if not obs["hand_visible"]:
            self.reset()
            return None

        now = obs["time"]
        self._update_buffers(obs, now)
        self._beckon.update(now, obs["openness"])

        return (
            self._detect_palm_wave(obs, now)
            or self._detect_beckon(obs, now)
            or self._detect_index_circle(obs, now)
            or self._detect_point_left_right(obs, now)
            or self._detect_stop(obs, now)
        )

    def _update_buffers(self, obs, now: float) -> None:
        openness = obs["openness"]
        if openness >= 0.7:
            if self._open_palm_since is None:
                self._open_palm_since = now
            self._palm_track.append((now, obs["center_x"], obs["center_y"]))
        else:
            self._open_palm_since = None
            if openness < 0.55:
                self._palm_track.clear()

        if obs["gesture"] == "POINTING" and obs.get("index_tip"):
            tip_x, tip_y = obs["index_tip"]
            self._point_track.append((now, tip_x, tip_y))
        elif obs["gesture"] != "POINTING":
            self._point_track.clear()
            self._point_direction = None
            self._point_direction_since = None

        while self._palm_track and now - self._palm_track[0][0] > self._palm_wave_window:
            self._palm_track.popleft()
        while self._point_track and now - self._point_track[0][0] > self._circle_window:
            self._point_track.popleft()

    def _can_emit(self, name: str, now: float, cooldown: float | None = None) -> bool:
        if cooldown is None:
            cooldown = self._cooldown
        return now - self._last_emit[name] >= cooldown

    def _emit(self, name: str, obs, reason: str, cooldown=None, extra=None):
        now = obs["time"]
        if not self._can_emit(name, now, cooldown):
            return None
        self._last_emit[name] = now

        command = {
            "id": f"{name}-{int(now * 1000)}",
            "command": name,
            "reason": reason,
            "confidence": round(float(obs.get("confidence", 0.0)), 3),
            "gesture": obs["gesture"],
            "center_x": round(float(obs["center_x"]), 4),
            "center_y": round(float(obs["center_y"]), 4),
            "openness": round(float(obs["openness"]), 3),
            "stamp_monotonic": round(now, 3),
        }
        if obs.get("pointing_direction"):
            command["pointing_direction"] = obs["pointing_direction"]
        if extra:
            command.update(extra)
        return command

    def _detect_beckon(self, obs, now):
        if not self._beckon.ready():
            return None
        curls = self._beckon.consume()
        self._palm_track.clear()
        return self._emit(
            "COME_CLOSER",
            obs,
            "beckon curls detected",
            cooldown=2.4,
            extra={"curls": curls},
        )

    def _detect_stop(self, obs, now):
        if obs["openness"] < 0.7 or self._open_palm_since is None:
            return None
        if now - self._open_palm_since < self._stop_hold:
            return None

        recent = [pt for pt in self._palm_track if now - pt[0] <= 0.5]
        if len(recent) >= 4:
            xs = [pt[1] for pt in recent]
            if max(xs) - min(xs) > self._stop_motion_tolerance * 3:
                return None

        return self._emit("STOP", obs, "open palm held still", cooldown=0.9)

    def _detect_palm_wave(self, obs, now):
        if obs["openness"] < 0.7:
            return None
        if len(self._palm_track) < 8:
            return None

        xs = [pt[1] for pt in self._palm_track]
        amp = max(xs) - min(xs)
        if amp < self._palm_wave_amp * 2:
            return None

        reversals = count_axis_reversals(xs, min_sweep=self._palm_wave_sweep / 2.4)
        if reversals < self._palm_wave_reversals:
            return None

        self._palm_track.clear()
        return self._emit(
            "TAIL_WAG",
            obs,
            "open palm waved left and right",
            cooldown=2.8,
            extra={"wave_reversals": reversals, "wave_amplitude": round(amp, 3)},
        )

    def _detect_index_circle(self, obs, now):
        if obs["gesture"] != "POINTING" or len(self._point_track) < 12:
            return None

        points = list(self._point_track)
        duration = points[-1][0] - points[0][0]
        if duration < 0.55:
            return None

        xs = np.array([p[1] for p in points], dtype=np.float32)
        ys = np.array([p[2] for p in points], dtype=np.float32)
        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
        radii = np.hypot(xs - cx, ys - cy)
        mean_radius = float(np.mean(radii))
        if mean_radius < 0.04 or float(np.std(radii)) > mean_radius * 0.95:
            return None

        angles = np.unwrap(np.arctan2(ys - cy, xs - cx))
        net = float(angles[-1] - angles[0])
        total = float(np.sum(np.abs(np.diff(angles))))
        raw_quadrants = {
            int((math.degrees(a) % 360) // 90)
            for a in np.arctan2(ys - cy, xs - cx)
        }
        if abs(net) < 1.55 * math.pi or total < 1.75 * math.pi or len(raw_quadrants) < 4:
            return None

        self._point_track.clear()
        self._point_direction = None
        self._point_direction_since = None
        return self._emit(
            "ROTATE_ONCE",
            obs,
            "index fingertip traced a circle",
            cooldown=3.0,
            extra={
                "circle_degrees": round(abs(math.degrees(net)), 1),
                "circle_direction": "clockwise" if net > 0 else "counterclockwise",
            },
        )

    def _detect_point_left_right(self, obs, now):
        if obs["gesture"] != "POINTING":
            return None
        direction = obs.get("pointing_direction")
        if direction not in ("LEFT", "RIGHT"):
            self._point_direction = None
            self._point_direction_since = None
            return None

        if direction != self._point_direction:
            self._point_direction = direction
            self._point_direction_since = now
            return None

        if self._point_direction_since is None or now - self._point_direction_since < self._point_hold:
            return None

        self._point_track.clear()
        command_name = "MOVE_LEFT_FOOT" if direction == "LEFT" else "MOVE_RIGHT_FOOT"
        self._point_direction_since = math.inf
        return self._emit(
            command_name,
            obs,
            f"index finger held {direction.lower()}",
            cooldown=2.2,
            extra={"direction": direction},
        )


class GestureCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("gesture_command_node")

        self.declare_parameter("image_topic", "/kinect/rgb/image_raw")
        self.declare_parameter("command_topic", "/gesture/command")
        self.declare_parameter("tracking_topic", "/gesture/tracking")
        self.declare_parameter("debug_image_topic", "/gesture/debug_image")
        self.declare_parameter("model_path", _find_default_model_path())
        self.declare_parameter("max_fps", 12.0)
        self.declare_parameter("min_confidence", 0.45)
        self.declare_parameter("history_size", 7)
        self.declare_parameter("history_votes", 4)
        self.declare_parameter("mirror_horizontal_commands", False)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("downscale_width", 480)
        self.declare_parameter("enable_clahe", True)
        self.declare_parameter("min_frame_luma", 22.0)
        self.declare_parameter("max_frame_luma", 245.0)
        self.declare_parameter("beckon_oscillations_required", 2)
        self.declare_parameter("beckon_window_sec", 4.5)
        self.declare_parameter("palm_wave_reversals", 3)
        self.declare_parameter("stop_hold_sec", 0.45)
        self.declare_parameter("command_cooldown_sec", 1.8)

        image_topic = self.get_parameter("image_topic").value
        command_topic = self.get_parameter("command_topic").value
        tracking_topic = self.get_parameter("tracking_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        model_path = self.get_parameter("model_path").value

        if not model_path or not Path(model_path).is_file():
            raise FileNotFoundError(
                "MediaPipe gesture model not found. Set gesture_node model_path "
                "or QBOT_GESTURE_MODEL to models/gesture_recognizer.task."
            )

        self._max_fps = max(1.0, float(self.get_parameter("max_fps").value))
        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._mirror_horizontal = bool(self.get_parameter("mirror_horizontal_commands").value)
        self._publish_debug = bool(self.get_parameter("publish_debug_image").value)
        self._downscale_width = int(self.get_parameter("downscale_width").value)
        self._enable_clahe = bool(self.get_parameter("enable_clahe").value)
        self._min_luma = float(self.get_parameter("min_frame_luma").value)
        self._max_luma = float(self.get_parameter("max_frame_luma").value)

        self._label_filter = MajorityLabelFilter(
            int(self.get_parameter("history_size").value),
            int(self.get_parameter("history_votes").value),
        )
        self._smoother = LandmarkSmoother(alpha=0.55, jump_threshold=0.22)
        self._decoder = TemporalCommandDecoder(
            beckon_oscillations_required=int(self.get_parameter("beckon_oscillations_required").value),
            beckon_window_sec=float(self.get_parameter("beckon_window_sec").value),
            palm_wave_reversals=int(self.get_parameter("palm_wave_reversals").value),
            stop_hold_sec=float(self.get_parameter("stop_hold_sec").value),
            command_cooldown_sec=float(self.get_parameter("command_cooldown_sec").value),
        )

        self._last_process_time = 0.0
        self._last_timestamp_ms = 0
        self._frame_drop_counter = 0

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._image_sub = self.create_subscription(Image, image_topic, self._image_callback, qos)
        self._command_pub = self.create_publisher(String, command_topic, 10)
        self._tracking_pub = self.create_publisher(String, tracking_topic, 10)
        self._debug_pub = (
            self.create_publisher(Image, debug_image_topic, 5) if self._publish_debug else None
        )

        options = mp.tasks.vision.GestureRecognizerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model_path),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.55,
            min_hand_presence_confidence=0.55,
            min_tracking_confidence=0.55,
        )
        self._recognizer = mp.tasks.vision.GestureRecognizer.create_from_options(options)

        self.get_logger().info(
            f"Gesture node on {image_topic}; downscale={self._downscale_width}px clahe={self._enable_clahe}"
        )

    def destroy_node(self):
        if getattr(self, "_recognizer", None) is not None:
            self._recognizer.close()
        super().destroy_node()

    def _image_callback(self, msg):
        now = _now()
        if now - self._last_process_time < 1.0 / self._max_fps:
            return
        self._last_process_time = now

        try:
            rgb = decode_ros_image(msg, target="rgb")
        except Exception as exc:
            self.get_logger().warning(f"Could not decode image: {exc}")
            return

        quality = assess_quality(rgb, self._min_luma, self._max_luma)
        if not quality.accepted:
            self._frame_drop_counter += 1
            if self._frame_drop_counter % 30 == 1:
                self.get_logger().warning(
                    f"Dropping frames - {quality.reason} (mean_luma={quality.mean_luma:.1f})"
                )
            self._publish_tracking(self._blank_observation(now, msg.header.frame_id))
            return
        self._frame_drop_counter = 0

        processed = rgb
        if self._enable_clahe:
            processed = apply_clahe(processed)
        processed = downscale(processed, self._downscale_width)
        processed = np.ascontiguousarray(processed)

        timestamp_ms = self._message_timestamp_ms(msg)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=processed)
        try:
            result = self._recognizer.recognize_for_video(mp_image, timestamp_ms)
        except Exception as exc:
            self.get_logger().warning(f"MediaPipe recognition failed: {exc}")
            return

        observation = self._result_to_observation(result, now, msg.header.frame_id)
        self._publish_tracking(observation)

        command = self._decoder.update(observation)
        if command:
            ros_msg = String()
            ros_msg.data = json.dumps(command, separators=(",", ":"))
            self._command_pub.publish(ros_msg)
            self.get_logger().info(
                f"Gesture command: {command['command']} ({command['reason']})"
            )

        if self._publish_debug:
            self._publish_debug_frame(processed, observation)

    def _message_timestamp_ms(self, msg) -> int:
        stamp = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000
        if stamp <= 0:
            stamp = int(time.time() * 1000)
        if stamp <= self._last_timestamp_ms:
            stamp = self._last_timestamp_ms + 1
        self._last_timestamp_ms = stamp
        return stamp

    @staticmethod
    def _blank_observation(now: float, frame_id: str) -> dict:
        return {
            "time": now,
            "frame_id": frame_id,
            "hand_visible": False,
            "gesture": "NO_HAND",
            "raw_gesture": "NO_HAND",
            "confidence": 0.0,
            "center_x": 0.5,
            "center_y": 0.5,
            "span": 0.0,
            "openness": 0.0,
            "pointing_direction": None,
            "index_tip": None,
            "fingers": [0, 0, 0, 0, 0],
        }

    def _result_to_observation(self, result, now: float, frame_id: str) -> dict:
        observation = self._blank_observation(now, frame_id)
        if not result.hand_landmarks:
            self._label_filter.clear()
            self._smoother.reset()
            return observation

        raw_landmarks = result.hand_landmarks[0]
        landmarks = self._smoother.update(raw_landmarks)

        center_x, center_y, span = hand_center_and_span(landmarks)
        if span < 0.055:
            self._label_filter.clear()
            self._smoother.reset()
            return observation

        model_label = "NONE"
        confidence = 0.0
        if result.gestures and result.gestures[0]:
            best = result.gestures[0][0]
            model_label = GESTURE_MAP.get(best.category_name, "CUSTOM")
            confidence = float(best.score)

        fingers = fingers_up(landmarks)
        pattern_label = label_from_finger_pattern(tuple(fingers))
        openness = hand_openness(landmarks)

        if confidence < self._min_confidence or model_label in ("NONE", "CUSTOM"):
            raw_label = pattern_label
        elif pattern_label == "POINTING":
            raw_label = "POINTING"
        else:
            raw_label = model_label

        if raw_label == "CUSTOM" and fingers == [0, 1, 0, 0, 0]:
            raw_label = "POINTING"

        stable_label = self._label_filter.update(raw_label)

        direction = None
        index_tip = None
        if stable_label == "POINTING":
            direction = pointing_direction(landmarks, self._mirror_horizontal)
            index_tip = (float(landmarks[8].x), float(landmarks[8].y))

        observation.update({
            "hand_visible": True,
            "gesture": stable_label,
            "raw_gesture": raw_label,
            "confidence": confidence,
            "center_x": float(center_x),
            "center_y": float(center_y),
            "span": float(span),
            "openness": float(openness),
            "pointing_direction": direction,
            "index_tip": index_tip,
            "fingers": fingers,
        })
        return observation

    def _publish_tracking(self, observation: dict) -> None:
        payload = {
            "hand_visible": observation["hand_visible"],
            "gesture": observation["gesture"],
            "raw_gesture": observation["raw_gesture"],
            "confidence": round(float(observation["confidence"]), 3),
            "center_x": round(float(observation["center_x"]), 4),
            "center_y": round(float(observation["center_y"]), 4),
            "offset_x": round(float(observation["center_x"] - 0.5), 4),
            "span": round(float(observation["span"]), 4),
            "openness": round(float(observation["openness"]), 3),
            "pointing_direction": observation.get("pointing_direction"),
            "frame_id": observation.get("frame_id", ""),
            "stamp_monotonic": round(float(observation["time"]), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._tracking_pub.publish(msg)

    def _publish_debug_frame(self, rgb_frame: np.ndarray, observation: dict) -> None:
        bgr = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        label = observation["gesture"]
        direction = observation.get("pointing_direction") or "-"
        openness = observation.get("openness", 0.0)
        color = (0, 255, 0) if observation["hand_visible"] else (180, 180, 180)
        cv2.putText(
            bgr,
            f"{label} {direction} open={openness:.2f}",
            (12, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
        try:
            out = Image()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = observation.get("frame_id", "")
            out.height = bgr.shape[0]
            out.width = bgr.shape[1]
            out.encoding = "bgr8"
            out.is_bigendian = False
            out.step = bgr.shape[1] * 3
            out.data = np.ascontiguousarray(bgr).tobytes()
            self._debug_pub.publish(out)
        except Exception as exc:
            self.get_logger().warning(f"Could not publish debug image: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = GestureCommandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
