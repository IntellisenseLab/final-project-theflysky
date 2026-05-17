"""Behaviour node that converts gesture commands into Kobuki motion.

Beyond the original ``twist`` / ``oscillate`` / ``align`` building blocks
this version adds three things that make the robot noticeably more
robust on a real Kobuki QBot driven by a Raspberry Pi 5:

* **Depth-aware come-closer** - if the Kinect depth stream is live the
  forward drive stops as soon as the person in front of the robot is at
  ``come_closer_target_m``. The original timed drive is kept as a hard
  fallback in case depth is unavailable or noisy.
* **Obstacle safety stop** - any forward ``twist`` segment is aborted
  early if the median depth in the centre of the frame falls below
  ``obstacle_stop_m``.
* **Smarter alignment** - when the target is lost the search direction is
  biased by the last known offset (rotate toward where the person was
  last seen) instead of always spinning the same way.

There is also an opt-in idle wag so the robot looks alive instead of
sitting perfectly still between gestures.
"""

from __future__ import annotations

import collections
import json
import math
import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from kobuki_ros_interfaces.msg import Sound  # type: ignore
except ImportError:
    Sound = None

from behavior_node.sound_profiles import profile_for


def _now() -> float:
    return time.monotonic()


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _depth_meters_center(depth_array: np.ndarray, roi_frac: float = 0.18) -> Optional[float]:
    if depth_array is None or depth_array.size == 0:
        return None
    h, w = depth_array.shape[:2]
    half = max(2, int(min(h, w) * roi_frac / 2))
    cx, cy = w // 2, h // 2
    roi = depth_array[cy - half : cy + half, cx - half : cx + half]
    valid = roi[roi > 0]
    if valid.size < 10:
        return None
    return float(np.median(valid)) / 1000.0


class PetBehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__("pet_behavior_node")

        self.declare_parameter("gesture_command_topic", "/gesture/command")
        self.declare_parameter("gesture_tracking_topic", "/gesture/tracking")
        self.declare_parameter("vision_target_topic", "/vision/target")
        self.declare_parameter("depth_image_topic", "/kinect/depth/image_raw")
        self.declare_parameter("cmd_vel_topic", "/commands/velocity")
        self.declare_parameter("sound_topic", "/commands/sound")
        self.declare_parameter("publish_idle_zero", True)
        self.declare_parameter("timer_hz", 20.0)

        self.declare_parameter("come_closer_distance_m", 0.45)
        self.declare_parameter("come_closer_target_m", 0.7)
        self.declare_parameter("come_closer_max_time_sec", 6.0)
        self.declare_parameter("obstacle_stop_m", 0.32)
        self.declare_parameter("use_depth_for_come_closer", True)
        self.declare_parameter("use_depth_for_safety", True)
        self.declare_parameter("foot_distance_m", 0.3048)
        self.declare_parameter("drive_speed_mps", 0.13)
        self.declare_parameter("turn_speed_radps", 0.65)
        self.declare_parameter("tail_wag_duration_sec", 3.2)
        self.declare_parameter("tail_wag_speed_radps", 1.05)
        self.declare_parameter("tail_wag_half_period_sec", 0.26)

        self.declare_parameter("align_after_actions", True)
        self.declare_parameter("align_timeout_sec", 3.0)
        self.declare_parameter("align_deadband", 0.06)
        self.declare_parameter("align_hold_sec", 0.25)
        self.declare_parameter("align_kp", 1.2)
        self.declare_parameter("align_max_speed_radps", 0.38)
        self.declare_parameter("align_search_speed_radps", 0.22)
        self.declare_parameter("align_search_max_sec", 1.6)
        self.declare_parameter("tracking_timeout_sec", 1.0)
        self.declare_parameter("target_turn_sign", -1.0)

        self.declare_parameter("enable_idle_wag", True)
        self.declare_parameter("idle_wag_interval_sec", 14.0)
        self.declare_parameter("idle_wag_duration_sec", 0.5)
        self.declare_parameter("idle_wag_speed_radps", 0.6)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        command_topic = self.get_parameter("gesture_command_topic").value
        tracking_topic = self.get_parameter("gesture_tracking_topic").value
        vision_topic = self.get_parameter("vision_target_topic").value
        depth_topic = self.get_parameter("depth_image_topic").value
        sound_topic = self.get_parameter("sound_topic").value

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._sound_pub = self.create_publisher(Sound, sound_topic, 10) if Sound is not None else None
        self._command_sub = self.create_subscription(String, command_topic, self._command_cb, 10)
        self._tracking_sub = self.create_subscription(String, tracking_topic, self._tracking_cb, 10)
        self._vision_sub = self.create_subscription(String, vision_topic, self._vision_cb, 10)
        depth_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._depth_sub = self.create_subscription(Image, depth_topic, self._depth_cb, depth_qos)

        self._action_name: Optional[str] = None
        self._segments: collections.deque = collections.deque()
        self._current_segment: Optional[dict] = None
        self._segment_started_at: Optional[float] = None
        self._active_command_id: Optional[str] = None
        self._completed_ids: collections.deque = collections.deque(maxlen=64)
        self._last_idle_zero = 0.0
        self._last_target: Optional[dict] = None
        self._last_vision: Optional[dict] = None
        self._align_centered_since: Optional[float] = None
        self._align_search_started_at: Optional[float] = None
        self._align_last_search_dir: float = 0.0
        self._sound_events: collections.deque = collections.deque()
        self._warned_no_sound = False
        self._latest_depth_distance: Optional[float] = None
        self._latest_depth_time: float = -1e6
        self._idle_wag_last_fired = _now()

        timer_hz = max(5.0, float(self.get_parameter("timer_hz").value))
        self._timer = self.create_timer(1.0 / timer_hz, self._timer_callback)

        self.get_logger().info(
            f"Pet behaviour node publishing Twist on {cmd_vel_topic}; depth on {depth_topic}"
        )
        if Sound is None:
            self.get_logger().warning(
                "kobuki_ros_interfaces missing; motion will work but Kobuki sounds are disabled."
            )

    def destroy_node(self):
        self._publish_twist(0.0, 0.0)
        super().destroy_node()

    # ------------------------------------------------------------------ subs

    def _command_cb(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Ignoring malformed gesture command: {msg.data}")
            return

        name = command.get("command")
        command_id = command.get("id")
        if not name or not command_id:
            self.get_logger().warning(f"Ignoring incomplete gesture command: {msg.data}")
            return
        if command_id in self._completed_ids or command_id == self._active_command_id:
            return

        if name == "STOP":
            self._stop_active_action(play_sound=True, reason="open palm")
            self._completed_ids.append(command_id)
            return

        if self._action_name is not None:
            self.get_logger().info(
                f"Ignoring {name}; already running {self._action_name}. Use open palm to stop."
            )
            return

        segments = self._segments_for_command(name)
        if not segments:
            self.get_logger().warning(f"No behaviour mapped for gesture command {name}")
            return

        if bool(self.get_parameter("align_after_actions").value):
            segments.append({
                "kind": "align",
                "duration": float(self.get_parameter("align_timeout_sec").value),
            })

        self._action_name = name
        self._active_command_id = command_id
        self._segments = collections.deque(segments)
        self._current_segment = None
        self._segment_started_at = None
        self._align_centered_since = None
        self._align_search_started_at = None
        self._queue_sound_profile(name)
        self.get_logger().info(
            f"Running behaviour for {name}: {command.get('reason', 'gesture command')}"
        )

    def _tracking_cb(self, msg):
        payload = self._parse_json(msg.data, "gesture tracking")
        if payload is None or not payload.get("hand_visible"):
            return
        self._last_target = {
            "source": "hand",
            "time": _now(),
            "center_x": float(payload.get("center_x", 0.5)),
            "offset_x": float(payload.get("offset_x", 0.0)),
            "confidence": float(payload.get("confidence", 0.0)),
        }

    def _vision_cb(self, msg):
        payload = self._parse_json(msg.data, "vision target")
        if payload is None or not payload.get("face_visible"):
            return
        self._last_vision = {
            "source": "face",
            "time": _now(),
            "center_x": float(payload.get("center_x", 0.5)),
            "offset_x": float(payload.get("offset_x", 0.0)),
            "confidence": float(payload.get("confidence", 0.0)),
        }

    def _depth_cb(self, msg):
        encoding = msg.encoding.lower()
        if encoding not in ("16uc1", "mono16", "type_16uc1"):
            return
        width, height = int(msg.width), int(msg.height)
        if width * height == 0:
            return
        try:
            array = np.frombuffer(msg.data, dtype=np.uint16)
            if array.size < width * height:
                return
            depth = array[: width * height].reshape((height, width))
        except Exception as exc:
            self.get_logger().warning(f"Depth decode failed: {exc}")
            return

        distance = _depth_meters_center(depth)
        if distance is not None:
            self._latest_depth_distance = distance
            self._latest_depth_time = _now()

    # ----------------------------------------------------------------- timer

    def _timer_callback(self):
        now = _now()
        self._publish_due_sounds(now)

        if self._action_name is None:
            self._maybe_idle_wag(now)
            if bool(self.get_parameter("publish_idle_zero").value) and now - self._last_idle_zero > 0.5:
                self._publish_twist(0.0, 0.0)
                self._last_idle_zero = now
            return

        if self._current_segment is None:
            self._start_next_segment(now)
            if self._current_segment is None:
                return

        segment = self._current_segment
        elapsed = now - self._segment_started_at
        kind = segment["kind"]

        if kind == "twist":
            if elapsed >= segment["duration"]:
                self._finish_segment(now)
                return
            linear = float(segment.get("linear", 0.0))
            angular = float(segment.get("angular", 0.0))
            if linear > 0.0 and self._obstacle_blocking_forward():
                self.get_logger().info("Forward motion aborted; obstacle within safety distance.")
                self._finish_segment(now)
                return
            self._publish_twist(linear, angular)
            return

        if kind == "drive_to_distance":
            if elapsed >= segment["duration"]:
                self._finish_segment(now)
                return
            if self._obstacle_blocking_forward():
                self.get_logger().info("Drive-to-distance halted; obstacle close.")
                self._finish_segment(now)
                return
            distance_now = self._fresh_depth_distance(now)
            if distance_now is not None and distance_now <= segment["target_distance_m"]:
                self.get_logger().info(
                    f"Reached target distance {segment['target_distance_m']:.2f} m (current {distance_now:.2f} m)."
                )
                self._finish_segment(now)
                return
            self._publish_twist(float(segment.get("speed", 0.1)), 0.0)
            return

        if kind == "oscillate":
            if elapsed >= segment["duration"]:
                self._finish_segment(now)
                return
            half_period = max(0.05, float(segment["half_period"]))
            direction = 1.0 if int(elapsed / half_period) % 2 == 0 else -1.0
            self._publish_twist(0.0, direction * float(segment["angular"]))
            return

        if kind == "align":
            if self._run_align_segment(now, elapsed, segment):
                self._finish_segment(now)
            return

        if kind == "pause":
            self._publish_twist(0.0, 0.0)
            if elapsed >= segment["duration"]:
                self._finish_segment(now)

    # -------------------------------------------------------- segment builds

    def _segments_for_command(self, name: str):
        drive_speed = float(self.get_parameter("drive_speed_mps").value)
        turn_speed = float(self.get_parameter("turn_speed_radps").value)
        foot_distance = float(self.get_parameter("foot_distance_m").value)
        come_distance = float(self.get_parameter("come_closer_distance_m").value)

        if name == "COME_CLOSER":
            if bool(self.get_parameter("use_depth_for_come_closer").value):
                return [
                    {
                        "kind": "drive_to_distance",
                        "speed": drive_speed,
                        "duration": float(self.get_parameter("come_closer_max_time_sec").value),
                        "target_distance_m": float(self.get_parameter("come_closer_target_m").value),
                    },
                    {"kind": "pause", "duration": 0.15},
                ]
            return [
                self._drive_segment(come_distance, drive_speed),
                {"kind": "pause", "duration": 0.12},
            ]

        if name == "ROTATE_ONCE":
            return [
                self._rotate_segment(2.0 * math.pi, turn_speed),
                {"kind": "pause", "duration": 0.16},
            ]

        if name == "MOVE_LEFT_FOOT":
            return [
                self._rotate_segment(math.pi / 2.0, turn_speed),
                self._drive_segment(foot_distance, drive_speed),
                self._rotate_segment(-math.pi / 2.0, turn_speed),
                {"kind": "pause", "duration": 0.12},
            ]

        if name == "MOVE_RIGHT_FOOT":
            return [
                self._rotate_segment(-math.pi / 2.0, turn_speed),
                self._drive_segment(foot_distance, drive_speed),
                self._rotate_segment(math.pi / 2.0, turn_speed),
                {"kind": "pause", "duration": 0.12},
            ]

        if name == "TAIL_WAG":
            return [
                {
                    "kind": "oscillate",
                    "duration": float(self.get_parameter("tail_wag_duration_sec").value),
                    "angular": float(self.get_parameter("tail_wag_speed_radps").value),
                    "half_period": float(self.get_parameter("tail_wag_half_period_sec").value),
                },
                {"kind": "pause", "duration": 0.15},
            ]

        return []

    @staticmethod
    def _drive_segment(distance: float, speed: float) -> dict:
        speed = max(0.03, abs(speed))
        sign = 1.0 if distance >= 0.0 else -1.0
        return {
            "kind": "twist",
            "duration": abs(distance) / speed,
            "linear": sign * speed,
            "angular": 0.0,
        }

    @staticmethod
    def _rotate_segment(angle: float, speed: float) -> dict:
        speed = max(0.12, abs(speed))
        sign = 1.0 if angle >= 0.0 else -1.0
        return {
            "kind": "twist",
            "duration": abs(angle) / speed,
            "linear": 0.0,
            "angular": sign * speed,
        }

    # ------------------------------------------------------ segment runtime

    def _start_next_segment(self, now: float) -> None:
        if not self._segments:
            self._finish_action()
            return
        self._current_segment = self._segments.popleft()
        self._segment_started_at = now
        self._align_centered_since = None
        self._align_search_started_at = None

    def _finish_segment(self, now: float) -> None:
        self._publish_twist(0.0, 0.0)
        self._current_segment = None
        self._segment_started_at = None
        self._start_next_segment(now)

    def _finish_action(self) -> None:
        name = self._action_name
        if self._active_command_id:
            self._completed_ids.append(self._active_command_id)
        self._publish_twist(0.0, 0.0)
        self._action_name = None
        self._active_command_id = None
        self._current_segment = None
        self._segment_started_at = None
        self._segments.clear()
        self._idle_wag_last_fired = _now()
        self.get_logger().info(f"Behaviour complete: {name}")

    def _stop_active_action(self, play_sound: bool, reason: str) -> None:
        if play_sound:
            self._queue_sound_profile("STOP")
        self._segments.clear()
        self._current_segment = None
        self._segment_started_at = None
        self._action_name = None
        self._active_command_id = None
        self._publish_twist(0.0, 0.0)
        self._idle_wag_last_fired = _now()
        self.get_logger().info(f"Robot stopped by {reason}.")

    # -------------------------------------------------------- align segment

    def _run_align_segment(self, now: float, elapsed: float, segment: dict) -> bool:
        if elapsed >= segment["duration"]:
            self._publish_twist(0.0, 0.0)
            return True

        target = self._fresh_target(now)
        if target is None:
            search_speed = float(self.get_parameter("align_search_speed_radps").value)
            search_max = float(self.get_parameter("align_search_max_sec").value)

            if self._align_search_started_at is None:
                self._align_search_started_at = now
                self._align_last_search_dir = self._biased_search_direction()

            if now - self._align_search_started_at > search_max:
                # Give up gracefully instead of spinning in place forever.
                self._publish_twist(0.0, 0.0)
                return True

            self._publish_twist(0.0, self._align_last_search_dir * search_speed)
            return False

        self._align_search_started_at = None

        offset = target["center_x"] - 0.5
        deadband = float(self.get_parameter("align_deadband").value)
        if abs(offset) <= deadband:
            self._publish_twist(0.0, 0.0)
            if self._align_centered_since is None:
                self._align_centered_since = now
            return now - self._align_centered_since >= float(self.get_parameter("align_hold_sec").value)

        self._align_centered_since = None
        turn_sign = float(self.get_parameter("target_turn_sign").value)
        kp = float(self.get_parameter("align_kp").value)
        max_speed = float(self.get_parameter("align_max_speed_radps").value)
        angular = _clamp(turn_sign * kp * offset, -max_speed, max_speed)
        self._publish_twist(0.0, angular)
        return False

    def _biased_search_direction(self) -> float:
        turn_sign = float(self.get_parameter("target_turn_sign").value)
        target = self._last_target or self._last_vision
        if target is None:
            return turn_sign
        offset = float(target.get("offset_x", 0.0))
        if offset == 0.0:
            return turn_sign
        return turn_sign * (1.0 if offset > 0 else -1.0)

    def _fresh_target(self, now: float) -> Optional[dict]:
        timeout = float(self.get_parameter("tracking_timeout_sec").value)
        if self._last_target and now - self._last_target["time"] <= timeout:
            return self._last_target
        if self._last_vision and now - self._last_vision["time"] <= timeout:
            return self._last_vision
        return None

    # ------------------------------------------------------------- depth IO

    def _fresh_depth_distance(self, now: float) -> Optional[float]:
        if self._latest_depth_distance is None:
            return None
        if now - self._latest_depth_time > 1.5:
            return None
        return self._latest_depth_distance

    def _obstacle_blocking_forward(self) -> bool:
        if not bool(self.get_parameter("use_depth_for_safety").value):
            return False
        distance = self._fresh_depth_distance(_now())
        if distance is None:
            return False
        threshold = float(self.get_parameter("obstacle_stop_m").value)
        return distance <= threshold

    # ---------------------------------------------------------------- sound

    def _queue_sound_profile(self, command_name: str) -> None:
        if self._sound_pub is None:
            if not self._warned_no_sound:
                self.get_logger().warning("No Kobuki sound publisher available.")
                self._warned_no_sound = True
            return
        now = _now()
        for delay, value in profile_for(command_name):
            self._sound_events.append((now + delay, int(value)))

    def _publish_due_sounds(self, now: float) -> None:
        if self._sound_pub is None:
            return
        while self._sound_events and self._sound_events[0][0] <= now:
            _, value = self._sound_events.popleft()
            msg = Sound()
            msg.value = value
            self._sound_pub.publish(msg)

    # ---------------------------------------------------------------- idle

    def _maybe_idle_wag(self, now: float) -> None:
        if not bool(self.get_parameter("enable_idle_wag").value):
            return
        interval = float(self.get_parameter("idle_wag_interval_sec").value)
        if now - self._idle_wag_last_fired < interval:
            return
        if self._fresh_target(now) is None:
            # Don't be cute when nobody is watching.
            self._idle_wag_last_fired = now
            return
        duration = float(self.get_parameter("idle_wag_duration_sec").value)
        speed = float(self.get_parameter("idle_wag_speed_radps").value)
        half_period = max(0.05, duration / 2.0)
        self._segments = collections.deque([
            {"kind": "oscillate", "duration": duration, "angular": speed, "half_period": half_period},
            {"kind": "pause", "duration": 0.1},
        ])
        self._action_name = "IDLE_WAG"
        self._active_command_id = f"IDLE_WAG-{int(now * 1000)}"
        self._current_segment = None
        self._segment_started_at = None
        self._queue_sound_profile("IDLE_WAG")
        self._idle_wag_last_fired = now

    # -------------------------------------------------------------- helpers

    def _publish_twist(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

    def _parse_json(self, data: str, label: str):
        try:
            return json.loads(data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Ignoring malformed {label}: {data}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PetBehaviorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
