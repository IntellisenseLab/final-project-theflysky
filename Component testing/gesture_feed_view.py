"""High-FPS Kinect feed viewer with live gesture-decision overlay.

Unlike the gesture node's built-in ``/gesture/debug_image`` (which is
rate-limited to ``max_fps`` and tied to MediaPipe inference), this viewer
subscribes to the *raw* Kinect RGB stream and draws the latest decision on
top. The video therefore runs at the full camera rate (~30 fps) while the
gesture state updates as fast as the gesture node publishes it.

It overlays:
* the smoothed per-frame gesture label, openness and pointing direction
  (from ``/gesture/tracking``);
* a dot at the tracked hand centre;
* the most recent confirmed command such as COME_CLOSER / STOP / ROTATE_ONCE
  (from ``/gesture/command``), with how long ago it fired;
* the measured display FPS.

Run (Kinect node + gesture node must already be publishing):

    source qbot_ws/install/setup.bash
    python3 "Component testing/gesture_feed_view.py"

Press 'q' or ESC to quit.
"""

from __future__ import annotations

import json
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image

RGB_TOPIC = "/kinect/rgb/image_raw"
TRACKING_TOPIC = "/gesture/tracking"
COMMAND_TOPIC = "/gesture/command"


class GestureFeedViewer(Node):
    def __init__(self) -> None:
        super().__init__("gesture_feed_viewer")
        self.create_subscription(Image, RGB_TOPIC, self._on_image, qos_profile_sensor_data)
        self.create_subscription(String, TRACKING_TOPIC, self._on_tracking, 10)
        self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)

        self._tracking: dict = {}
        self._last_command: dict = {}
        self._last_command_time: float = 0.0

        self._headless = bool(int(os.environ.get("QBOT_VIEWER_HEADLESS", "0")))
        self._frame_count = 0
        self._fps = 0.0
        self._fps_t0 = time.monotonic()
        self._log_t0 = time.monotonic()

    def _on_tracking(self, msg: String) -> None:
        try:
            self._tracking = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_command(self, msg: String) -> None:
        try:
            self._last_command = json.loads(msg.data)
            self._last_command_time = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _on_image(self, msg: Image) -> None:
        frame = self._decode(msg)
        if frame is None:
            return
        self._tick_fps()
        self._draw_overlay(frame)
        if not self._headless:
            try:
                cv2.imshow("Kinect feed + gesture decisions", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    rclpy.shutdown()
            except cv2.error:
                # No display available (e.g. headless Pi over SSH): keep
                # running and just report FPS to the console instead.
                self._headless = True
                self.get_logger().warning("No display; running headless, logging FPS only.")

    @staticmethod
    def _decode(msg: Image):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            img = buf.reshape(msg.height, msg.width, 3)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgr8":
            return buf.reshape(msg.height, msg.width, 3).copy()
        return None

    def _tick_fps(self) -> None:
        self._frame_count += 1
        now = time.monotonic()
        if now - self._fps_t0 >= 0.5:
            self._fps = self._frame_count / (now - self._fps_t0)
            self._frame_count = 0
            self._fps_t0 = now
        if now - self._log_t0 >= 1.0:
            self._log_t0 = now
            gesture = self._tracking.get("gesture", "-") if self._tracking else "-"
            cmd = self._last_command.get("command", "-") if self._last_command else "-"
            self.get_logger().info(f"feed {self._fps:.1f} fps | gesture={gesture} | last_cmd={cmd}")

    def _draw_overlay(self, frame) -> None:
        h, w = frame.shape[:2]
        t = self._tracking

        cv2.putText(frame, f"{self._fps:4.1f} fps", (w - 130, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

        if t.get("hand_visible"):
            gesture = t.get("gesture", "-")
            direction = t.get("pointing_direction") or "-"
            openness = float(t.get("openness", 0.0))
            cv2.putText(frame, f"{gesture}  dir={direction}  open={openness:.2f}",
                        (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

            # openness bar
            bar_w = int(180 * max(0.0, min(1.0, openness)))
            cv2.rectangle(frame, (12, 44), (192, 60), (80, 80, 80), 1)
            cv2.rectangle(frame, (12, 44), (12 + bar_w, 60), (0, 255, 0), -1)

            # hand centre dot
            cx = int(float(t.get("center_x", 0.5)) * w)
            cy = int(float(t.get("center_y", 0.5)) * h)
            cv2.circle(frame, (cx, cy), 8, (0, 200, 255), 2)
        else:
            cv2.putText(frame, "no hand", (12, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2, cv2.LINE_AA)

        cmd = self._last_command
        if cmd:
            age = time.monotonic() - self._last_command_time
            color = (0, 0, 255) if age < 1.5 else (0, 140, 255)
            cv2.putText(frame, f"CMD: {cmd.get('command', '-')}  ({age:.1f}s ago)",
                        (12, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)


def main() -> None:
    rclpy.init()
    node = GestureFeedViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
