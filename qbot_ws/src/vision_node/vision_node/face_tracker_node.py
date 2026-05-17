"""Face tracker used by the behaviour node to re-centre on the user.

Prefers MediaPipe FaceDetection (BlazeFace) because it is roughly an
order of magnitude faster and more accurate than OpenCV's Haar cascade
on a Raspberry Pi 5. Falls back to the Haar cascade only if MediaPipe
cannot be imported, so the node still functions on a stripped-down image.

Outputs a JSON payload on ``/vision/target`` containing the smoothed
normalised face centre, the bounding box, and a small confidence value
that the behaviour node can compare against the hand tracking signal.
"""

from __future__ import annotations

import collections
import json
import os
import time

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
os.environ.setdefault("GLOG_minloglevel", "2")
os.environ.setdefault("ABSL_MIN_LOG_LEVEL", "2")

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import mediapipe as mp  # type: ignore
    _MEDIAPIPE_AVAILABLE = True
except Exception:  # pragma: no cover - import-time fallback
    mp = None
    _MEDIAPIPE_AVAILABLE = False


def _now() -> float:
    return time.monotonic()


def _decode_bgr(msg) -> np.ndarray:
    encoding = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    width, height, step = int(msg.width), int(msg.height), int(msg.step)

    if encoding in ("rgb8", "bgr8"):
        rows = data.reshape((height, step))
        image = rows[:, : width * 3].reshape((height, width, 3))
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)
    if encoding in ("rgba8", "bgra8"):
        rows = data.reshape((height, step))
        image = rows[:, : width * 4].reshape((height, width, 4))
        code = cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR
        return np.ascontiguousarray(cv2.cvtColor(image, code))
    if encoding in ("mono8", "8uc1"):
        rows = data.reshape((height, step))
        gray = rows[:, :width].reshape((height, width))
        return np.ascontiguousarray(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


class _MediaPipeDetector:
    def __init__(self, min_confidence: float) -> None:
        self._detector = mp.solutions.face_detection.FaceDetection(
            model_selection=0,
            min_detection_confidence=min_confidence,
        )

    def detect(self, bgr: np.ndarray):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        results = self._detector.process(rgb)
        if not results or not results.detections:
            return []
        h, w = bgr.shape[:2]
        out = []
        for det in results.detections:
            box = det.location_data.relative_bounding_box
            if box.width <= 0 or box.height <= 0:
                continue
            x = max(0.0, float(box.xmin))
            y = max(0.0, float(box.ymin))
            bw = min(1.0 - x, float(box.width))
            bh = min(1.0 - y, float(box.height))
            score = float(det.score[0]) if det.score else 0.5
            out.append({
                "center_x": x + bw / 2.0,
                "center_y": y + bh / 2.0,
                "width": bw,
                "height": bh,
                "confidence": max(0.05, min(1.0, score)),
                "_pixel_area": bw * bh * w * h,
            })
        return out

    def close(self):
        try:
            self._detector.close()
        except Exception:
            pass


class _HaarDetector:
    def __init__(self, scale_factor: float, min_neighbors: int, min_face_size: int) -> None:
        cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        self._cascade = cv2.CascadeClassifier(cascade_path)
        if self._cascade.empty():
            raise RuntimeError(f"Could not load OpenCV face cascade: {cascade_path}")
        self._scale_factor = scale_factor
        self._min_neighbors = min_neighbors
        self._min_face_size = min_face_size

    def detect(self, bgr: np.ndarray):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        h, w = gray.shape[:2]
        faces = self._cascade.detectMultiScale(
            gray,
            scaleFactor=self._scale_factor,
            minNeighbors=self._min_neighbors,
            minSize=(self._min_face_size, self._min_face_size),
        )
        out = []
        for x, y, fw, fh in faces:
            area = fw * fh
            out.append({
                "center_x": (x + fw * 0.5) / float(w),
                "center_y": (y + fh * 0.5) / float(h),
                "width": fw / float(w),
                "height": fh / float(h),
                "confidence": min(1.0, area / (w * h) * 12.0),
                "_pixel_area": area,
            })
        return out

    def close(self):
        pass


class FaceTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("face_tracker_node")

        self.declare_parameter("image_topic", "/kinect/rgb/image_raw")
        self.declare_parameter("target_topic", "/vision/target")
        self.declare_parameter("max_fps", 8.0)
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("stable_frames_required", 2)
        self.declare_parameter("hold_last_detection_sec", 0.6)
        self.declare_parameter("ema_alpha", 0.5)
        self.declare_parameter("downscale_width", 480)
        self.declare_parameter("force_haar", False)
        # Haar-only knobs
        self.declare_parameter("scale_factor", 1.12)
        self.declare_parameter("min_neighbors", 5)
        self.declare_parameter("min_face_size_px", 42)

        image_topic = self.get_parameter("image_topic").value
        target_topic = self.get_parameter("target_topic").value
        force_haar = bool(self.get_parameter("force_haar").value)
        min_conf = float(self.get_parameter("min_confidence").value)

        if _MEDIAPIPE_AVAILABLE and not force_haar:
            self._detector = _MediaPipeDetector(min_conf)
            self.get_logger().info("Face tracker using MediaPipe FaceDetection")
        else:
            self._detector = _HaarDetector(
                scale_factor=float(self.get_parameter("scale_factor").value),
                min_neighbors=int(self.get_parameter("min_neighbors").value),
                min_face_size=int(self.get_parameter("min_face_size_px").value),
            )
            self.get_logger().info("Face tracker using OpenCV Haar cascade")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._sub = self.create_subscription(Image, image_topic, self._image_callback, qos)
        self._pub = self.create_publisher(String, target_topic, 10)

        self._last_process_time = 0.0
        self._recent_detections: collections.deque = collections.deque(maxlen=6)
        self._smoothed = None
        self._last_visible_time = -1e6

    def destroy_node(self):
        try:
            self._detector.close()
        except Exception:
            pass
        super().destroy_node()

    def _image_callback(self, msg):
        now = _now()
        max_fps = max(1.0, float(self.get_parameter("max_fps").value))
        if now - self._last_process_time < 1.0 / max_fps:
            return
        self._last_process_time = now

        try:
            bgr = _decode_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f"Could not decode image: {exc}")
            return

        target_width = int(self.get_parameter("downscale_width").value)
        if 0 < target_width < bgr.shape[1]:
            scale = target_width / float(bgr.shape[1])
            bgr_small = cv2.resize(
                bgr,
                (target_width, max(1, int(round(bgr.shape[0] * scale)))),
                interpolation=cv2.INTER_AREA,
            )
        else:
            bgr_small = bgr

        try:
            detections = self._detector.detect(bgr_small)
        except Exception as exc:
            self.get_logger().warning(f"Face detection failed: {exc}")
            detections = []

        face = max(detections, key=lambda d: d["_pixel_area"]) if detections else None

        if face is not None:
            self._recent_detections.append((now, face))
            self._smoothed = self._smooth(face)
            self._last_visible_time = now
        else:
            hold = float(self.get_parameter("hold_last_detection_sec").value)
            if self._smoothed is not None and now - self._last_visible_time > hold:
                self._smoothed = None

        stable_required = int(self.get_parameter("stable_frames_required").value)
        recent_visible = sum(1 for stamp, _ in self._recent_detections if now - stamp <= 0.8)
        face_visible = self._smoothed is not None and recent_visible >= stable_required

        payload = {
            "face_visible": bool(face_visible),
            "center_x": round(float(self._smoothed["center_x"]), 4) if face_visible else 0.5,
            "center_y": round(float(self._smoothed["center_y"]), 4) if face_visible else 0.5,
            "offset_x": round(float(self._smoothed["center_x"] - 0.5), 4) if face_visible else 0.0,
            "width": round(float(self._smoothed["width"]), 4) if face_visible else 0.0,
            "height": round(float(self._smoothed["height"]), 4) if face_visible else 0.0,
            "confidence": round(float(self._smoothed["confidence"]), 3) if face_visible else 0.0,
            "frame_id": msg.header.frame_id,
            "stamp_monotonic": round(now, 3),
        }
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self._pub.publish(out)

    def _smooth(self, detection: dict) -> dict:
        if self._smoothed is None:
            return {k: float(detection[k]) for k in ("center_x", "center_y", "width", "height", "confidence")}
        alpha = float(self.get_parameter("ema_alpha").value)
        return {
            key: alpha * float(detection[key]) + (1.0 - alpha) * float(self._smoothed[key])
            for key in ("center_x", "center_y", "width", "height", "confidence")
        }


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
