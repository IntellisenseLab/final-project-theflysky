"""Frame preprocessing utilities shared by the perception nodes.

Adds two layers of robustness that the original pipeline lacked:

1. **Quality gating** - very dark or near-saturated frames are dropped
   before they reach the expensive recognisers, which both saves CPU on
   the Raspberry Pi 5 and avoids feeding the model garbage.
2. **CLAHE** (Contrast Limited Adaptive Histogram Equalisation) is
   applied to the luma channel for low-light robustness. Indoor scenes
   under a desk lamp are a common case where the Kinect's auto-gain is
   not enough.

It also centralises the ``sensor_msgs/Image`` decoder so the gesture and
vision nodes share one implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


_CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


@dataclass
class FrameQuality:
    mean_luma: float
    accepted: bool
    reason: str


def decode_ros_image(msg, target: str = "rgb") -> np.ndarray:
    """Convert a ``sensor_msgs/Image`` to BGR or RGB ndarray.

    Supports the encodings the Kinect publisher and common simulators
    emit. Raises ``ValueError`` for anything else.
    """

    encoding = msg.encoding.lower()
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in ("rgb8", "bgr8"):
        channels = 3
        rows = data.reshape((height, step))
        image = rows[:, : width * channels].reshape((height, width, channels))
        if target == "rgb" and encoding == "bgr8":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif target == "bgr" and encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)

    if encoding in ("rgba8", "bgra8"):
        channels = 4
        rows = data.reshape((height, step))
        image = rows[:, : width * channels].reshape((height, width, channels))
        if target == "rgb":
            code = cv2.COLOR_RGBA2RGB if encoding == "rgba8" else cv2.COLOR_BGRA2RGB
        else:
            code = cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR
        return np.ascontiguousarray(cv2.cvtColor(image, code))

    if encoding in ("mono8", "8uc1"):
        rows = data.reshape((height, step))
        gray = rows[:, :width].reshape((height, width))
        code = cv2.COLOR_GRAY2RGB if target == "rgb" else cv2.COLOR_GRAY2BGR
        return np.ascontiguousarray(cv2.cvtColor(gray, code))

    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def decode_depth_image(msg) -> Optional[np.ndarray]:
    """Decode a depth image (16-bit mm) into a 2-D ``uint16`` ndarray."""

    encoding = msg.encoding.lower()
    if encoding not in ("16uc1", "mono16", "type_16uc1"):
        return None
    width = int(msg.width)
    height = int(msg.height)
    array = np.frombuffer(msg.data, dtype=np.uint16)
    if array.size < width * height:
        return None
    return array[: width * height].reshape((height, width))


def assess_quality(
    image: np.ndarray,
    min_luma: float = 22.0,
    max_luma: float = 245.0,
) -> FrameQuality:
    """Reject frames that are way too dark or way too bright to be useful."""

    if image.ndim == 3 and image.shape[2] == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    else:
        gray = image
    mean_luma = float(gray.mean())
    if mean_luma < min_luma:
        return FrameQuality(mean_luma, False, "frame too dark")
    if mean_luma > max_luma:
        return FrameQuality(mean_luma, False, "frame saturated")
    return FrameQuality(mean_luma, True, "ok")


def apply_clahe(image_rgb: np.ndarray) -> np.ndarray:
    """Return an RGB copy with CLAHE applied to the luma channel."""

    if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
        return image_rgb
    ycrcb = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2YCrCb)
    ycrcb[:, :, 0] = _CLAHE.apply(ycrcb[:, :, 0])
    return cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2RGB)


def downscale(image: np.ndarray, max_width: int) -> np.ndarray:
    """Shrink to ``max_width`` keeping aspect ratio. No-op if already small."""

    if max_width <= 0 or image.shape[1] <= max_width:
        return image
    scale = max_width / float(image.shape[1])
    new_size = (max_width, max(1, int(round(image.shape[0] * scale))))
    return cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)


def depth_meters_in_roi(depth_mm: np.ndarray, cx: float, cy: float, roi_frac: float = 0.18) -> Optional[float]:
    """Return the median depth in metres inside a centred ROI.

    ``cx`` / ``cy`` are normalised image coordinates (0..1). Invalid
    samples (depth == 0) are filtered out before taking the median so the
    result is robust to small specular dropouts.
    """

    h, w = depth_mm.shape[:2]
    half = max(2, int(min(h, w) * roi_frac / 2))
    px = int(max(half, min(w - half - 1, cx * w)))
    py = int(max(half, min(h - half - 1, cy * h)))
    roi = depth_mm[py - half : py + half, px - half : px + half]
    valid = roi[roi > 0]
    if valid.size < 10:
        return None
    return float(np.median(valid)) / 1000.0
