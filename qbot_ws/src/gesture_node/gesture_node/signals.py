"""Continuous signal helpers used by the gesture decoder.

The original decoder relied on MediaPipe's discrete gesture labels
(Open_Palm / Closed_Fist / Pointing_Up) flipping back and forth to
recognise a beckon. That coupling was brittle: a half-curled hand often
classified as "Custom" and broke the oscillation count.

These helpers replace label-edge counting with hysteresis around a
continuous ``openness`` metric and add EMA smoothing with outlier
rejection on the raw MediaPipe landmarks so a single noisy frame cannot
disturb the temporal state.
"""

from __future__ import annotations

import collections
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


PALM_BASE_INDICES = (5, 17)
FINGER_TIP_INDICES = (8, 12, 16, 20)


@dataclass
class _Vec3:
    """Tiny stand-in compatible with mediapipe's NormalizedLandmark."""

    x: float
    y: float
    z: float


def hand_openness(landmarks) -> float:
    """Return a value in [0, 1]; 0 = closed fist, 1 = fully extended palm.

    The metric divides the mean fingertip-to-wrist distance by the palm
    width (index-MCP to pinky-MCP), normalising for hand size and distance
    from the camera.
    """

    wrist = landmarks[0]
    palm_width = math.hypot(
        landmarks[PALM_BASE_INDICES[0]].x - landmarks[PALM_BASE_INDICES[1]].x,
        landmarks[PALM_BASE_INDICES[0]].y - landmarks[PALM_BASE_INDICES[1]].y,
    )
    if palm_width < 1e-3:
        return 0.5

    total = 0.0
    for tip_index in FINGER_TIP_INDICES:
        tip = landmarks[tip_index]
        total += math.hypot(tip.x - wrist.x, tip.y - wrist.y)
    mean_tip_distance = total / len(FINGER_TIP_INDICES)

    ratio = mean_tip_distance / palm_width
    return max(0.0, min(1.0, (ratio - 1.0) / 1.6))


class LandmarkSmoother:
    """EMA smoother with jump rejection across consecutive frames."""

    def __init__(self, alpha: float = 0.55, jump_threshold: float = 0.22) -> None:
        self._alpha = float(alpha)
        self._jump_threshold = float(jump_threshold)
        self._state: Optional[list] = None

    def reset(self) -> None:
        self._state = None

    def update(self, landmarks) -> list:
        if self._state is None or len(self._state) != len(landmarks):
            self._state = [_Vec3(lm.x, lm.y, lm.z) for lm in landmarks]
            return list(self._state)

        for index, lm in enumerate(landmarks):
            prev = self._state[index]
            jump = math.hypot(lm.x - prev.x, lm.y - prev.y)
            if jump > self._jump_threshold:
                # Suspicious outlier: nudge gently instead of trusting it.
                blend = 0.20
            else:
                blend = self._alpha
            prev.x = blend * lm.x + (1.0 - blend) * prev.x
            prev.y = blend * lm.y + (1.0 - blend) * prev.y
            prev.z = blend * lm.z + (1.0 - blend) * prev.z
        return list(self._state)


class MajorityLabelFilter:
    """Vote across the last ``size`` labels; require ``min_votes`` to switch."""

    def __init__(self, size: int, min_votes: int) -> None:
        self._history: collections.deque = collections.deque(maxlen=int(size))
        self._min_votes = int(min_votes)

    def clear(self) -> None:
        self._history.clear()

    def update(self, label: str) -> str:
        self._history.append(label)
        counts = collections.Counter(self._history)
        top, votes = counts.most_common(1)[0]
        if votes >= self._min_votes:
            return top
        return self._history[-1]


class BeckonOscillationDetector:
    """Counts open <-> closed oscillations from a continuous openness signal.

    Uses Schmitt-trigger hysteresis around the ``open_threshold`` /
    ``closed_threshold`` band so jitter near the midpoint does not produce
    phantom oscillations. A minimum half-period (``min_half_period_sec``)
    is enforced to ignore unrealistically fast flips.
    """

    def __init__(
        self,
        oscillations_required: int = 2,
        window_sec: float = 4.5,
        open_threshold: float = 0.62,
        closed_threshold: float = 0.32,
        min_half_period_sec: float = 0.18,
    ) -> None:
        self.oscillations_required = int(oscillations_required)
        self.window_sec = float(window_sec)
        self.open_threshold = float(open_threshold)
        self.closed_threshold = float(closed_threshold)
        self.min_half_period_sec = float(min_half_period_sec)
        self._curl_times: collections.deque = collections.deque()
        self._state: str = "neutral"
        self._last_change: float = -math.inf

    def reset(self) -> None:
        self._curl_times.clear()
        self._state = "neutral"
        self._last_change = -math.inf

    def update(self, now: float, openness: float) -> int:
        while self._curl_times and now - self._curl_times[0] > self.window_sec:
            self._curl_times.popleft()

        if openness >= self.open_threshold and self._state != "open":
            if self._state == "closed" and now - self._last_change >= self.min_half_period_sec:
                self._curl_times.append(now)
            self._state = "open"
            self._last_change = now
        elif openness <= self.closed_threshold and self._state != "closed":
            self._state = "closed"
            self._last_change = now

        return len(self._curl_times)

    def ready(self) -> bool:
        return len(self._curl_times) >= self.oscillations_required

    def consume(self) -> int:
        count = len(self._curl_times)
        self._curl_times.clear()
        self._state = "neutral"
        self._last_change = -math.inf
        return count


def count_axis_reversals(values: Iterable[float], min_sweep: float) -> int:
    """Count direction reversals along a 1-D series with minimum sweep gating."""

    seq = list(values)
    if len(seq) < 2:
        return 0

    direction = 0
    extreme = seq[0]
    anchor = seq[0]
    reversals = 0

    for value in seq[1:]:
        if direction == 0:
            delta = value - anchor
            if abs(delta) >= min_sweep:
                direction = 1 if delta > 0 else -1
                extreme = value
            continue

        if direction > 0:
            if value > extreme:
                extreme = value
            elif extreme - value >= min_sweep:
                reversals += 1
                direction = -1
                extreme = value
        else:
            if value < extreme:
                extreme = value
            elif value - extreme >= min_sweep:
                reversals += 1
                direction = 1
                extreme = value

    return reversals


def label_from_finger_pattern(fingers: Tuple[int, int, int, int, int]) -> str:
    """Map a fingers-up tuple to a coarse gesture label."""

    patterns = {
        (0, 0, 0, 0, 0): "FIST",
        (1, 1, 1, 1, 1): "OPEN_PALM",
        (0, 1, 0, 0, 0): "POINTING",
        (1, 0, 0, 0, 0): "THUMBS_UP",
        (0, 1, 1, 0, 0): "PEACE",
        (1, 1, 0, 0, 1): "I_LOVE_YOU",
    }
    return patterns.get(tuple(fingers), "CUSTOM")
