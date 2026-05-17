"""Unit tests for the temporal command decoder.

These tests exercise the temporal logic directly with synthetic
observations - no ROS, no MediaPipe required. They are the safety net
that makes it safe to keep tuning thresholds without manually walking
through every gesture on the robot.

Run from the package root:

    pytest qbot_ws/src/gesture_node/test/test_decoder.py
"""

from __future__ import annotations

import math
import os
import sys

import pytest


# Allow the tests to import the package without a ROS install.
PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from gesture_node.gesture_command_node import TemporalCommandDecoder  # noqa: E402
from gesture_node.signals import (  # noqa: E402
    BeckonOscillationDetector,
    LandmarkSmoother,
    MajorityLabelFilter,
    count_axis_reversals,
    hand_openness,
)


class _LM:
    """Stand-in for mediapipe NormalizedLandmark."""

    __slots__ = ("x", "y", "z")

    def __init__(self, x, y, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


def _hand(openness: float):
    """Construct a 21-landmark hand whose openness metric matches the request.

    The wrist sits at the origin, the palm-base landmarks are at fixed
    positions and the four fingertips share a single distance from the
    wrist scaled to land on the requested openness value.
    """

    landmarks = [_LM(0.5, 0.7) for _ in range(21)]
    # Palm-base landmarks: ~0.10 apart horizontally.
    landmarks[5] = _LM(0.45, 0.65)
    landmarks[17] = _LM(0.55, 0.65)
    palm_width = 0.10
    target_ratio = 1.0 + 1.6 * max(0.0, min(1.0, openness))
    tip_distance = target_ratio * palm_width
    tip_y = 0.7 - tip_distance  # wrist is at y=0.7
    for tip_index in (8, 12, 16, 20):
        landmarks[tip_index] = _LM(0.5, tip_y)
    landmarks[0] = _LM(0.5, 0.7)
    return landmarks


def _obs(time_s: float, *, gesture="FIST", openness=0.1, center_x=0.5,
         center_y=0.5, pointing=None, index_tip=None, confidence=0.8):
    return {
        "time": float(time_s),
        "frame_id": "test",
        "hand_visible": True,
        "gesture": gesture,
        "raw_gesture": gesture,
        "confidence": float(confidence),
        "center_x": float(center_x),
        "center_y": float(center_y),
        "span": 0.2,
        "openness": float(openness),
        "pointing_direction": pointing,
        "index_tip": index_tip,
        "fingers": [0, 0, 0, 0, 0],
    }


# ---------------------------------------------------------------- signals


class TestHandOpenness:
    def test_returns_low_for_curled_fingertips(self):
        # Fingertips near wrist (~palm_width away).
        landmarks = _hand(openness=0.0)
        assert hand_openness(landmarks) < 0.1

    def test_returns_high_for_extended_fingertips(self):
        landmarks = _hand(openness=1.0)
        assert hand_openness(landmarks) > 0.9

    def test_monotonic_between_curl_and_extend(self):
        prev = -1.0
        for value in (0.1, 0.3, 0.5, 0.7, 0.9):
            current = hand_openness(_hand(value))
            assert current >= prev
            prev = current


class TestLandmarkSmoother:
    def test_initial_frame_preserved(self):
        smoother = LandmarkSmoother(alpha=0.5, jump_threshold=0.5)
        out = smoother.update([_LM(0.2, 0.3, 0.0)] * 21)
        assert out[0].x == pytest.approx(0.2)
        assert out[0].y == pytest.approx(0.3)

    def test_outlier_jump_rejected(self):
        smoother = LandmarkSmoother(alpha=0.8, jump_threshold=0.1)
        smoother.update([_LM(0.5, 0.5, 0.0)] * 21)
        # Inject a wild jump on landmark 0 only.
        next_frame = [_LM(0.5, 0.5, 0.0)] * 21
        next_frame[0] = _LM(0.99, 0.99, 0.0)
        out = smoother.update(next_frame)
        # Jump should be heavily damped, well below the 0.8 EMA blend.
        assert out[0].x < 0.7
        assert out[0].y < 0.7

    def test_smooth_motion_followed(self):
        smoother = LandmarkSmoother(alpha=0.5, jump_threshold=0.5)
        smoother.update([_LM(0.0, 0.0, 0.0)] * 21)
        out = smoother.update([_LM(0.1, 0.0, 0.0)] * 21)
        assert 0.04 < out[0].x < 0.06


class TestMajorityLabelFilter:
    def test_holds_majority(self):
        f = MajorityLabelFilter(size=5, min_votes=3)
        for label in ["OPEN_PALM", "OPEN_PALM", "FIST", "OPEN_PALM", "OPEN_PALM"]:
            result = f.update(label)
        assert result == "OPEN_PALM"

    def test_returns_latest_when_no_majority(self):
        f = MajorityLabelFilter(size=4, min_votes=3)
        f.update("A")
        f.update("B")
        assert f.update("C") == "C"


class TestBeckonOscillationDetector:
    def test_two_oscillations_required_by_default(self):
        det = BeckonOscillationDetector(oscillations_required=2)
        assert not det.ready()
        t = 0.0
        # One full curl (open -> closed -> open) ~ one oscillation.
        det.update(t, 0.9)
        t += 0.3
        det.update(t, 0.1)
        t += 0.3
        det.update(t, 0.9)
        assert not det.ready()  # only 1 oscillation
        t += 0.3
        det.update(t, 0.1)
        t += 0.3
        det.update(t, 0.9)
        assert det.ready()  # 2 oscillations

    def test_hysteresis_band_ignored(self):
        det = BeckonOscillationDetector(open_threshold=0.6, closed_threshold=0.4)
        det.update(0.0, 0.5)
        det.update(0.1, 0.55)
        det.update(0.2, 0.45)
        assert not det.ready()

    def test_window_expiry(self):
        det = BeckonOscillationDetector(window_sec=1.0)
        det.update(0.0, 0.9)
        det.update(0.1, 0.1)
        det.update(0.2, 0.9)  # 1 oscillation
        det.update(5.0, 0.9)  # should have been pruned
        assert not det.ready()


class TestAxisReversals:
    def test_no_reversals_for_monotonic_signal(self):
        assert count_axis_reversals([0.0, 0.1, 0.2, 0.3], min_sweep=0.05) == 0

    def test_counts_reversals(self):
        # Goes up, down, up — 2 reversals.
        assert count_axis_reversals([0.0, 0.3, 0.0, 0.3], min_sweep=0.1) == 2

    def test_ignores_small_jitter(self):
        signal = [0.0, 0.01, -0.01, 0.02, 0.04, 0.02, 0.05]
        assert count_axis_reversals(signal, min_sweep=0.2) == 0


# ----------------------------------------------------------- full decoder


def _step(decoder, frames):
    last = None
    for obs in frames:
        last = decoder.update(obs)
        if last is not None:
            return last
    return last


class TestTemporalCommandDecoder:
    def test_beckon_fires_after_two_curls(self):
        decoder = TemporalCommandDecoder()
        # Two open <-> closed oscillations.
        frames = []
        t = 0.0
        for opn in (0.9, 0.1, 0.9, 0.1, 0.9):
            frames.append(_obs(t, gesture="OPEN_PALM" if opn > 0.5 else "FIST", openness=opn))
            t += 0.3
        cmd = _step(decoder, frames)
        assert cmd is not None
        assert cmd["command"] == "COME_CLOSER"

    def test_open_palm_hold_triggers_stop(self):
        decoder = TemporalCommandDecoder(stop_hold_sec=0.3)
        frames = [
            _obs(t, gesture="OPEN_PALM", openness=0.95, center_x=0.5)
            for t in (0.0, 0.1, 0.2, 0.35, 0.5)
        ]
        cmd = _step(decoder, frames)
        assert cmd is not None and cmd["command"] == "STOP"

    def test_palm_wave_triggers_tail_wag_not_stop(self):
        decoder = TemporalCommandDecoder()
        frames = []
        # Build a wave with clear reversals well above the stop motion tolerance.
        xs = [0.5, 0.65, 0.35, 0.65, 0.35, 0.65, 0.35, 0.65, 0.35]
        for i, x in enumerate(xs):
            frames.append(_obs(0.05 * i, gesture="OPEN_PALM", openness=0.95, center_x=x))
        cmd = _step(decoder, frames)
        assert cmd is not None
        assert cmd["command"] == "TAIL_WAG"

    def test_pointing_left_held_triggers_move(self):
        decoder = TemporalCommandDecoder(point_hold_sec=0.3)
        frames = [
            _obs(t, gesture="POINTING", openness=0.4,
                 pointing="LEFT", index_tip=(0.3, 0.5))
            for t in (0.0, 0.1, 0.25, 0.35, 0.5)
        ]
        cmd = _step(decoder, frames)
        assert cmd is not None and cmd["command"] == "MOVE_LEFT_FOOT"

    def test_pointing_right_held_triggers_move(self):
        decoder = TemporalCommandDecoder(point_hold_sec=0.3)
        frames = [
            _obs(t, gesture="POINTING", openness=0.4,
                 pointing="RIGHT", index_tip=(0.7, 0.5))
            for t in (0.0, 0.1, 0.25, 0.35, 0.5)
        ]
        cmd = _step(decoder, frames)
        assert cmd is not None and cmd["command"] == "MOVE_RIGHT_FOOT"

    def test_index_circle_triggers_rotate(self):
        decoder = TemporalCommandDecoder()
        frames = []
        steps = 24
        for i in range(steps):
            angle = 2.0 * math.pi * (i / float(steps - 1))
            tip = (0.5 + 0.10 * math.cos(angle), 0.5 + 0.10 * math.sin(angle))
            frames.append(_obs(0.07 * i, gesture="POINTING", openness=0.35,
                               pointing=None, index_tip=tip))
        cmd = _step(decoder, frames)
        assert cmd is not None and cmd["command"] == "ROTATE_ONCE"

    def test_lost_hand_resets_state(self):
        decoder = TemporalCommandDecoder()
        decoder.update(_obs(0.0, gesture="OPEN_PALM", openness=0.95))
        decoder.update({**_obs(0.1), "hand_visible": False})
        # First frame after recovery alone must not satisfy stop/beckon yet.
        result = decoder.update(_obs(0.2, gesture="OPEN_PALM", openness=0.95))
        assert result is None

    def test_command_cooldown(self):
        decoder = TemporalCommandDecoder(stop_hold_sec=0.2, command_cooldown_sec=2.0)
        # Trigger stop.
        for t in (0.0, 0.1, 0.25):
            cmd = decoder.update(_obs(t, gesture="OPEN_PALM", openness=0.95, center_x=0.5))
        assert cmd is not None and cmd["command"] == "STOP"
        # Holding palm should not retrigger immediately.
        second = decoder.update(_obs(0.5, gesture="OPEN_PALM", openness=0.95, center_x=0.5))
        assert second is None
