"""Distinctive Kobuki sound sequences for each gesture command.

The Kobuki base only exposes seven canned tones (ON, OFF, RECHARGE,
BUTTON, ERROR, CLEANINGSTART, CLEANINGEND). To still give each gesture a
recognisable, vaguely "pet-like" voice we layer those tones into short
sequences with deliberate pacing - the brain easily learns "ascending
chirp = come closer", "rapid double beep = stop", etc.

Each profile is a list of ``(delay_seconds, sound_value)`` tuples
relative to the start of the command.
"""

from __future__ import annotations

from typing import List, Tuple

try:
    from kobuki_ros_interfaces.msg import Sound  # type: ignore
    _ON = int(Sound.ON)
    _OFF = int(Sound.OFF)
    _RECHARGE = int(Sound.RECHARGE)
    _BUTTON = int(Sound.BUTTON)
    _ERROR = int(Sound.ERROR)
    _CLEAN_START = int(Sound.CLEANINGSTART)
    _CLEAN_END = int(Sound.CLEANINGEND)
    SOUNDS_AVAILABLE = True
except Exception:  # pragma: no cover - Kobuki not installed
    _ON, _OFF, _RECHARGE, _BUTTON, _ERROR, _CLEAN_START, _CLEAN_END = range(7)
    SOUNDS_AVAILABLE = False


SoundEvent = Tuple[float, int]


PROFILES: dict = {
    # Two short rising chirps + a friendly trill — "I am coming".
    "COME_CLOSER": [
        (0.00, _BUTTON),
        (0.18, _ON),
        (0.55, _CLEAN_START),
    ],
    # Quick double-beep ending in OFF — "halt".
    "STOP": [
        (0.00, _BUTTON),
        (0.12, _OFF),
    ],
    # Three ascending notes for the full-circle spin.
    "ROTATE_ONCE": [
        (0.00, _ON),
        (0.22, _BUTTON),
        (0.45, _CLEAN_END),
    ],
    # Soft notice + button — direction "to my left".
    "MOVE_LEFT_FOOT": [
        (0.00, _RECHARGE),
        (0.22, _BUTTON),
    ],
    # Mirror of left, ordered differently so the ear can tell them apart.
    "MOVE_RIGHT_FOOT": [
        (0.00, _BUTTON),
        (0.22, _RECHARGE),
    ],
    # A happy little jingle for tail-wagging.
    "TAIL_WAG": [
        (0.00, _CLEAN_START),
        (0.18, _BUTTON),
        (0.36, _CLEAN_END),
        (0.60, _BUTTON),
    ],
    # Optional idle wag — gentle button click only.
    "IDLE_WAG": [
        (0.00, _BUTTON),
    ],
    # Fallback if a command name is unknown.
    "UNKNOWN": [
        (0.00, _BUTTON),
    ],
}


def profile_for(command: str) -> List[SoundEvent]:
    return list(PROFILES.get(command, PROFILES["UNKNOWN"]))
