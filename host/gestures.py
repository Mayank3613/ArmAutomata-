"""
Gesture classification and continuous claw/wrist-rotation extraction
from MediaPipe hand landmarks.

Gesture classification uses geometric distances (no trained model).
Claw angle is computed proportionally from thumb-to-finger distance.
Wrist rotation is computed from the hand's roll angle in the camera frame.
"""

from __future__ import annotations

import math
from typing import List, Tuple

from config import (
    PINCH_THRESHOLD,
    FIST_CURL_THRESHOLD,
    OPEN_PALM_THRESHOLD,
    CLAW_DIST_MIN,
    CLAW_DIST_MAX,
    CLAW_OPEN_ANGLE,
    CLAW_CLOSED_ANGLE,
    WRIST_ROT_NEUTRAL_ANGLE,
    WRIST_EXT_NEUTRAL_ANGLE,
)


# MediaPipe hand landmark indices
_WRIST = 0
_THUMB_TIP = 4
_INDEX_TIP = 8
_MIDDLE_TIP = 12
_RING_TIP = 16
_PINKY_TIP = 20

_THUMB_MCP = 2
_INDEX_MCP = 5
_MIDDLE_MCP = 9
_RING_MCP = 13
_PINKY_MCP = 17

_FINGER_TIPS = [_INDEX_TIP, _MIDDLE_TIP, _RING_TIP, _PINKY_TIP]
_FINGER_MCPS = [_INDEX_MCP, _MIDDLE_MCP, _RING_MCP, _PINKY_MCP]


def _dist(lm_a: object, lm_b: object) -> float:
    """Euclidean distance between two NormalizedLandmark objects."""
    return math.sqrt(
        (lm_a.x - lm_b.x) ** 2
        + (lm_a.y - lm_b.y) ** 2
        + (lm_a.z - lm_b.z) ** 2
    )


def _dist_2d(lm_a: object, lm_b: object) -> float:
    """2D Euclidean distance (x, y only) between two landmarks."""
    return math.sqrt((lm_a.x - lm_b.x) ** 2 + (lm_a.y - lm_b.y) ** 2)


def _palm_size(landmarks: List[object]) -> float:
    """Return the wrist-to-middle-MCP distance as a normalisation reference."""
    return _dist(landmarks[_WRIST], landmarks[_MIDDLE_MCP])


def classify_gesture(landmarks: List[object]) -> str:
    """Classify a hand gesture from 21 MediaPipe landmarks.

    Args:
        landmarks: List/sequence of 21 ``NormalizedLandmark`` objects.

    Returns:
        One of ``"TRACKING"``, ``"FIST"``, or ``"UNKNOWN"``.

        - ``"TRACKING"`` — hand is open or pinching; arm follows the hand
          and the claw angle is set proportionally.
        - ``"FIST"`` — all fingers curled; arm holds current pose.
        - ``"UNKNOWN"`` — ambiguous; frame discarded.
    """
    palm = _palm_size(landmarks)
    if palm < 1e-6:
        return "UNKNOWN"

    # Per-finger curl ratios (tip-to-MCP distance normalised by palm size)
    curls = [
        _dist(landmarks[tip], landmarks[mcp]) / palm
        for tip, mcp in zip(_FINGER_TIPS, _FINGER_MCPS)
    ]

    if all(c < FIST_CURL_THRESHOLD for c in curls):
        return "FIST"

    # Any other pose with at least some fingers extended → track the hand
    return "TRACKING"


def compute_claw_angle(landmarks: List[object]) -> int:
    """Compute a proportional claw angle from thumb-to-finger distance.

    The claw maps the distance between the **thumb tip** and the average
    position of the four **fingertips** (index, middle, ring, pinky).

    - Thumb close to fingers → claw closed (``CLAW_CLOSED_ANGLE``)
    - Thumb far from fingers  → claw open  (``CLAW_OPEN_ANGLE``)

    Args:
        landmarks: 21 MediaPipe NormalizedLandmark objects.

    Returns:
        Claw servo angle as an integer (clamped to the configured range).
    """
    palm = _palm_size(landmarks)
    if palm < 1e-6:
        return CLAW_OPEN_ANGLE

    # Average position of the four fingertips (the "rest of the hand")
    avg_x = sum(landmarks[t].x for t in _FINGER_TIPS) / 4.0
    avg_y = sum(landmarks[t].y for t in _FINGER_TIPS) / 4.0
    avg_z = sum(landmarks[t].z for t in _FINGER_TIPS) / 4.0

    # Distance from thumb tip to that average, normalised by palm size
    thumb = landmarks[_THUMB_TIP]
    dx = thumb.x - avg_x
    dy = thumb.y - avg_y
    dz = thumb.z - avg_z
    dist_norm = math.sqrt(dx * dx + dy * dy + dz * dz) / palm

    # Linear interpolation: CLAW_DIST_MIN → closed, CLAW_DIST_MAX → open
    t = (dist_norm - CLAW_DIST_MIN) / (CLAW_DIST_MAX - CLAW_DIST_MIN)
    t = max(0.0, min(1.0, t))  # clamp to [0, 1]

    angle = CLAW_CLOSED_ANGLE + t * (CLAW_OPEN_ANGLE - CLAW_CLOSED_ANGLE)
    return int(round(angle))


def compute_wrist_rotation(landmarks: List[object]) -> int:
    """Compute wrist rotation angle from the hand's roll in the camera frame.

    Uses the angle of the vector from the wrist (landmark 0) to the
    middle-finger MCP (landmark 9) projected onto the image plane.

    - Hand vertical (fingers up)   → 90° (neutral)
    - Hand tilted clockwise        → <90°
    - Hand tilted counter-clockwise → >90°

    Args:
        landmarks: 21 MediaPipe NormalizedLandmark objects.

    Returns:
        Wrist rotation servo angle as an integer (0–180).
    """
    wrist = landmarks[_WRIST]
    mcp = landmarks[_MIDDLE_MCP]

    # Vector from wrist to middle MCP in image coordinates
    dx = mcp.x - wrist.x
    dy = mcp.y - wrist.y  # Note: MediaPipe Y increases downward

    # atan2 gives the angle from the positive X axis; we want the angle
    # from the vertical (Y-up). In image coords, "straight up" means
    # dy is large negative (mcp above wrist) and dx ≈ 0.
    angle_rad = math.atan2(dx, -dy)  # 0 when fingers point straight up
    angle_deg = math.degrees(angle_rad)

    # Map to servo range: 0° hand-roll → 90° servo (neutral)
    servo_angle = WRIST_ROT_NEUTRAL_ANGLE + angle_deg
    return int(round(max(0, min(180, servo_angle))))


def compute_wrist_extension(landmarks: List[object]) -> int:
    """Compute wrist extension (pitch) angle from the hand's tilt in the camera frame.

    Uses the angle of the vector from the wrist (landmark 0) to the
    middle-finger MCP (landmark 9) projected onto the vertical (Y) axis.

    - Hand pointing forward (fingers up)   → 90° (neutral)
    - Hand tilted upward (fingers away)     → >90°
    - Hand tilted downward (fingers toward) → <90°

    Args:
        landmarks: 21 MediaPipe NormalizedLandmark objects.

    Returns:
        Wrist extension servo angle as an integer (0–180).
    """
    wrist = landmarks[_WRIST]
    mcp = landmarks[_MIDDLE_MCP]

    # Vector from wrist to middle MCP
    dx = mcp.x - wrist.x
    dy = mcp.y - wrist.y  # MediaPipe Y increases downward
    dz = mcp.z - wrist.z  # Z: negative = closer to camera

    # Pitch: angle of the wrist→MCP vector in the vertical plane.
    # We use atan2(dz, -dy) so that:
    #   fingers pointing up (-dy large)  → ~0° → servo neutral
    #   fingers tilting toward camera (+dz) → positive angle → servo > neutral
    #   fingers tilting away (-dz) → negative angle → servo < neutral
    angle_rad = math.atan2(-dz, -dy)
    angle_deg = math.degrees(angle_rad)

    servo_angle = WRIST_EXT_NEUTRAL_ANGLE + angle_deg
    return int(round(max(0, min(180, servo_angle))))

