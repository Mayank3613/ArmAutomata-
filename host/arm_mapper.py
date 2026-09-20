"""
Arm angle mapper — computes robot joint angles from human body + hand landmarks.

Implements hierarchical tracking:
  1. FULL_ARM  — shoulder + elbow + wrist visible → direct joint-angle mirroring
  2. FOREARM   — only elbow + wrist visible → elbow angle + estimated shoulder
  3. HAND_ONLY — only hand visible → fall back to IK from wrist position

Claw and wrist rotation always come from hand landmarks when available.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from config import (
    SERVO_MIN_ANGLE,
    SERVO_MAX_ANGLE,
    WRIST_EXT_NEUTRAL_ANGLE,
    WRIST_ROT_NEUTRAL_ANGLE,
    CLAW_OPEN_ANGLE,
    JOINT_MIN_ANGLES,
    JOINT_MAX_ANGLES,
    BASE_SPEED_DEG_PER_SEC,
    SHOULDER_SPEED_DEG_PER_SEC,
    ELBOW_SPEED_DEG_PER_SEC,
    POSITIVE_SPEED_FACTOR,
    NEGATIVE_SPEED_FACTOR,
    BASE_DEADBAND_DEG,
    BASE_INVERT_DIRECTION,
    SHOULDER_INVERT_DIRECTION,
    ELBOW_INVERT_DIRECTION,
)
from body_tracker import (
    RIGHT_SHOULDER,
    RIGHT_ELBOW,
    RIGHT_WRIST,
    LEFT_SHOULDER,
    LEFT_ELBOW,
    LEFT_WRIST,
)
from gestures import compute_claw_angle, compute_wrist_rotation
from calibration import map_to_workspace
from kinematics import inverse_kinematics

# Minimum visibility score to consider a landmark "present"
_VIS_THRESHOLD = 0.5

# Scale factor for mapping horizontal wrist displacement to base rotation
_BASE_SCALE = 250.0  # [TUNE] degrees per unit of normalised x-displacement


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp *value* to [lo, hi]."""
    return max(lo, min(hi, value))


class ContinuousJointTracker:
    """Tracks virtual joint position and generates speed commands for continuous rotation.

    Applies to continuous-rotation MG996R motors (Ch 0 Base, Ch 1 Shoulder, Ch 2 Elbow).
    Since they cannot seek directly to an absolute angle, this tracker maintains an estimated
    virtual position over time based on commanded speed and joint velocity (deg/sec),
    and applies a proportional velocity controller to steer towards the target angle.
    """

    def __init__(
        self,
        initial_angle: float = 90.0,
        speed_deg_per_sec: float = BASE_SPEED_DEG_PER_SEC,
        deadband_deg: float = BASE_DEADBAND_DEG,
        invert: bool = False,
        min_angle: float = 0.0,
        max_angle: float = 180.0,
        positive_factor: float = POSITIVE_SPEED_FACTOR,
        negative_factor: float = NEGATIVE_SPEED_FACTOR,
    ) -> None:
        self.virtual_angle = float(initial_angle)
        self.speed_deg_per_sec = float(speed_deg_per_sec)
        self.deadband = float(deadband_deg)
        self.invert = bool(invert)
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)
        self.positive_factor = float(positive_factor)
        self.negative_factor = float(negative_factor)
        self.current_speed = 0

    def update(self, target_angle: float, dt: float) -> int:
        """Update virtual position using elapsed time dt and compute speed command.

        Args:
            target_angle: Desired joint angle in degrees.
            dt: Time elapsed since last update in seconds.

        Returns:
            speed: integer in [-100, 100] (0 = stopped).
        """
        target_angle = _clamp(target_angle, self.min_angle, self.max_angle)

        # Advance virtual position based on speed during interval dt
        if self.current_speed != 0 and dt > 0:
            effective_speed = -self.current_speed if self.invert else self.current_speed
            speed_dps = self.speed_deg_per_sec
            if effective_speed > 0 and self.positive_factor > 0:
                speed_dps = speed_dps / self.positive_factor
            elif effective_speed < 0 and self.negative_factor > 0:
                speed_dps = speed_dps / self.negative_factor
            movement = (effective_speed / 100.0) * speed_dps * dt
            self.virtual_angle = _clamp(
                self.virtual_angle + movement,
                self.min_angle,
                self.max_angle,
            )

        error = target_angle - self.virtual_angle

        # If within deadband, shut off motor
        if abs(error) <= self.deadband:
            self.current_speed = 0
            return 0

        # Proportional controller: full speed at >= 30° error
        kp = 100.0 / 30.0
        raw_speed = kp * error

        # Minimum kick threshold (25%) to overcome static gearbox friction
        direction = 1 if error > 0 else -1
        speed_mag = min(100.0, max(25.0, abs(raw_speed)))
        speed = int(round(direction * speed_mag))

        if self.invert:
            speed = -speed

        self.current_speed = speed
        return self.current_speed

    def stop(self) -> int:
        """Immediately command 0 speed (stop joint rotation)."""
        self.current_speed = 0
        return 0


# Backwards compatibility alias
BaseTracker = ContinuousJointTracker


def _angle_at_vertex(
    a_x: float, a_y: float,
    b_x: float, b_y: float,
    c_x: float, c_y: float,
) -> float:
    """Compute the angle (degrees) at vertex B in the triangle A-B-C.

    Uses 2D coordinates (image plane).  Returns a value in [0, 180].
    """
    ba_x, ba_y = a_x - b_x, a_y - b_y
    bc_x, bc_y = c_x - b_x, c_y - b_y

    dot = ba_x * bc_x + ba_y * bc_y
    mag_ba = math.sqrt(ba_x ** 2 + ba_y ** 2)
    mag_bc = math.sqrt(bc_x ** 2 + bc_y ** 2)

    if mag_ba * mag_bc < 1e-8:
        return 0.0

    cos_angle = _clamp(dot / (mag_ba * mag_bc), -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


def _shoulder_angle(shoulder: object, elbow: object) -> float:
    """Compute the shoulder servo angle from the upper arm's orientation.

    Measures the angle of the shoulder→elbow vector relative to straight down.

    Returns:
        Servo angle in degrees: 0 = arm hanging down, 90 = horizontal, 180 = up.
    """
    dx = elbow.x - shoulder.x
    dy = elbow.y - shoulder.y  # positive = downward in image coords

    # For the right arm, when raised to the right, dx is negative (leftward in image).
    # atan2(-dx, dy) gives 0 when hanging (dy>0, dx≈0) and +90 when horizontal right.
    angle = math.degrees(math.atan2(-dx, dy))
    return _clamp(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)


def _elbow_angle(shoulder: object, elbow: object, wrist: object) -> float:
    """Compute the elbow servo angle — interior angle at the elbow joint.

    Returns:
        Servo angle: 180 = fully extended, small = fully bent.
    """
    angle = _angle_at_vertex(
        shoulder.x, shoulder.y,
        elbow.x, elbow.y,
        wrist.x, wrist.y,
    )
    return _clamp(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)


def _base_from_pose(shoulder: object, wrist: object) -> float:
    """Compute base rotation from the horizontal displacement of the wrist.

    Maps the wrist position relative to the shoulder in the x-axis.
    Centre (wrist below shoulder) = 90°.
    """
    dx = wrist.x - shoulder.x
    # Invert: moving wrist to the right (negative dx in image for right arm)
    # should rotate the base correspondingly
    base = 90.0 - dx * _BASE_SCALE
    return _clamp(base, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)


def _pick_arm(pose_landmarks: List[object]) -> str:
    """Choose which arm to track based on visibility scores.

    Returns ``"right"`` or ``"left"``.
    """
    r_vis = min(
        pose_landmarks[RIGHT_SHOULDER].visibility,
        pose_landmarks[RIGHT_ELBOW].visibility,
    )
    l_vis = min(
        pose_landmarks[LEFT_SHOULDER].visibility,
        pose_landmarks[LEFT_ELBOW].visibility,
    )
    return "right" if r_vis >= l_vis else "left"


def _get_arm_landmarks(
    pose_landmarks: List[object], side: str
) -> Tuple[object, object, object]:
    """Return (shoulder, elbow, wrist) landmarks for the chosen side."""
    if side == "right":
        return (
            pose_landmarks[RIGHT_SHOULDER],
            pose_landmarks[RIGHT_ELBOW],
            pose_landmarks[RIGHT_WRIST],
        )
    return (
        pose_landmarks[LEFT_SHOULDER],
        pose_landmarks[LEFT_ELBOW],
        pose_landmarks[LEFT_WRIST],
    )


def determine_tracking_mode(
    pose_landmarks: Optional[List[object]],
) -> Tuple[str, str]:
    """Determine the tracking mode based on which body parts are visible.

    Returns:
        (mode, arm_side) where mode is one of:
        ``"FULL_ARM"``, ``"FOREARM"``, ``"HAND_ONLY"``.
    """
    if pose_landmarks is None:
        return "HAND_ONLY", "right"

    side = _pick_arm(pose_landmarks)
    shoulder, elbow, wrist = _get_arm_landmarks(pose_landmarks, side)

    shoulder_vis = shoulder.visibility > _VIS_THRESHOLD
    elbow_vis = elbow.visibility > _VIS_THRESHOLD
    wrist_vis = wrist.visibility > _VIS_THRESHOLD

    if shoulder_vis and elbow_vis:
        return "FULL_ARM", side
    if elbow_vis and wrist_vis:
        return "FOREARM", side
    return "HAND_ONLY", side


def compute_robot_angles(
    pose_landmarks: Optional[List[object]],
    hand_landmarks: Optional[List[object]],
    mode: str,
    arm_side: str,
    prev_angles: Tuple[float, float, float, float, float, float],
) -> Tuple[float, float, float, float, float, float]:
    """Compute all 6 robot joint angles from human landmarks.

    Args:
        pose_landmarks: 33 body landmarks from Pose Landmarker, or None.
        hand_landmarks: 21 hand landmarks from Hand Landmarker, or None.
        mode: One of ``"FULL_ARM"``, ``"FOREARM"``, ``"HAND_ONLY"``.
        arm_side: ``"right"`` or ``"left"``.
        prev_angles: Previous frame's angles (used when data is missing).

    Returns:
        (base, shoulder, elbow, wrist_rot, wrist_ext, claw) — all in degrees.
    """
    p_base, p_shoulder, p_elbow, p_wrist_rot, p_wrist_ext, p_claw = prev_angles

    base = p_base
    shoulder = p_shoulder
    elbow = p_elbow
    wrist_rot = p_wrist_rot
    wrist_ext = p_wrist_ext
    claw = p_claw

    # ---- Arm position (base, shoulder, elbow) ----
    if mode == "FULL_ARM" and pose_landmarks is not None:
        sh, el, wr = _get_arm_landmarks(pose_landmarks, arm_side)

        base = _base_from_pose(sh, wr)

        # Mirror shoulder angle for left arm
        raw_shoulder = _shoulder_angle(sh, el)
        if arm_side == "left":
            raw_shoulder = 180.0 - raw_shoulder
        shoulder = raw_shoulder

        elbow = _elbow_angle(sh, el, wr)

        # Wrist extension: keep end-effector level
        # Compensate for shoulder and elbow angles
        wrist_ext = _clamp(
            180.0 - shoulder - (180.0 - elbow) + 90.0,
            SERVO_MIN_ANGLE,
            SERVO_MAX_ANGLE,
        )

    elif mode == "FOREARM" and pose_landmarks is not None:
        sh, el, wr = _get_arm_landmarks(pose_landmarks, arm_side)

        # Elbow angle is reliable
        elbow = _elbow_angle(sh, el, wr)

        # Base from wrist horizontal position (use elbow as reference)
        base = _base_from_pose(el, wr)

        # Shoulder: estimate from forearm elevation
        forearm_dy = wr.y - el.y
        forearm_dx = wr.x - el.x
        forearm_angle = math.degrees(math.atan2(-forearm_dx, forearm_dy))
        shoulder = _clamp(forearm_angle + 45.0, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)

        wrist_ext = _clamp(
            180.0 - shoulder - (180.0 - elbow) + 90.0,
            SERVO_MIN_ANGLE,
            SERVO_MAX_ANGLE,
        )

    elif mode == "HAND_ONLY" and hand_landmarks is not None:
        # Fall back to IK from hand wrist position
        wrist_lm = hand_landmarks[0]
        x_mm, y_mm, z_mm = map_to_workspace(
            wrist_lm.x, wrist_lm.y, wrist_lm.z + 0.5
        )
        base, shoulder, elbow, wrist_ext = inverse_kinematics(x_mm, y_mm, z_mm)

    # ---- Wrist rotation + Claw (always from hand landmarks) ----
    if hand_landmarks is not None:
        wrist_rot = compute_wrist_rotation(hand_landmarks)
        claw = compute_claw_angle(hand_landmarks)

    # Clamp all angles to physical joint limits
    base = _clamp(base, JOINT_MIN_ANGLES[0], JOINT_MAX_ANGLES[0])
    shoulder = _clamp(shoulder, JOINT_MIN_ANGLES[1], JOINT_MAX_ANGLES[1])
    elbow = _clamp(elbow, JOINT_MIN_ANGLES[2], JOINT_MAX_ANGLES[2])
    wrist_rot = _clamp(wrist_rot, JOINT_MIN_ANGLES[3], JOINT_MAX_ANGLES[3])
    wrist_ext = _clamp(wrist_ext, JOINT_MIN_ANGLES[4], JOINT_MAX_ANGLES[4])
    claw = _clamp(claw, JOINT_MIN_ANGLES[5], JOINT_MAX_ANGLES[5])

    return base, shoulder, elbow, wrist_rot, wrist_ext, claw

