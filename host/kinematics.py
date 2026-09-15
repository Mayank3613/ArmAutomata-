"""
Geometric inverse kinematics (IK) and forward kinematics (FK)
for a 5-DOF robotic arm (base rotation + shoulder + elbow + wrist).

Coordinate frame
-----------------
* Origin at the base rotation axis, on the table surface.
* Z axis points **up**.
* The arm reaches outward in the XY horizontal plane;
  ``theta_base`` rotates in that plane (0° = +X axis).

Joint convention (all in degrees for the external API)
------------------------------------------------------
* ``theta_base``     — rotation about Z  (0–180, 90 = straight ahead)
* ``theta_shoulder`` — shoulder angle from horizontal  (0 = arm horizontal, 90 = straight up)
* ``theta_elbow``    — elbow "servo" angle  (0 = fully folded back on L1,
                       90 = L2 perpendicular to L1, 180 = fully extended)
* ``theta_wrist``    — wrist angle (computed to keep the end-effector level)
"""

from __future__ import annotations

import math
from typing import Tuple

from config import L1_MM, L2_MM, BASE_HEIGHT_MM, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp *value* to [lo, hi]."""
    return max(lo, min(hi, value))


def inverse_kinematics(
    x: float, y: float, z: float
) -> Tuple[float, float, float, float]:
    """Compute joint angles (degrees) to reach the target (x, y, z) in mm.

    Args:
        x: Target X in mm (left/right from base centre).
        y: Target Y in mm (forward/backward from base centre).
        z: Target Z in mm (height above table surface).

    Returns:
        (theta_base, theta_shoulder, theta_elbow, theta_wrist) in degrees,
        each clamped to [SERVO_MIN_ANGLE, SERVO_MAX_ANGLE].

    Raises:
        Nothing — unreachable targets are clamped to the workspace edge
        so the arm stretches toward the target without domain errors.
    """
    # --- Base rotation (top-down view) ---
    theta_base_rad = math.atan2(y, x)
    theta_base = math.degrees(theta_base_rad)
    theta_base = _clamp(theta_base, 0.0, 180.0)

    # --- Planar 2-link IK (in the vertical plane containing the target) ---
    r = math.sqrt(x * x + y * y)          # horizontal reach
    z_eff = z - BASE_HEIGHT_MM             # height relative to shoulder pivot

    d = math.sqrt(r * r + z_eff * z_eff)   # distance to target from shoulder

    # Clamp reach to avoid acos domain errors
    max_reach = L1_MM + L2_MM
    min_reach = abs(L1_MM - L2_MM)
    if d > max_reach:
        scale = max_reach / d
        r *= scale
        z_eff *= scale
        d = max_reach
    if d < min_reach:
        d = min_reach

    # --- Law of cosines for the interior elbow angle ---
    # gamma = interior angle at elbow between L1 and L2 (π = straight, 0 = folded)
    cos_gamma = (L1_MM * L1_MM + L2_MM * L2_MM - d * d) / (2.0 * L1_MM * L2_MM)
    cos_gamma = _clamp(cos_gamma, -1.0, 1.0)
    gamma = math.acos(cos_gamma)

    # Servo elbow angle: 180° = fully extended (gamma = π), 0° = fully folded
    theta_elbow = math.degrees(gamma)

    # --- Shoulder angle ---
    # alpha = angle at shoulder in the shoulder-target-elbow triangle
    cos_alpha = (L1_MM * L1_MM + d * d - L2_MM * L2_MM) / (2.0 * L1_MM * d)
    cos_alpha = _clamp(cos_alpha, -1.0, 1.0)
    alpha = math.acos(cos_alpha)

    # phi = elevation angle of the target from horizontal
    phi = math.atan2(z_eff, r)

    # shoulder angle = elevation to target + offset within triangle
    theta_shoulder = math.degrees(phi + alpha)

    # --- Wrist: keep end-effector level ---
    # The angle that L2 makes below horizontal is (theta_shoulder - (180 - theta_elbow))
    # = theta_shoulder - 180 + theta_elbow.
    # To make the end-effector horizontal, wrist must compensate:
    theta_wrist = 90.0 + (theta_shoulder - 180.0 + theta_elbow)
    # Simplified: theta_wrist = theta_shoulder + theta_elbow - 90.0

    # Clamp all to servo range
    theta_base = _clamp(theta_base, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
    theta_shoulder = _clamp(theta_shoulder, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
    theta_elbow = _clamp(theta_elbow, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
    theta_wrist = _clamp(theta_wrist, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)

    return theta_base, theta_shoulder, theta_elbow, theta_wrist


def forward_kinematics(
    theta_base: float, theta_shoulder: float, theta_elbow: float
) -> Tuple[float, float, float]:
    """Compute end-effector position (mm) from joint angles (degrees).

    Uses the **same** angle convention as ``inverse_kinematics``:
    * ``theta_shoulder`` is measured from horizontal (0 = horizontal forward).
    * ``theta_elbow`` is the interior angle at the elbow joint
      (180 = fully extended, 0 = fully folded).

    Args:
        theta_base: Base rotation angle in degrees.
        theta_shoulder: Shoulder angle in degrees.
        theta_elbow: Elbow angle in degrees.

    Returns:
        (x, y, z) position of the wrist point in mm.
    """
    base_rad = math.radians(theta_base)
    shoulder_rad = math.radians(theta_shoulder)
    elbow_interior_rad = math.radians(theta_elbow)  # interior angle at elbow

    # The angle L2 makes from horizontal:
    # After the shoulder lifts by shoulder_rad, L1 points at angle shoulder_rad
    # from horizontal. The elbow interior angle is between the continuation of
    # L1 and L2. So the absolute angle of L2 from horizontal is:
    #   shoulder_rad - (π - elbow_interior_rad)
    angle_L2 = shoulder_rad - (math.pi - elbow_interior_rad)

    # Planar reach and height
    r = L1_MM * math.cos(shoulder_rad) + L2_MM * math.cos(angle_L2)
    z = BASE_HEIGHT_MM + L1_MM * math.sin(shoulder_rad) + L2_MM * math.sin(angle_L2)

    x = r * math.cos(base_rad)
    y = r * math.sin(base_rad)

    return x, y, z


# ---------------------------------------------------------------------------
# Quick self-test — run with:  python kinematics.py
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    test_points = [
        (100.0, 100.0, 100.0),
        (0.0, 150.0, 60.0),
        (50.0, 50.0, 200.0),
        (-80.0, 120.0, 80.0),
        (0.0, 200.0, 60.0),   # near max reach
    ]

    tolerance_mm = 5.0
    all_ok = True

    print("IK → FK round-trip validation")
    print("=" * 60)
    for x, y, z in test_points:
        b, s, e, w = inverse_kinematics(x, y, z)
        fx, fy, fz = forward_kinematics(b, s, e)
        err = math.sqrt((fx - x) ** 2 + (fy - y) ** 2 + (fz - z) ** 2)
        status = "OK" if err < tolerance_mm else "FAIL"
        if status == "FAIL":
            all_ok = False
        print(
            f"  Target ({x:7.1f}, {y:7.1f}, {z:7.1f}) → "
            f"IK [B={b:5.1f} S={s:5.1f} E={e:5.1f} W={w:5.1f}] → "
            f"FK ({fx:7.1f}, {fy:7.1f}, {fz:7.1f})  "
            f"err={err:6.2f} mm  [{status}]"
        )
    print("=" * 60)
    print("ALL PASSED ✓" if all_ok else "SOME TESTS FAILED ✗")
    sys.exit(0 if all_ok else 1)
