"""
Workspace calibration: map MediaPipe normalised coordinates to real-world mm.

MediaPipe returns wrist position as (x, y, z) in [0, 1] (image-normalised).
This module linearly maps those values into the arm's physical workspace.
"""

from __future__ import annotations

from config import (
    WORKSPACE_X_MIN, WORKSPACE_X_MAX,
    WORKSPACE_Y_MIN, WORKSPACE_Y_MAX,
    WORKSPACE_Z_MIN, WORKSPACE_Z_MAX,
)


def map_to_workspace(
    norm_x: float,
    norm_y: float,
    norm_z: float,
) -> tuple[float, float, float]:
    """Map MediaPipe normalised wrist coordinates to workspace mm.

    Args:
        norm_x: Normalised X from MediaPipe (0 = left edge, 1 = right edge).
        norm_y: Normalised Y from MediaPipe (0 = top edge, 1 = bottom edge).
        norm_z: Normalised Z from MediaPipe (roughly proportional to hand
                size — smaller = farther from camera).

    Returns:
        (X_mm, Y_mm, Z_mm) in the arm's coordinate frame.

    Notes:
        * X is mirrored (MediaPipe X=0 → workspace right, X=1 → workspace left)
          so that moving your hand right moves the arm right from the operator's
          perspective.
        * Y is inverted (MediaPipe Y=0 = top → arm Y_max, Y=1 = bottom → Y_min).
        * Z mapping is a simple linear interpolation; real depth estimation from
          a monocular camera is approximate — tweak bounds to taste.
    """
    # Clamp inputs to [0, 1]
    nx = max(0.0, min(1.0, norm_x))
    ny = max(0.0, min(1.0, norm_y))
    nz = max(0.0, min(1.0, norm_z))

    # Mirror X so that user's right → arm's right
    x_mm = WORKSPACE_X_MAX - nx * (WORKSPACE_X_MAX - WORKSPACE_X_MIN)
    # Invert Y so that higher hand → higher arm
    y_mm = WORKSPACE_Y_MAX - ny * (WORKSPACE_Y_MAX - WORKSPACE_Y_MIN)
    # Z: 0 → near, 1 → far
    z_mm = WORKSPACE_Z_MIN + nz * (WORKSPACE_Z_MAX - WORKSPACE_Z_MIN)

    return x_mm, y_mm, z_mm
