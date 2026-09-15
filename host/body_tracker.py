"""
Body (pose) tracking module using MediaPipe Pose Landmarker (Tasks API).

Detects 33 body landmarks including shoulders, elbows, and wrists.
Used alongside HandTracker for full-arm gesture control.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Optional, List

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from config import CAMERA_WIDTH, CAMERA_HEIGHT

# Path to the pose landmarker model (next to this file by default)
_MODEL_PATH = os.path.join(os.path.dirname(__file__), "pose_landmarker.task")

# Pose landmark indices
NOSE = 0
LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_ELBOW = 13
RIGHT_ELBOW = 14
LEFT_WRIST = 15
RIGHT_WRIST = 16
LEFT_HIP = 23
RIGHT_HIP = 24

# Connections to draw for the arms (subset of full skeleton)
_ARM_CONNECTIONS = [
    (LEFT_SHOULDER, RIGHT_SHOULDER),    # shoulder line
    (LEFT_SHOULDER, LEFT_ELBOW),        # left upper arm
    (LEFT_ELBOW, LEFT_WRIST),           # left forearm
    (RIGHT_SHOULDER, RIGHT_ELBOW),      # right upper arm
    (RIGHT_ELBOW, RIGHT_WRIST),         # right forearm
    (LEFT_SHOULDER, LEFT_HIP),          # left torso
    (RIGHT_SHOULDER, RIGHT_HIP),        # right torso
]


@dataclass
class BodyTracker:
    """Wraps the MediaPipe Pose Landmarker for body tracking.

    Attributes:
        min_detection_confidence: Minimum confidence for pose detection.
        min_tracking_confidence: Minimum confidence for pose tracking.
        draw_landmarks: Whether to draw body landmarks on the frame.
        model_path: Path to the ``pose_landmarker.task`` model file.
    """

    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5
    draw_landmarks: bool = True
    model_path: str = _MODEL_PATH

    _landmarker: Optional[mp_vision.PoseLandmarker] = field(
        default=None, init=False, repr=False
    )
    _frame_timestamp_ms: int = field(default=0, init=False, repr=False)

    def open(self) -> None:
        """Initialise the Pose Landmarker.

        Raises:
            FileNotFoundError: If the model file is missing.
        """
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"Pose landmarker model not found at: {self.model_path}\n"
                "Download it with:\n"
                "  curl -L -o host/pose_landmarker.task "
                "https://storage.googleapis.com/mediapipe-models/"
                "pose_landmarker/pose_landmarker_lite/float16/1/"
                "pose_landmarker_lite.task"
            )

        base_options = mp_python.BaseOptions(
            model_asset_path=self.model_path,
            delegate=mp_python.BaseOptions.Delegate.CPU,
        )
        options = mp_vision.PoseLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )
        self._landmarker = mp_vision.PoseLandmarker.create_from_options(options)
        self._frame_timestamp_ms = 0

    def detect(
        self, frame: np.ndarray
    ) -> tuple[Optional[List[object]], np.ndarray]:
        """Detect pose landmarks in *frame*.

        Args:
            frame: BGR image from the camera.

        Returns:
            (landmarks, annotated_frame)
            *landmarks* is a list of 33 PoseLandmark objects or ``None``.
            Each landmark has ``.x``, ``.y``, ``.z``, ``.visibility``.
        """
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        self._frame_timestamp_ms += 33  # ~30 FPS
        result = self._landmarker.detect_for_video(
            mp_image, self._frame_timestamp_ms
        )

        landmarks = None
        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]

            if self.draw_landmarks:
                frame = self._draw(frame, landmarks)

        return landmarks, frame

    def _draw(self, frame: np.ndarray, landmarks: List[object]) -> np.ndarray:
        """Draw arm skeleton on the frame."""
        h, w, _ = frame.shape
        points = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]

        for start_idx, end_idx in _ARM_CONNECTIONS:
            # Only draw if both landmarks are visible enough
            if (landmarks[start_idx].visibility > 0.5 and
                    landmarks[end_idx].visibility > 0.5):
                cv2.line(frame, points[start_idx], points[end_idx],
                         (0, 200, 255), 3)

        # Draw landmark dots for shoulder, elbow, wrist
        for idx in [LEFT_SHOULDER, RIGHT_SHOULDER,
                    LEFT_ELBOW, RIGHT_ELBOW,
                    LEFT_WRIST, RIGHT_WRIST]:
            if landmarks[idx].visibility > 0.5:
                cv2.circle(frame, points[idx], 6, (0, 0, 255), -1)

        return frame

    def close(self) -> None:
        """Release MediaPipe resources."""
        if self._landmarker is not None:
            self._landmarker.close()

    def __enter__(self) -> "BodyTracker":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
