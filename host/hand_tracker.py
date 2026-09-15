"""
Hand tracking module using MediaPipe Hand Landmarker (Tasks API).

Compatible with MediaPipe ≥ 0.10.14 / 1.x which removed the legacy
``mp.solutions.hands`` interface.

Detects 21 hand landmarks from a webcam feed and optionally
draws them on the frame for debugging.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from config import CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT

# Path to the hand landmarker model (next to this file by default)
_MODEL_PATH = os.path.join(os.path.dirname(__file__), "hand_landmarker.task")

# Hand skeleton connections (pairs of landmark indices) — hardcoded because
# mediapipe.solutions is removed in MediaPipe 1.x.
_HAND_CONNECTIONS = frozenset([
    (0, 1), (1, 2), (2, 3), (3, 4),       # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),       # index
    (0, 9), (9, 10), (10, 11), (11, 12),   # middle  (0→9 via 5→9)
    (0, 13), (13, 14), (14, 15), (15, 16), # ring    (0→13 via 9→13)
    (0, 17), (17, 18), (18, 19), (19, 20), # pinky
    (5, 9), (9, 13), (13, 17),             # palm cross-connections
])


def _draw_landmarks_on_image(
    frame: np.ndarray,
    detection_result: mp_vision.HandLandmarkerResult,
) -> np.ndarray:
    """Draw hand landmarks and connections on the frame."""
    if not detection_result.hand_landmarks:
        return frame

    h, w, _ = frame.shape
    for hand_landmarks in detection_result.hand_landmarks:
        # Convert normalised landmarks to pixel coords
        points = [
            (int(lm.x * w), int(lm.y * h)) for lm in hand_landmarks
        ]

        # Draw connections
        for start_idx, end_idx in _HAND_CONNECTIONS:
            cv2.line(frame, points[start_idx], points[end_idx], (0, 255, 0), 2)

        # Draw landmark dots
        for px, py in points:
            cv2.circle(frame, (px, py), 4, (255, 0, 0), -1)

    return frame


@dataclass
class HandTracker:
    """Wraps the MediaPipe Hand Landmarker (Tasks API) for single-hand detection.

    Attributes:
        camera_index: OpenCV capture device index.
        max_num_hands: Maximum hands to detect (kept at 1).
        min_detection_confidence: MediaPipe detection confidence threshold.
        min_tracking_confidence: MediaPipe tracking confidence threshold.
        draw_landmarks: Whether to annotate the frame with landmarks.
        model_path: Path to the ``hand_landmarker.task`` model file.
    """

    camera_index: int = CAMERA_INDEX
    max_num_hands: int = 1
    min_detection_confidence: float = 0.7
    min_tracking_confidence: float = 0.5
    draw_landmarks: bool = True
    model_path: str = _MODEL_PATH

    # --- private, set in open() ---
    _cap: Optional[cv2.VideoCapture] = field(default=None, init=False, repr=False)
    _landmarker: Optional[mp_vision.HandLandmarker] = field(
        default=None, init=False, repr=False
    )
    _frame_timestamp_ms: int = field(default=0, init=False, repr=False)

    def open(self) -> None:
        """Open the camera and initialise the Hand Landmarker.

        Raises:
            RuntimeError: If the camera cannot be opened.
            FileNotFoundError: If the model file is missing.
        """
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f"Hand landmarker model not found at: {self.model_path}\n"
                "Download it with:\n"
                "  curl -L -o host/hand_landmarker.task "
                "https://storage.googleapis.com/mediapipe-models/"
                "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
            )

        self._cap = cv2.VideoCapture(self.camera_index)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open camera at index {self.camera_index}. "
                "Check that a webcam is connected."
            )
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        base_options = mp_python.BaseOptions(
            model_asset_path=self.model_path,
            delegate=mp_python.BaseOptions.Delegate.CPU,
        )
        options = mp_vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.VIDEO,
            num_hands=self.max_num_hands,
            min_hand_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )
        self._landmarker = mp_vision.HandLandmarker.create_from_options(options)
        self._frame_timestamp_ms = 0

    def read_frame(self) -> tuple[bool, Optional[np.ndarray]]:
        """Read a raw BGR frame from the camera.

        Returns:
            (success, frame) — *frame* is ``None`` when the read fails.
        """
        if self._cap is None:
            return False, None
        ret, frame = self._cap.read()
        if not ret:
            return False, None
        return True, frame

    def detect(
        self, frame: np.ndarray
    ) -> tuple[Optional[list[object]], np.ndarray]:
        """Detect hand landmarks in *frame*.

        Args:
            frame: BGR image (e.g. from ``read_frame``).

        Returns:
            (landmarks, annotated_frame)
            *landmarks* is a list of 21 landmark objects (each with ``.x``,
            ``.y``, ``.z`` attributes) or ``None`` when no hand is found.
            *annotated_frame* has landmarks drawn when ``draw_landmarks`` is True.
        """
        # Convert BGR to RGB for MediaPipe
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

        # Monotonically increasing timestamp (required for VIDEO mode)
        self._frame_timestamp_ms += 33  # ~30 FPS
        result = self._landmarker.detect_for_video(
            mp_image, self._frame_timestamp_ms
        )

        landmarks = None
        if result.hand_landmarks:
            # Return the first hand's landmarks
            landmarks = result.hand_landmarks[0]

            if self.draw_landmarks:
                frame = _draw_landmarks_on_image(frame, result)

        return landmarks, frame

    def close(self) -> None:
        """Release the camera and MediaPipe resources."""
        if self._cap is not None:
            self._cap.release()
        if self._landmarker is not None:
            self._landmarker.close()

    # -- context-manager support --
    def __enter__(self) -> "HandTracker":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
