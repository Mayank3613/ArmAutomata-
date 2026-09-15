"""
Main control loop for the Gesture-Controlled 6-DOF Robotic Arm.

Uses TWO trackers for hierarchical body tracking:
  - Pose Landmarker → shoulder, elbow, wrist (arm joint angles)
  - Hand Landmarker → fingers (claw + wrist rotation + gesture state)

Tracking modes (automatic, based on what's visible):
  FULL_ARM  — shoulder + elbow + wrist in view → direct joint mirroring
  FOREARM   — elbow + wrist in view → elbow angle + estimated shoulder
  HAND_ONLY — only hand in view → IK from wrist position (fallback)

Motor layout:
  Ch 0 — Base Rotation      (MG996R)
  Ch 1 — Shoulder Extension  (MG996R)
  Ch 2 — Elbow Extension     (MG996R)
  Ch 3 — Wrist Rotation      (MG90S)   ← from hand roll
  Ch 4 — Wrist Extension     (MG90S)   ← keeps end-effector level
  Ch 5 — Claw                (SG90)    ← thumb-to-fingers distance

Usage:
    python main.py                           # defaults from config.py
    python main.py --port COM3               # override serial port
    python main.py --no-serial               # run without Arduino
"""

from __future__ import annotations

import argparse
import sys

import cv2

from config import (
    CLAW_OPEN_ANGLE,
    WRIST_EXT_NEUTRAL_ANGLE,
    WRIST_ROT_NEUTRAL_ANGLE,
    WINDOW_NAME,
    SERIAL_PORT,
    SERIAL_BAUD,
)
from hand_tracker import HandTracker
from body_tracker import BodyTracker
from gestures import classify_gesture
from arm_mapper import determine_tracking_mode, compute_robot_angles
from smoothing import AngleSmoother
from serial_link import SerialLink


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description="Gesture-controlled 6-DOF robotic arm host application."
    )
    parser.add_argument(
        "--port",
        type=str,
        default=SERIAL_PORT,
        help=f"Serial port for the Arduino (default: {SERIAL_PORT}).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=SERIAL_BAUD,
        help=f"Serial baud rate (default: {SERIAL_BAUD}).",
    )
    parser.add_argument(
        "--no-serial",
        action="store_true",
        help="Run without opening a serial connection (for testing).",
    )
    return parser.parse_args()


def main() -> None:
    """Entry point — runs the capture/control loop."""
    args = parse_args()

    # ---- Initialise components ----
    hand_tracker = HandTracker()
    body_tracker = BodyTracker()
    # 5 arm channels + 1 claw channel (smoothed together now)
    smoother = AngleSmoother(num_channels=6)
    link: SerialLink | None = None

    try:
        hand_tracker.open()
    except (RuntimeError, FileNotFoundError) as exc:
        print(f"[ERROR] {exc}")
        sys.exit(1)

    try:
        body_tracker.open()
    except FileNotFoundError as exc:
        print(f"[WARN] {exc}")
        print("[INFO] Continuing with hand-only tracking.")
        body_tracker = None  # type: ignore[assignment]

    if not args.no_serial:
        link = SerialLink(port=args.port, baud=args.baud)
        try:
            link.open()
        except RuntimeError as exc:
            print(f"[ERROR] {exc}")
            print("[INFO]  Continuing without serial — pass --no-serial to suppress.")
            link = None

    # ---- State ----
    prev_angles = (
        90.0,                       # base
        90.0,                       # shoulder
        90.0,                       # elbow
        float(WRIST_ROT_NEUTRAL_ANGLE),  # wrist rotation
        float(WRIST_EXT_NEUTRAL_ANGLE),  # wrist extension
        float(CLAW_OPEN_ANGLE),          # claw
    )

    # Colours for tracking mode display
    MODE_COLOURS = {
        "FULL_ARM": (0, 255, 0),    # green
        "FOREARM": (0, 200, 255),   # orange
        "HAND_ONLY": (255, 200, 0), # cyan
    }

    print(f"[main] Starting capture loop.  Press 'q' in the {WINDOW_NAME} window to quit.")

    try:
        while True:
            ok, frame = hand_tracker.read_frame()
            if not ok or frame is None:
                print("[WARN] Failed to read frame — retrying…")
                continue

            # ---- Detect body pose ----
            pose_landmarks = None
            if body_tracker is not None:
                pose_landmarks, frame = body_tracker.detect(frame)

            # ---- Detect hand landmarks ----
            hand_landmarks, frame = hand_tracker.detect(frame)

            # ---- Determine tracking mode ----
            mode, arm_side = determine_tracking_mode(pose_landmarks)

            # ---- Check gesture (FIST = freeze) ----
            gesture = "UNKNOWN"
            if hand_landmarks is not None:
                gesture = classify_gesture(hand_landmarks)

            # ---- Compute and send angles ----
            if gesture == "FIST":
                # Freeze — hold current pose
                cv2.putText(
                    frame, "FIST — HOLD", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                )

            elif gesture != "UNKNOWN" or (pose_landmarks is not None and hand_landmarks is None):
                # Compute robot angles from available landmarks
                angles = compute_robot_angles(
                    pose_landmarks, hand_landmarks,
                    mode, arm_side, prev_angles,
                )

                smoothed, changed = smoother.update(angles)
                prev_angles = smoothed

                # Convert to integers for serial
                int_angles = tuple(int(round(a)) for a in smoothed)

                if link is not None and changed:
                    link.send_angles(*int_angles)

                # ---- HUD overlay ----
                colour = MODE_COLOURS.get(mode, (200, 200, 200))
                cv2.putText(
                    frame, f"{mode} ({arm_side})", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2,
                )
                cv2.putText(
                    frame,
                    f"B:{int_angles[0]} S:{int_angles[1]} E:{int_angles[2]} "
                    f"WR:{int_angles[3]} WE:{int_angles[4]} C:{int_angles[5]}",
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                )

            else:
                cv2.putText(
                    frame, "NO TRACKING", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2,
                )

            # Show annotated feed
            cv2.imshow(WINDOW_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\n[main] Interrupted.")

    finally:
        hand_tracker.close()
        if body_tracker is not None:
            body_tracker.close()
        if link is not None:
            link.close()
        cv2.destroyAllWindows()
        print("[main] Shutdown complete.")


if __name__ == "__main__":
    main()
