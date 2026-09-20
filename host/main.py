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
  Ch 0 — Base Rotation       (MG996R, 360° Continuous)
  Ch 1 — Shoulder Extension  (MG996R, 360° Continuous)
  Ch 2 — Elbow Extension     (MG996R, 360° Continuous)
  Ch 3 — Wrist Rotation      (MG90S, 180° Positional)   ← from hand roll
  Ch 4 — Wrist Extension     (MG90S, 180° Positional)   ← keeps end-effector level
  Ch 5 — Claw                (SG90, 180° Positional)    ← thumb-to-fingers distance

Usage:
    python main.py                           # defaults from config.py
    python main.py --port COM3               # override serial port
    python main.py --no-serial               # run without Arduino
"""

from __future__ import annotations

import argparse
import sys
import time

import cv2

from config import (
    CLAW_OPEN_ANGLE,
    WRIST_EXT_NEUTRAL_ANGLE,
    WRIST_ROT_NEUTRAL_ANGLE,
    WINDOW_NAME,
    SERIAL_PORT,
    SERIAL_BAUD,
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
from hand_tracker import HandTracker
from body_tracker import BodyTracker
from gestures import classify_gesture
from arm_mapper import determine_tracking_mode, compute_robot_angles, ContinuousJointTracker
from smoothing import AngleSmoother
from serial_link import SerialLink


def list_serial_ports() -> None:
    """Print all available serial ports and exit."""
    from serial.tools.list_ports import comports

    ports = sorted(comports(), key=lambda p: p.device)
    if not ports:
        print("No serial ports found. Is the Arduino plugged in?")
    else:
        print("Available serial ports:")
        for p in ports:
            print(f"  {p.device:20s}  {p.description}")
    sys.exit(0)


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
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List available serial ports and exit.",
    )
    args = parser.parse_args()
    if args.list_ports:
        list_serial_ports()
    return args


def main() -> None:
    """Entry point — runs the capture/control loop."""
    args = parse_args()

    # ---- Initialise components ----
    hand_tracker = HandTracker()
    body_tracker = BodyTracker()
    base_tracker = ContinuousJointTracker(
        initial_angle=90.0,
        speed_deg_per_sec=BASE_SPEED_DEG_PER_SEC,
        deadband_deg=BASE_DEADBAND_DEG,
        invert=BASE_INVERT_DIRECTION,
        min_angle=float(JOINT_MIN_ANGLES[0]),
        max_angle=float(JOINT_MAX_ANGLES[0]),
    )
    shoulder_tracker = ContinuousJointTracker(
        initial_angle=90.0,
        speed_deg_per_sec=SHOULDER_SPEED_DEG_PER_SEC,
        deadband_deg=BASE_DEADBAND_DEG,
        invert=SHOULDER_INVERT_DIRECTION,
        min_angle=float(JOINT_MIN_ANGLES[1]),
        max_angle=float(JOINT_MAX_ANGLES[1]),
    )
    elbow_tracker = ContinuousJointTracker(
        initial_angle=90.0,
        speed_deg_per_sec=ELBOW_SPEED_DEG_PER_SEC,
        deadband_deg=BASE_DEADBAND_DEG,
        invert=ELBOW_INVERT_DIRECTION,
        min_angle=float(JOINT_MIN_ANGLES[2]),
        max_angle=float(JOINT_MAX_ANGLES[2]),
    )
    # 3 positional channels (WristRot, WristExt, Claw) smoothed together
    smoother = AngleSmoother(num_channels=3)
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
        90.0,                            # base
        90.0,                            # shoulder
        90.0,                            # elbow
        float(WRIST_ROT_NEUTRAL_ANGLE),  # wrist rotation
        float(WRIST_EXT_NEUTRAL_ANGLE),  # wrist extension
        float(CLAW_OPEN_ANGLE),          # claw
    )
    last_sent_speeds = (0, 0, 0)
    int_micro = (
        int(WRIST_ROT_NEUTRAL_ANGLE),
        int(WRIST_EXT_NEUTRAL_ANGLE),
        int(CLAW_OPEN_ANGLE),
    )
    last_time = time.time()

    # Colours for tracking mode display
    MODE_COLOURS = {
        "FULL_ARM": (0, 255, 0),    # green
        "FOREARM": (0, 200, 255),   # orange
        "HAND_ONLY": (255, 200, 0), # cyan
    }

    print(f"[main] Starting capture loop.  Press 'q' in the {WINDOW_NAME} window to quit.")

    try:
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

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

            # ---- Compute and send angles / continuous speeds ----
            if gesture == "FIST":
                # Freeze — hold current pose, immediately halt all continuous motors
                b_spd = base_tracker.stop()
                s_spd = shoulder_tracker.stop()
                e_spd = elbow_tracker.stop()
                current_speeds = (b_spd, s_spd, e_spd)
                if link is not None and any(s != 0 for s in last_sent_speeds):
                    link.send_angles(0, 0, 0, *int_micro)
                    last_sent_speeds = (0, 0, 0)

                cv2.putText(
                    frame, "FIST — HOLD", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                )
                cv2.putText(
                    frame,
                    f"Spd:[{b_spd},{s_spd},{e_spd}] "
                    f"Pos:[{int(round(base_tracker.virtual_angle))},{int(round(shoulder_tracker.virtual_angle))},{int(round(elbow_tracker.virtual_angle))}] "
                    f"WR:{int_micro[0]} WE:{int_micro[1]} C:{int_micro[2]}",
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1,
                )

            elif gesture != "UNKNOWN" or (pose_landmarks is not None and hand_landmarks is None):
                # Compute robot angles from available landmarks
                angles = compute_robot_angles(
                    pose_landmarks, hand_landmarks,
                    mode, arm_side, prev_angles,
                )
                prev_angles = angles

                # Smooth the 3 positional micro-servos (WristRot, WristExt, Claw)
                smoothed_micro, micro_changed = smoother.update(angles[3:])
                int_micro = tuple(int(round(a)) for a in smoothed_micro)

                # ContinuousJointTrackers calculate real-time speeds for continuous MG996R motors (Ch 0, 1, 2)
                b_spd = base_tracker.update(angles[0], dt)
                s_spd = shoulder_tracker.update(angles[1], dt)
                e_spd = elbow_tracker.update(angles[2], dt)
                current_speeds = (b_spd, s_spd, e_spd)
                speeds_changed = (current_speeds != last_sent_speeds)

                if link is not None and (micro_changed or speeds_changed):
                    link.send_angles(b_spd, s_spd, e_spd, *int_micro)
                    last_sent_speeds = current_speeds

                # ---- HUD overlay ----
                colour = MODE_COLOURS.get(mode, (200, 200, 200))
                cv2.putText(
                    frame, f"{mode} ({arm_side})", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2,
                )
                cv2.putText(
                    frame,
                    f"Spd:[{b_spd},{s_spd},{e_spd}] "
                    f"Pos:[{int(round(base_tracker.virtual_angle))},{int(round(shoulder_tracker.virtual_angle))},{int(round(elbow_tracker.virtual_angle))}] "
                    f"WR:{int_micro[0]} WE:{int_micro[1]} C:{int_micro[2]}",
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                )

            else:
                # Loss of tracking: emergency halt all continuous motors immediately
                b_spd = base_tracker.stop()
                s_spd = shoulder_tracker.stop()
                e_spd = elbow_tracker.stop()
                if link is not None and any(s != 0 for s in last_sent_speeds):
                    link.send_angles(0, 0, 0, *int_micro)
                    last_sent_speeds = (0, 0, 0)

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
