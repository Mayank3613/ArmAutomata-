"""
Main control loop for the Gesture-Controlled Robotic Arm (wrist-only mode).

Uses ONLY the Hand Landmarker to control the 3 positional micro-servos:
  Ch 3 — Wrist Rotation  (MG90S, 180° Positional) ← from hand roll
  Ch 4 — Wrist Extension (MG90S, 180° Positional) ← from hand pitch
  Ch 5 — Claw            (SG90,  180° Positional)  ← thumb-to-fingers distance

MG996R continuous motors (Ch 0–2: base, shoulder, elbow) are DISABLED
(always sent speed 0) because the wrong motor variant is installed.

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
    JOINT_MIN_ANGLES,
    JOINT_MAX_ANGLES,
)
from hand_tracker import HandTracker
from gestures import (
    classify_gesture,
    compute_claw_angle,
    compute_wrist_rotation,
    compute_wrist_extension,
)
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
        description="Gesture-controlled robotic arm — wrist-only mode."
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


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp *value* to [lo, hi]."""
    return max(lo, min(hi, value))


def main() -> None:
    """Entry point — runs the capture/control loop."""
    args = parse_args()

    # ---- Initialise components ----
    hand_tracker = HandTracker()
    # 3 positional channels: WristRot, WristExt, Claw
    smoother = AngleSmoother(num_channels=3)
    link: SerialLink | None = None

    try:
        hand_tracker.open()
    except (RuntimeError, FileNotFoundError) as exc:
        print(f"[ERROR] {exc}")
        sys.exit(1)

    if not args.no_serial:
        link = SerialLink(port=args.port, baud=args.baud)
        try:
            link.open()
        except RuntimeError as exc:
            print(f"[ERROR] {exc}")
            print("[INFO]  Continuing without serial — pass --no-serial to suppress.")
            link = None

    # ---- State ----
    int_angles = (
        int(WRIST_ROT_NEUTRAL_ANGLE),
        int(WRIST_EXT_NEUTRAL_ANGLE),
        int(CLAW_OPEN_ANGLE),
    )

    print(f"[main] Wrist-only mode. Press 'q' in the {WINDOW_NAME} window to quit.")

    try:
        while True:
            ok, frame = hand_tracker.read_frame()
            if not ok or frame is None:
                print("[WARN] Failed to read frame — retrying…")
                continue

            # ---- Detect hand landmarks ----
            hand_landmarks, frame = hand_tracker.detect(frame)

            # ---- Check gesture (FIST = freeze) ----
            gesture = "UNKNOWN"
            if hand_landmarks is not None:
                gesture = classify_gesture(hand_landmarks)

            # ---- Compute and send angles ----
            if gesture == "FIST":
                # Freeze — hold current servo positions
                cv2.putText(
                    frame, "FIST — HOLD", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
                )
                cv2.putText(
                    frame,
                    f"WR:{int_angles[0]}  WE:{int_angles[1]}  Claw:{int_angles[2]}",
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1,
                )

            elif gesture == "TRACKING":
                # Compute all 3 positional angles from hand landmarks
                wrist_rot = compute_wrist_rotation(hand_landmarks)
                wrist_ext = compute_wrist_extension(hand_landmarks)
                claw = compute_claw_angle(hand_landmarks)

                # Clamp to joint limits (channels 3, 4, 5)
                wrist_rot = int(_clamp(wrist_rot, JOINT_MIN_ANGLES[3], JOINT_MAX_ANGLES[3]))
                wrist_ext = int(_clamp(wrist_ext, JOINT_MIN_ANGLES[4], JOINT_MAX_ANGLES[4]))
                claw = int(_clamp(claw, JOINT_MIN_ANGLES[5], JOINT_MAX_ANGLES[5]))

                # Smooth
                raw_angles = (float(wrist_rot), float(wrist_ext), float(claw))
                smoothed, changed = smoother.update(raw_angles)
                int_angles = tuple(int(round(a)) for a in smoothed)

                if link is not None and changed:
                    link.send_angles(*int_angles)

                # ---- HUD overlay ----
                cv2.putText(
                    frame, "TRACKING", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                )
                cv2.putText(
                    frame,
                    f"WR:{int_angles[0]}  WE:{int_angles[1]}  Claw:{int_angles[2]}",
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1,
                )

            else:
                # No hand detected
                cv2.putText(
                    frame, "NO HAND", (10, 40),
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
        if link is not None:
            link.close()
        cv2.destroyAllWindows()
        print("[main] Shutdown complete.")


if __name__ == "__main__":
    main()
