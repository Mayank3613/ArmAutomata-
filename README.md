# Gesture-Controlled 6-DOF Robotic Arm

A real-time gesture-controlled robotic arm system. A USB webcam captures your hand; a Python host detects hand landmarks, classifies gestures, computes inverse kinematics, and streams joint angles to an Arduino Uno over USB serial. The Arduino drives six servos via a PCA9685 PWM board.

## Architecture

```
Webcam → MediaPipe Hands → Gesture + Claw + Wrist Rotation
       → IK Solver → Smoother → Serial → Arduino → PCA9685 → Servos
```

### Motor Layout

| PCA9685 Ch | Joint              | Servo  | Control Source                    |
|------------|--------------------|--------|-----------------------------------|
| 0          | Base Rotation      | MG996R | Hand X/Y position (atan2)         |
| 1          | Shoulder Extension | MG996R | IK from hand position             |
| 2          | Elbow Extension    | MG996R | IK from hand position             |
| 3          | Wrist Rotation     | MG90S  | Hand roll angle in camera frame   |
| 4          | Wrist Extension    | MG90S  | IK (keeps end-effector level)     |
| 5          | Claw (2-finger)    | SG90S  | Thumb-to-fingers distance         |

### Gesture Control

| Gesture       | Action                                                   |
|---------------|----------------------------------------------------------|
| **Open hand** | Arm follows hand position; claw opens proportionally      |
| **Pinch/Grab**| Arm follows hand position; claw closes proportionally     |
| **Fist**      | Freeze — arm holds current pose                           |

**Claw control**: The claw angle is mapped **proportionally** from the distance between your thumb tip and the average position of your other four fingertips. Move your thumb toward your fingers to close the claw; spread them apart to open it.

**Wrist rotation**: Tilt/roll your hand left or right to rotate the wrist servo.

## Hardware

### Components

- **Arduino Uno** (or compatible)
- **PCA9685** 16-channel PWM driver board
- **MG996R** × 3 — base rotation (ch 0), shoulder (ch 1), elbow (ch 2)
- **MG90S** × 2 — wrist rotation (ch 3), wrist extension (ch 4)
- **SG90S** × 1 — claw / gripper (ch 5)
- **USB webcam** (any UVC-compatible camera)
- **5V power supply** (≥3A recommended) for the servos

### Wiring

```
Arduino Uno        PCA9685
-----------        -------
A4  (SDA)  ──────  SDA
A5  (SCL)  ──────  SCL
GND        ──────  GND
5V         ──────  VCC  (logic power)

External 5V PSU    PCA9685
(≥3A)              -------
+5V        ──────  V+   (servo power — screw terminal)
GND        ──────  GND  (screw terminal)
```

> ⚠️ **Do NOT power the servos from the Arduino's 5V pin.** Use a separate supply (5–6V, ≥3A) connected to the PCA9685 V+ screw terminal.

## Software Setup

### 1. Python Host (requires Python 3.10–3.12)

```bash
cd ~/Projects/RoboArm
python3.12 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 2. Download the Hand Landmarker Model

```bash
curl -L -o host/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

### 3. Arduino Firmware

1. Open `firmware/arm_firmware/arm_firmware.ino` in the Arduino IDE.
2. Install the **Adafruit PWM Servo Driver Library** via Library Manager.
3. Select **Board → Arduino Uno**, choose the correct port, and upload.

### 4. Find Your Serial Port

| OS      | Command                              | Typical result        |
|---------|--------------------------------------|-----------------------|
| macOS   | `ls /dev/tty.usb*`                   | `/dev/tty.usbmodem…`  |
| Linux   | `ls /dev/ttyACM* /dev/ttyUSB*`       | `/dev/ttyACM0`        |
| Windows | Device Manager → Ports               | `COM3`                |

Update `SERIAL_PORT` in `host/config.py`, or pass it at runtime.

## Running

```bash
cd ~/Projects/RoboArm/host
source ../venv/bin/activate

python3 main.py --no-serial          # test without Arduino
python3 main.py --port COM3          # with Arduino on COM3
python3 main.py --port /dev/ttyACM0  # with Arduino on Linux
```

- An OpenCV window shows the annotated camera feed with gesture labels and joint angles.
- Press **`q`** in the window to quit.

### IK Validation

```bash
cd host
python3 kinematics.py
```

## Project Structure

```
RoboArm/
├── README.md
├── requirements.txt
├── host/
│   ├── config.py          # All tunable constants
│   ├── hand_tracker.py    # MediaPipe hand detection (Tasks API)
│   ├── gestures.py        # Gesture classification + claw + wrist rotation
│   ├── calibration.py     # Normalised coords → workspace mm
│   ├── kinematics.py      # Inverse & forward kinematics
│   ├── smoothing.py       # EMA + deadband filter
│   ├── serial_link.py     # Arduino serial communication (6 angles)
│   ├── main.py            # Main control loop
│   └── hand_landmarker.task  # MediaPipe model (downloaded)
└── firmware/
    └── arm_firmware/
        └── arm_firmware.ino   # Arduino sketch (6 servos)
```

## Configuration

All tunable constants live in [`host/config.py`](host/config.py). Key settings:

| Constant             | Default | Description                                  |
|----------------------|---------|----------------------------------------------|
| `L1_MM` / `L2_MM`   | 105/98  | Link lengths in mm                           |
| `BASE_HEIGHT_MM`     | 60      | Base/shoulder pivot height in mm             |
| `SMOOTHING_ALPHA`    | 0.3     | EMA weight (0 = max smooth, 1 = none)        |
| `DEADBAND_DEGREES`   | 2.0     | Min angle change to retransmit               |
| `CLAW_DIST_MIN`      | 0.05    | Thumb-finger dist → claw fully closed        |
| `CLAW_DIST_MAX`      | 0.35    | Thumb-finger dist → claw fully open          |
| `CLAW_OPEN_ANGLE`    | 90      | Servo angle for claw open                    |
| `CLAW_CLOSED_ANGLE`  | 30      | Servo angle for claw closed                  |
| `SERIAL_PORT`        | /dev/ttyUSB0 | Default serial port                     |

## Calibration Notes

- **Servo pulse bounds** (`SERVOMIN`/`SERVOMAX` in the `.ino`): Defaults (150/600) work for most servos. Fine-tune per servo.
- **Claw thresholds** (`CLAW_DIST_MIN`/`CLAW_DIST_MAX`): Adjust based on your hand size and camera distance. Lower values = more sensitive.
- **Workspace bounds**: Adjust `WORKSPACE_*` constants to match your physical setup.
- **Wrist rotation**: If the wrist rotation direction is inverted, negate the angle in `compute_wrist_rotation()` in `gestures.py`.

## License

MIT
