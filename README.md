# ArmAutomata — Gesture-Controlled 6-DOF Robotic Arm

A real-time gesture-controlled robotic arm that uses computer vision to track your hand and mirror its movements onto physical servos. A webcam captures your hand; a Python host uses **MediaPipe Hand Landmarker** (a pre-trained ML model) to detect 21 hand landmarks in real-time, computes servo angles from hand geometry, and streams them to an Arduino Uno over USB serial. The Arduino drives servos via a PCA9685 PWM board.

> **Current Mode: Wrist-Only** — The MG996R motors (Ch 0–2) are continuous rotation variants and are **permanently disabled**. Only the 3 positional micro-servos (wrist rotation, wrist extension, claw) are vision-controlled.

## Architecture

```
Webcam → Hand Landmarker (21 hand landmarks)
       → Gesture Classifier (TRACKING / FIST)
       → Angle Computation (wrist rot, wrist ext, claw)
       → EMA Smoother + Deadband
       → Serial → Arduino → PCA9685 → 3 Positional Servos
```

---

## ML Model — MediaPipe Hand Landmarker

The system uses Google's **MediaPipe Hand Landmarker** (Tasks API), a pre-trained deep learning model that runs entirely on-device (CPU, no GPU required).

### Model Details

| Property | Value |
|----------|-------|
| **Model file** | `hand_landmarker.task` (~7.5 MB, float16) |
| **Architecture** | Two-stage: Palm Detector (BlazePalm) → Hand Landmark Model (BlazHand) |
| **Input** | RGB video frames (640×480 @ ~30 FPS) |
| **Output** | 21 3D hand landmarks per detected hand |
| **Inference** | ~15–30ms per frame on CPU |
| **Running mode** | `VIDEO` (sequential frames with monotonic timestamps) |

### The 21 Hand Landmarks

```
         ╭── 8 (INDEX_TIP)
         │
    7────6────5 (INDEX_MCP)
    │              │
    8              │
                   │
4 (THUMB_TIP)     9 (MIDDLE_MCP) ──10──11──12 (MIDDLE_TIP)
│                  │
3                 13 (RING_MCP) ──14──15──16 (RING_TIP)
│                  │
2 (THUMB_MCP)     17 (PINKY_MCP) ──18──19──20 (PINKY_TIP)
│                  │
└──────── 0 (WRIST) ──────────┘
```

Each landmark provides normalised `(x, y, z)` coordinates:
- **x, y** ∈ [0, 1] — position in the image frame
- **z** — relative depth (negative = closer to camera), normalised by palm size

### How We Use the Landmarks

| Servo | Landmarks Used | Computation |
|-------|---------------|-------------|
| **Wrist Rotation** | 0 (Wrist) → 9 (Middle MCP) | Roll angle: `atan2(dx, -dy)` of the wrist-to-MCP vector in the image plane |
| **Wrist Extension** | 0 (Wrist) → 9 (Middle MCP) | Pitch angle: `atan2(-dz, -dy)` using the Z (depth) component |
| **Claw** | 4 (Thumb tip) → 8, 12, 16, 20 (Fingertips) | Minimum 3D distance from thumb to any fingertip, normalised by palm size |
| **Gesture (FIST)** | 5, 9, 13, 17 (MCPs) + 8, 12, 16, 20 (Tips) | All four fingertip-to-MCP distances below threshold → FIST |

### Model Download

The model is **not** included in the repo (too large for git). Download it:

```bash
# macOS / Linux
curl -L -o host/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

# Windows (PowerShell)
Invoke-WebRequest -Uri "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task" -OutFile "host\hand_landmarker.task"
```

---

## Inverse Kinematics (Reference)

> **Note:** IK is currently unused in wrist-only mode but is retained in [`host/kinematics.py`](host/kinematics.py) for future use when positional MG996R motors are installed.

The IK solver computes joint angles for a **planar 2-link arm** with base rotation, given a target (x, y, z) position in millimetres.

### Coordinate Frame

- **Origin**: Base rotation axis, on the table surface
- **Z axis**: Points **up**
- **XY plane**: Horizontal; the arm reaches outward in this plane
- `θ_base` rotates in the XY plane (0° = +X, 90° = straight ahead)

### Arm Geometry

```
                    ┌─── Wrist (end effector)
                    │
              L2 (98mm)
                    │
              Elbow ┤ ← θ_elbow (interior angle)
                    │
              L1 (105mm)
                    │
           Shoulder ┤ ← θ_shoulder (from horizontal)
                    │
         Base (60mm height)
         ═══════════╧═══════════  ← Table
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| `L1_MM` | 105 mm | Shoulder-to-elbow link length |
| `L2_MM` | 98 mm | Elbow-to-wrist link length |
| `BASE_HEIGHT_MM` | 60 mm | Height of the shoulder pivot above the table |
| **Max reach** | 203 mm | L1 + L2 (fully extended) |
| **Min reach** | 7 mm | \|L1 − L2\| (fully folded) |

### IK Algorithm

**Step 1 — Base rotation** (top-down view):

$$\theta_{base} = \text{atan2}(y, x)$$

**Step 2 — Planar 2-link IK** (in the vertical plane):

Compute the distance from shoulder to target:

$$r = \sqrt{x^2 + y^2}, \quad z_{eff} = z - h_{base}$$

$$d = \sqrt{r^2 + z_{eff}^2}$$

If $d$ exceeds the arm's reach, it's clamped to $L_1 + L_2$ (arm stretches toward target).

**Step 3 — Elbow angle** via the Law of Cosines:

$$\cos(\gamma) = \frac{L_1^2 + L_2^2 - d^2}{2 \cdot L_1 \cdot L_2}$$

$$\theta_{elbow} = \arccos(\gamma)$$

Where $\gamma$ is the interior angle at the elbow: 180° = fully extended, 0° = fully folded.

**Step 4 — Shoulder angle**:

$$\cos(\alpha) = \frac{L_1^2 + d^2 - L_2^2}{2 \cdot L_1 \cdot d}, \quad \phi = \text{atan2}(z_{eff}, r)$$

$$\theta_{shoulder} = \phi + \alpha$$

**Step 5 — Wrist compensation** (keeps end-effector level):

$$\theta_{wrist} = \theta_{shoulder} + \theta_{elbow} - 90°$$

### FK Round-Trip Validation

```bash
python host/kinematics.py
```

This runs IK → FK on 5 test points and verifies the round-trip error is < 5mm.

---

## Motor Layout

| PCA9685 Ch | Joint | Servo | Operating Range | Status | Control Source |
|:----------:|-------|-------|:---------------:|:------:|---------------|
| 0 | Base Rotation | MG996R (360° Continuous) | — | **DISABLED** | PWM permanently off |
| 1 | Shoulder Extension | MG996R (360° Continuous) | — | **DISABLED** | PWM permanently off |
| 2 | Elbow Extension | MG996R (360° Continuous) | — | **DISABLED** | PWM permanently off |
| **3** | **Wrist Rotation** | MG90S (180° Positional) | **0° – 180°** (90° = neutral) | ✅ Active | Hand roll angle |
| **4** | **Wrist Extension** | MG90S (180° Positional) | **0° – 180°** (90° = upright) | ✅ Active | Hand pitch angle |
| **5** | **Claw (2-finger)** | SG90 (180° Positional) | **20° – 80°** (80° = open, 20° = closed) | ✅ Active | Thumb-to-fingertip distance |

### Servo Specifications

| Servo | Type | Torque | Voltage | Pulse Range (PCA9685) |
|-------|------|--------|---------|----------------------|
| **MG996R** | 360° Continuous | 9.4 kg·cm | 4.8–7.2V | 150 – 464 (stop: 307) |
| **MG90S** | 180° Positional | 1.8 kg·cm | 4.8–6V | 102 – 512 (center: 307) |
| **SG90** | 180° Positional | 1.2 kg·cm | 4.8–5V | 102 – 492 (center: 297) |

### Pulse Width ↔ PCA9685 Counts (at 50 Hz)

At 50 Hz PWM, one period = 20 ms, and the PCA9685 divides it into 4096 counts:

$$\text{counts} = \frac{\text{pulse width (ms)}}{20\text{ ms}} \times 4096$$

| Pulse Width | Counts | Typical Meaning |
|:-----------:|:------:|:---------------:|
| 0.5 ms | 102 | 0° (min position) |
| 1.0 ms | 205 | ~45° |
| 1.5 ms | 307 | 90° (center / stop) |
| 2.0 ms | 410 | ~135° |
| 2.5 ms | 512 | 180° (max position) |

---

## Gesture Control

| Gesture | Detection Method | Action |
|---------|-----------------|--------|
| **Open hand** (TRACKING) | Any finger extended (tip-to-MCP distance > threshold) | Servos track hand — wrist rotates, extends, claw opens |
| **Pinch / Grab** (TRACKING) | Thumb close to fingertips | Claw closes proportionally |
| **Fist** (FIST) | All 4 fingertip-to-MCP distances < 0.15 (normalised) | Freeze — all servos hold current position |
| **No hand** (UNKNOWN) | No hand detected in frame | Servos hold last known position |

### Claw Proportional Mapping

The claw angle is computed from the **minimum** 3D distance between the thumb tip and any of the four fingertips (index, middle, ring, pinky), normalised by palm size:

| Normalised Distance | Claw Angle | State |
|:-------------------:|:----------:|:-----:|
| ≤ 0.15 | 20° | Fully closed |
| 0.30 | 50° | Half open |
| ≥ 0.45 | 80° | Fully open |

---

## Signal Processing — Smoothing & Deadband

Raw landmark data from MediaPipe is noisy. Two filters prevent servo jitter:

### Exponential Moving Average (EMA)

$$\hat{\theta}_t = \alpha \cdot \theta_{raw} + (1 - \alpha) \cdot \hat{\theta}_{t-1}$$

| Parameter | Value | Effect |
|-----------|:-----:|--------|
| `SMOOTHING_ALPHA` | 0.15 | Lower = smoother but slower response |

### Deadband

Ignore angle changes smaller than `DEADBAND_DEGREES` = **2.0°**. A new serial packet is only sent when at least one channel has moved more than 2° since the last transmission.

---

## Serial Protocol

The Python host sends ASCII packets over USB serial at **115200 baud**:

```
0,0,0,<wrist_rot>,<wrist_ext>,<claw>\n
```

| Field | Range | Description |
|-------|:-----:|-------------|
| Fields 0–2 | Always `0` | MG996R speeds (disabled) |
| `wrist_rot` | 0–180 | Wrist rotation angle |
| `wrist_ext` | 0–180 | Wrist extension angle |
| `claw` | 20–80 | Claw angle |

The firmware parses this as 6 comma-separated integers, ignores fields 0–2, and drives channels 3–5 via `angleToPulse()`.

---

## Hardware

### Components

| # | Component | Spec | Qty |
|---|-----------|------|:---:|
| 1 | Arduino Uno R3 | ATmega328P | 1 |
| 2 | PCA9685 Servo Driver | 16-ch I2C PWM | 1 |
| 3 | MG996R Servo | Base, Shoulder, Elbow (currently disabled) | 3 |
| 4 | MG90S Servo | Wrist Rotation, Wrist Extension | 2 |
| 5 | SG90 Servo | Claw | 1 |
| 6 | USB Webcam | Any UVC camera | 1 |
| 7 | SMPS | High-voltage DC power supply | 1 |
| 8 | Buck Converter (XL4015) | Steps down SMPS voltage to 5V | 1 |

### Wiring

> 📖 **Full Guide**: See [WIRING_GUIDE.md](WIRING_GUIDE.md) for detailed schematics, pin-by-pin charts, and pre-flight checklists.

```
1. Power Path (Servo Rail):
   SMPS (higher voltage) ──► Buck Converter (XL4015, set to 5.0V) ──► PCA9685 V+ screw terminal ──► Servos (Ch 0–5)

2. Control Path:
   Computer / Laptop ──► USB Cable (power + data) ──► Arduino Uno
                                                        ├── A4 (SDA) ──► PCA9685 SDA
                                                        ├── A5 (SCL) ──► PCA9685 SCL
                                                        ├── 5V       ──► PCA9685 VCC (logic)
                                                        └── GND      ──► PCA9685 GND
```

> ⚠️ **Note on Power**: The Arduino is powered directly by the laptop via USB. The buck converter steps the SMPS voltage down to a regulated 5.0V for the PCA9685 servo driver board. **Never** power the servos directly from the Arduino's 5V pin.

---

## Software Setup

### Prerequisites

- **Python 3.10–3.12** (3.12 recommended)
- **Git** (to clone the repo)
- **Arduino IDE** (for firmware upload)

### 1. Clone the repo

```bash
git clone https://github.com/Mayank3613/ArmAutomata-.git
cd ArmAutomata-
```

### 2. Create a virtual environment & install dependencies

#### Windows (Command Prompt)
```cmd
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

#### Windows (PowerShell)
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

#### macOS
```bash
python3.12 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

#### Linux
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. Download the MediaPipe hand model

#### macOS / Linux
```bash
curl -L -o host/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

#### Windows (PowerShell)
```powershell
Invoke-WebRequest -Uri "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task" -OutFile "host\hand_landmarker.task"
```

### 4. Upload firmware to Arduino

1. Open Arduino IDE
2. Install library: **Sketch → Include Library → Manage Libraries → search "Adafruit PWM Servo Driver" → Install**
3. Open `firmware/arm_firmware/arm_firmware.ino`
4. Select board: **Tools → Board → Arduino Uno**
5. Select port: **Tools → Port → your Arduino port**
6. Click **Upload** (→ button)
7. Open Serial Monitor (115200 baud) — you should see: `ARM_READY`

> If no port appears, install the [CH340 driver](https://sparks.gogo.co.nz/ch340.html) (clone boards) or the official [Arduino drivers](https://www.arduino.cc/en/Guide/DriverInstallation).

### 5. Find your serial port

```bash
python host/main.py --list-ports
```

| OS | Manual method |
|----|---------------|
| **Windows** | Device Manager → Ports (COM & LPT) |
| **macOS** | Terminal: `ls /dev/cu.usb*` |
| **Linux** | Terminal: `ls /dev/ttyACM* /dev/ttyUSB*` |

---

## Running

```bash
# Activate venv first
source venv/bin/activate          # macOS/Linux
# venv\Scripts\activate           # Windows

# Test without Arduino (camera + tracking only)
python host/main.py --no-serial

# With Arduino
python host/main.py --port /dev/cu.usbserial-1130    # macOS
python host/main.py --port COM3                       # Windows
python host/main.py --port /dev/ttyACM0               # Linux
```

- An OpenCV window shows the annotated camera feed with hand skeleton, gesture state, and servo angles.
- Press **`q`** in the window to quit.

---

## Project Structure

```
ArmAutomata-/
├── README.md                    # This file
├── WIRING_GUIDE.md              # Detailed hardware wiring guide
├── requirements.txt             # Python dependencies
├── .gitignore
├── host/
│   ├── main.py                  # Main control loop (wrist-only mode)
│   ├── config.py                # All tunable constants (auto-detects OS)
│   ├── hand_tracker.py          # MediaPipe Hand Landmarker wrapper
│   ├── gestures.py              # Gesture classification + claw + wrist angles
│   ├── smoothing.py             # EMA + deadband filter
│   ├── serial_link.py           # Arduino serial communication (3 angles)
│   ├── hand_landmarker.task     # ML model (downloaded, not in git)
│   │
│   │  ── Reference (not used in wrist-only mode) ──
│   ├── body_tracker.py          # MediaPipe Pose Landmarker (arm tracking)
│   ├── arm_mapper.py            # Maps human arm → robot joint angles
│   ├── kinematics.py            # Inverse & forward kinematics
│   └── calibration.py           # Normalised coords → workspace mm
│
└── firmware/
    ├── arm_firmware/
    │   └── arm_firmware.ino      # Arduino sketch (wrist-only, Ch 0-2 disabled)
    └── servo_calibration/
        └── servo_calibration.ino # Servo calibration utility
```

## Configuration

All tunable constants live in [`host/config.py`](host/config.py). Key values for the current wrist-only mode:

### Active Servo Parameters

| Constant | Value | Description |
|----------|:-----:|-------------|
| `WRIST_ROT_NEUTRAL_ANGLE` | 90° | Wrist rotation neutral (fingers up) |
| `WRIST_EXT_NEUTRAL_ANGLE` | 90° | Wrist extension neutral (hand upright) |
| `CLAW_OPEN_ANGLE` | 80° | Claw fully open |
| `CLAW_CLOSED_ANGLE` | 20° | Claw fully closed |
| `CLAW_DIST_MIN` | 0.15 | Normalised thumb-finger distance → fully closed |
| `CLAW_DIST_MAX` | 0.45 | Normalised thumb-finger distance → fully open |

### Joint Limits

| Joint | Channel | Min Angle | Max Angle |
|-------|:-------:|:---------:|:---------:|
| Base Rotation | 0 | 0° | 180° |
| Shoulder Extension | 1 | 15° | 165° |
| Elbow Extension | 2 | 10° | 170° |
| **Wrist Rotation** | **3** | **0°** | **180°** |
| **Wrist Extension** | **4** | **0°** | **180°** |
| **Claw** | **5** | **20°** | **80°** |

### Signal Processing

| Constant | Value | Description |
|----------|:-----:|-------------|
| `SMOOTHING_ALPHA` | 0.15 | EMA weight (lower = smoother, slower) |
| `DEADBAND_DEGREES` | 2.0° | Min change to trigger retransmission |
| `FIST_CURL_THRESHOLD` | 0.15 | Fingertip-to-MCP ratio for fist detection |

### Serial

| Constant | Value | Description |
|----------|:-----:|-------------|
| `SERIAL_BAUD` | 115200 | Must match firmware |
| `SERIAL_RESET_WAIT_S` | 2.0s | Wait for Arduino bootloader after connect |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `ModuleNotFoundError` | Activate the venv: `source venv/bin/activate` (Mac/Linux) or `venv\Scripts\activate` (Win) |
| Camera not opening | Grant camera permission. macOS: System Settings → Privacy → Camera |
| `FileNotFoundError: hand_landmarker.task` | Download the model file (step 3 above) |
| Serial port not found | Run `python host/main.py --list-ports` to find the correct port |
| `Resource busy` serial error | Close Arduino IDE Serial Monitor or kill other processes using the port: `lsof /dev/cu.usbserial-*` |
| MediaPipe crash on macOS (`DrishtiMetalHelper`) | Use `mediapipe<1.0` (already pinned in requirements.txt) and Python 3.12 |
| Servos don't move | Check: firmware flashed? Serial Monitor shows `ARM_READY`? Buck converter outputting 5V? |
| Claw doesn't fully close | Tune `CLAW_DIST_MIN` in `config.py` (increase for easier closing) |
| `python` not found (Mac/Linux) | Use `python3` instead |

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `opencv-python` | ≥ 4.8.0 | Camera capture + display |
| `mediapipe` | ≥ 0.10.0, < 1.0 | Hand landmark detection (ML model) |
| `numpy` | ≥ 1.24.0 | Array operations (used by MediaPipe/OpenCV) |
| `pyserial` | ≥ 3.5 | USB serial communication with Arduino |

## License

MIT
