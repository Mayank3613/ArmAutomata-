# ArmAutomata — Gesture-Controlled 6-DOF Robotic Arm

A real-time gesture-controlled robotic arm. A webcam captures your arm and hand; a Python host detects body pose + hand landmarks, computes joint angles, and streams them to an Arduino Uno over USB serial. The Arduino drives six servos via a PCA9685 PWM board.

## Architecture

```
Webcam → Pose Landmarker (shoulder/elbow/wrist)
       → Hand Landmarker (fingers/gestures)
       → Arm Mapper → Smoother → Serial → Arduino → PCA9685 → Servos
```

### Tracking Modes (automatic)

| Mode | What's visible | How arm is controlled |
|------|----------------|----------------------|
| 🟢 **FULL_ARM** | Shoulder + elbow + wrist | Direct joint-angle mirroring |
| 🟠 **FOREARM** | Elbow + wrist | Elbow angle + estimated shoulder |
| 🔵 **HAND_ONLY** | Just the hand | IK from wrist position |

### Motor Layout

| PCA9685 Ch | Joint              | Servo  | Range / Limits | Pulse Count | Control Source                    |
|------------|--------------------|--------|----------------|-------------|-----------------------------------|
| 0          | Base Rotation      | MG996R (360° Continuous) | -100 to +100 speed | 150 – 464 (stop: 307) | Virtual tracker from wrist horizontal position |
| 1          | Shoulder Extension | MG996R (360° Continuous) | -100 to +100 speed | 150 – 464 (stop: 307) | Virtual tracker from upper arm angle |
| 2          | Elbow Extension    | MG996R (360° Continuous) | -100 to +100 speed | 150 – 464 (stop: 307) | Virtual tracker from elbow interior angle |
| 3          | Wrist Rotation     | MG90S  (180° Positional) | 0° – 180° angle    | 102 – 512   | Hand roll angle in camera         |
| 4          | Wrist Extension    | MG90S  (180° Positional) | 0° – 180° angle    | 102 – 512   | Auto-levels end-effector          |
| 5          | Claw (2-finger)    | SG90   (180° Positional) | 30° – 90° angle    | 102 – 492   | Thumb-to-fingers distance         |

### Gesture Control

| Gesture       | Action                                                   |
|---------------|----------------------------------------------------------|
| **Open hand** | Arm follows your arm; claw opens proportionally           |
| **Pinch/Grab**| Arm follows your arm; claw closes proportionally          |
| **Fist**      | Freeze — arm holds current pose                           |

## Hardware

### Components

| # | Component | Spec | Qty |
|---|-----------|------|-----|
| 1 | Arduino Uno R3 | ATmega328P | 1 |
| 2 | PCA9685 Servo Driver | 16-ch I2C PWM | 1 |
| 3 | MG996R Servo | Base, Shoulder, Elbow | 3 |
| 4 | MG90S Servo | Wrist Rotation, Wrist Extension | 2 |
| 5 | SG90 Servo | Claw | 1 |
| 6 | USB Webcam | Any UVC camera | 1 |
| 7 | SMPS | High-voltage DC power supply | 1 |
| 8 | Buck Converter (XL4015) | Steps down SMPS voltage to 5V | 1 |

### Wiring

> 📖 **Full Guide**: See [WIRING_GUIDE.md](WIRING_GUIDE.md) for detailed schematics, pin-by-pin charts, and pre-flight checklists.

The architecture has two independent paths:

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

> ⚠️ **Note on Power**: The Arduino is powered directly by the laptop via USB. The buck converter is used because the SMPS supplies a higher voltage than recommended for the servos; it steps down the SMPS voltage to a regulated 5.0V for the PCA9685 servo driver board. **Never** power the servos directly from the Arduino's 5V pin.

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

### 3. Download MediaPipe models

#### Windows (PowerShell)
```powershell
Invoke-WebRequest -Uri "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task" -OutFile "host\hand_landmarker.task"

Invoke-WebRequest -Uri "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task" -OutFile "host\pose_landmarker.task"
```

#### macOS / Linux
```bash
curl -L -o host/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

curl -L -o host/pose_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task
```

### 4. Upload firmware to Arduino

The file `firmware/arm_firmware/arm_firmware.ino` is the Arduino sketch that receives joint angles over USB serial and drives the 6 servos via the PCA9685 board. You need to upload this file to your Arduino Uno **once**.

#### Step 4a — Install Arduino IDE

Download and install from [arduino.cc/en/software](https://www.arduino.cc/en/software) (Windows, macOS, or Linux).

#### Step 4b — Install the required library

The firmware depends on the **Adafruit PWM Servo Driver Library** to communicate with the PCA9685 board over I2C.

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries…** (or press `Ctrl+Shift+I`)
3. In the search box, type **"Adafruit PWM Servo Driver"**
4. Find **"Adafruit PWM Servo Driver Library"** by Adafruit — click **Install**
5. If prompted to install dependencies (like "Adafruit BusIO"), click **Install All**

#### Step 4c — Open the sketch

1. In Arduino IDE, go to **File → Open…**
2. Navigate to the cloned repo folder: `ArmAutomata-/firmware/arm_firmware/`
3. Open the file **`arm_firmware.ino`**

The sketch will open in a new window. You should see the code with `#include <Adafruit_PWMServoDriver.h>` at the top.

#### Step 4d — Connect and configure

1. **Plug the Arduino Uno into your computer** via USB-B cable
2. In Arduino IDE, go to **Tools → Board** and select **"Arduino Uno"**
3. Go to **Tools → Port** and select the port that appeared when you plugged in:
   - Windows: **COM3**, **COM4**, etc.
   - macOS: **/dev/cu.usbmodem14201** or similar
   - Linux: **/dev/ttyACM0** or similar

> If no port appears, install the Arduino USB driver: [CH340 driver](https://sparks.gogo.co.nz/ch340.html) (for clone boards) or the official [Arduino drivers](https://www.arduino.cc/en/Guide/DriverInstallation).

#### Step 4e — Upload

1. Click the **Upload** button (→ arrow icon) or press `Ctrl+U`
2. Wait for the IDE to compile and upload. You should see:
   ```
   Sketch uses XXXX bytes (XX%) of program storage space.
   Done uploading.
   ```
3. The Arduino will reset, and all 6 servos will move to **90° (neutral position)**
4. Open **Tools → Serial Monitor** (set baud to **115200**) — you should see: `ARM_READY`

The Arduino is now ready to receive angle commands from the Python host. You only need to upload once — the firmware stays in the Arduino's flash memory even after power cycling.

### 5. Find your serial port

The easiest way — use the built-in port scanner:

```bash
python host/main.py --list-ports
```
```
Available serial ports:
  COM3                  Arduino Uno (COM3)
```

Or find it manually:

| OS | Method |
|----|--------|
| **Windows** | Device Manager → Ports (COM & LPT) → look for "Arduino Uno (COMx)" |
| **macOS** | Terminal: `ls /dev/tty.usb*` |
| **Linux** | Terminal: `ls /dev/ttyACM* /dev/ttyUSB*` |

---

## Running

Activate your virtual environment first, then:

```bash
# Test without Arduino (any platform)
python host/main.py --no-serial

# With Arduino
python host/main.py --port COM3              # Windows
python host/main.py --port /dev/tty.usbmodem14201  # macOS
python host/main.py --port /dev/ttyACM0      # Linux

# Auto-detect serial ports
python host/main.py --list-ports
```

> **Note:** On macOS/Linux, use `python3` instead of `python` if your system default is Python 2.

- An OpenCV window shows the annotated camera feed with tracking mode and joint angles.
- Press **`q`** in the window to quit.

### IK Validation

```bash
python host/kinematics.py
```

---

## Project Structure

```
ArmAutomata-/
├── README.md
├── requirements.txt
├── .gitignore
├── host/
│   ├── config.py          # All tunable constants (auto-detects OS)
│   ├── hand_tracker.py    # MediaPipe Hand Landmarker (fingers)
│   ├── body_tracker.py    # MediaPipe Pose Landmarker (arm)
│   ├── arm_mapper.py      # Maps human arm → robot joint angles
│   ├── gestures.py        # Gesture classification + claw + wrist rotation
│   ├── calibration.py     # Normalised coords → workspace mm
│   ├── kinematics.py      # Inverse & forward kinematics (IK fallback)
│   ├── smoothing.py       # EMA + deadband filter
│   ├── serial_link.py     # Arduino serial communication (6 angles)
│   └── main.py            # Main control loop (--list-ports, --no-serial)
└── firmware/
    └── arm_firmware/
        └── arm_firmware.ino   # Arduino sketch (6 servos, non-blocking)
```

## Configuration

All tunable constants live in [`host/config.py`](host/config.py). The serial port default auto-detects your OS.

| Constant                 | Default | Description                                  |
|--------------------------|---------|----------------------------------------------|
| `L1_MM` / `L2_MM`        | 105/98  | Link lengths in mm                           |
| `BASE_HEIGHT_MM`         | 60      | Base/shoulder pivot height in mm             |
| `SMOOTHING_ALPHA`        | 0.15    | EMA weight (halved for smooth, half-speed motion) |
| `DEADBAND_DEGREES`       | 2.0     | Min angle change to retransmit (positional)  |
| `BASE_SPEED_DEG_PER_SEC` | 120.0   | Base rotational speed (deg/sec)              |
| `SHOULDER_SPEED_DEG_PER_SEC` | 120.0 | Shoulder rotational speed (deg/sec)          |
| `ELBOW_SPEED_DEG_PER_SEC`| 120.0   | Elbow rotational speed (deg/sec)             |
| `POSITIVE_SPEED_FACTOR`  | 1.25    | Positive (CCW) move time multiplier (+25%)   |
| `BASE_STOP_PULSE`        | 307     | Neutral stop pulse count for continuous MG996R (1.5ms at 50Hz) |
| `BASE_DEADBAND_DEG`      | 3.0     | Error deadband for continuous motor controllers|
| `BASE_INVERT_DIRECTION`  | False   | Invert base rotation direction if needed     |
| `SHOULDER_INVERT_DIRECTION` | False| Invert shoulder rotation direction if needed |
| `ELBOW_INVERT_DIRECTION` | False   | Invert elbow rotation direction if needed    |
| `JOINT_MIN_ANGLES`       | [0, 15, 10, 0, 0, 30] | Min angles per joint (collision prevention) |
| `JOINT_MAX_ANGLES`       | [180, 165, 170, 180, 180, 90] | Max angles per joint |
| `CLAW_DIST_MIN`          | 0.05    | Thumb-finger dist -> claw fully closed       |
| `CLAW_DIST_MAX`          | 0.35    | Thumb-finger dist -> claw fully open         |
| `CLAW_OPEN_ANGLE`        | 90      | Servo angle for claw open                    |
| `CLAW_CLOSED_ANGLE`      | 30      | Servo angle for claw closed                  |
| `SERIAL_PORT`            | auto    | COM3 (Win) / /dev/tty.usbmodem... (Mac) / /dev/ttyACM0 (Linux) |

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `ModuleNotFoundError` | Activate the venv: `venv\Scripts\activate` (Win) or `source venv/bin/activate` (Mac/Linux) |
| Camera not opening | Grant camera permission. Windows: Settings → Privacy → Camera. macOS: System Settings → Privacy → Camera |
| `FileNotFoundError: hand_landmarker.task` | Download the model files (step 3 above) |
| Serial port not found | Run `python host/main.py --list-ports` to find the correct port |
| MediaPipe crash on macOS (`DrishtiMetalHelper`) | Use `mediapipe<1.0` (already pinned in requirements.txt) and Python 3.12 |
| `python` not found (Mac/Linux) | Use `python3` instead |

## License

MIT
