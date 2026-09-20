"""
Central configuration for the Gesture-Controlled 6-DOF Robotic Arm.

Motor layout (from hardware diagram):
    Ch 0 — Base Rotation     — MG996R
    Ch 1 — Shoulder Extension — MG996R
    Ch 2 — Elbow Extension    — MG996R
    Ch 3 — Wrist Rotation     — MG90S
    Ch 4 — Wrist Extension    — MG90S
    Ch 5 — Claw (2-finger)    — SG90

All tunable constants are grouped here for easy adjustment.
Values marked with [TUNE] will likely need real-hardware calibration.
"""

# ---------------------------------------------------------------------------
# Arm geometry (millimetres)
# ---------------------------------------------------------------------------
BASE_HEIGHT_MM: float = 60.0          # Height of the base/shoulder pivot
L1_MM: float = 105.0                  # Shoulder-to-elbow link length
L2_MM: float = 98.0                   # Elbow-to-wrist link length

# ---------------------------------------------------------------------------
# Workspace bounds — the real-world volume the arm can reach (mm)
# MediaPipe normalised [0,1] wrist coords are mapped into this box.
# ---------------------------------------------------------------------------
WORKSPACE_X_MIN: float = -200.0       # [TUNE] Left limit
WORKSPACE_X_MAX: float = 200.0        # [TUNE] Right limit
WORKSPACE_Y_MIN: float = 0.0          # [TUNE] Bottom (table surface)
WORKSPACE_Y_MAX: float = 250.0        # [TUNE] Top
WORKSPACE_Z_MIN: float = 50.0         # [TUNE] Nearest reach
WORKSPACE_Z_MAX: float = 200.0        # [TUNE] Farthest reach

# ---------------------------------------------------------------------------
# Gesture detection thresholds (normalised landmark distances)
# ---------------------------------------------------------------------------
PINCH_THRESHOLD: float = 0.05         # [TUNE] Max thumb-index tip distance for PINCH
FIST_CURL_THRESHOLD: float = 0.15     # [TUNE] Max fingertip-to-MCP distance for FIST
OPEN_PALM_THRESHOLD: float = 0.15     # [TUNE] Min fingertip-to-MCP distance for OPEN_PALM

# ---------------------------------------------------------------------------
# Claw (gripper) control — proportional thumb-to-fingers mapping
# ---------------------------------------------------------------------------
CLAW_DIST_MIN: float = 0.15           # [TUNE] Normalised thumb-finger distance → fully closed
CLAW_DIST_MAX: float = 0.45           # [TUNE] Normalised thumb-finger distance → fully open
CLAW_OPEN_ANGLE: int = 80             # [TUNE] Servo angle for claw fully open
CLAW_CLOSED_ANGLE: int = 20           # [TUNE] Servo angle for claw fully closed

# ---------------------------------------------------------------------------
# Smoothing
# ---------------------------------------------------------------------------
SMOOTHING_ALPHA: float = 0.15         # EMA weight (halved for smoother, half-speed motion)
DEADBAND_DEGREES: float = 2.0         # Ignore angle changes smaller than this

# ---------------------------------------------------------------------------
# Servo angle limits (degrees, sent over serial)
# ---------------------------------------------------------------------------
SERVO_MIN_ANGLE: int = 0
SERVO_MAX_ANGLE: int = 180
WRIST_EXT_NEUTRAL_ANGLE: int = 90     # [TUNE] Wrist extension neutral (level)
WRIST_ROT_NEUTRAL_ANGLE: int = 90     # [TUNE] Wrist rotation neutral (no twist)

# Actuation limits per joint to prevent mechanical collisions
# Order: [Base, Shoulder, Elbow, WristRot, WristExt, Claw]
JOINT_MIN_ANGLES: list[int] = [0, 15, 10, 0, 0, 20]
JOINT_MAX_ANGLES: list[int] = [180, 165, 170, 180, 180, 80]

# Per-motor pulse counts for PCA9685 (at 50 Hz)
# Ch 0-2 (MG996R): 150-464 (center=307), Ch 3-4 (MG90S): 102-512 (center=307), Ch 5 (SG90): 102-492
PULSE_MIN: list[int] = [150, 150, 150, 102, 102, 102]
PULSE_MAX: list[int] = [464, 464, 464, 512, 512, 492]

# ---------------------------------------------------------------------------
# Continuous MG996R Motors Tuning Parameters (Ch 0 Base, Ch 1 Shoulder, Ch 2 Elbow)
# ---------------------------------------------------------------------------
BASE_SPEED_DEG_PER_SEC: float = 120.0     # Calibrated: 120.0 deg/sec
SHOULDER_SPEED_DEG_PER_SEC: float = 120.0 # Calibrated: 120.0 deg/sec
ELBOW_SPEED_DEG_PER_SEC: float = 120.0    # Calibrated: 120.0 deg/sec
POSITIVE_SPEED_FACTOR: float = 1.25       # Positive (CCW) rotation is slower -> takes 25% longer
NEGATIVE_SPEED_FACTOR: float = 1.0        # Neutral/standard multiplier for negative rotation

BASE_STOP_PULSE: int = 307               # Calibrated neutral stop point (1.5ms at 50Hz)
BASE_DEADBAND_DEG: float = 3.0            # Error deadband below which continuous motors stop
BASE_INVERT_DIRECTION: bool = False       # Invert base rotation direction if needed
SHOULDER_INVERT_DIRECTION: bool = False   # Invert shoulder rotation direction if needed
ELBOW_INVERT_DIRECTION: bool = False      # Invert elbow rotation direction if needed

# ---------------------------------------------------------------------------
# PCA9685 channel assignments (must match firmware)
# ---------------------------------------------------------------------------
CH_BASE: int = 0
CH_SHOULDER: int = 1
CH_ELBOW: int = 2
CH_WRIST_ROT: int = 3
CH_WRIST_EXT: int = 4
CH_CLAW: int = 5
NUM_JOINTS: int = 6

# ---------------------------------------------------------------------------
# Serial communication
# ---------------------------------------------------------------------------
import platform as _platform
if _platform.system() == "Windows":
    SERIAL_PORT: str = "COM3"          # [TUNE] Check Device Manager → Ports
elif _platform.system() == "Darwin":
    SERIAL_PORT: str = "/dev/tty.usbmodem14201"  # [TUNE] ls /dev/tty.usb*
else:
    SERIAL_PORT: str = "/dev/ttyACM0"  # [TUNE] ls /dev/ttyACM* /dev/ttyUSB*

SERIAL_BAUD: int = 115200
SERIAL_RESET_WAIT_S: float = 2.0      # Seconds to wait after opening port for Arduino reset

# ---------------------------------------------------------------------------
# Camera / display
# ---------------------------------------------------------------------------
CAMERA_INDEX: int = 0                 # OpenCV VideoCapture device index
CAMERA_WIDTH: int = 640
CAMERA_HEIGHT: int = 480
WINDOW_NAME: str = "RoboArm - Gesture Control"
