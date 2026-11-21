"""Jetson runtime and calibration settings for the competition chassis."""

import os

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CENTER_X = CAMERA_WIDTH // 2
CAMERA_DEVICE = os.environ.get("ROBOTICS_CAMERA_DEVICE")
CAMERA_RETRY_DT = 1.0

YOLO_MODEL_PATH = os.environ.get("ROBOTICS_MODEL")
CONFIDENCE_THRESHOLD = 0.45
YOLO_IMGSZ = 640
SIM_THRESHOLD = 0.4
HIST_UPDATE_THRESHOLD = 0.6
# Hold position when multiple people have similarly matching clothing.
SIM_MIN_MARGIN = 0.08
REGISTRATION_TIMEOUT = 5.0

CONTROL_DT = 0.03
MAX_STEP = 35
ALPHA = 0.35
START_TURN = 120
STOP_TURN = 80
VISION_STALE_TIMEOUT = 0.5
CONTROL_STALE_TIMEOUT = 0.2

# Pixel thresholds for this camera framing. Calibrate on a new chassis.
# FAR and TARGET describe proximity on the display. The shield uses fixed speed.
DISTANCE_ALPHA = 0.20
STOP_ENTER_HEIGHT = 430
STOP_EXIT_HEIGHT = 380
FAR_ENTER_HEIGHT = 120
FAR_EXIT_HEIGHT = 150

# A fresh frame without a match stops the cart. Rotation is an opt-in bench mode.
SEARCH_ROTATION_ENABLED = False
SEARCH_TIME = 2.0
LOST_SWEEP_TIME = 6.0
LOST_ROTATE_TIME = 0.6
LOST_STOP_TIME = 0.4

WEB_HOST = os.environ.get("ROBOTICS_WEB_HOST", "127.0.0.1")
WEB_PORT = 5000
JPEG_QUALITY = 45
FOLLOW_MODE = os.environ.get("ROBOTICS_FOLLOW_MODE", "vision").lower()
UWB_FRONT_GUARD = os.environ.get("ROBOTICS_UWB_FRONT_GUARD", "1") != "0"

ROBOT_SERIAL_ENABLED = os.environ.get("ROBOTICS_MOTOR_ENABLED", "0") == "1"
ROBOT_SERIAL_PORT = os.environ.get("ROBOTICS_MOTOR_PORT", "/dev/ttyUSB0")
ROBOT_SERIAL_BAUD = 115200
ROBOT_SERIAL_DT = 0.05
ROBOT_SERIAL_ACK_TIMEOUT = 1.0
ROBOT_SERIAL_RECONNECT_DT = 1.0

# Match stable /dev/serial/by-id paths to ANCHOR1 (left) and ANCHOR2 (right).
UWB_ENABLED = os.environ.get("ROBOTICS_UWB_ENABLED", "1") == "1"
UWB_LEFT_PORT = os.environ.get("ROBOTICS_UWB_LEFT", "/dev/ttyACM1")
UWB_RIGHT_PORT = os.environ.get("ROBOTICS_UWB_RIGHT", "/dev/ttyACM0")
UWB_BAUD = 115200
UWB_RECONNECT_DT = 1.0
# Physical distance between the antenna centers in metres.
UWB_ANCHOR_BASELINE_M = float(os.environ.get("ROBOTICS_UWB_BASELINE_M", "0.30"))
UWB_STALE_TIMEOUT = 1.5
UWB_MAX_PAIR_SKEW = 0.45

# Driving uses a stricter lifetime and only pairs two new, identified reports.
UWB_DRIVE_MAX_AGE_S = 0.65
UWB_DRIVE_MAX_SKEW_S = 0.32
UWB_STOP_M = 0.85
UWB_RESUME_M = 1.05
UWB_MAX_FOLLOW_M = 3.0
UWB_MIN_FORWARD_M = 0.20
UWB_MAX_BEARING_DEG = 55.0
UWB_TURN_START_DEG = 18.0
UWB_TURN_STOP_DEG = 10.0
UWB_CONFIRM_PAIRS = 2
# The retained AceBot protocol has fixed-speed drive masks.
UWB_MOTION_PULSE_S = 0.14
UWB_SETTLE_S = 0.10
UWB_RECOVERY_PAIR_WAIT_S = 0.60
