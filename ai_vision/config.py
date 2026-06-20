# =========================================================
# Camera
# =========================================================
CAMERA_WIDTH = 640

CAMERA_HEIGHT = 480

CAMERA_FPS = 30

CENTER_X = CAMERA_WIDTH // 2

# =========================================================
# YOLO
# =========================================================
CONFIDENCE_THRESHOLD = 0.45

YOLO_IMGSZ = 640

# =========================================================
# Similarity
# =========================================================
SIM_THRESHOLD = 0.4

HIST_UPDATE_THRESHOLD = 0.6

# =========================================================
# Control
# =========================================================
CONTROL_DT = 0.03

MAX_STEP = 35  # 이전: 15 → 더 빠른 반응

ALPHA = 0.35   # 이전: 0.15 → 더 빠른 스무싱

START_TURN = 40

STOP_TURN = 20

SEARCH_ERROR = 120

# Target bounding-box height is used as a monocular distance estimate.
# Separate enter/exit thresholds prevent commands from chattering at edges.
DISTANCE_ALPHA = 0.20

STOP_ENTER_HEIGHT = 250

# Leave STOP mode only when the target height has dropped back below this value.
STOP_EXIT_HEIGHT = 230

FAR_ENTER_HEIGHT = 150

FAR_EXIT_HEIGHT = 180

NEAR_ENTER_HEIGHT = 430

NEAR_EXIT_HEIGHT = 400

# =========================================================
# LOST FSM
# =========================================================
SEARCH_TIME = 1.0

LOST_RIGHT_TIME = 3.0

LOST_LEFT_TIME = 5.0

# =========================================================
# Stream
# =========================================================
JPEG_QUALITY = 45

# =========================================================
# Robot USB Serial
# =========================================================
ROBOT_SERIAL_ENABLED = True

# CH340 USB serial converter.  The by-id name remains stable even if Linux
# assigns a different ttyUSB number after a reboot.
ROBOT_SERIAL_PORT = "/dev/ttyUSB0"

ROBOT_SERIAL_BAUD = 115200

ROBOT_SERIAL_DT = 0.05

ROBOT_SERIAL_ACK_TIMEOUT = 1.0

ROBOT_SERIAL_RECONNECT_DT = 1.0
