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

START_TURN = 120

STOP_TURN = 80

SEARCH_ERROR = 120

# Target bounding-box height is used as a monocular distance estimate.
# Separate enter/exit thresholds prevent commands from chattering at edges.
DISTANCE_ALPHA = 0.20

STOP_ENTER_HEIGHT = 430

# Leave STOP mode only when the target height has dropped back below this value.
STOP_EXIT_HEIGHT = 380

FAR_ENTER_HEIGHT = 120

FAR_EXIT_HEIGHT = 150

NEAR_ENTER_HEIGHT = 430

NEAR_EXIT_HEIGHT = 400

# =========================================================
# LOST FSM
# =========================================================
SEARCH_TIME = 1.0

LOST_RIGHT_TIME = 7.0

LOST_LEFT_TIME = 7.0

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

# =========================================================
# UWB (로봇 앵커 2개 / 사람 태그 1개)
# =========================================================
# 앵커는 전용 USB CDC 포트를 사용합니다. ESP32 모터 제어 포트인
# ROBOT_SERIAL_PORT와 UWB 포트를 같은 값으로 설정하면 안 됩니다.
UWB_ENABLED = True

# 오른쪽 앵커: 부팅 로그에서 DEVICE ID: deca0302 확인됨.
UWB_RIGHT_PORT = "/dev/ttyACM0"

# 왼쪽 앵커를 연결한 뒤 실제 포트로 바꿉니다(보통 /dev/ttyACM1).
UWB_LEFT_PORT = "/dev/ttyACM1"

UWB_BAUD = 115200
UWB_RECONNECT_DT = 1.0

# 로봇에 붙인 왼쪽·오른쪽 앵커의 UWB 안테나 중심 사이 실제 거리(m)입니다.
# 예: 두 앵커의 모듈 중앙이 30cm 떨어졌다면 0.30으로 설정합니다.
# 이 값이 있어야 젯슨이 두 거리로 사람의 좌우/전방 위치를 계산할 수 있습니다.
UWB_ANCHOR_BASELINE_M = 0.30

# 이 시간보다 오래된 거리 데이터는 사용할 수 없는 값으로 처리합니다.
UWB_STALE_TIMEOUT = 1.5
