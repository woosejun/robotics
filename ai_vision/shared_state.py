import threading
import time

# =========================================================
# Frame
# =========================================================
current_frame = None

frame_lock = threading.Lock()

# =========================================================
# Detection
# =========================================================
detections = []

sim_scores = []

target_idx = -1

data_lock = threading.Lock()

# =========================================================
# Target Histogram
# =========================================================
target_hist = None

target_hist_lock = threading.Lock()

# =========================================================
# Control State
# =========================================================
control_error_x = 0

robot_state = "IDLE"

motor_command = "S"

target_height = 0

distance_zone = "LOST"

last_direction = 0

turning = False

control_lock = threading.Lock()

# =========================================================
# Robot Serial Status
# =========================================================
robot_serial_connected = False

robot_serial_last_ack = ""

robot_serial_last_ack_time = 0.0

robot_serial_lock = threading.Lock()

# =========================================================
# Internal State
# =========================================================
virtual_error_x = 0

prev_control_error = 0

virtual_target_height = 0.0

target_height_initialized = False

lost_start_time = time.time()

# =========================================================
# Save Request
# =========================================================
save_requested = False

save_lock = threading.Lock()

# =========================================================
# Running
# =========================================================
running = True
