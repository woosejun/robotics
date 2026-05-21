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

last_direction = 0

turning = False

control_lock = threading.Lock()

# =========================================================
# Internal State
# =========================================================
virtual_error_x = 0

prev_control_error = 0

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