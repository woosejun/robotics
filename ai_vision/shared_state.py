"""State shared by the camera, inference, control, serial and web threads."""

import threading

frame_lock = threading.Lock()
current_frame = None
frame_at = float("-inf")

data_lock = threading.Lock()
detections = []
sim_scores = []
target_idx = -1
detection_at = float("-inf")
registration_requested_at = None
registration_status = "UNREGISTERED"

# When both locks are needed, acquire data_lock before control_lock.
control_lock = threading.Lock()
motion_enabled = False
motion_epoch = 0
control_error_x = 0
robot_state = "IDLE"
motor_command = "S"
motor_command_at = float("-inf")
motion_input_until = float("-inf")
motion_input_fault = "VISION_STALE"
motor_pulse_until = float("inf")
uwb_armed_generation = None
uwb_intent = "S"
target_height = 0
distance_zone = "LOST"
last_direction = 0

robot_serial_lock = threading.Lock()
robot_serial_connected = False
robot_serial_last_ack = ""
robot_serial_last_ack_time = float("-inf")

running = True


def visual_target_is_current(now, timeout):
    """Caller holds data_lock."""
    return (
        registration_status == "REGISTERED"
        and 0 <= target_idx < len(detections)
        and now - detection_at < timeout
    )
