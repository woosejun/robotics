"""Vision tracking, motion authorization and fail-safe command generation."""

import time

import config
import shared_state as state
from uwb_drive import UwbFollower, read_uwb_input


def update_distance_zone(current_zone, height, raw_height):
    # Stop immediately on a large raw box. Use the smoothed height for release.
    if raw_height >= config.STOP_ENTER_HEIGHT or height >= config.STOP_ENTER_HEIGHT:
        return "STOP"
    if current_zone == "STOP" and max(height, raw_height) > config.STOP_EXIT_HEIGHT:
        return "STOP"
    far_threshold = (
        config.FAR_EXIT_HEIGHT if current_zone == "FAR" else config.FAR_ENTER_HEIGHT
    )
    return "FAR" if height < far_threshold else "TARGET"


def set_motion_enabled(enabled, ranges=None):
    """Authorize the selected source with fresh input and a confirmed motor link."""
    with state.data_lock, state.control_lock:
        now = time.monotonic()
        deadline = state.detection_at + config.VISION_STALE_TIMEOUT
        generation = None
        expiry_status = "VISION_STALE"
        if enabled:
            if config.FOLLOW_MODE == "uwb":
                if not config.UWB_ENABLED or ranges is None:
                    return "UWB 수신기가 필요합니다."
                _, generation, _, fault, deadline = read_uwb_input(ranges, now)
                if fault == "UWB_FRONT_LOST":
                    return "카메라에서 등록한 대상의 정면 위치를 확인해야 합니다."
                if fault == "UWB_OUTSIDE":
                    return "태그를 카트 정면의 동작 범위 안에 놓아 주세요."
                if fault:
                    return "좌우 앵커에서 유효한 태그 거리를 기다려 주세요."
                expiry_status = "UWB_INPUT_STALE"
            elif not state.visual_target_is_current(now, config.VISION_STALE_TIMEOUT):
                return "등록한 대상이 실시간 영상에서 확인되어야 합니다."
            if config.ROBOT_SERIAL_ENABLED:
                with state.robot_serial_lock:
                    if not state.robot_serial_connected:
                        return "모터 제어기의 연결 확인을 기다려 주세요."
        if state.motion_enabled != enabled:
            state.motion_enabled = enabled
            state.motion_epoch += 1
            state.motor_command = "S"
            state.motor_command_at = now
            state.motion_input_until = deadline
            state.motion_input_fault = expiry_status
            state.motor_pulse_until = float("inf")
            state.uwb_armed_generation = generation
        return None


def request_registration():
    with state.data_lock, state.control_lock:
        state.registration_requested_at = time.monotonic()
        state.registration_status = "PENDING"
        state.target_idx = -1
        state.motion_enabled = False
        state.motor_command = "S"


def expire_registration(now):
    with state.data_lock:
        requested_at = state.registration_requested_at
        if requested_at is not None and now - requested_at >= config.REGISTRATION_TIMEOUT:
            state.registration_requested_at = None
            state.registration_status = "TIMED_OUT"


class Follower:
    def __init__(self):
        self.reset()

    def reset(self):
        self.filtered_error = 0.0
        self.filtered_height = None
        self.previous_error = 0
        self.turning = False
        self.zone = "LOST"
        self.lost_at = None
        self.last_frame_at = float("-inf")
        self.last_direction = 0
        self.tracking_result = (0, 0, "LOST", "S")

    def track(self, box):
        x1, y1, x2, y2 = box
        raw_height = min(config.CAMERA_HEIGHT, max(0, y2 - y1))
        if self.filtered_height is None:
            self.filtered_height = raw_height
        else:
            self.filtered_height += config.DISTANCE_ALPHA * (raw_height - self.filtered_height)
        self.zone = update_distance_zone(self.zone, self.filtered_height, raw_height)

        raw_error = (x1 + x2) // 2 - config.CENTER_X
        self.filtered_error += config.ALPHA * (raw_error - self.filtered_error)
        delta = int(self.filtered_error) - self.previous_error
        self.previous_error += max(-config.MAX_STEP, min(config.MAX_STEP, delta))
        turn_threshold = config.STOP_TURN if self.turning else config.START_TURN
        self.turning = abs(self.previous_error) >= turn_threshold
        error = self.previous_error if self.turning else 0
        if error:
            self.last_direction = 1 if error > 0 else -1

        if self.zone == "STOP":
            command = "S"
        elif self.turning:
            command = "R" if error > 0 else "L"
        else:
            command = "F"
        return error, int(self.filtered_height), self.zone, command

    def update(self, now):
        expire_registration(now)
        with state.data_lock, state.control_lock:
            frame_at = state.detection_at
            fresh = now - frame_at < config.VISION_STALE_TIMEOUT
            registered = state.registration_status == "REGISTERED"
            detection_present = registered and 0 <= state.target_idx < len(state.detections)
            status, error, height, zone, command = "IDLE", 0, 0, "LOST", "S"

            # Keep the tracker display alive with motion disabled.
            if detection_present and fresh:
                if frame_at != self.last_frame_at:
                    self.tracking_result = self.track(state.detections[state.target_idx])
                error, height, zone, tracking_command = self.tracking_result
                self.lost_at = None
                status = "TRACK" if state.motion_enabled else "READY"
                if state.motion_enabled:
                    command = tracking_command
            elif not fresh:
                if state.motion_enabled:
                    status = "VISION_STALE"
                elif state.robot_state == "VISION_STALE":
                    status = state.robot_state
                state.motion_enabled = False
                self.reset()
            elif state.motion_enabled:
                if self.lost_at is None:
                    self.reset()
                    self.lost_at = now
                elapsed = now - self.lost_at
                status = "SEARCH"
                search_end = config.SEARCH_TIME
                if config.SEARCH_ROTATION_ENABLED:
                    search_end += 2 * config.LOST_SWEEP_TIME
                    if config.SEARCH_TIME <= elapsed < search_end:
                        sweep = elapsed - config.SEARCH_TIME
                        clockwise = sweep < config.LOST_SWEEP_TIME
                        status = "LOST_RIGHT" if clockwise else "LOST_LEFT"
                        if sweep % (config.LOST_ROTATE_TIME + config.LOST_STOP_TIME) < config.LOST_ROTATE_TIME:
                            command = "C" if clockwise else "K"
                if elapsed >= search_end:
                    status = "WAIT_UWB"
                    state.motion_enabled = False
            else:
                if registered and state.robot_state == "WAIT_UWB":
                    status = "WAIT_UWB"
                self.reset()

            self.last_frame_at = frame_at
            state.robot_state = status
            state.control_error_x = error
            state.target_height = height
            state.distance_zone = zone
            state.last_direction = self.last_direction
            state.motor_command = command
            state.motor_command_at = now
            state.motion_input_until = frame_at + config.VISION_STALE_TIMEOUT
            state.motion_input_fault = "VISION_STALE"
            state.motor_pulse_until = float("inf")


def control_thread(ranges=None):
    follower = UwbFollower(ranges) if config.FOLLOW_MODE == "uwb" else Follower()
    while state.running:
        now = time.monotonic()
        if config.FOLLOW_MODE == "uwb" and config.UWB_FRONT_GUARD:
            expire_registration(now)
        follower.update(now)
        time.sleep(config.CONTROL_DT)
