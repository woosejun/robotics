"""Turn two-anchor UWB positions into short motor commands."""

import math

import config
import shared_state as state


def read_uwb_input(ranges, now):
    """Caller holds data_lock while checking the optional camera interlock."""
    fix, generation, nearest, live_until = ranges.motion_sample(now)
    if config.UWB_FRONT_GUARD and not state.visual_target_is_current(now, config.VISION_STALE_TIMEOUT):
        return fix, generation, nearest, "UWB_FRONT_LOST", now
    if fix is None:
        if config.UWB_FRONT_GUARD:
            live_until = min(live_until, state.detection_at + config.VISION_STALE_TIMEOUT)
        return None, generation, nearest, "UWB_SIGNAL_LOST", live_until

    bearing = math.degrees(math.atan2(fix.x_m, fix.y_m))
    distance = math.hypot(fix.x_m, fix.y_m)
    if (fix.y_m < config.UWB_MIN_FORWARD_M or distance > config.UWB_MAX_FOLLOW_M
            or abs(bearing) > config.UWB_MAX_BEARING_DEG):
        return fix, generation, nearest, "UWB_OUTSIDE", now

    deadline = fix.observed_at + config.UWB_DRIVE_MAX_AGE_S
    if config.UWB_FRONT_GUARD:
        deadline = min(deadline, state.detection_at + config.VISION_STALE_TIMEOUT)
    return fix, generation, nearest, None, deadline


class UwbFollower:
    def __init__(self, ranges):
        self.ranges = ranges
        self.arm_epoch = None
        self.reset()

    def reset(self):
        self.near = False
        self.last_sequence = None
        self.intent = "S"
        self.confirmations = 0
        self.pulse_until = float("-inf")
        self.earliest_recheck_at = float("-inf")
        self.resample_until = float("-inf")

    def track(self, fix, nearest, now):
        distance = math.hypot(fix.x_m, fix.y_m)
        if (nearest <= config.UWB_STOP_M or distance <= config.UWB_STOP_M
                or (self.near and distance < config.UWB_RESUME_M)):
            self.near = True
            self.last_sequence = fix.sequence
            self.intent = "S"
            self.confirmations = 0
            self.pulse_until = now
            return "UWB_CLOSE", "S"
        self.near = False

        if fix.sequence != self.last_sequence and fix.observed_at >= self.earliest_recheck_at:
            self.last_sequence = fix.sequence
            bearing = math.degrees(math.atan2(fix.x_m, fix.y_m))
            turn = "C" if bearing > 0 else "K"
            threshold = config.UWB_TURN_STOP_DEG if self.intent == turn else config.UWB_TURN_START_DEG
            requested = turn if abs(bearing) >= threshold else "F"
            if requested != self.intent:
                self.intent = requested
                self.confirmations = 0
                self.pulse_until = now
            self.confirmations += 1
            if self.confirmations >= config.UWB_CONFIRM_PAIRS:
                self.pulse_until = now + config.UWB_MOTION_PULSE_S
                # Give the serial loop a tick to send S before the settling interval.
                settled_at = self.pulse_until + config.ROBOT_SERIAL_DT + config.UWB_SETTLE_S
                self.earliest_recheck_at = settled_at
                self.ranges.defer_motion_pairs_until(settled_at)
                self.resample_until = settled_at + config.UWB_RECOVERY_PAIR_WAIT_S
        return "UWB_TRACK", self.intent if now < self.pulse_until else "S"

    def update(self, now):
        with state.data_lock, state.control_lock:
            fix, generation, nearest, fault, deadline = read_uwb_input(self.ranges, now)
            command = "S"
            status = fault or "UWB_READY"
            if state.motion_enabled:
                if self.arm_epoch != state.motion_epoch:
                    self.reset()
                    self.arm_epoch = state.motion_epoch
                if generation != state.uwb_armed_generation:
                    fault = fault or "UWB_SIGNAL_LOST"
                if (fault == "UWB_SIGNAL_LOST" and generation == state.uwb_armed_generation
                        and now < min(deadline, self.resample_until)):
                    status = "UWB_MEASURING"
                    deadline = min(deadline, self.resample_until)
                elif fault:
                    state.motion_enabled = False
                    status = fault
                    self.reset()
                else:
                    status, command = self.track(fix, nearest, now)
            else:
                self.reset()

            state.robot_state = status
            state.distance_zone = "STOP" if status == "UWB_CLOSE" else "LOST"
            state.uwb_intent = self.intent if self.confirmations >= config.UWB_CONFIRM_PAIRS else "S"
            state.motor_command = command
            state.motor_command_at = now
            state.motion_input_until = deadline
            state.motion_input_fault = "UWB_INPUT_STALE"
            state.motor_pulse_until = self.pulse_until
