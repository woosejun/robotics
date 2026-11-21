"""Transport the motor command to the ESP32 over a single-byte protocol."""

import argparse
import math
import select
import time

import config
import shared_state as state
from serial_io import LineBuffer, open_serial

MOTION_COMMANDS = frozenset("FBLRCKS")
ALIASES = {"W": "F", "A": "L", "D": "R", "X": "B"}


def normalize_command(command):
    if not isinstance(command, str) or len(command) != 1:
        raise ValueError("A motor command must be a single character.")
    command = command.upper()
    command = ALIASES.get(command, command)
    if command not in MOTION_COMMANDS:
        raise ValueError(f"Unsupported motor command: {command!r}")
    return command


def read_acknowledgements(stream, buffer, timeout=0):
    readable, _, _ = select.select([stream], [], [], timeout)
    if not readable:
        return []
    chunk = stream.read(4096)
    if chunk == b"":
        raise OSError("Motor serial device disconnected")
    if not chunk:
        return []
    return [
        line[3:] for line in buffer.feed(chunk)
        if line.startswith("OK ") and line[3:] in MOTION_COMMANDS
    ]


def write_command(stream, command):
    if stream.write(command.encode("ascii")) != 1:
        raise OSError("Motor serial write did not complete")


def set_link(connected, *, ack="", now=float("-inf")):
    # Match the lock order used when authorizing motion.
    with state.control_lock, state.robot_serial_lock:
        state.robot_serial_connected = connected
        state.robot_serial_last_ack = ack
        state.robot_serial_last_ack_time = now
        if not connected:
            state.motion_enabled = False
            state.motor_command = "S"


def safe_command(now, link_ready):
    # Caller holds control_lock. Either stale producer can force a stop.
    if not (state.running and state.motion_enabled and link_ready):
        return "S"
    if now >= state.motion_input_until and now < state.motor_pulse_until:
        state.robot_state = state.motion_input_fault
    elif now - state.motor_command_at >= config.CONTROL_STALE_TIMEOUT:
        state.robot_state = "CONTROL_STALE"
    elif state.motor_command in MOTION_COMMANDS:
        return state.motor_command if now < state.motor_pulse_until else "S"
    state.motion_enabled = False
    state.motor_command = "S"
    state.motor_command_at = float("-inf")
    return state.motor_command


def run_serial_session(*, open_port=open_serial):
    with open_port(config.ROBOT_SERIAL_PORT, config.ROBOT_SERIAL_BAUD, writable=True) as stream:
        buffer = LineBuffer()
        last_sent = ""
        last_ack_at = float("-inf")
        link_ready = False
        set_link(False)
        try:
            while state.running:
                received = read_acknowledgements(stream, buffer)
                now = time.monotonic()
                if link_ready and now - last_ack_at >= config.ROBOT_SERIAL_ACK_TIMEOUT:
                    print("[WARNING] 모터 ACK 시간 초과. 주행을 다시 활성화해야 합니다.")
                    link_ready = False
                    last_sent = ""
                    set_link(False)
                if last_sent and last_sent in received:
                    # Until linked, last_sent can only be the stop command.
                    last_ack_at = now
                    link_ready = True
                    set_link(True, ack=last_sent, now=now)

                with state.control_lock:
                    last_sent = safe_command(now, link_ready)
                    write_command(stream, last_sent)
                time.sleep(config.ROBOT_SERIAL_DT)
        finally:
            set_link(False)
            try:
                write_command(stream, "S")
            except OSError:
                # The ESP32 has its own timeout when the cable is gone.
                pass


def robot_comm_thread():
    if not config.ROBOT_SERIAL_ENABLED:
        return
    while state.running:
        try:
            run_serial_session()
        except (OSError, ValueError) as exc:
            set_link(False)
            print(f"[WARNING] 모터 직렬 포트 {config.ROBOT_SERIAL_PORT}: {exc}")
            time.sleep(config.ROBOT_SERIAL_RECONNECT_DT)


def verify_robot_connection(timeout, command="S"):
    command = normalize_command(command)
    pending = "S"
    buffer = LineBuffer()
    with open_serial(config.ROBOT_SERIAL_PORT, config.ROBOT_SERIAL_BAUD, writable=True) as stream:
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                write_command(stream, pending)
                wait = min(0.1, max(0.0, deadline - time.monotonic()))
                if pending in read_acknowledgements(stream, buffer, wait):
                    if pending == command:
                        return True
                    pending = command
            return False
        finally:
            try:
                write_command(stream, "S")
            except OSError:
                pass


def main():
    parser = argparse.ArgumentParser(description="Check the ESP32 serial link")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--command", default="S", help="Default S checks ACK without motion")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive")
    try:
        connected = verify_robot_connection(args.timeout, args.command)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"[FAIL] {exc}\n")
    if not connected:
        parser.exit(2, "[FAIL] No matching motor ACK\n")
    print(f"[PASS] Motor ACK {normalize_command(args.command)}")


if __name__ == "__main__":
    main()
