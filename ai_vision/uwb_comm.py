"""Receive UWB range reports from the robot's left and right anchors.

This module deliberately does not send configuration commands to an anchor.
Different Decawave/Qorvo firmwares use different command protocols, while the
anchor already connected to this robot is known to print a boot banner.  Once
the firmware streams ranges, common text and JSON range formats are accepted.
"""

import json
import math
import os
import re
import select
import termios
import time

import config
import shared_state as state


BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}

# Examples accepted: "DISTANCE: 1.42", "range=142cm",
# "RANGE right 1.42 m", and JSON such as {"distance_m": 1.42}.
DISTANCE_PATTERN = re.compile(
    r"(?:distance|range|dist)\b[^\d+-]{0,32}"
    r"([-+]?\d+(?:\.\d+)?)\s*(mm|cm|m)?\b",
    re.IGNORECASE,
)

ANCHOR_DISTANCE_PATTERN = re.compile(
    r"^\s*anchor[12]\s*,\s*([-+]?\d+(?:\.\d+)?)\s*$",
    re.IGNORECASE,
)


def open_uwb_port(port, baud=None):
    """Open a raw, non-blocking serial port without transmitting to it."""
    baud = baud or config.UWB_BAUD
    try:
        baud_flag = BAUD_RATES[baud]
    except KeyError as exc:
        raise ValueError(f"지원하지 않는 UWB baud rate: {baud}") from exc

    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = baud_flag
        attrs[5] = baud_flag
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def _meters(value, unit):
    unit = (unit or "m").lower()
    if unit == "mm":
        return value / 1000.0
    if unit == "cm":
        return value / 100.0
    return value


def parse_distance_m(line):
    """Return a plausible range in metres, or None for non-range log lines."""
    try:
        payload = json.loads(line)
    except (TypeError, json.JSONDecodeError):
        payload = None

    if isinstance(payload, dict):
        for key, factor in (("distance_m", 1.0), ("range_m", 1.0),
                            ("distance_cm", 0.01), ("range_cm", 0.01),
                            ("distance_mm", 0.001), ("range_mm", 0.001)):
            value = payload.get(key)
            if isinstance(value, (int, float)):
                distance_m = float(value) * factor
                return distance_m if 0.05 <= distance_m <= 100.0 else None

    match = ANCHOR_DISTANCE_PATTERN.match(line)
    if match:
        distance_m = float(match.group(1))
        return distance_m if 0.05 <= distance_m <= 100.0 else None

    match = DISTANCE_PATTERN.search(line)
    if not match:
        return None
    distance_m = _meters(float(match.group(1)), match.group(2))
    return distance_m if 0.05 <= distance_m <= 100.0 else None


def _update_position_locked():
    left = state.uwb_left_distance_m
    right = state.uwb_right_distance_m
    baseline = config.UWB_ANCHOR_BASELINE_M
    if left is None or right is None or baseline <= 0:
        state.uwb_position_x_m = None
        state.uwb_position_y_m = None
        return

    # Left=(0, 0), right=(baseline, 0).  x is rightward from left anchor;
    # y is the forward (positive) solution for a person in front of robot.
    x = (left * left - right * right + baseline * baseline) / (2 * baseline)
    y_squared = left * left - x * x
    if y_squared < -0.02:  # impossible triangle: wait for a cleaner reading
        state.uwb_position_x_m = None
        state.uwb_position_y_m = None
        return
    state.uwb_position_x_m = x - baseline / 2.0  # robot-centre lateral axis
    state.uwb_position_y_m = math.sqrt(max(0.0, y_squared))


def record_range(anchor, distance_m, line):
    """Store a range report and update the two-anchor tag position."""
    now = time.monotonic()
    with state.uwb_lock:
        if anchor == "left":
            state.uwb_left_distance_m = distance_m
        else:
            state.uwb_right_distance_m = distance_m
        state.uwb_last_update_time = now
        state.uwb_last_message = line
        _update_position_locked()


def _set_connected(anchor, connected):
    with state.uwb_lock:
        setattr(state, f"uwb_{anchor}_connected", connected)


def _read_lines(fd, buffer):
    try:
        chunk = os.read(fd, 4096)
    except BlockingIOError:
        return [], buffer
    if not chunk:
        return [], buffer
    buffer += chunk
    parts = buffer.split(b"\n")
    lines = [part.rstrip(b"\r").decode("utf-8", errors="replace")
             for part in parts[:-1]]
    return lines, parts[-1]


def run_anchor_session(anchor, port):
    fd = open_uwb_port(port)
    print(f"[INFO] UWB {anchor} anchor 수신 시작: {port} @ {config.UWB_BAUD}")
    _set_connected(anchor, True)
    buffer = b""
    try:
        while state.running:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if not readable:
                continue
            lines, buffer = _read_lines(fd, buffer)
            for line in lines:
                distance_m = parse_distance_m(line)
                if distance_m is not None:
                    record_range(anchor, distance_m, line)
    finally:
        _set_connected(anchor, False)
        os.close(fd)


def _anchor_thread(anchor, port):
    while state.running:
        try:
            run_anchor_session(anchor, port)
        except (OSError, ValueError) as exc:
            _set_connected(anchor, False)
            print(f"[WARNING] UWB {anchor} anchor 연결 실패: {port}: {exc}")
            if state.running:
                time.sleep(config.UWB_RECONNECT_DT)


def uwb_comm_thread():
    """Run in one daemon thread; starts a receiver for each configured anchor."""
    if not config.UWB_ENABLED:
        print("[INFO] UWB 비활성화")
        return

    import threading
    workers = []
    for anchor, port in (("left", config.UWB_LEFT_PORT),
                         ("right", config.UWB_RIGHT_PORT)):
        if port:
            worker = threading.Thread(target=_anchor_thread, args=(anchor, port),
                                      daemon=True)
            worker.start()
            workers.append(worker)
    while state.running:
        time.sleep(0.5)
