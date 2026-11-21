"""Receive and pair distance measurements from the two robot anchors."""

from dataclasses import dataclass
import json
import math
import os
import re
import select
import threading
import time

import config
from serial_io import LineBuffer, open_serial

ANCHOR_IDS = {"left": 1, "right": 2}
ANCHOR_REPORT = re.compile(r"^\s*ANCHOR([12])\s*,\s*(.+?)\s*$", re.IGNORECASE)
TEXT_REPORT = re.compile(
    r"^\s*(?:distance|range|dist)\b\s*(?:(?:left|right)\s+)?[:=]?\s*"
    r"([-+]?\d+(?:\.\d+)?)\s*(mm|cm|m)?\s*$", re.IGNORECASE,
)
UNIT_FACTOR = {"m": 1.0, "cm": 0.01, "mm": 0.001}


def parse_distance_m(line):
    """Accept the firmware's CSV, named text fields, and named JSON fields."""
    report = ANCHOR_REPORT.fullmatch(line)
    if report:
        try:
            distance = float(report[2])
        except ValueError:
            return None
        return distance if 0.05 <= distance <= 100.0 else None

    match = TEXT_REPORT.fullmatch(line)
    if match:
        distance = float(match[1]) * UNIT_FACTOR[(match[2] or "m").lower()]
        return distance if 0.05 <= distance <= 100.0 else None

    if not line.lstrip().startswith("{"):
        return None
    try:
        payload = json.loads(line)
        for name in ("distance", "range"):
            for unit, factor in UNIT_FACTOR.items():
                value = payload.get(f"{name}_{unit}")
                if type(value) in (int, float) and 0 < value <= 100.0 / factor:
                    distance = value * factor
                    return distance if 0.05 <= distance <= 100.0 else None
    except (ValueError, AttributeError):
        pass
    return None


def estimate_position(left, right, baseline):
    """Return (rightward x, positive forward y) in metres, assuming a front tag."""
    if not math.isfinite(baseline) or baseline <= 0:
        return None
    if not abs(left - right) <= baseline <= left + right:
        return None
    x = (left * left - right * right) / (2 * baseline)
    y_squared = left * left - (x + baseline / 2) ** 2
    return x, math.sqrt(max(0.0, y_squared))


def validate_anchor_ports(ports, motor_port=None):
    active = [os.path.realpath(port) for port in ports.values() if port]
    if len(set(active)) != len(active):
        raise ValueError("왼쪽과 오른쪽 UWB 앵커는 서로 다른 포트를 사용해야 합니다.")
    if motor_port and os.path.realpath(motor_port) in active:
        raise ValueError("UWB와 모터 컨트롤러에 같은 포트가 지정되어 있습니다.")


@dataclass
class AnchorReading:
    connected: bool = False
    distance: float = None
    measured_at: float = None
    wrong_id: int = None
    drive_sequence: int = 0


@dataclass(frozen=True)
class MotionFix:
    x_m: float
    y_m: float
    observed_at: float
    sequence: int


class Ranges:
    def __init__(self, baseline, *, clock=time.monotonic):
        if not math.isfinite(baseline) or baseline <= 0:
            raise ValueError("UWB 안테나 간격은 양의 유한한 값이어야 합니다.")
        self.baseline = baseline
        self.clock = clock
        self.lock = threading.Lock()
        self.readings = {anchor: AnchorReading() for anchor in ANCHOR_IDS}
        self.sequence = 0
        self.motion_generation = 0
        self.motion_fix = None
        self.paired_sequences = (0, 0)
        self.pair_after = float("-inf")

    def _invalidate_motion(self):
        self.motion_fix = None
        self.motion_generation += 1
        self.paired_sequences = (self.sequence, self.sequence)

    def set_connected(self, anchor, connected):
        with self.lock:
            # Reconnect starts with a new measurement on this port.
            self.readings[anchor] = AnchorReading(connected=connected)
            self._invalidate_motion()

    def record_line(self, anchor, line):
        distance = parse_distance_m(line)
        if distance is None:
            return
        report = ANCHOR_REPORT.fullmatch(line)
        anchor_id = int(report[1]) if report else ANCHOR_IDS[anchor]
        with self.lock:
            reading = self.readings[anchor]
            now = self.clock()
            if anchor_id != ANCHOR_IDS[anchor]:
                reading.wrong_id = anchor_id
                reading.distance = reading.measured_at = None
                reading.drive_sequence = 0
                self._invalidate_motion()
                return
            if reading.drive_sequence and now - reading.measured_at >= config.UWB_DRIVE_MAX_AGE_S:
                self._invalidate_motion()
            reading.distance = distance
            reading.measured_at = now
            reading.wrong_id = None
            self.sequence += 1
            reading.drive_sequence = self.sequence if report and reading.connected else 0
            if not reading.drive_sequence:
                self._invalidate_motion()
                return

            left, right = self.readings["left"], self.readings["right"]
            if left.drive_sequence <= self.paired_sequences[0] or right.drive_sequence <= self.paired_sequences[1]:
                return
            if min(left.measured_at, right.measured_at) < self.pair_after:
                return
            if abs(left.measured_at - right.measured_at) > config.UWB_DRIVE_MAX_SKEW_S:
                return
            self.paired_sequences = (left.drive_sequence, right.drive_sequence)
            position = estimate_position(left.distance, right.distance, self.baseline)
            if position is None:
                self._invalidate_motion()
                return
            self.motion_fix = MotionFix(
                *position, min(left.measured_at, right.measured_at), self.sequence,
            )

    def motion_sample(self, now):
        """Return the current pair, continuity generation, closest range and live deadline."""
        with self.lock:
            fix = self.motion_fix
            if fix is not None and now - fix.observed_at >= config.UWB_DRIVE_MAX_AGE_S:
                fix = None
            latest = [reading for reading in self.readings.values() if reading.drive_sequence]
            live_until = float("-inf")
            if len(latest) == len(ANCHOR_IDS):
                live_until = min(reading.measured_at for reading in latest) + config.UWB_DRIVE_MAX_AGE_S
            nearest = min((reading.distance for reading in latest), default=float("inf"))
            return fix, self.motion_generation, nearest, live_until

    def defer_motion_pairs_until(self, deadline):
        """After movement, compose the next position from two stationary readings."""
        with self.lock:
            self.pair_after = max(self.pair_after, deadline)

    def snapshot(self):
        with self.lock:
            now = self.clock()
            anchors = {}
            for anchor, reading in self.readings.items():
                age = now - reading.measured_at if reading.measured_at is not None else None
                if not reading.connected:
                    status = "DISCONNECTED"
                elif reading.wrong_id is not None:
                    status = "ID_MISMATCH"
                elif age is None:
                    status = "WAITING"
                elif age >= config.UWB_STALE_TIMEOUT:
                    status = "STALE"
                else:
                    status = "RANGING"
                anchors[anchor] = {
                    "status": status,
                    "range_m": reading.distance if status == "RANGING" else None,
                    "age_s": age,
                    "received_id": reading.wrong_id,
                }

            position = None
            pair_status = "WAITING"
            if all(reading["status"] == "RANGING" for reading in anchors.values()):
                left, right = anchors["left"], anchors["right"]
                if abs(left["age_s"] - right["age_s"]) > config.UWB_MAX_PAIR_SKEW:
                    pair_status = "UNSYNCED"
                else:
                    position = estimate_position(left["range_m"], right["range_m"], self.baseline)
                    pair_status = "OK" if position is not None else "INVALID_GEOMETRY"
            return {"anchors": anchors, "pair_status": pair_status, "position_m": position}


def run_anchor_session(anchor, port, ranges, stop, *, baud=config.UWB_BAUD,
                       open_port=open_serial, on_line=None):
    with open_port(port, baud) as stream:
        ranges.set_connected(anchor, True)
        lines = LineBuffer()
        try:
            while not stop.is_set():
                readable, _, _ = select.select([stream], [], [], 0.2)
                if not readable:
                    continue
                chunk = stream.read(4096)
                if chunk == b"":
                    raise OSError("Serial device disconnected")
                if chunk:
                    for line in lines.feed(chunk):
                        ranges.record_line(anchor, line)
                        if on_line is not None:
                            on_line(anchor, line)
        finally:
            ranges.set_connected(anchor, False)


def anchor_worker(anchor, port, ranges, stop, **options):
    while not stop.is_set():
        try:
            run_anchor_session(anchor, port, ranges, stop, **options)
        except (OSError, ValueError) as exc:
            print(f"[WARNING] UWB {anchor} ({port}): {exc}")
            stop.wait(config.UWB_RECONNECT_DT)


def start_anchor_workers(ports, ranges, stop, **options):
    workers = [
        threading.Thread(
            target=anchor_worker, args=(anchor, port, ranges, stop),
            kwargs=options, name=f"uwb-{anchor}", daemon=True,
        )
        for anchor, port in ports.items() if port
    ]
    for worker in workers:
        worker.start()
    return workers
