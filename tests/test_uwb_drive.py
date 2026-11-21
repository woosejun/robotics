"""Exercise UWB motion with a robot model and virtual USB streams."""

import importlib
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ai_vision"))

import config
import controller
import shared_state as state
from robot_comm import run_serial_session, safe_command
from uwb_comm import Ranges, start_anchor_workers
from uwb_drive import UwbFollower


def anchor_message(anchor, x, y, baseline, error=0.0):
    offset = baseline / 2 if anchor == "left" else -baseline / 2
    distance = math.hypot(x + offset, y) + error
    return f"ANCHOR{1 if anchor == 'left' else 2},{distance:.3f}\n"


class UwbDriveTests(unittest.TestCase):
    def setUp(self):
        importlib.reload(state)
        self.addCleanup(patch.stopall)
        patch.multiple(config, FOLLOW_MODE="uwb", UWB_FRONT_GUARD=False,
                       UWB_ENABLED=True, ROBOT_SERIAL_ENABLED=False).start()
        self.now = 10.0
        self.ranges = Ranges(0.30, clock=lambda: self.now)
        self.ranges.set_connected("left", True)
        self.ranges.set_connected("right", True)
        self.follower = UwbFollower(self.ranges)

    def report(self, anchor, x, y):
        self.ranges.record_line(anchor, anchor_message(anchor, x, y, self.ranges.baseline))

    def pair(self, x, y):
        for anchor in ("left", "right"):
            self.report(anchor, x, y)

    def arm(self):
        with patch.object(controller.time, "monotonic", return_value=self.now):
            return controller.set_motion_enabled(True, self.ranges)

    def tick(self, dt):
        self.now += dt
        self.follower.update(self.now)
        with state.control_lock:
            return safe_command(self.now, True)

    def test_independent_pairs_stale_and_required_rearm(self):
        self.report("left", 0, 1.8)
        self.assertIsNotNone(self.arm())
        self.report("right", 0, 1.8)
        self.assertIsNone(self.arm())
        self.assertEqual(self.tick(0.01), "S")
        self.report("left", 0, 1.8)
        self.assertEqual(self.tick(0.01), "S")
        self.report("right", 0, 1.8)
        self.assertEqual(self.tick(0.01), "F")
        # Production sender expires the pulse even if the controller stalls.
        with state.control_lock:
            expiry = state.motor_pulse_until
            self.assertEqual(safe_command(expiry, True), "S")
            self.assertTrue(state.motion_enabled)

        self.tick(config.UWB_DRIVE_MAX_AGE_S + 0.01)
        self.assertFalse(state.motion_enabled)
        self.pair(0, 1.8)
        self.assertEqual(self.tick(0.01), "S")
        self.assertFalse(state.motion_enabled)

    def test_mismatched_and_unidentified_reports_cannot_drive(self):
        self.pair(0, 1.8)
        self.assertIsNone(self.arm())
        self.tick(0.01)
        # Even if two new valid reports arrive before the controller tick, the
        # fault generation forces an explicit restart.
        self.ranges.record_line("left", "ANCHOR2,1.8")
        self.pair(0, 1.8)
        self.assertEqual(self.tick(0.01), "S")
        self.assertFalse(state.motion_enabled)
        self.ranges.record_line("left", "DISTANCE: 1.8")
        self.report("right", 0, 1.8)
        self.assertIsNotNone(self.arm())

    def test_near_range_stops_current_pulse_before_pair_updates(self):
        self.pair(0, 1.8)
        self.assertIsNone(self.arm())
        self.tick(0.01)
        self.pair(0, 1.8)
        self.assertEqual(self.tick(0.01), "F")
        self.report("left", 0, 0.65)
        self.assertEqual(self.tick(0.01), "S")
        self.assertEqual(state.robot_state, "UWB_CLOSE")

    def test_front_guard_and_close_hold_hysteresis(self):
        with patch.object(config, "UWB_FRONT_GUARD", True):
            self.pair(0, 1.5)
            self.assertIsNotNone(self.arm())
            state.registration_status = "REGISTERED"
            state.target_idx = 0
            state.detections = [(270, 100, 370, 340)]
            state.detection_at = self.now
            self.assertIsNone(self.arm())
            self.tick(0.01)
            state.target_idx = -1
            self.assertEqual(self.tick(0.01), "S")
            self.assertEqual(state.robot_state, "UWB_FRONT_LOST")
            self.assertFalse(state.motion_enabled)
        self.pair(0, 0.80)
        self.assertIsNone(self.arm())
        self.assertEqual(self.tick(0.01), "S")
        self.assertEqual(state.robot_state, "UWB_CLOSE")
        self.pair(0, 0.97)
        self.assertEqual(self.tick(0.01), "S")
        self.assertEqual(state.robot_state, "UWB_CLOSE")
        for _ in range(2):
            self.pair(0, 1.15)
            command = self.tick(0.01)
        self.assertEqual(command, "F")

    def test_outside_following_envelope_requires_rearm(self):
        self.pair(0, 1.5)
        self.assertIsNone(self.arm())
        self.tick(0.01)
        self.pair(1.4, 0.30)
        self.assertEqual(self.tick(0.01), "S")
        self.assertEqual(state.robot_state, "UWB_OUTSIDE")
        self.assertFalse(state.motion_enabled)
        self.pair(0, config.UWB_MAX_FOLLOW_M + 0.3)
        self.assertIsNotNone(self.arm())

    def test_closed_loop_converges_from_left_and_right(self):
        for starting_x in (1.0, -1.0):
            with self.subTest(starting_x=starting_x):
                self.reset_scenario(starting_x)
                duration, distance, commands = self.simulate_chassis(starting_x)
                self.assertGreaterEqual(distance, 0.67)
                self.assertLessEqual(distance, 0.88)
                self.assertIn("C" if starting_x > 0 else "K", commands)
                self.assertIn("F", commands)
                self.assertNotIn("B", commands)
                print(f"Closed-loop x={starting_x:+.1f} m: stop at {distance:.3f} m in {duration:.2f} s")

    def reset_scenario(self, x):
        importlib.reload(state)
        self.now = 10.0
        self.ranges = Ranges(0.30, clock=lambda: self.now)
        self.follower = UwbFollower(self.ranges)
        for anchor in ("left", "right"):
            self.ranges.set_connected(anchor, True)
        self.pair(x, 2.30)
        self.assertIsNone(self.arm())

    def simulate_chassis(self, x):
        y = 2.30
        start_time = self.now
        times = {"left": self.now + 0.23, "right": self.now + 0.27}
        periods = {"left": 0.23, "right": 0.27}
        used = set()
        command = "S"
        closest = math.hypot(x, y)
        for step in range(3000):
            self.now += 0.01
            for anchor in times:
                if self.now >= times[anchor]:
                    noise = 0.002 * math.sin(self.now * (15 if anchor == "left" else 13))
                    self.ranges.record_line(anchor, anchor_message(anchor, x, y, 0.30, noise))
                    times[anchor] += periods[anchor]
            if step % 3 == 0:
                self.follower.update(self.now)
            if step % 5 == 0:
                with state.control_lock:
                    command = safe_command(self.now, True)
                used.add(command)
            if command == "F":
                y -= 0.45 * 0.01
            elif command in ("C", "K"):
                angle = math.radians(80 * 0.01) * (1 if command == "C" else -1)
                x, y = x * math.cos(angle) - y * math.sin(angle), x * math.sin(angle) + y * math.cos(angle)
            closest = min(closest, math.hypot(x, y))
            self.assertTrue(state.motion_enabled, f"unexpected stop at {self.now}: {state.robot_state}")
            if state.robot_state == "UWB_CLOSE" and self.now - start_time > 4 and command == "S":
                return self.now - start_time, closest, used
        self.fail(f"Chassis did not settle: x={x:.2f}, y={y:.2f}, state={state.robot_state}")


class UwbToMotorIntegration(unittest.TestCase):
    def test_anchor_serial_to_firmware_masks_and_recovery(self):
        try:
            import web_tracker
        except ModuleNotFoundError as exc:
            if exc.name != "flask":
                raise
            self.skipTest("Flask is required to test the operator dashboard")
        importlib.reload(state)
        ranges = Ranges(0.30)
        records = []
        errors = []
        bridge_stop = threading.Event()
        anchor_stop = threading.Event()
        endpoints = {}
        peers = {}

        def attach(port, mode):
            peer, sock = socket.socketpair()
            try:
                stream = sock.makefile(mode, buffering=0)
            finally:
                sock.close()
            self.addCleanup(peer.close)
            self.addCleanup(stream.close)
            peers[port], endpoints[port] = peer, stream

        def open_port(port, baud, **kwargs):
            try:
                return endpoints.pop(port)
            except KeyError as exc:
                raise OSError(f"Virtual device {port} is unplugged") from exc

        firmware_path = os.environ.get("ROBOTICS_TEST_MOTOR_FIRMWARE")
        firmware = (
            subprocess.Popen([firmware_path], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             text=True, encoding="ascii")
            if firmware_path else None
        )
        masks = {"S": 0x00, "F": 0xA3, "C": 0xAC, "K": 0x53}

        def exercise_motor(command):
            if firmware is None:
                return "OK " + command, masks[command]
            firmware.stdin.write(command + "\n")
            firmware.stdin.flush()
            ack = ""
            for line in firmware.stdout:
                line = line.rstrip()
                if line.startswith("MASK "):
                    return ack, int(line[5:])
                if line.startswith("OK "):
                    ack = line
            raise OSError("Native ESP32 sketch stopped responding")

        def emulate_peripheral():
            peer = peers["motor"]
            peer.settimeout(0.1)
            try:
                while not bridge_stop.is_set():
                    try:
                        payload = peer.recv(1)
                    except socket.timeout:
                        continue
                    if not payload:
                        return
                    command = payload.decode("ascii")
                    ack, mask = exercise_motor(command)
                    if ack != "OK " + command:
                        raise OSError(f"Expected ACK for {command}, received {ack!r}")
                    records.append((command, mask))
                    peer.sendall((ack + "\r\n").encode("ascii"))
            except (OSError, ValueError, KeyError) as exc:
                if state.running or not isinstance(exc, (ConnectionResetError, BrokenPipeError)):
                    errors.append(exc)

        def run_motor_port():
            try:
                run_serial_session(open_port=open_port)
            except OSError as exc:
                errors.append(exc)

        def wait_for(condition, message):
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                if condition():
                    return
                if errors:
                    self.fail(str(errors))
                time.sleep(0.01)
            self.fail(f"{message}; state={state.robot_state}; tail={records[-8:]}")

        def report(x, y):
            for name in ("left", "right"):
                peers[name].sendall(anchor_message(name, x, y, ranges.baseline).encode("ascii"))

        def report_until(x, y, condition, message):
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                report(x, y)
                time.sleep(0.06)
                if condition():
                    return
                if errors:
                    self.fail(str(errors))
            self.fail(f"{message}; state={state.robot_state}; tail={records[-8:]}")

        def expect_mask(x, y, command):
            cursor = len(records)
            report_until(x, y, lambda: (command, masks[command]) in records[cursor:],
                         f"Expected firmware response to {command}")

        for name in ("left", "right", "motor"):
            attach(name, "rwb" if name == "motor" else "rb")
        with patch.multiple(config, FOLLOW_MODE="uwb", UWB_FRONT_GUARD=False,
                            UWB_ENABLED=True, ROBOT_SERIAL_ENABLED=True,
                            ROBOT_SERIAL_PORT="motor", UWB_RECONNECT_DT=0.05), patch.object(
                                web_tracker, "ranges", ranges
                            ):
            control = threading.Thread(target=controller.control_thread, args=(ranges,), daemon=True)
            motor_port = threading.Thread(target=run_motor_port, daemon=True)
            motor_bridge = threading.Thread(target=emulate_peripheral, daemon=True)
            anchor_workers = start_anchor_workers(
                {"left": "left", "right": "right"}, ranges, anchor_stop, open_port=open_port,
            )
            for worker in (control, motor_bridge, motor_port):
                worker.start()
            try:
                client = web_tracker.app.test_client()
                self.assertNotIn(b"/video_feed", client.get("/").data)
                self.assertEqual(client.post("/motion", json={"enabled": True}).status_code, 409)
                wait_for(lambda: state.robot_serial_connected, "ESP32 stop ACK missing")
                wait_for(
                    lambda: all(a["status"] == "WAITING" for a in ranges.snapshot()["anchors"].values()),
                    "Virtual anchors did not open",
                )
                report(0, 1.75)
                wait_for(lambda: state.robot_state == "UWB_READY", "UWB pair not ready")
                self.assertTrue(all(record == ("S", 0) for record in records))
                self.assertEqual(client.post("/motion", json={"enabled": True}).status_code, 200)
                expect_mask(0, 1.75, "F")
                expect_mask(0.85, 1.70, "C")
                expect_mask(-0.85, 1.70, "K")
                expect_mask(0, 0.70, "S")
                wait_for(lambda: state.robot_state == "UWB_CLOSE", "Near target did not hold")
                expect_mask(0, 1.75, "F")

                peers["left"].close()
                cutoff = len(records)
                wait_for(lambda: not state.motion_enabled, "Disconnect did not disarm")
                wait_for(lambda: ("S", 0) in records[cutoff:], "Disconnect did not stop motor")
                attach("left", "rb")
                wait_for(
                    lambda: ranges.snapshot()["anchors"]["left"]["status"] == "WAITING",
                    "Left anchor did not reconnect",
                )
                report_until(0, 1.75, lambda: state.robot_state == "UWB_READY",
                             "No fresh pair after reconnect")
                cutoff = len(records)
                report(0, 1.75)
                time.sleep(0.20)
                self.assertTrue(records[cutoff:] and all(record == ("S", 0) for record in records[cutoff:]))
                self.assertEqual(client.post("/motion", json={"enabled": True}).status_code, 200)
                expect_mask(0, 1.75, "F")
                self.assertEqual(client.post("/motion", json={"enabled": False}).status_code, 200)
                cutoff = len(records)
                wait_for(lambda: ("S", 0) in records[cutoff:], "Operator stop did not reach firmware")
                self.assertFalse(state.motion_enabled)
                self.assertEqual(client.get("/get_data").get_json()["follow_mode"], "uwb")
                print(f"UWB serial -> {'production .ino' if firmware else 'virtual ESP32'}: "
                      "F/A3, C/AC, K/53, S/00, disconnect, re-arm")
            finally:
                state.running = False
                anchor_stop.set()
                for worker in [control, motor_port, *anchor_workers]:
                    worker.join(timeout=1)
                bridge_stop.set()
                motor_bridge.join(timeout=1)
                if firmware:
                    firmware.stdin.close()
                    firmware.wait(timeout=2)
                    firmware.stdout.close()
            self.assertEqual(errors, [])
