import importlib
from pathlib import Path
import socket
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ai_vision"))

import config
import shared_state as state
from controller import Follower, request_registration, set_motion_enabled, update_distance_zone
from robot_comm import run_serial_session, safe_command
from serial_io import LineBuffer


def publish_target(frame_at, box=(270, 100, 370, 340)):
    with state.data_lock:
        state.detection_at = frame_at
        state.detections = [box]
        state.target_idx = 0
        state.registration_status = "REGISTERED"


class MotionTests(unittest.TestCase):
    def setUp(self):
        importlib.reload(state)
        self.follower = Follower()

    def test_requires_registration_and_explicit_enable(self):
        self.follower.update(time.monotonic() + 30)
        self.assertEqual((state.robot_state, state.motor_command), ("IDLE", "S"))
        self.assertIsNotNone(set_motion_enabled(True))
        publish_target(time.monotonic())
        self.follower.update(time.monotonic())
        self.assertEqual((state.robot_state, state.motor_command), ("READY", "S"))
        with patch.object(config, "ROBOT_SERIAL_ENABLED", False):
            self.assertIsNone(set_motion_enabled(True))
        self.follower.update(time.monotonic())
        self.assertEqual((state.robot_state, state.motor_command), ("TRACK", "F"))
        request_registration()
        self.follower.update(time.monotonic())
        self.assertEqual((state.motion_enabled, state.motor_command), (False, "S"))
        self.assertIsNotNone(set_motion_enabled(True))
        self.follower.update(time.monotonic() + config.REGISTRATION_TIMEOUT + 1)
        self.assertEqual(state.registration_status, "TIMED_OUT")

    def test_frozen_vision_latches_stop(self):
        now = time.monotonic()
        publish_target(now)
        with patch.object(config, "ROBOT_SERIAL_ENABLED", False):
            self.assertIsNone(set_motion_enabled(True))
        self.follower.update(now + 0.01)
        self.assertEqual(state.motor_command, "F")
        self.follower.update(now + config.VISION_STALE_TIMEOUT + 0.01)
        self.assertEqual((state.robot_state, state.motion_enabled, state.motor_command),
                         ("VISION_STALE", False, "S"))
        publish_target(now + 1)
        self.follower.update(now + 1)
        self.assertEqual((state.robot_state, state.motor_command), ("READY", "S"))

    def test_lost_target_stays_stationary_without_search_opt_in(self):
        now = time.monotonic()
        publish_target(now)
        with patch.object(config, "ROBOT_SERIAL_ENABLED", False):
            self.assertIsNone(set_motion_enabled(True))
        self.follower.update(now)
        state.target_idx = -1
        state.detection_at = now + 0.01
        self.follower.update(now + 0.01)
        self.assertEqual((state.robot_state, state.motor_command), ("SEARCH", "S"))
        # Keep supplying fresh frames without a match.
        state.detection_at = now + config.SEARCH_TIME + 0.02
        self.follower.update(state.detection_at)
        self.assertEqual((state.robot_state, state.motion_enabled, state.motor_command),
                         ("WAIT_UWB", False, "S"))

    def test_raw_height_stops_before_filter_and_release_uses_hysteresis(self):
        now = time.monotonic()
        publish_target(now)
        with patch.object(config, "ROBOT_SERIAL_ENABLED", False):
            self.assertIsNone(set_motion_enabled(True))
        self.follower.update(now)
        publish_target(now + 0.01, (270, 0, 370, 435))
        self.follower.update(now + 0.01)
        self.assertEqual((state.distance_zone, state.motor_command), ("STOP", "S"))
        self.assertEqual(update_distance_zone("STOP", 320, 420), "STOP")
        self.assertEqual(update_distance_zone("STOP", 381, 370), "STOP")
        self.assertEqual(update_distance_zone("STOP", 380, 370), "TARGET")

    def test_sender_arms_with_stop_then_latches_on_expired_command(self):
        now = time.monotonic()
        publish_target(now)
        with patch.object(config, "ROBOT_SERIAL_ENABLED", False):
            self.assertIsNone(set_motion_enabled(True))
        with state.control_lock:
            self.assertEqual(safe_command(time.monotonic(), True), "S")
            self.assertTrue(state.motion_enabled)
        self.follower.update(now + 0.01)
        with state.control_lock:
            self.assertEqual(safe_command(now + 0.02, True), "F")
            self.assertEqual(safe_command(now + config.CONTROL_STALE_TIMEOUT + 0.03, True), "S")
            self.assertFalse(state.motion_enabled)

    def test_ack_handshake_and_loss_through_virtual_serial(self):
        peer, endpoint = socket.socketpair()
        self.addCleanup(peer.close)
        stream = endpoint.makefile("rwb", buffering=0)
        endpoint.close()
        self.addCleanup(stream.close)
        peer.settimeout(1)
        errors = []

        def serve():
            try:
                run_serial_session(open_port=lambda *args, **kwargs: stream)
            except OSError as exc:
                errors.append(exc)

        worker = threading.Thread(target=serve, daemon=True)
        with patch.multiple(config, ROBOT_SERIAL_DT=0.01, ROBOT_SERIAL_ACK_TIMEOUT=0.12,
                            ROBOT_SERIAL_ENABLED=True):
            worker.start()
            try:
                self.assertEqual(peer.recv(1), b"S")
                publish_target(time.monotonic())
                self.assertIsNotNone(set_motion_enabled(True))
                peer.sendall(b"OK F\r\n")
                self.assertEqual(peer.recv(1), b"S")
                self.assertFalse(state.robot_serial_connected)
                peer.sendall(b"OK S\r\n")
                deadline = time.monotonic() + 1
                while not state.robot_serial_connected and time.monotonic() < deadline:
                    time.sleep(0.005)
                self.assertTrue(state.robot_serial_connected)
                self.assertIsNone(set_motion_enabled(True))
                self.follower.update(time.monotonic())
                while time.monotonic() < deadline and peer.recv(1) != b"F":
                    pass
                self.assertEqual(state.motor_command, "F")
                while state.motion_enabled and time.monotonic() < deadline:
                    # Keep control alive to isolate the missing serial acknowledgement.
                    self.follower.update(time.monotonic())
                    time.sleep(0.005)
                self.assertFalse(state.motion_enabled)
                self.assertFalse(state.robot_serial_connected)
                while time.monotonic() < deadline:
                    if peer.recv(1) == b"S":
                        break
                else:
                    self.fail("ESP32 stop was not sent after the ACK timeout")
            finally:
                state.running = False
                worker.join(timeout=1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])

    def test_line_buffer_discards_overlong_fragment(self):
        buffer = LineBuffer()
        self.assertEqual(buffer.feed(b"x" * 300), [])
        self.assertEqual(buffer.feed(b"ANCHOR1,2.00\nOK"), [])
        self.assertEqual(buffer.feed(b" S\r\n"), ["OK S"])


try:
    from web_tracker import app, ranges
except ModuleNotFoundError as exc:
    if exc.name != "flask":
        raise
    app = None


@unittest.skipIf(app is None, "Flask is needed for dashboard tests")
class DashboardTests(unittest.TestCase):
    def setUp(self):
        importlib.reload(state)
        self.client = app.test_client()

    def test_actions_and_uwb_payload(self):
        self.assertEqual(self.client.get("/").status_code, 200)
        self.assertEqual(self.client.get("/save").status_code, 405)
        self.assertEqual(self.client.post("/save").status_code, 415)
        self.assertEqual(self.client.post("/save", json={}).status_code, 202)
        self.assertEqual(self.client.post("/motion", json={"enabled": True}).status_code, 409)
        publish_target(time.monotonic())
        with patch.object(config, "ROBOT_SERIAL_ENABLED", False):
            self.assertEqual(self.client.post("/motion", json={"enabled": True}).status_code, 200)
        self.assertEqual(self.client.post("/motion", json={"enabled": False}).status_code, 200)

        ranges.set_connected("left", True)
        ranges.set_connected("right", True)
        self.addCleanup(ranges.set_connected, "left", False)
        self.addCleanup(ranges.set_connected, "right", False)
        ranges.record_line("left", "ANCHOR1,1.300")
        ranges.record_line("right", "ANCHOR2,1.217")
        payload = self.client.get("/get_data").get_json()
        self.assertEqual(payload["uwb_pair_status"], "OK")
        self.assertAlmostEqual(payload["uwb_position_xy_m"][0], 0.348, places=2)
        self.assertFalse(payload["motion_enabled"])
