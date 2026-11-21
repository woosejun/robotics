"""Run UWB receiver sessions with virtual serial streams and optional Linux PTYs."""

import os
from pathlib import Path
import queue
import socket
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ai_vision"))

import config
from uwb_comm import (
    Ranges, estimate_position, parse_distance_m,
    start_anchor_workers, validate_anchor_ports,
)


class UwbProtocolTests(unittest.TestCase):
    def test_formats_and_invalid_values(self):
        cases = {
            "ANCHOR1,1.234": 1.234, "DISTANCE: 1.42 m": 1.42, "range=142cm": 1.42,
            "RANGE right 1.42 m": 1.42, '{"distance_mm":1420}': 1.42,
            "ANCHOR2 NO VALID RESPONSE": None, "DEVICE ID: deca0302": None,
            "ANCHOR1,-1.2": None, "ANCHOR2,nan": None, "ANCHOR2,inf": None,
            '{"distance_m":true}': None, '{"range_m":101}': None,
        }
        for line, expected in cases.items():
            with self.subTest(line=line):
                self.assertEqual(parse_distance_m(line), expected)

    def test_geometry(self):
        for x in (-0.35, 0.0, 0.35):
            left = ((x + 0.15) ** 2 + 1.2 ** 2) ** 0.5
            right = ((x - 0.15) ** 2 + 1.2 ** 2) ** 0.5
            estimated_x, estimated_y = estimate_position(left, right, 0.30)
            self.assertAlmostEqual(estimated_x, x)
            self.assertAlmostEqual(estimated_y, 1.2)
        self.assertIsNone(estimate_position(1.0, 2.0, 0.30))

    def test_port_configuration(self):
        with self.assertRaises(ValueError):
            validate_anchor_ports({"left": "port0", "right": "port0"})
        with self.assertRaises(ValueError):
            validate_anchor_ports({"left": "port0", "right": "port1"}, "port1")
        for baseline in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                Ranges(baseline)


class VirtualUwbTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.ranges = Ranges(0.30, clock=lambda: self.now)
        self.stop = threading.Event()
        self.addCleanup(self.stop.set)
        self.streams = {anchor: queue.Queue() for anchor in ("left", "right")}
        self.connections = []
        self.addCleanup(self.close_connections)

    def close_connections(self):
        for peer, stream in self.connections:
            peer.close()
            stream.close()

    def connect(self, anchor):
        peer, reader = socket.socketpair()
        stream = reader.makefile("rb", buffering=0)
        reader.close()
        self.connections.append((peer, stream))
        self.streams[anchor].put(stream)
        return peer

    def start(self):
        def open_port(port, baud):
            try:
                return self.streams[port].get_nowait()
            except queue.Empty as exc:
                raise OSError("Virtual port not attached") from exc
        workers = start_anchor_workers(
            {"left": "left", "right": "right"}, self.ranges, self.stop, open_port=open_port,
        )

        def cleanup():
            self.stop.set()
            for worker in workers:
                worker.join(timeout=2)
        self.addCleanup(cleanup)

    def wait_for(self, condition):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            snapshot = self.ranges.snapshot()
            if condition(snapshot):
                return snapshot
            time.sleep(0.005)
        self.fail(f"Virtual UWB state did not converge: {self.ranges.snapshot()}")

    def test_two_streams_fragmentation_pair_skew_timeout_and_reconnect(self):
        left, right = self.connect("left"), self.connect("right")
        with patch.object(config, "UWB_RECONNECT_DT", 0.02):
            self.start()
            self.wait_for(lambda s: all(a["status"] == "WAITING" for a in s["anchors"].values()))

            left.sendall(b"DEVICE ID: deca0302\r\nANCHOR1,1.")
            right.sendall(b"ANCHOR2,1.217\r\n")
            self.wait_for(lambda s: s["anchors"]["right"]["status"] == "RANGING")
            self.assertIsNone(self.ranges.snapshot()["position_m"])
            left.sendall(b"300\r\n")
            snapshot = self.wait_for(lambda s: s["pair_status"] == "OK")
            self.assertAlmostEqual(snapshot["position_m"][0], 0.348, places=2)
            self.assertAlmostEqual(snapshot["position_m"][1], 1.20, places=2)

            self.now += config.UWB_MAX_PAIR_SKEW + 0.1
            right.sendall(b"ANCHOR2,1.218\n")
            self.wait_for(lambda s: s["pair_status"] == "UNSYNCED")

            self.now += config.UWB_STALE_TIMEOUT
            right.sendall(b"ANCHOR2,1.217\n")
            stale = self.wait_for(lambda s: (
                s["anchors"]["left"]["status"] == "STALE"
                and s["anchors"]["right"]["status"] == "RANGING"
            ))
            self.assertIsNone(stale["position_m"])
            self.assertIsNone(stale["anchors"]["left"]["range_m"])

            left.sendall(b"ANCHOR2,1.300\n")
            bad_id = self.wait_for(lambda s: s["anchors"]["left"]["status"] == "ID_MISMATCH")
            self.assertIsNone(bad_id["position_m"])
            self.assertEqual(bad_id["anchors"]["left"]["received_id"], 2)

            left.close()
            self.wait_for(lambda s: s["anchors"]["left"]["status"] == "DISCONNECTED")
            replacement = self.connect("left")
            self.wait_for(lambda s: s["anchors"]["left"]["status"] == "WAITING")
            replacement.sendall(b"ANCHOR1,1.300\n")
            self.wait_for(lambda s: s["pair_status"] == "OK")

            replacement.sendall(b"ANCHOR1,2.000\n")
            invalid = self.wait_for(lambda s: s["pair_status"] == "INVALID_GEOMETRY")
            self.assertIsNone(invalid["position_m"])

    @unittest.skipUnless(os.name == "posix", "Linux/POSIX PTY test")
    def test_real_termios_with_two_pty_devices(self):
        import pty
        masters, ports = {}, {}
        for anchor in ("left", "right"):
            master, slave = pty.openpty()
            self.addCleanup(os.close, master)
            self.addCleanup(os.close, slave)
            masters[anchor], ports[anchor] = master, os.ttyname(slave)
        workers = start_anchor_workers(ports, self.ranges, self.stop)

        def cleanup():
            self.stop.set()
            for worker in workers:
                worker.join(timeout=2)
        self.addCleanup(cleanup)

        self.wait_for(lambda s: all(a["status"] == "WAITING" for a in s["anchors"].values()))
        os.write(masters["left"], b"ANCHOR1,1.300\r\n")
        os.write(masters["right"], b"ANCHOR2,1.217\r\n")
        snapshot = self.wait_for(lambda s: s["pair_status"] == "OK")
        self.assertAlmostEqual(snapshot["position_m"][0], 0.348, places=2)
