import importlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

VISION_DIR = Path(__file__).resolve().parents[1] / "ai_vision"
sys.path.insert(0, str(VISION_DIR))

import config
import shared_state as state
from controller import Follower, request_registration, set_motion_enabled


class InferenceTests(unittest.TestCase):
    def setUp(self):
        importlib.reload(state)
        spec = importlib.util.spec_from_file_location("isolated_inference", VISION_DIR / "inference.py")
        self.inference = importlib.util.module_from_spec(spec)
        self.cv2 = ModuleType("cv2")
        self.cv2.HISTCMP_CORREL = 0
        self.cv2.compareHist = lambda registered, candidate, method: candidate
        with patch.dict(sys.modules, cv2=self.cv2):
            spec.loader.exec_module(self.inference)

    def run_inference(self, now, detection_boxes):
        ultralytics = ModuleType("ultralytics")
        ultralytics.YOLO = lambda *args, **kwargs: object()
        clock = SimpleNamespace(
            monotonic=lambda: now,
            sleep=lambda _: setattr(state, "running", False),
        )
        with tempfile.TemporaryDirectory() as temporary:
            engine = Path(temporary) / "model.engine"
            engine.touch()
            with patch.dict(sys.modules, ultralytics=ultralytics), patch.multiple(
                self.inference, time=clock, detection_boxes=detection_boxes, get_hist=lambda *args: 1.0,
            ), patch.object(config, "YOLO_MODEL_PATH", str(engine)):
                self.inference.inference_thread()

    def test_registration_and_frozen_frame_is_processed_only_once(self):
        request_registration()
        state.current_frame = object()
        state.frame_at = time.monotonic()
        calls = []
        box = (270, 80, 370, 340)

        def detect(model, frame):
            calls.append(frame)
            return [box]

        self.run_inference(state.frame_at + 0.01, detect)
        self.assertEqual(calls, [state.current_frame])
        self.assertEqual((state.target_idx, state.registration_status), (0, "REGISTERED"))
        self.assertEqual(state.detection_at, state.frame_at)

    def test_inference_failure_invalidates_last_motor_input(self):
        now = time.monotonic()
        state.current_frame = object()
        state.frame_at = now
        state.detection_at = now - 0.01
        state.detections = [(270, 80, 370, 340)]
        state.target_idx = 0
        state.registration_status = "REGISTERED"
        with patch.object(config, "ROBOT_SERIAL_ENABLED", False):
            self.assertIsNone(set_motion_enabled(True))
        follower = Follower()
        follower.update(now)
        self.assertEqual(state.motor_command, "F")

        def detect(model, frame):
            raise RuntimeError("Virtual model failure")

        self.run_inference(now + 0.01, detect)
        follower.update(now + 0.01)
        self.assertEqual((state.robot_state, state.motor_command, state.motion_enabled),
                         ("VISION_STALE", "S", False))

    def test_ambiguous_clothing_candidates_hold_position(self):
        target, _ = self.inference.match_target(object(), [0.65, 0.62, None])
        self.assertEqual(target, -1)
        target, _ = self.inference.match_target(object(), [0.65, 0.25, None])
        self.assertEqual(target, 0)
