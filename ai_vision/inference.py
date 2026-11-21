"""Run person detection and match a registered clothing histogram."""

import os
import time

import cv2

import config
import shared_state as state


def get_hist(frame, box):
    x1, y1, x2, y2 = box
    roi = frame[y1 + (y2 - y1) // 4:y2, x1 + (x2 - x1) // 4:x2 - (x2 - x1) // 4]
    if roi.size == 0:
        return None

    hsv = cv2.cvtColor(cv2.resize(roi, (64, 128)), cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
    return cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)


def detection_boxes(model, frame):
    result = model(
        frame, classes=0, conf=config.CONFIDENCE_THRESHOLD, verbose=False,
        imgsz=config.YOLO_IMGSZ,
    )[0]
    height, width = frame.shape[:2]
    boxes = []
    for x1, y1, x2, y2 in result.boxes.xyxy.cpu().tolist():
        x1, x2 = max(0, int(x1)), min(width, int(x2))
        y1, y2 = max(0, int(y1)), min(height, int(y2))
        if x1 < x2 and y1 < y2:
            boxes.append((x1, y1, x2, y2))
    return boxes


def match_target(target_hist, histograms):
    scores = [
        float(cv2.compareHist(target_hist, hist, cv2.HISTCMP_CORREL))
        if target_hist is not None and hist is not None else 0.0
        for hist in histograms
    ]
    ranked = sorted(range(len(scores)), key=scores.__getitem__, reverse=True)
    if not ranked or scores[ranked[0]] <= config.SIM_THRESHOLD:
        return -1, scores
    if len(ranked) > 1 and scores[ranked[0]] - scores[ranked[1]] < config.SIM_MIN_MARGIN:
        return -1, scores
    return ranked[0], scores


def inference_thread():
    model_path = config.YOLO_MODEL_PATH or os.path.join(
        os.path.dirname(__file__), "models", "yolov8n.engine"
    )
    if not os.path.isfile(model_path):
        print(f"[ERROR] YOLO model missing: {model_path}")
        return
    try:
        from ultralytics import YOLO

        model = YOLO(model_path, task="detect")
    except Exception as exc:
        print(f"[ERROR] YOLO model could not be initialized: {exc}")
        return

    target_hist = None
    processed_at = float("-inf")
    while state.running:
        now = time.monotonic()
        with state.data_lock:
            request_at = state.registration_requested_at
            if request_at is not None and now - request_at >= config.REGISTRATION_TIMEOUT:
                state.registration_requested_at = None
                state.registration_status = "TIMED_OUT"
                request_at = None
            if state.registration_status != "REGISTERED":
                target_hist = None

        with state.frame_lock:
            frame, frame_at = state.current_frame, state.frame_at
        if frame is None or frame_at <= processed_at or now - frame_at >= config.VISION_STALE_TIMEOUT:
            time.sleep(0.01)
            continue

        processed_at = frame_at
        try:
            boxes = detection_boxes(model, frame)
            histograms = [get_hist(frame, box) for box in boxes]
            target_index, scores = match_target(target_hist, histograms)
            if target_index >= 0 and scores[target_index] > config.HIST_UPDATE_THRESHOLD:
                target_hist = cv2.addWeighted(
                    target_hist, 0.9, histograms[target_index], 0.1, 0
                )
        except Exception as exc:
            with state.data_lock:
                state.detections = []
                state.sim_scores = []
                state.target_idx = -1
                state.detection_at = float("-inf")
            print(f"[ERROR] YOLO inference: {exc}")
            time.sleep(0.1)
            continue

        with state.data_lock:
            # A new registration requested during this inference needs a newer frame.
            if request_at != state.registration_requested_at:
                continue
            now = time.monotonic()
            if now - frame_at >= config.VISION_STALE_TIMEOUT:
                continue

            if request_at is not None and frame_at >= request_at:
                if now - request_at >= config.REGISTRATION_TIMEOUT:
                    state.registration_requested_at = None
                    state.registration_status = "TIMED_OUT"
                else:
                    candidates = [i for i, hist in enumerate(histograms) if hist is not None]
                    if candidates:
                        target_index = min(
                            candidates,
                            key=lambda i: abs((boxes[i][0] + boxes[i][2]) / 2 - config.CENTER_X),
                        )
                        target_hist = histograms[target_index]
                        scores[target_index] = 1.0
                        state.registration_requested_at = None
                        state.registration_status = "REGISTERED"
            state.detections = boxes
            state.sim_scores = scores
            state.target_idx = target_index
            # Keep the camera timestamp so a frozen frame cannot refresh the command.
            state.detection_at = frame_at
