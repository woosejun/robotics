import time

import cv2

import config
import shared_state as state


def draw_frame(frame):
    with state.data_lock:
        fresh = time.monotonic() - state.detection_at < config.VISION_STALE_TIMEOUT
        boxes = state.detections if fresh else []
        scores = state.sim_scores
        target_idx = state.target_idx

    with state.control_lock:
        status = state.robot_state
        error = state.control_error_x
        height = state.target_height
        zone = state.distance_zone
        command = state.motor_command

    cv2.line(frame, (config.CENTER_X, 0), (config.CENTER_X, config.CAMERA_HEIGHT), (0, 255, 255), 2)
    for index, ((x1, y1, x2, y2), score) in enumerate(zip(boxes, scores)):
        selected = index == target_idx
        color = (0, 255, 0) if selected else (255, 0, 0)
        label = f"{'TARGET' if selected else 'PERSON'} {score:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(y1 - 5, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    text = f"{status}  {zone}  H={height}  X={error}  CMD={command}"
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return frame
