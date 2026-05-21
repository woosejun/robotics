import cv2
import time

from ultralytics import YOLO

import config
import shared_state as state

# =========================================================
# TensorRT
# =========================================================
print("[INFO] TensorRT 엔진 로드 중...")

model = YOLO(
    "yolov8n.engine",
    task="detect"
)

print("[SUCCESS] TensorRT 엔진 로드 완료")

# =========================================================
# Histogram
# =========================================================
def get_hist(frame, box):

    x1, y1, x2, y2 = map(
        int,
        box
    )

    h, w, _ = frame.shape

    x1 = max(0, x1)
    y1 = max(0, y1)

    x2 = min(w, x2)
    y2 = min(h, y2)

    roi = frame[
        y1 + (y2 - y1)//4 : y2,
        x1 + (x2 - x1)//4 :
        x2 - (x2 - x1)//4
    ]

    if roi.size == 0:

        return None

    roi = cv2.resize(
        roi,
        (64, 128)
    )

    hsv = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2HSV
    )

    hist = cv2.calcHist(
        [hsv],
        [0, 1],
        None,
        [16, 16],
        [0, 180, 0, 256]
    )

    cv2.normalize(
        hist,
        hist,
        0,
        1,
        cv2.NORM_MINMAX
    )

    return hist

# =========================================================
# Inference Thread
# =========================================================
def inference_thread():

    while state.running:

        with state.frame_lock:

            if state.current_frame is None:

                frame_copy = None

            else:

                frame_copy = (
                    state.current_frame.copy()
                )

        if frame_copy is None:

            time.sleep(0.01)

            continue

        try:

            results = model(
                frame_copy,
                classes=0,
                conf=config.CONFIDENCE_THRESHOLD,
                verbose=False,
                imgsz=config.YOLO_IMGSZ
            )

            local_boxes = (
                results[0].boxes
            )

            local_scores = []

            local_target_idx = -1

            best_sim = -1

            with state.target_hist_lock:

                current_target_hist = (
                    state.target_hist
                )

            # =================================================
            # Person Compare
            # =================================================
            for i, box in enumerate(
                local_boxes
            ):

                b = (
                    box.xyxy[0]
                    .cpu()
                    .numpy()
                )

                curr_hist = get_hist(
                    frame_copy,
                    b
                )

                if (
                    current_target_hist
                    is not None
                    and
                    curr_hist
                    is not None
                ):

                    sim = cv2.compareHist(
                        current_target_hist,
                        curr_hist,
                        cv2.HISTCMP_CORREL
                    )

                else:

                    sim = 0.0

                local_scores.append(sim)

                if (
                    sim >
                    config.SIM_THRESHOLD
                    and
                    sim > best_sim
                ):

                    best_sim = sim

                    local_target_idx = i

            # =================================================
            # Histogram Update
            # =================================================
            if (
                local_target_idx != -1
                and
                current_target_hist
                is not None
            ):

                best_box = (
                    local_boxes[
                        local_target_idx
                    ]
                    .xyxy[0]
                    .cpu()
                    .numpy()
                )

                new_hist = get_hist(
                    frame_copy,
                    best_box
                )

                if (
                    new_hist is not None
                    and
                    local_scores[
                        local_target_idx
                    ]
                    >
                    config.HIST_UPDATE_THRESHOLD
                ):

                    updated_hist = (
                        cv2.addWeighted(
                            current_target_hist,
                            0.9,
                            new_hist,
                            0.1,
                            0
                        )
                    )

                    with state.target_hist_lock:

                        state.target_hist = (
                            updated_hist
                        )

            # =================================================
            # SAVE
            # =================================================
            with state.save_lock:

                do_save = (
                    state.save_requested
                )

                if do_save:

                    state.save_requested = False

            if (
                do_save
                and
                len(local_boxes) > 0
            ):

                boxes_np = (
                    local_boxes
                    .xyxy
                    .cpu()
                    .numpy()
                )

                centers = [

                    (b[0] + b[2]) / 2

                    for b in boxes_np
                ]

                dist = [

                    abs(
                        c -
                        config.CENTER_X
                    )

                    for c in centers
                ]

                best_idx = dist.index(
                    min(dist)
                )

                new_hist = get_hist(
                    frame_copy,
                    boxes_np[best_idx]
                )

                if new_hist is not None:

                    with state.target_hist_lock:

                        state.target_hist = (
                            new_hist
                        )

                    print(
                        "[INFO] "
                        "타겟 등록 완료"
                    )

            # =================================================
            # Update Detection
            # =================================================
            with state.data_lock:

                state.detections = (
                    list(local_boxes)
                )

                state.sim_scores = (
                    list(local_scores)
                )

                state.target_idx = (
                    local_target_idx
                )

        except Exception as e:

            print(
                "[TensorRT ERROR]",
                e
            )

        time.sleep(0.02)