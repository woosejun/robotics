import cv2

import config
import shared_state as state

# =========================================================
# Draw Frame
# =========================================================
def draw_frame(frame):

    # =====================================================
    # Detection Copy
    # =====================================================
    with state.data_lock:

        local_detections = (
            list(state.detections)
        )

        local_scores = (
            list(state.sim_scores)
        )

        local_target_idx = (
            state.target_idx
        )

    # =====================================================
    # Control Copy
    # =====================================================
    with state.control_lock:

        local_error_x = (
            state.control_error_x
        )

        local_state = (
            state.robot_state
        )

        local_direction = (
            state.last_direction
        )

        local_target_height = (
            state.target_height
        )

        local_distance_zone = (
            state.distance_zone
        )

        local_motor_command = (
            state.motor_command
        )

    # =====================================================
    # Center Line
    # =====================================================
    cv2.line(
        frame,
        (
            config.CENTER_X,
            0
        ),
        (
            config.CENTER_X,
            config.CAMERA_HEIGHT
        ),
        (0, 255, 255),
        2
    )

    # =====================================================
    # Draw Detection
    # =====================================================
    for i, box in enumerate(
        local_detections
    ):

        if i >= len(local_scores):

            break

        b = (
            box.xyxy[0]
            .cpu()
            .numpy()
            .astype(int)
        )

        x1, y1, x2, y2 = b

        center_x = (
            x1 + x2
        ) // 2

        is_target = (
            i == local_target_idx
        )

        color = (
            (0, 255, 0)
            if is_target
            else (255, 0, 0)
        )

        score_str = (
            f"{local_scores[i]:.2f}"
        )

        # =============================================
        # MASTER
        # =============================================
        if is_target:

            label = (
                f"MASTER "
                f"{score_str} "
                f"| ERR: "
                f"{local_error_x}"
            )

            cv2.line(
                frame,
                (
                    config.CENTER_X,
                    240
                ),
                (
                    center_x,
                    240
                ),
                (0, 255, 0),
                2
            )

        # =============================================
        # PERSON
        # =============================================
        else:

            label = (
                f"PERSON "
                f"{score_str}"
            )

        # =============================================
        # Draw Box
        # =============================================
        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            color,
            2
        )

        cv2.putText(
            frame,
            label,
            (
                x1,
                max(y1 - 5, 12)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2
        )

    # =====================================================
    # State
    # =====================================================
    cv2.putText(
        frame,
        f"STATE: {local_state}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"ERR_X: {local_error_x}",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        f"DIR: {local_direction}",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.putText(
        frame,
        (
            f"DIST: {local_distance_zone} "
            f"H={local_target_height} "
            f"CMD={local_motor_command}"
        ),
        (10, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    # =====================================================
    # WAIT UWB
    # =====================================================
    if local_state == "WAIT_UWB":

        cv2.putText(
            frame,
            "UWB RECOVERY",
            (180, 240),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            3
        )

    return frame
