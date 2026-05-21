import time

import config
import shared_state as state

# =========================================================
# Control Thread
# =========================================================
def control_thread():

    while state.running:

        # =====================================================
        # Detection Copy
        # =====================================================
        with state.data_lock:

            local_target_idx = (
                state.target_idx
            )

            local_detections = (
                list(state.detections)
            )

        # =====================================================
        # TRACK
        # =====================================================
        if (
            local_target_idx != -1
            and
            local_target_idx <
            len(local_detections)
        ):

            box = local_detections[
                local_target_idx
            ]

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

            # =============================================
            # Raw Error
            # =============================================
            raw_error_x = (
                center_x -
                config.CENTER_X
            )

            # =============================================
            # Virtual Target
            # =============================================
            state.virtual_error_x += (

                config.ALPHA * (

                    raw_error_x -
                    state.virtual_error_x
                )
            )

            target_error = int(
                state.virtual_error_x
            )

            # =============================================
            # Rate Limiter
            # =============================================
            delta = (
                target_error -
                state.prev_control_error
            )

            delta = max(
                -config.MAX_STEP,
                min(
                    config.MAX_STEP,
                    delta
                )
            )

            new_error = (
                state.prev_control_error
                + delta
            )

            state.prev_control_error = (
                new_error
            )

            # =============================================
            # Hysteresis Turning Logic
            # =============================================
            if state.turning == False:

                if (
                    abs(new_error)
                    >
                    config.START_TURN
                ):

                    state.turning = True

            else:

                if (
                    abs(new_error)
                    <
                    config.STOP_TURN
                ):

                    state.turning = False

            if state.turning == False:

                new_error = 0

            # =============================================
            # Direction Memory
            # =============================================
            if new_error > 0:

                state.last_direction = 1

            elif new_error < 0:

                state.last_direction = -1

            # =============================================
            # State Update
            # =============================================
            state.lost_start_time = (
                time.time()
            )

            new_state = "TRACK"

        # =====================================================
        # LOST FSM
        # =====================================================
        else:

            lost_duration = (
                time.time()
                -
                state.lost_start_time
            )

            # =============================================
            # SEARCH
            # =============================================
            if (
                lost_duration
                <
                config.SEARCH_TIME
            ):

                new_state = "SEARCH"

                new_error = 0

            # =============================================
            # LOST RIGHT
            # =============================================
            elif (
                lost_duration
                <
                config.LOST_RIGHT_TIME
            ):

                new_state = (
                    "LOST_RIGHT"
                )

                new_error = (
                    state.last_direction
                    *
                    config.SEARCH_ERROR
                )

            # =============================================
            # LOST LEFT
            # =============================================
            elif (
                lost_duration
                <
                config.LOST_LEFT_TIME
            ):

                new_state = (
                    "LOST_LEFT"
                )

                new_error = (
                    -state.last_direction
                    *
                    config.SEARCH_ERROR
                )

            # =============================================
            # WAIT UWB
            # =============================================
            else:

                new_state = (
                    "WAIT_UWB"
                )

                new_error = 0

            # =============================================
            # Internal Sync
            # =============================================
            state.virtual_error_x = (
                new_error
            )

            state.prev_control_error = (
                new_error
            )

            state.turning = False

        # =====================================================
        # Final Update
        # =====================================================
        with state.control_lock:

            state.control_error_x = (
                new_error
            )

            state.robot_state = (
                new_state
            )

        time.sleep(
            config.CONTROL_DT
        )