import time

import config
import shared_state as state

SEARCH_TIME = 2.0
LOST_TOTAL_TIME = 12.0
LOST_ROTATE_TIME = 0.6
LOST_STOP_TIME = 0.4


MOTOR_COMMAND_TABLE = {
    "FAR": {
        "LEFT": "L",
        "CENTER": "F",
        "RIGHT": "R",
    },
    "TARGET": {
        "LEFT": "L",
        "CENTER": "F",
        "RIGHT": "R",
    },
    "NEAR": {
        "LEFT": "L",
        "CENTER": "F",
        "RIGHT": "R",
    },
    "STOP": {
        "LEFT": "S",
        "CENTER": "S",
        "RIGHT": "S",
    },
}


def update_distance_zone(current_zone, target_height):
    if current_zone == "STOP":
        if target_height <= config.STOP_EXIT_HEIGHT:
            return "TARGET"

        return "STOP"

    if target_height >= config.STOP_ENTER_HEIGHT:
        return "STOP"

    if current_zone == "FAR":
        if target_height >= config.FAR_EXIT_HEIGHT:
            return "TARGET"

        return "FAR"

    if target_height < config.FAR_ENTER_HEIGHT:
        return "FAR"

    if target_height > config.NEAR_ENTER_HEIGHT:
        return "NEAR"

    return "TARGET"


def select_motor_command(distance_zone, control_error_x):
    # 좌우 판정 경계값 (픽셀)
    # 이 범위 내에서는 CENTER 명령만 보냄
    # 사람 추종 시 세로 방향 유지가 더 중요하므로
    # 좌우 선회 판단을 조금 더 좁게 잡습니다.
    LATERAL_THRESHOLD = 120

    if control_error_x < -LATERAL_THRESHOLD:
        horizontal_zone = "LEFT"
    elif control_error_x > LATERAL_THRESHOLD:
        horizontal_zone = "RIGHT"
    else:
        horizontal_zone = "CENTER"

    return MOTOR_COMMAND_TABLE[distance_zone][horizontal_zone]


def select_search_command(control_error_x):
    if control_error_x > 0:
        return "C"

    if control_error_x < 0:
        return "K"

    return "S"


def get_remembered_search_error():
    if state.last_direction > 0:
        return config.SEARCH_ERROR

    if state.last_direction < 0:
        return -config.SEARCH_ERROR

    return config.SEARCH_ERROR

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

        with state.target_hist_lock:
            local_target_hist = (
                state.target_hist
            )

        # =====================================================
        # TRACK
        # =====================================================
        if (
            local_target_hist is not None
            and
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

            raw_target_height = min(
                config.CAMERA_HEIGHT,
                max(0, y2 - y1),
            )

            if not state.target_height_initialized:
                state.virtual_target_height = raw_target_height
                state.target_height_initialized = True
            else:
                state.virtual_target_height += (
                    config.DISTANCE_ALPHA
                    * (
                        raw_target_height
                        - state.virtual_target_height
                    )
                )

            new_target_height = int(
                state.virtual_target_height
            )

            new_distance_zone = update_distance_zone(
                state.distance_zone,
                new_target_height,
            )

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

            new_command = select_motor_command(
                new_distance_zone,
                new_error,
            )

            if (
                not isinstance(new_command, str)
                or len(new_command) != 1
            ):
                print(
                    "[WARNING] controller에서 잘못된 명령 생성:",
                    repr(new_command),
                    "-> S로 대체"
                )
                new_command = "S"

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
                SEARCH_TIME
            ):

                new_state = "SEARCH"

                new_error = 0

                new_command = "S"

            # =============================================
            # LOST RIGHT
            # =============================================
            elif (
                lost_duration
                <
                SEARCH_TIME +
                LOST_TOTAL_TIME / 2.0
            ):

                phase = (
                    lost_duration -
                    SEARCH_TIME
                ) % (
                    LOST_ROTATE_TIME +
                    LOST_STOP_TIME
                )

                new_state = (
                    "LOST_RIGHT"
                )

                new_error = 0

                if phase < LOST_ROTATE_TIME:
                    new_command = "C"
                else:
                    new_command = "S"

            # =============================================
            # LOST LEFT
            # =============================================
            elif (
                lost_duration
                <
                SEARCH_TIME +
                LOST_TOTAL_TIME
            ):

                phase = (
                    lost_duration -
                    SEARCH_TIME
                ) % (
                    LOST_ROTATE_TIME +
                    LOST_STOP_TIME
                )

                new_state = (
                    "LOST_LEFT"
                )

                new_error = 0

                if phase < LOST_ROTATE_TIME:
                    new_command = "K"
                else:
                    new_command = "S"

            # =============================================
            # WAIT UWB
            # =============================================
            elif (
                lost_duration
                >
                SEARCH_TIME +
                LOST_TOTAL_TIME
            ):

                new_state = (
                    "WAIT_UWB"
                )

                new_error = 0

                new_command = "S"

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

            state.target_height_initialized = False
            new_target_height = 0
            new_distance_zone = "LOST"

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

            state.motor_command = (
                new_command
            )

            state.target_height = (
                new_target_height
            )

            state.distance_zone = (
                new_distance_zone
            )

        time.sleep(
            config.CONTROL_DT
        )
