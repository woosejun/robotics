import os
import termios
import time

import config
import shared_state as state

# =========================================================
# UART Robot Command Thread
# =========================================================
def checksum(text):

    value = 0

    for char in text:

        value ^= ord(char)

    return value


def make_packet(sequence, robot_state, control_error_x):

    body = (
        f"{sequence},"
        f"{robot_state},"
        f"{control_error_x}"
    )

    return (
        f"${body}*"
        f"{checksum(body):02X}\n"
    )


def open_serial_port():

    fd = os.open(
        config.ROBOT_SERIAL_PORT,
        os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
    )

    attrs = termios.tcgetattr(fd)

    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = (
        termios.CS8 |
        termios.CREAD |
        termios.CLOCAL
    )
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200

    termios.tcsetattr(
        fd,
        termios.TCSANOW,
        attrs
    )

    return fd


def robot_comm_thread():

    if not config.ROBOT_SERIAL_ENABLED:

        print("[INFO] Robot UART 비활성화")

        return

    try:

        serial_fd = open_serial_port()

    except OSError as exc:

        print(
            "[WARNING] Robot UART 열기 실패:",
            config.ROBOT_SERIAL_PORT,
            exc
        )

        return

    print(
        "[INFO] Robot UART 송신:",
        config.ROBOT_SERIAL_PORT,
        config.ROBOT_SERIAL_BAUD
    )

    sequence = 0

    while state.running:

        with state.control_lock:

            local_state = state.robot_state

            local_error = int(
                state.control_error_x
            )

        packet = make_packet(
            sequence,
            local_state,
            local_error
        )

        try:

            os.write(
                serial_fd,
                packet.encode("ascii")
            )

        except OSError as exc:

            print(
                "[WARNING] Robot UART 송신 실패:",
                exc
            )

        sequence = (
            sequence + 1
        ) % 10000

        time.sleep(
            config.ROBOT_SERIAL_DT
        )

    os.close(serial_fd)
