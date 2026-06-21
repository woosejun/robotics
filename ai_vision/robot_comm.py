import argparse
import os
import select
import termios
import time

import config
import shared_state as state


BAUD_RATES = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}

ROBOT_COMMANDS = frozenset("FBLRCKS12345678!@#$")

COMMAND_ALIASES = {
    "W": "F",  # keyboard forward
    "A": "L",  # keyboard left
    "D": "R",  # keyboard right
    "X": "B",  # keyboard backward; S is kept as stop
}

COMMAND_DESCRIPTIONS = {
    "F": "forward",
    "B": "backward",
    "L": "left",
    "R": "right",
    "C": "clockwise rotate",
    "K": "counter-clockwise rotate",
    "S": "stop",
}


def normalize_command(command):
    if not isinstance(command, str) or len(command) != 1:
        raise ValueError(
            f"지원하지 않는 로봇 명령: {command!r} (명령은 단일 문자여야 합니다)"
        )

    command = command.upper()
    command = COMMAND_ALIASES.get(command, command)

    if command not in ROBOT_COMMANDS:
        allowed = ", ".join(sorted(ROBOT_COMMANDS | frozenset(COMMAND_ALIASES)))
        raise ValueError(f"지원하지 않는 로봇 명령: {command} (허용: {allowed})")

    return command


def encode_command(command):
    return normalize_command(command).encode("ascii")


def open_serial_port(port=None, baud=None):
    port = port or config.ROBOT_SERIAL_PORT
    baud = baud or config.ROBOT_SERIAL_BAUD

    try:
        baud_flag = BAUD_RATES[baud]
    except KeyError as exc:
        raise ValueError(f"지원하지 않는 baud rate: {baud}") from exc

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = baud_flag
        attrs[5] = baud_flag
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
    except Exception:
        os.close(fd)
        raise

    return fd


def read_serial_lines(fd, buffer):
    """Read all available bytes and return complete ASCII lines."""
    # Bound each pass so a continuously-chatty controller cannot starve the
    # command loop.
    for _ in range(16):
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            break

        if not chunk:
            break

        buffer += chunk

    parts = buffer.split(b"\n")
    lines = [part.rstrip(b"\r").decode("ascii", errors="replace")
             for part in parts[:-1]]
    return lines, parts[-1]


def parse_ack(line):
    if not line.startswith("OK "):
        return None

    command = line[3:].strip()
    return command if command in ROBOT_COMMANDS else None


def set_connection_status(connected, command=""):
    with state.robot_serial_lock:
        state.robot_serial_connected = connected

        if command:
            state.robot_serial_last_ack = command
            state.robot_serial_last_ack_time = time.monotonic()


def verify_robot_connection(timeout=3.0, verbose=False, command="S"):
    """Send one command and wait for the matching ESP32 ACK."""
    fd = open_serial_port()
    command = normalize_command(command)
    encoded_command = encode_command(command)
    deadline = time.monotonic() + timeout
    next_send_time = 0.0
    buffer = b""

    try:
        while time.monotonic() < deadline:
            now = time.monotonic()

            if now >= next_send_time:
                os.write(fd, encoded_command)
                next_send_time = now + 0.25

                if verbose:
                    print(f"[TX] {command}")

            wait = min(0.1, max(0.0, deadline - time.monotonic()))
            readable, _, _ = select.select([fd], [], [], wait)

            if not readable:
                continue

            lines, buffer = read_serial_lines(fd, buffer)

            for line in lines:
                if verbose and line:
                    print(f"[RX] {line}")

                if parse_ack(line) == command:
                    return True, command

        return False, command
    finally:
        os.close(fd)


def run_serial_session():
    serial_fd = open_serial_port()
    print(
        "[INFO] Robot USB serial 송신:",
        config.ROBOT_SERIAL_PORT,
        config.ROBOT_SERIAL_BAUD,
    )

    receive_buffer = b""
    last_ack_time = 0.0
    ack_confirmed = False
    ack_wait_warned = False
    session_start_time = time.monotonic()

    try:
        while state.running:
            with state.control_lock:
                command = state.motor_command

            if not isinstance(command, str) or len(command) != 1:
                print(
                    "[WARNING] shared_state.motor_command가 단일 문자가 아닙니다:",
                    repr(command),
                    "-> S로 대체합니다."
                )
                command = "S"

            try:
                command = normalize_command(command)
            except ValueError:
                print(
                    "[WARNING] shared_state.motor_command가 유효하지 않은 명령입니다:",
                    repr(command),
                    "-> S로 대체합니다."
                )
                command = "S"

            os.write(serial_fd, encode_command(command))

            lines, receive_buffer = read_serial_lines(
                serial_fd,
                receive_buffer,
            )

            for line in lines:
                ack_command = parse_ack(line)

                if ack_command is not None:
                    last_ack_time = time.monotonic()
                    set_connection_status(True, ack_command)

                    if not ack_confirmed:
                        print(
                            "[INFO] Robot USB serial 통신 확인: ACK",
                            ack_command,
                        )
                        ack_confirmed = True
                        ack_wait_warned = False

            if (
                not ack_confirmed
                and not ack_wait_warned
                and time.monotonic() - session_start_time
                > config.ROBOT_SERIAL_ACK_TIMEOUT
            ):
                print(
                    "[WARNING] USB 포트는 열렸지만 Robot ACK 응답이 없음"
                )
                ack_wait_warned = True

            if (
                ack_confirmed
                and time.monotonic() - last_ack_time
                > config.ROBOT_SERIAL_ACK_TIMEOUT
            ):
                print("[WARNING] Robot USB serial ACK 응답 없음")
                set_connection_status(False)
                ack_confirmed = False

            time.sleep(config.ROBOT_SERIAL_DT)
    finally:
        set_connection_status(False)
        os.close(serial_fd)


def robot_comm_thread():
    if not config.ROBOT_SERIAL_ENABLED:
        print("[INFO] Robot USB serial 비활성화")
        return

    while state.running:
        try:
            run_serial_session()
        except (OSError, ValueError) as exc:
            set_connection_status(False)
            print(
                "[WARNING] Robot USB serial 연결 실패:",
                config.ROBOT_SERIAL_PORT,
                exc,
            )

            if state.running:
                time.sleep(config.ROBOT_SERIAL_RECONNECT_DT)


def main():
    parser = argparse.ArgumentParser(
        description="ESP32 USB serial 연결과 ACK를 확인합니다."
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="ACK 대기 시간(초), 기본값: 3",
    )
    parser.add_argument(
        "--command",
        default="S",
        help=(
            "시험할 모터 명령. 기본값: S(정지). "
            "허용: F/B/L/R/C/K/S, 별칭: W=F, A=L, D=R, X=B"
        ),
    )
    args = parser.parse_args()

    try:
        connected, command = verify_robot_connection(
            args.timeout,
            verbose=True,
            command=args.command,
        )
    except (OSError, ValueError) as exc:
        print(f"[FAIL] USB serial 포트 열기 실패: {exc}")
        raise SystemExit(1) from exc

    if connected:
        print(f"[PASS] USB serial 통신 정상: ACK {command}")
        return

    print(f"[FAIL] 명령 {command} 송신 후 ACK 응답 없음")
    raise SystemExit(2)


if __name__ == "__main__":
    main()
