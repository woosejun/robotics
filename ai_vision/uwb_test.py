#!/usr/bin/env python3
"""Read UWB anchor output only; this does not start the camera or motors."""

import argparse
import os
import select
import termios
import time


BAUD_RATES = {
    9600: termios.B9600,
    115200: termios.B115200,
}


def open_serial(port, baud):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = BAUD_RATES[baud]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def main():
    parser = argparse.ArgumentParser(description="Print UWB anchor serial output")
    parser.add_argument("--right", default="/dev/ttyACM0")
    parser.add_argument("--left", default="/dev/ttyACM1")
    parser.add_argument("--baud", type=int, default=115200, choices=BAUD_RATES)
    args = parser.parse_args()

    ports = {"RIGHT": open_serial(args.right, args.baud),
             "LEFT": open_serial(args.left, args.baud)}
    buffers = {fd: b"" for fd in ports.values()}
    labels = {fd: name for name, fd in ports.items()}
    print("UWB serial test started. Stop with Ctrl+C.")
    try:
        while True:
            readable, _, _ = select.select(list(labels), [], [], 1.0)
            for fd in readable:
                data = os.read(fd, 4096)
                if not data:
                    continue
                buffers[fd] += data
                lines = buffers[fd].split(b"\n")
                buffers[fd] = lines.pop()
                for line in lines:
                    print(f"[{labels[fd]}] {line.rstrip().decode('utf-8', 'replace')}")
    except KeyboardInterrupt:
        print("\nUWB serial test stopped.")
    finally:
        for fd in ports.values():
            os.close(fd)


if __name__ == "__main__":
    main()
