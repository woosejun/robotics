#!/usr/bin/env python3
"""Inspect UWB anchors and position estimates with the production receiver."""

import argparse
import json
import math
import threading
import time

import config
from serial_io import BAUD_RATES
from uwb_comm import Ranges, start_anchor_workers, validate_anchor_ports


def main():
    parser = argparse.ArgumentParser(description="Inspect UWB range, pairing and port health")
    parser.add_argument("--left", default=config.UWB_LEFT_PORT, help="ANCHOR1 port (empty to skip)")
    parser.add_argument("--right", default=config.UWB_RIGHT_PORT, help="ANCHOR2 port (empty to skip)")
    parser.add_argument("--baud", type=int, default=config.UWB_BAUD, choices=BAUD_RATES)
    parser.add_argument("--baseline", type=float, default=config.UWB_ANCHOR_BASELINE_M)
    parser.add_argument("--duration", type=float, default=0, help="Seconds, 0 runs until Ctrl+C")
    parser.add_argument("--raw", action="store_true", help="Also print raw firmware lines")
    args = parser.parse_args()
    ports = {"left": args.left, "right": args.right}
    if not any(ports.values()) or not math.isfinite(args.duration) or args.duration < 0:
        parser.error("Supply at least one serial port and a nonnegative duration")
    try:
        validate_anchor_ports(ports, config.ROBOT_SERIAL_PORT)
        ranges = Ranges(args.baseline)
    except ValueError as exc:
        parser.error(str(exc))

    stop = threading.Event()
    workers = start_anchor_workers(
        ports, ranges, stop, baud=args.baud,
        on_line=(lambda anchor, line: print(f"[{anchor}] {line}")) if args.raw else None,
    )
    deadline = time.monotonic() + args.duration if args.duration else float("inf")
    try:
        while not stop.wait(min(1.0, max(0.0, deadline - time.monotonic()))):
            print(json.dumps(ranges.snapshot(), ensure_ascii=False))
            if time.monotonic() >= deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for worker in workers:
            worker.join()


if __name__ == "__main__":
    main()
