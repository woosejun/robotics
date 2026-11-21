"""Raw Linux serial setup and bounded ASCII line framing."""

import os

BAUD_RATES = (9600, 19200, 38400, 57600, 115200)


def open_serial(port, baud, *, writable=False):
    try:
        import termios
    except ImportError as exc:
        raise OSError("Physical serial devices require Linux.") from exc
    if baud not in BAUD_RATES:
        raise ValueError(f"Unsupported baud rate: {baud}")

    access = os.O_RDWR if writable else os.O_RDONLY
    fd = os.open(port, access | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = getattr(termios, f"B{baud}")
        attrs[6][termios.VMIN] = 1
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH if writable else termios.TCIFLUSH)
        return os.fdopen(fd, "r+b" if writable else "rb", buffering=0)
    except termios.error as exc:
        os.close(fd)
        raise OSError(f"Could not configure serial port {port}: {exc}") from exc
    except (Exception, KeyboardInterrupt):
        os.close(fd)
        raise


class LineBuffer:
    MAX_LINE_BYTES = 256

    def __init__(self):
        self.pending = b""
        self.discarding = False

    def feed(self, chunk):
        parts = (self.pending + chunk).split(b"\n")
        tail = parts.pop()
        if self.discarding and parts:
            parts.pop(0)
            self.discarding = False
        if len(tail) > self.MAX_LINE_BYTES:
            self.discarding = True
        self.pending = b"" if self.discarding else tail
        return [
            part.rstrip(b"\r").decode("ascii", errors="replace")
            for part in parts if len(part) <= self.MAX_LINE_BYTES
        ]
