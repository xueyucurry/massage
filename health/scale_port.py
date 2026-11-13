"""Linux serial-port setup shared by the scale probe and GUI adapter."""

from __future__ import annotations

import copy
import termios


def configure_9600_8n1(fd: int) -> list:
    previous = termios.tcgetattr(fd)
    settings = copy.deepcopy(previous)
    settings[0] = 0
    settings[1] = 0
    settings[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    settings[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    if hasattr(termios, "CRTSCTS"):
        settings[2] &= ~termios.CRTSCTS
    settings[3] = 0
    settings[4] = termios.B9600
    settings[5] = termios.B9600
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    return previous
