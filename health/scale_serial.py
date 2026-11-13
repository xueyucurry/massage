"""Optional, receive-only serial adapter for completed scale measurements."""

from __future__ import annotations

import os
import termios
import time
from collections import deque
from pathlib import Path

from PyQt5.QtCore import QCoreApplication, QObject, QTimer, pyqtSignal

from scale_port import configure_9600_8n1
from scale_protocol import ScaleFrameStream, ScaleMeasurement


DEFAULT_PORT = Path(
    "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_AVBIb2B3V13-if00-port0"
)


class ScaleSerialAdapter(QObject):
    measurementReceived = pyqtSignal(object)
    statusChanged = pyqtSignal(str)

    def __init__(self, parent: QObject | None = None, port: Path | str | None = None) -> None:
        super().__init__(parent)
        self.port = Path(port or os.getenv("MASSAGE_SCALE_PORT") or DEFAULT_PORT)
        self._fd: int | None = None
        self._previous_settings: list | None = None
        self._stream = ScaleFrameStream()
        self._activity_times: deque[float] = deque()
        self._next_open_at = 0.0
        self._status = ""
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._poll)
        app = QCoreApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.stop)

    @property
    def status(self) -> str:
        return self._status

    def _set_status(self, value: str) -> None:
        if value != self._status:
            self._status = value
            self.statusChanged.emit(value)

    def start(self) -> None:
        self._timer.start()
        self._poll()

    def stop(self) -> None:
        self._timer.stop()
        self._close()

    def _close(self) -> None:
        if self._fd is None:
            return
        try:
            if self._previous_settings is not None:
                termios.tcsetattr(self._fd, termios.TCSANOW, self._previous_settings)
        except (OSError, termios.error):
            pass
        os.close(self._fd)
        self._fd = None
        self._previous_settings = None
        self._stream = ScaleFrameStream()
        self._activity_times.clear()

    def _open(self) -> None:
        try:
            fd = os.open(self.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                previous = configure_9600_8n1(fd)
            except Exception:
                os.close(fd)
                raise
        except PermissionError:
            self._set_status("电子秤串口无权限，请将当前用户加入 dialout 组后重新登录")
        except FileNotFoundError:
            self._set_status("电子秤未连接")
        except OSError as exc:
            self._set_status(f"电子秤串口打开失败：{exc.strerror or exc}")
        else:
            self._fd = fd
            self._previous_settings = previous
            self._stream = ScaleFrameStream()
            self._set_status("电子秤已连接，等待测量")

    def _poll(self) -> None:
        if self._fd is None:
            now = time.monotonic()
            if now >= self._next_open_at:
                self._next_open_at = now + 5
                self._open()
            return
        try:
            data = os.read(self._fd, 4096)
        except BlockingIOError:
            return
        except OSError:
            self._close()
            self._next_open_at = time.monotonic() + 5
            self._set_status("电子秤连接中断，正在重连")
            return
        if not data:
            return
        activity_count, measurements = self._stream.feed(data)
        now = time.monotonic()
        while self._activity_times and now - self._activity_times[0] > 20:
            self._activity_times.popleft()
        if activity_count:
            self._activity_times.extend([now] * activity_count)
        for measurement in measurements:
            if len(self._activity_times) >= 3:
                self.measurementReceived.emit(measurement)
                self._activity_times.clear()
