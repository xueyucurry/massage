#!/usr/bin/env python3
"""Read-only FAIRINO six-axis force monitor."""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import socket
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Tuple

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QFrame,
    QGridLayout,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


FORCE_FIELDS: Tuple[Tuple[str, int, str, int], ...] = (
    ("Fx", 0, "N", 2),
    ("Fy", 1, "N", 2),
    ("Fz", 2, "N", 2),
    ("Mx", 3, "N·m", 3),
    ("My", 4, "N·m", 3),
    ("Mz", 5, "N·m", 3),
)


class ForceMonitorWindow(QMainWindow):
    def __init__(
        self,
        telemetry_host: str,
        telemetry_port: int,
        state_file: Path,
        axis_sign: float,
        display_hz: float,
        history_rows: int,
        stale_after_s: float,
        disconnected_after_s: float,
    ) -> None:
        super().__init__()
        self.telemetry_host = telemetry_host
        self.telemetry_port = int(telemetry_port)
        self.state_file = Path(state_file)
        self.axis_sign = -1.0 if float(axis_sign) < 0 else 1.0
        self.display_hz = min(50.0, max(1.0, float(display_hz)))
        self.history_rows = max(20, int(history_rows))
        self.stale_after_s = max(0.1, float(stale_after_s))
        self.disconnected_after_s = max(
            self.stale_after_s, float(disconnected_after_s)
        )

        self.latest_values: Dict[str, float] | None = None
        self.target_n: float | None = None
        self.session_status = ""
        self.last_message_at: float | None = None
        self.first_sample_at: float | None = None
        self.message_sequence = 0
        self.displayed_sequence = 0
        self.sample_count = 0
        self.invalid_count = 0
        self._closed = False

        self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.telemetry_socket.bind((self.telemetry_host, self.telemetry_port))
        self.telemetry_socket.setblocking(False)

        self._build_ui()
        self._refresh_target_state()

        self.telemetry_timer = QTimer(self)
        self.telemetry_timer.timeout.connect(self._poll_telemetry)
        self.telemetry_timer.start(10)

        self.sample_timer = QTimer(self)
        self.sample_timer.timeout.connect(self._display_latest_sample)
        self.sample_timer.start(max(20, int(round(1000.0 / self.display_hz))))

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_status)
        self.status_timer.start(100)

        self.target_timer = QTimer(self)
        self.target_timer.timeout.connect(self._refresh_target_state)
        self.target_timer.start(500)

    def _build_ui(self) -> None:
        self.setWindowTitle("FAIRINO 六维力传感器实时读数")
        self.setMinimumSize(840, 600)
        self.resize(1060, 760)

        central = QWidget(self)
        central.setObjectName("root")
        root = QVBoxLayout(central)
        root.setContentsMargins(20, 18, 20, 18)
        root.setSpacing(12)

        header = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("六维力传感器实时读数")
        title.setObjectName("title")
        subtitle = QLabel("显示 ft.py 已校验的力控读数，不额外调用传感器服务")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)

        self.status_label = QLabel("等待 ft.py 力数据")
        self.status_label.setObjectName("statusDisconnected")
        self.status_label.setAlignment(Qt.AlignCenter)
        header.addLayout(title_box)
        header.addStretch(1)
        header.addWidget(self.status_label)
        root.addLayout(header)

        comparison_frame = QFrame()
        comparison_frame.setObjectName("panel")
        comparison_layout = QGridLayout(comparison_frame)
        comparison_layout.setContentsMargins(12, 12, 12, 12)
        comparison_layout.setHorizontalSpacing(8)

        self.target_label = self._add_summary_cell(
            comparison_layout, 0, "目标力", "summaryTarget"
        )
        self.pressure_label = self._add_summary_cell(
            comparison_layout, 1, "实测 Fz", "summaryActual"
        )
        self.error_label = self._add_summary_cell(
            comparison_layout, 2, "|Fz| - 目标", "summaryError"
        )
        root.addWidget(comparison_frame)

        current_frame = QFrame()
        current_frame.setObjectName("panel")
        current_layout = QGridLayout(current_frame)
        current_layout.setContentsMargins(12, 12, 12, 12)
        current_layout.setHorizontalSpacing(8)
        current_layout.setVerticalSpacing(8)

        self.current_labels: Dict[str, QLabel] = {}
        for index, (label, _, unit, precision) in enumerate(FORCE_FIELDS):
            item = QFrame()
            item.setObjectName("forceCell" if index < 3 else "torqueCell")
            item_layout = QVBoxLayout(item)
            item_layout.setContentsMargins(12, 8, 12, 9)
            item_layout.setSpacing(2)

            name = QLabel(f"{label} ({unit})")
            name.setObjectName("axisName")
            value = QLabel(self._placeholder(precision))
            value.setObjectName(
                "axisForceValue" if index < 3 else "axisTorqueValue"
            )
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            item_layout.addWidget(name)
            item_layout.addWidget(value)
            current_layout.addWidget(item, index // 3, index % 3)
            self.current_labels[label] = value
        root.addWidget(current_frame)

        table_frame = QFrame()
        table_frame.setObjectName("panel")
        table_layout = QVBoxLayout(table_frame)
        table_layout.setContentsMargins(12, 10, 12, 12)
        table_layout.setSpacing(8)

        table_header = QHBoxLayout()
        table_title = QLabel("实时采样")
        table_title.setObjectName("sectionTitle")
        self.sample_label = QLabel("有效样本 0")
        self.sample_label.setObjectName("metadata")
        table_header.addWidget(table_title)
        table_header.addStretch(1)
        table_header.addWidget(self.sample_label)
        table_layout.addLayout(table_header)

        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(
            [
                "时间",
                "Time (s)",
                "Fx (N)",
                "Fy (N)",
                "Fz (N)",
                "Mx (N·m)",
                "My (N·m)",
                "Mz (N·m)",
            ]
        )
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.horizontalHeader().setMinimumSectionSize(82)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setAlternatingRowColors(True)
        self.table.setShowGrid(False)
        self.table.setFocusPolicy(Qt.NoFocus)
        table_layout.addWidget(self.table, 1)
        root.addWidget(table_frame, 1)

        self.info_label = QLabel(self._source_text())
        self.info_label.setObjectName("metadata")
        root.addWidget(self.info_label)

        self.setCentralWidget(central)
        self.setStyleSheet(
            """
            QWidget#root {
                background: #f4f7fa;
                color: #172235;
                font-family: "Microsoft YaHei UI", "Noto Sans CJK SC", sans-serif;
            }
            QLabel#title {
                font-size: 22px;
                font-weight: 700;
            }
            QLabel#subtitle, QLabel#metadata {
                color: #66758a;
                font-size: 13px;
            }
            QFrame#panel {
                background: white;
                border: 1px solid #d5dee8;
                border-radius: 5px;
            }
            QFrame#summaryCell {
                background: #f8fafc;
                border: 1px solid #d9e1ea;
                border-radius: 4px;
            }
            QLabel#summaryName {
                color: #66758a;
                font-size: 13px;
                font-weight: 700;
            }
            QLabel#summaryTarget, QLabel#summaryActual, QLabel#summaryError {
                font-family: "DejaVu Sans Mono", monospace;
                font-size: 28px;
                font-weight: 700;
            }
            QLabel#summaryTarget { color: #9a5b00; }
            QLabel#summaryActual { color: #087565; }
            QLabel#summaryError { color: #34445a; }
            QFrame#forceCell {
                background: #f1faf7;
                border: 1px solid #b9dfd3;
                border-radius: 4px;
            }
            QFrame#torqueCell {
                background: #f3f7fc;
                border: 1px solid #c5d5e8;
                border-radius: 4px;
            }
            QLabel#axisName {
                color: #5d6c80;
                font-size: 13px;
                font-weight: 700;
            }
            QLabel#axisForceValue, QLabel#axisTorqueValue {
                font-family: "DejaVu Sans Mono", monospace;
                font-size: 25px;
                font-weight: 700;
            }
            QLabel#axisForceValue { color: #087565; }
            QLabel#axisTorqueValue { color: #285f91; }
            QLabel#sectionTitle {
                font-size: 16px;
                font-weight: 700;
            }
            QLabel#statusLive, QLabel#statusStale, QLabel#statusDisconnected {
                border-radius: 4px;
                padding: 6px 12px;
                font-size: 13px;
                font-weight: 700;
            }
            QLabel#statusLive {
                color: #086b55;
                background: #dff5ee;
                border: 1px solid #9edbc8;
            }
            QLabel#statusStale {
                color: #875700;
                background: #fff3d6;
                border: 1px solid #e7c66e;
            }
            QLabel#statusDisconnected {
                color: #626d7c;
                background: #e9edf2;
                border: 1px solid #cbd4de;
            }
            QTableWidget {
                background: white;
                alternate-background-color: #f7f9fb;
                border: 1px solid #dbe2ea;
                color: #1d2939;
                font-family: "DejaVu Sans Mono", monospace;
                font-size: 13px;
            }
            QHeaderView::section {
                background: #e8eef5;
                color: #34445a;
                border: 0;
                border-right: 1px solid #d3dce6;
                border-bottom: 1px solid #ccd6e1;
                padding: 7px 4px;
                font-weight: 700;
            }
            QTableWidget::item { padding: 4px; }
            """
        )

    def _add_summary_cell(
        self, layout: QGridLayout, column: int, title: str, value_name: str
    ) -> QLabel:
        frame = QFrame()
        frame.setObjectName("summaryCell")
        cell_layout = QVBoxLayout(frame)
        cell_layout.setContentsMargins(14, 8, 14, 9)
        cell_layout.setSpacing(2)
        name = QLabel(title)
        name.setObjectName("summaryName")
        value = QLabel("--.-- N")
        value.setObjectName(value_name)
        value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        cell_layout.addWidget(name)
        cell_layout.addWidget(value)
        layout.addWidget(frame, 0, column)
        return value

    @staticmethod
    def _placeholder(precision: int) -> str:
        return "--." + ("-" * precision)

    def _source_text(self) -> str:
        return (
            f"数据源：ft.py 有效力读数 · 本机端口：{self.telemetry_port}"
            f" · 显示频率：{self.display_hz:g} Hz"
        )

    def _poll_telemetry(self) -> None:
        while True:
            try:
                payload, _ = self.telemetry_socket.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError:
                return

            try:
                message = json.loads(payload.decode("utf-8"))
                values = [float(v) for v in message["force"][:6]]
                if len(values) != 6 or not all(math.isfinite(v) for v in values):
                    raise ValueError("invalid force sample")
                if max(abs(v) for v in values[:3]) > 1000.0:
                    raise ValueError("force sample out of range")
                if max(abs(v) for v in values[3:]) > 1000.0:
                    raise ValueError("torque sample out of range")
            except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
                self.invalid_count += 1
                continue

            self.latest_values = {
                label: values[index] for label, index, _, _ in FORCE_FIELDS
            }
            try:
                target_n = abs(float(message.get("target_n")))
                if math.isfinite(target_n):
                    self.target_n = target_n
            except (TypeError, ValueError):
                pass
            self.last_message_at = time.monotonic()
            self.message_sequence += 1

    def _refresh_target_state(self) -> None:
        try:
            state = json.loads(self.state_file.read_text(encoding="utf-8"))
            target_n = abs(float(state.get("force_target_n")))
            if math.isfinite(target_n):
                self.target_n = target_n
            self.session_status = str(state.get("status") or "")
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            return
        self._render_summary()

    def _render_summary(self) -> None:
        if self.target_n is None:
            self.target_label.setText("--.-- N")
        else:
            self.target_label.setText(f"{self.target_n:.2f} N")

        if self.latest_values is None:
            self.pressure_label.setText("--.-- N")
            self.error_label.setText("--.-- N")
            return

        fz_n = self.latest_values["Fz"]
        self.pressure_label.setText(f"{fz_n:+.2f} N")
        if self.target_n is None:
            self.error_label.setText("--.-- N")
        else:
            self.error_label.setText(f"{abs(fz_n) - self.target_n:+.2f} N")

    def _display_latest_sample(self) -> None:
        if (
            self.latest_values is None
            or self.message_sequence == self.displayed_sequence
        ):
            return

        now = time.monotonic()
        if self.first_sample_at is None:
            self.first_sample_at = now
        elapsed = now - self.first_sample_at
        self.displayed_sequence = self.message_sequence
        self.sample_count += 1

        for label, _, _, precision in FORCE_FIELDS:
            self.current_labels[label].setText(
                f"{self.latest_values[label]:+.{precision}f}"
            )
        self._render_summary()

        row = self.table.rowCount()
        self.table.insertRow(row)
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        cells = [
            timestamp,
            f"{elapsed:.2f}",
            f"{self.latest_values['Fx']:.2f}",
            f"{self.latest_values['Fy']:.2f}",
            f"{self.latest_values['Fz']:.2f}",
            f"{self.latest_values['Mx']:.3f}",
            f"{self.latest_values['My']:.3f}",
            f"{self.latest_values['Mz']:.3f}",
        ]
        for column, text in enumerate(cells):
            item = QTableWidgetItem(text)
            item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.table.setItem(row, column, item)

        while self.table.rowCount() > self.history_rows:
            self.table.removeRow(0)
        self.table.scrollToBottom()
        self.sample_label.setText(
            f"有效样本 {self.sample_count} · 丢弃异常 {self.invalid_count}"
        )

    def _refresh_status(self) -> None:
        if self.last_message_at is None:
            self._set_status("等待 ft.py 力数据", "statusDisconnected")
            status_suffix = f" · 任务状态：{self.session_status}" if self.session_status else ""
            self.info_label.setText(self._source_text() + status_suffix)
            return

        age_s = time.monotonic() - self.last_message_at
        if age_s > self.disconnected_after_s:
            self._set_status("当前未采样", "statusDisconnected")
        elif age_s > self.stale_after_s:
            self._set_status("数据延迟", "statusStale")
        else:
            self._set_status("实时", "statusLive")
        self.info_label.setText(
            self._source_text() + f" · 最新数据 {age_s:.2f} 秒前"
        )

    def _set_status(self, text: str, object_name: str) -> None:
        if (
            self.status_label.text() == text
            and self.status_label.objectName() == object_name
        ):
            return
        self.status_label.setText(text)
        self.status_label.setObjectName(object_name)
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)

    def shutdown(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.telemetry_timer.stop()
        self.sample_timer.stop()
        self.status_timer.stop()
        self.target_timer.stop()
        self.telemetry_socket.close()


def _parse_args() -> argparse.Namespace:
    default_state_file = Path(__file__).resolve().parent / "ft_agent_state" / "current_session.json"
    parser = argparse.ArgumentParser(description="FAIRINO 六维力传感器实时读数")
    parser.add_argument(
        "--host",
        default=os.environ.get("FT_AGENT_FORCE_TELEMETRY_HOST", "127.0.0.1"),
        help="ft.py 力数据监听地址",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("FT_AGENT_FORCE_TELEMETRY_PORT", "45822")),
        help="ft.py 力数据监听端口",
    )
    parser.add_argument(
        "--state-file",
        type=Path,
        default=Path(os.environ.get("FT_AGENT_STATE_FILE", str(default_state_file))),
        help="按摩任务状态文件，用于显示目标力",
    )
    parser.add_argument(
        "--axis-sign",
        type=float,
        default=float(os.environ.get("LASTTIME_FORCE_AXIS_SIGN", "-1.0")),
        help="Fz 转换为按压力时使用的符号",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=float(os.environ.get("FORCE_MONITOR_HZ", "10.0")),
        help="界面采样显示频率，默认 10 Hz",
    )
    parser.add_argument(
        "--history",
        type=int,
        default=int(os.environ.get("FORCE_MONITOR_HISTORY", "200")),
        help="表格保留的最大行数，默认 200",
    )
    parser.add_argument("--stale-after", type=float, default=0.8)
    parser.add_argument("--disconnected-after", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    app = QApplication(sys.argv[:1])
    app.setFont(QFont("Microsoft YaHei UI", 10))

    try:
        window = ForceMonitorWindow(
            telemetry_host=args.host,
            telemetry_port=args.port,
            state_file=args.state_file,
            axis_sign=args.axis_sign,
            display_hz=args.hz,
            history_rows=args.history,
            stale_after_s=args.stale_after,
            disconnected_after_s=args.disconnected_after,
        )
    except OSError as exc:
        print(
            f"无法启动力传感器监视器：本机端口 {args.host}:{args.port} "
            f"不可用 ({exc})",
            file=sys.stderr,
        )
        return 1

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    window.show()

    try:
        return app.exec_()
    finally:
        window.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
