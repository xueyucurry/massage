# -*- coding: utf-8 -*-
"""
GUI 显示窗口数据模型 - 用于 QML 数据绑定.
"""

from PyQt5.QtCore import QObject, pyqtProperty, pyqtSignal


class GuiDisplayModel(QObject):
    """
    GUI 主窗口的数据模型，用于 Python 和 QML 之间的数据绑定.
    """

    # 属性变化信号
    statusTextChanged = pyqtSignal()
    emotionPathChanged = pyqtSignal()
    ttsTextChanged = pyqtSignal()
    buttonTextChanged = pyqtSignal()
    modeTextChanged = pyqtSignal()
    autoModeChanged = pyqtSignal()
    massageStateChanged = pyqtSignal()
    shoulderCheckChanged = pyqtSignal()
    # Compatibility signal for legacy, hidden QML layouts.  The live health
    # panel owns its data and logic in health/gui.
    healthStateChanged = pyqtSignal()

    # 用户操作信号
    manualButtonPressed = pyqtSignal()
    manualButtonReleased = pyqtSignal()
    autoButtonClicked = pyqtSignal()
    abortButtonClicked = pyqtSignal()
    modeButtonClicked = pyqtSignal()
    sendButtonClicked = pyqtSignal(str)  # 携带输入的文本
    settingsButtonClicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        # 私有属性
        self._status_text = "状态: 未连接"
        self._emotion_path = ""  # 表情资源路径（GIF/图片）或 emoji 字符
        self._tts_text = "就绪"
        self._button_text = "开始对话"  # 自动模式按钮文本
        self._mode_text = "手动对话"  # 模式切换按钮文本
        self._auto_mode = False  # 是否自动模式
        self._is_connected = False
        self._shoulder_check_running = False
        self._shoulder_check_message = ""
        self._massage_state = {
            "robot_status_text": "未连接",
            "robot_status_color": "#8A94A6",
            "status_badge_text": "就绪",
            "task_title": "按摩机器人就绪",
            "target_label": "未选择",
            "trajectory_text": "未保存",
            "action_label": "就绪",
            "stage_label": "-",
            "point_text": "0 / 0",
            "progress_value": 0.0,
            "force_text": "-",
            "worker_text": "未运行",
            "updated_at": "-",
            "message": "就绪，等待语音指令",
        }
        # Kept only so dormant pre-existing QML components can load without
        # binding warnings.  These values are never updated or displayed; the
        # standalone health module is the sole source of health UI state.
        self._legacy_health_state = {
            "status_label": "等待接收", "status_color": "#8A94A6",
            "heart_rate_text": "--", "spo2_text": "--",
            "temperature_text": "--", "battery_text": "--",
            "received_at_text": "--", "message": "",
            "receiver_status_text": "", "receiver_running": False,
            "heart_rate_trend": [], "spo2_trend": [], "temperature_trend": [],
            "height_text": "--", "weight_text": "--", "bmi_text": "--",
            "body_metrics_source_text": "",
        }

    # 状态文本属性
    @pyqtProperty(str, notify=statusTextChanged)
    def statusText(self):
        return self._status_text

    @statusText.setter
    def statusText(self, value):
        if self._status_text != value:
            self._status_text = value
            self.statusTextChanged.emit()

    # 表情路径属性
    @pyqtProperty(str, notify=emotionPathChanged)
    def emotionPath(self):
        return self._emotion_path

    @emotionPath.setter
    def emotionPath(self, value):
        if self._emotion_path != value:
            self._emotion_path = value
            self.emotionPathChanged.emit()

    # TTS 文本属性
    @pyqtProperty(str, notify=ttsTextChanged)
    def ttsText(self):
        return self._tts_text

    @ttsText.setter
    def ttsText(self, value):
        if self._tts_text != value:
            self._tts_text = value
            self.ttsTextChanged.emit()

    # 自动模式按钮文本属性
    @pyqtProperty(str, notify=buttonTextChanged)
    def buttonText(self):
        return self._button_text

    @buttonText.setter
    def buttonText(self, value):
        if self._button_text != value:
            self._button_text = value
            self.buttonTextChanged.emit()

    # 模式切换按钮文本属性
    @pyqtProperty(str, notify=modeTextChanged)
    def modeText(self):
        return self._mode_text

    @modeText.setter
    def modeText(self, value):
        if self._mode_text != value:
            self._mode_text = value
            self.modeTextChanged.emit()

    # 自动模式标志属性
    @pyqtProperty(bool, notify=autoModeChanged)
    def autoMode(self):
        return self._auto_mode

    @autoMode.setter
    def autoMode(self, value):
        if self._auto_mode != value:
            self._auto_mode = value
            self.autoModeChanged.emit()

    # 便捷方法
    def update_status(self, status: str, connected: bool):
        """
        更新状态文本和连接状态.
        """
        self.statusText = f"状态: {status}"
        self._is_connected = connected

    def update_text(self, text: str):
        """
        更新 TTS 文本.
        """
        self.ttsText = text

    def update_emotion(self, emotion_path: str):
        """
        更新表情路径.
        """
        self.emotionPath = emotion_path

    def update_button_text(self, text: str):
        """
        更新自动模式按钮文本.
        """
        self.buttonText = text

    def update_mode_text(self, text: str):
        """
        更新模式按钮文本.
        """
        self.modeText = text

    def set_auto_mode(self, is_auto: bool):
        """
        设置自动模式.
        """
        self.autoMode = is_auto
        if is_auto:
            self.modeText = "自动对话"
        else:
            self.modeText = "手动对话"

    def _massage_value(self, key: str):
        return self._massage_state.get(key, "")

    def _set_massage_state(self, updates: dict):
        changed = False
        for key, value in updates.items():
            if self._massage_state.get(key) != value:
                self._massage_state[key] = value
                changed = True
        if changed:
            self.massageStateChanged.emit()

    @staticmethod
    def _label_action(action: str) -> str:
        labels = {
            "dian_jin": "点筋",
            "fen_jin": "分筋",
            "shun_jin": "顺筋",
            "prepare": "准备",
            "start": "启动中",
            "move_to_hover": "贴近悬空位",
            "move_to_hover_next": "移动到下一点",
            "press_down": "下压",
            "release": "释放",
            "completed": "完成",
        }
        return labels.get(str(action or "").strip(), str(action or "就绪"))

    @staticmethod
    def _label_stage(stage: str) -> str:
        labels = {
            "point_actions": "点筋 / 分筋",
            "shun_jin": "顺筋",
            "completed": "已完成",
        }
        return labels.get(str(stage or "").strip(), str(stage or "-"))

    @staticmethod
    def _label_status(status: str) -> str:
        labels = {
            "idle": "就绪",
            "detecting": "检测中",
            "detected": "检测完成",
            "running": "按摩中",
            "pausing": "暂停中",
            "paused": "已暂停",
            "stopping": "停止中",
            "stopped": "已停止",
            "completed": "已完成",
            "error": "异常",
        }
        return labels.get(str(status or "").strip(), str(status or "就绪"))

    @staticmethod
    def _status_color(status: str) -> str:
        colors = {
            "detecting": "#1D7AF3",
            "detected": "#12A87A",
            "running": "#0B8F77",
            "pausing": "#D9822B",
            "paused": "#D9822B",
            "stopping": "#7A869A",
            "stopped": "#8A94A6",
            "completed": "#12A87A",
            "error": "#D64545",
        }
        return colors.get(str(status or "").strip(), "#8A94A6")

    def update_massage_state(self, state: dict):
        """
        更新按摩机器人任务状态.
        """
        state = state or {}
        status = str(state.get("status") or "idle")
        target_label = str(state.get("target_label") or "未选择")
        point_count = int(state.get("point_count") or 0)
        current_index = int(state.get("current_point_index") or 0)
        display_index = 0
        completed_and_stopped = status == "stopped" and state.get("stage") == "completed"
        if point_count > 0 and status in {"running", "pausing", "paused"}:
            display_index = min(point_count, max(1, current_index + 1))
        elif point_count > 0 and (status == "completed" or completed_and_stopped):
            display_index = point_count

        progress_value = 0.0
        if point_count > 0:
            progress_value = max(0.0, min(1.0, float(display_index) / float(point_count)))

        actions = state.get("actions") or []
        if isinstance(actions, str):
            actions = [actions]
        action_names = [self._label_action(action) for action in actions]
        action_label = self._label_action(state.get("current_action"))
        if action_label == "就绪" and action_names:
            action_label = " / ".join(action_names)

        trajectory_path = state.get("trajectory_path") or ""
        trajectory_text = "已保存" if trajectory_path else "未保存"
        if trajectory_path:
            trajectory_text = f"已保存: {str(trajectory_path).split('/')[-1]}"

        force = state.get("force_target_n")
        force_text = "-"
        try:
            if force not in (None, ""):
                force_value = float(force)
                force_number = (
                    f"{force_value:.1f}"
                    if force_value.is_integer() or abs(force_value * 10 - round(force_value * 10)) < 1e-9
                    else f"{force_value:.2f}".rstrip("0").rstrip(".")
                )
                force_text = f"{force_number} N"
        except (TypeError, ValueError):
            force_text = str(force)

        status_label = self._label_status(status)
        message = str(state.get("message") or "就绪，等待语音指令")
        worker_alive = bool(state.get("worker_alive"))
        worker_text = "运行中" if worker_alive or status in {"detecting", "running", "pausing", "stopping"} else "未运行"

        self._set_massage_state(
            {
                "robot_status_text": "已连接" if self._is_connected else "未连接",
                "robot_status_color": self._status_color(status),
                "status_badge_text": status_label,
                "task_title": f"{target_label} · {status_label}" if target_label != "未选择" else f"按摩机器人 · {status_label}",
                "target_label": target_label,
                "trajectory_text": trajectory_text,
                "action_label": action_label,
                "stage_label": self._label_stage(state.get("stage")),
                "point_text": f"{display_index} / {point_count}",
                "progress_value": progress_value,
                "force_text": force_text,
                "worker_text": worker_text,
                "updated_at": str(state.get("updated_at") or "-"),
                "message": message,
            }
        )

    @pyqtProperty(str, notify=massageStateChanged)
    def robotStatusText(self):
        return self._massage_value("robot_status_text")

    @pyqtProperty(str, notify=massageStateChanged)
    def robotStatusColor(self):
        return self._massage_value("robot_status_color")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageStatusBadgeText(self):
        return self._massage_value("status_badge_text")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageTaskTitle(self):
        return self._massage_value("task_title")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageTargetLabel(self):
        return self._massage_value("target_label")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageTrajectoryText(self):
        return self._massage_value("trajectory_text")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageActionLabel(self):
        return self._massage_value("action_label")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageStageLabel(self):
        return self._massage_value("stage_label")

    @pyqtProperty(str, notify=massageStateChanged)
    def massagePointText(self):
        return self._massage_value("point_text")

    @pyqtProperty(float, notify=massageStateChanged)
    def massageProgressValue(self):
        return float(self._massage_value("progress_value") or 0.0)

    @pyqtProperty(str, notify=massageStateChanged)
    def massageForceText(self):
        return self._massage_value("force_text")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageWorkerText(self):
        return self._massage_value("worker_text")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageUpdatedAt(self):
        return self._massage_value("updated_at")

    @pyqtProperty(str, notify=massageStateChanged)
    def massageMessage(self):
        return self._massage_value("message")

    def update_shoulder_check(self, running: bool, message: str):
        if (self._shoulder_check_running, self._shoulder_check_message) != (running, message):
            self._shoulder_check_running = running
            self._shoulder_check_message = message
            self.shoulderCheckChanged.emit()

    @pyqtProperty(bool, notify=shoulderCheckChanged)
    def shoulderCheckRunning(self):
        return self._shoulder_check_running

    @pyqtProperty(str, notify=shoulderCheckChanged)
    def shoulderCheckMessage(self):
        return self._shoulder_check_message

    def _legacy_health_value(self, key: str):
        return self._legacy_health_state.get(key, "")

    @pyqtProperty(str, notify=healthStateChanged)
    def healthStatusLabel(self): return self._legacy_health_value("status_label")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthStatusColor(self): return self._legacy_health_value("status_color")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthHeartRateText(self): return self._legacy_health_value("heart_rate_text")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthSpo2Text(self): return self._legacy_health_value("spo2_text")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthTemperatureText(self): return self._legacy_health_value("temperature_text")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthBatteryText(self): return self._legacy_health_value("battery_text")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthReceivedAtText(self): return self._legacy_health_value("received_at_text")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthMessage(self): return self._legacy_health_value("message")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthReceiverStatusText(self): return self._legacy_health_value("receiver_status_text")
    @pyqtProperty(bool, notify=healthStateChanged)
    def healthReceiverRunning(self): return bool(self._legacy_health_value("receiver_running"))
    @pyqtProperty("QVariantList", notify=healthStateChanged)
    def healthHeartRateTrend(self): return self._legacy_health_value("heart_rate_trend")
    @pyqtProperty("QVariantList", notify=healthStateChanged)
    def healthSpo2Trend(self): return self._legacy_health_value("spo2_trend")
    @pyqtProperty("QVariantList", notify=healthStateChanged)
    def healthTemperatureTrend(self): return self._legacy_health_value("temperature_trend")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthHeightText(self): return self._legacy_health_value("height_text")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthWeightText(self): return self._legacy_health_value("weight_text")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthBmiText(self): return self._legacy_health_value("bmi_text")
    @pyqtProperty(str, notify=healthStateChanged)
    def healthBodyMetricsSourceText(self): return self._legacy_health_value("body_metrics_source_text")
