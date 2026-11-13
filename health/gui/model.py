"""All health-panel state and persistence lives in the standalone health project."""

import json
import os
import sqlite3
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from PyQt5.QtCore import QObject, QTimer, pyqtProperty, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QGuiApplication

HEALTH_DIR = Path(__file__).resolve().parents[1]
if str(HEALTH_DIR) not in sys.path:
    sys.path.insert(0, str(HEALTH_DIR))
from massage_profile import MassageRecommendationStore, recommendation_for_bmi
from vital_alerts import VitalAlertEngine, valid_vital_value
from data_source import resolve_active_source
from scale_serial import ScaleSerialAdapter


DATABASE_PATH = HEALTH_DIR / "data" / "health_band.sqlite3"
BODY_METRICS_PATH = HEALTH_DIR / "data" / "body_metrics.json"
SETTINGS_PATH = HEALTH_DIR / "data" / "gui_health_settings.json"
HEALTH_SCRIPT = HEALTH_DIR / "health"
VITAL_COLUMNS = ("heart_rate_bpm", "spo2_percent", "temperature_c")
MAX_TREND_POINTS = 12
LOW_BATTERY_PERCENT = 20
IDLE_ALERT_TEXT = {
    "spo2_watch": ("血氧持续低于95%，请检查佩戴并复测。", "血氧持续低于95%，请检查手环佩戴，并确认是否不适。"),
    "spo2_urgent": ("血氧持续低于或等于88%，请立即复测并尽快就医。", "血氧持续低于88%，请立即复测并尽快就医。"),
    "temp_high": ("手环体温读数持续达到或超过38℃，请用体温计复测；如确认发热或有不适，请咨询医生。", "手环体温读数持续高于38℃，请立即用体温计复测；如确认发热或有不适，请及时就医。"),
    "temp_urgent": ("手环体温读数持续低于或等于35℃，请立即用体温计复测；如复测仍低或有不适，请及时就医。", "手环体温读数持续低于35℃，请立即用体温计复测；如仍偏低或有不适，请及时就医。"),
    "hr_rise": ("心率较个人基线明显升高，请确认是否不适并检查手环佩戴。", "心率较个人基线明显升高，请检查手环佩戴，并确认是否不适。"),
    "hr_fall": ("心率较个人基线明显下降，请确认是否头晕或不适并检查手环佩戴。", "心率较个人基线明显下降，请检查手环佩戴，并确认是否头晕或不适。"),
}


def _number(value: Any) -> Optional[float]:
    try:
        return None if value in (None, "") else float(value)
    except (TypeError, ValueError):
        return None


def _as_utc(value: Any) -> Optional[datetime]:
    try:
        parsed = datetime.fromisoformat(str(value))
        return parsed.replace(tzinfo=timezone.utc) if parsed.tzinfo is None else parsed.astimezone(timezone.utc)
    except (TypeError, ValueError):
        return None


class BodyMetricsStore:
    """Private local profile data; later devices use ``update_from_device``."""

    def __init__(self, path: Optional[Path | str] = None) -> None:
        self.path = Path(path or BODY_METRICS_PATH)

    def summary(self) -> Dict[str, Any]:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            data = {}
        height, weight = _number(data.get("height_cm")), _number(data.get("weight_kg"))
        bmi = weight / ((height / 100) ** 2) if height and weight else None
        return {"height_cm": height, "weight_kg": weight, "bmi": bmi, "source": str(data.get("source") or "未录入")}

    def _save(self, height_cm: Any, weight_kg: Any, source: str) -> Dict[str, Any]:
        height, weight = _number(height_cm), _number(weight_kg)
        if height is None or not 50 <= height <= 250:
            raise ValueError("身高应在 50–250 cm 之间")
        if weight is None or not 0 < weight <= 500:
            raise ValueError("体重应大于 0 且不超过 500 kg")
        if not recommendation_for_bmi(weight / ((height / 100) ** 2))["available"]:
            raise ValueError("身高体重计算出的 BMI 超出支持范围")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"height_cm": height, "weight_kg": weight, "source": source, "updated_at": datetime.now(timezone.utc).isoformat()}
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
        temporary.replace(self.path)
        return self.summary()

    def set_manual(self, height_cm: Any, weight_kg: Any) -> Dict[str, Any]:
        return self._save(height_cm, weight_kg, "手动录入")

    def update_from_device(self, height_cm: Any, weight_kg: Any, source: str = "测量设备") -> Dict[str, Any]:
        return self._save(height_cm, weight_kg, source)


class HealthGuiModel(QObject):
    """Qt-facing health module. It reads SQLite only and never uses MQTT credentials."""

    stateChanged = pyqtSignal()
    alertRaised = pyqtSignal(str, int)

    def __init__(self) -> None:
        super().__init__()
        self._session_started = datetime.now(timezone.utc)
        self._body_store = BodyMetricsStore()
        self._massage_recommendation_store = MassageRecommendationStore()
        self._bucket_seconds = self._load_bucket_seconds()
        self._has_valid_measurement = False
        self._alert_engine = VitalAlertEngine()
        self._massage_session_key = ""
        self._massage_active = False
        self._alert_started_at = self._session_started
        self._source = resolve_active_source()
        if self._source.mode == "demo":
            self._alert_engine = VitalAlertEngine(cooldown_seconds=0)
        if self._source.mode == "demo":
            self._session_started -= timedelta(minutes=5)
            self._alert_started_at = self._session_started
        # Retain the private profile on disk, but do not reveal the previous
        # session's values when a new GUI session opens.
        self._body_entered_this_session = False
        self._state: Dict[str, Any] = {}
        self._scale_status = "电子秤未连接"
        self._scale_adapter = ScaleSerialAdapter(self)
        self._scale_adapter.statusChanged.connect(self._on_scale_status)
        self._scale_adapter.measurementReceived.connect(self._on_scale_measurement)
        if isinstance(QGuiApplication.instance(), QGuiApplication):
            self._scale_adapter.start()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.refresh)
        self._timer.start(3000)
        self.refresh()

    def _load_bucket_seconds(self) -> int:
        try:
            saved = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
            return max(5, min(300, int(saved.get("trend_bucket_seconds", 30))))
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return 30

    def _save_bucket_seconds(self) -> None:
        SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)
        SETTINGS_PATH.write_text(json.dumps({"trend_bucket_seconds": self._bucket_seconds}), encoding="utf-8")

    def _samples(self) -> List[Dict[str, Any]]:
        path = self._source.path if self._source.mode == "demo" else DATABASE_PATH
        if not path.is_file():
            return []
        try:
            with sqlite3.connect(f"file:{path}?mode=ro", uri=True, timeout=3) as connection:
                connection.row_factory = sqlite3.Row
                rows = connection.execute(
                    """SELECT *
                    FROM health_band_samples WHERE julianday(received_at) >= julianday(?)
                    AND (? = 0 OR julianday(received_at) <= julianday('now'))
                    ORDER BY received_at ASC""",
                    [self._session_started.isoformat(), int(self._source.mode == "demo")],
                ).fetchall()
        except sqlite3.Error:
            return []
        samples = [dict(row) for row in rows]
        latest_mac = str(samples[-1].get("device_mac") or "") if samples else ""
        return [sample for sample in samples if str(sample.get("device_mac") or "") == latest_mac]

    @staticmethod
    def _valid(sample: Dict[str, Any]) -> bool:
        return all(valid_vital_value(sample, key) is not None for key in VITAL_COLUMNS)

    def _trends(self, samples: List[Dict[str, Any]], latest_at: datetime) -> Dict[str, List[float]]:
        start = latest_at - timedelta(minutes=5)
        buckets: Dict[int, Dict[str, List[float]]] = {}
        for sample in samples:
            moment = _as_utc(sample.get("received_at"))
            if not moment or moment < start:
                continue
            key = int((moment - start).total_seconds() // self._bucket_seconds)
            grouped = buckets.setdefault(key, {metric: [] for metric in VITAL_COLUMNS})
            for metric in VITAL_COLUMNS:
                value = valid_vital_value(sample, metric)
                if value is not None:
                    grouped[metric].append(value)
        return {
            metric: [sum(values[metric]) / len(values[metric]) for _, values in sorted(buckets.items()) if values[metric]][-MAX_TREND_POINTS:]
            for metric in VITAL_COLUMNS
        }

    def refresh(self) -> None:
        source = resolve_active_source()
        if source.token != self._source.token:
            self._source = source
            self._session_started = datetime.now(timezone.utc) - timedelta(minutes=5) if source.mode == "demo" else datetime.now(timezone.utc)
            self._alert_started_at = self._session_started
            self._has_valid_measurement = False
            self._alert_engine = VitalAlertEngine(cooldown_seconds=0 if source.mode == "demo" else 180)
        samples = self._samples()
        self._has_valid_measurement = self._has_valid_measurement or any(self._valid(sample) for sample in samples)
        saved_body = self._body_store.summary()
        body = saved_body if self._body_entered_this_session else {
            "height_cm": None, "weight_kg": None, "bmi": None, "source": "本次未录入"
        }
        recommendation = recommendation_for_bmi(body["bmi"])
        pending_recommendation = self._massage_recommendation_store.pending()
        if recommendation.get("available") and pending_recommendation.get("bmi") == recommendation.get("bmi"):
            recommendation = {**recommendation, "scheduled": True}
        latest = samples[-1] if samples else None
        state: Dict[str, Any] = {
            "height": body["height_cm"], "weight": body["weight_kg"], "bmi": body["bmi"], "body_source": body["source"],
            "massage_recommendation": recommendation,
            "bucket_seconds": self._bucket_seconds, "trends": {metric: [] for metric in VITAL_COLUMNS},
            "receiver": self._receiver_status(),
            "physiological_alert": None,
            "source_mode": self._source.mode,
        }
        alert_samples = [sample for sample in samples if (stamp := _as_utc(sample.get("received_at"))) and stamp >= self._alert_started_at]
        device_key = str(alert_samples[-1].get("device_mac") or "") if alert_samples else ""
        session_key = self._massage_session_key if self._massage_active else "health-monitor"
        evaluated = self._alert_engine.evaluate(alert_samples, session_key=f"{session_key}:{device_key}")
        alert = evaluated["alert"]
        if alert and not self._massage_active:
            message, speech = IDLE_ALERT_TEXT[alert["key"]]
            alert = {**alert, "message": message, "speech": speech}
        state["physiological_alert"] = alert
        if evaluated["announcement"]:
            announcement = evaluated["announcement"]
            speech = announcement["speech"] if self._massage_active else IDLE_ALERT_TEXT[announcement["key"]][1]
            self.alertRaised.emit(speech, announcement["priority"])
        if not latest or (not self._has_valid_measurement and not self._valid(latest)):
            state.update(status="等待接收", color="#8A94A6", message="等待本次会话的首条有效体征数据")
        else:
            received_at = _as_utc(latest["received_at"])
            age = int((datetime.now(timezone.utc) - received_at).total_seconds()) if received_at else 9999
            battery = _number(latest.get("battery_percent"))
            battery_low = battery is not None and battery <= LOW_BATTERY_PERCENT
            state.update(latest)
            state["trends"] = self._trends(samples, received_at)
            if self._valid(latest):
                self._has_valid_measurement = True
                if age > 20:
                    state.update(status="数据超时", color="#D9822B", message="超过 20 秒未收到新的体征数据")
                elif battery_low:
                    state.update(status="电量偏低", color="#D9822B", message=f"手环电量 {battery:.0f}%，建议及时充电")
                else:
                    state.update(status="数据正常", color="#12A87A", message="正在接收手环体征数据")
            elif not self._has_valid_measurement:
                state.update(status="等待接收", color="#8A94A6", message="本次体征数据无效，等待下一次有效上报")
            elif age > 20:
                state.update(status="数据超时", color="#D9822B", message="超过 20 秒未收到新的体征数据")
            else:
                state.update(status="数据待确认", color="#D9822B", message="本次体征数据不完整，趋势已略过该点")
        if state != self._state:
            self._state = state
            self.stateChanged.emit()

    def set_massage_session(self, session_key: str, active: bool) -> None:
        """Called by the host's existing massage-state monitor."""
        if session_key != self._massage_session_key or active != self._massage_active:
            if active and (session_key != self._massage_session_key or not self._massage_active):
                self._alert_started_at = datetime.now(timezone.utc) - timedelta(seconds=60)
            self._massage_session_key = session_key
            self._massage_active = active
            if not active:
                self._alert_engine.reset(session_key)
            self.refresh()

    def _receiver_status(self) -> Dict[str, Any]:
        try:
            result = subprocess.run([str(HEALTH_SCRIPT), "receiver-status"], cwd=HEALTH_DIR, capture_output=True, text=True, timeout=5, check=False)
            return {"running": result.returncode == 0, "text": (result.stdout or result.stderr).strip() or "接收器状态未知"}
        except (OSError, subprocess.TimeoutExpired):
            return {"running": False, "text": "接收器状态未知"}

    @pyqtSlot(str)
    def setTrendBucketSeconds(self, value: str) -> None:
        try:
            self._bucket_seconds = max(5, min(300, int(float(value))))
        except (TypeError, ValueError):
            return
        self._save_bucket_seconds()
        self.refresh()

    @pyqtSlot(str, str)
    def saveManualMetrics(self, height_cm: str, weight_kg: str) -> None:
        try:
            body = self._body_store.set_manual(height_cm, weight_kg)
            self._massage_recommendation_store.confirm(body["bmi"], body["source"])
        except ValueError:
            return
        self._body_entered_this_session = True
        self.refresh()

    def updateBodyMetricsFromDevice(self, height_cm: Any, weight_kg: Any, source: str = "测量设备") -> None:
        """Persist a device measurement and reveal it for this GUI session."""
        body = self._body_store.update_from_device(height_cm, weight_kg, source)
        self._massage_recommendation_store.confirm(body["bmi"], body["source"])
        self._body_entered_this_session = True
        self.refresh()

    def _on_scale_status(self, status: str) -> None:
        self._scale_status = status
        self.stateChanged.emit()

    def _on_scale_measurement(self, measurement: Any) -> None:
        if measurement.age_years < 18:
            self._on_scale_status("电子秤年龄未达到成人范围，本次读数未采用")
            return
        try:
            self.updateBodyMetricsFromDevice(measurement.height_cm, measurement.weight_kg, "电子秤")
        except ValueError as exc:
            self._on_scale_status(f"电子秤读数未采用：{exc}")
            return
        self._on_scale_status(
            f"已采用电子秤测量：{measurement.height_cm} cm、{measurement.weight_kg:.1f} kg"
        )

    @pyqtSlot()
    def startReceiver(self) -> None:
        try:
            subprocess.run([str(HEALTH_SCRIPT), "start"], cwd=HEALTH_DIR, capture_output=True, text=True, timeout=10, check=False)
        except (OSError, subprocess.TimeoutExpired):
            pass
        self._session_started = datetime.now(timezone.utc)
        self._has_valid_measurement = False
        self._alert_engine.reset(self._massage_session_key)
        self.refresh()

    @pyqtSlot()
    def stopReceiver(self) -> None:
        try:
            subprocess.run([str(HEALTH_SCRIPT), "stop"], cwd=HEALTH_DIR, capture_output=True, text=True, timeout=10, check=False)
        except (OSError, subprocess.TimeoutExpired):
            pass
        self.refresh()

    def _value(self, key: str, suffix: str = "", decimals: int = 1) -> str:
        value = _number(self._state.get(key))
        if key in VITAL_COLUMNS:
            value = valid_vital_value(self._state, key)
        return "--" if value is None else f"{value:.{decimals}f}{suffix}"

    @pyqtProperty(str, notify=stateChanged)
    def statusLabel(self): return self._state.get("status", "等待接收")
    @pyqtProperty(bool, notify=stateChanged)
    def simulatedSource(self): return self._state.get("source_mode") == "demo"
    @pyqtProperty(str, notify=stateChanged)
    def statusColor(self): return self._state.get("color", "#8A94A6")
    @pyqtProperty(str, notify=stateChanged)
    def badgeLabel(self):
        if self.alertMessage and self.statusLabel in ("数据正常", "电量偏低"):
            return "紧急预警" if self.alertColor == "#B42323" else "体征提醒"
        return self.statusLabel
    @pyqtProperty(str, notify=stateChanged)
    def badgeColor(self):
        if self.alertMessage and self.statusLabel in ("数据正常", "电量偏低"):
            return self.alertColor
        return self.statusColor
    @pyqtProperty(str, notify=stateChanged)
    def message(self): return self._state.get("message", "等待首条有效体征数据")
    @pyqtProperty(str, notify=stateChanged)
    def alertMessage(self): return (self._state.get("physiological_alert") or {}).get("message", "")
    @pyqtProperty(str, notify=stateChanged)
    def alertColor(self):
        alert = self._state.get("physiological_alert") or {}
        return "#B42323" if alert.get("priority", 0) >= 2 else "#B76E00"
    @pyqtProperty(str, notify=stateChanged)
    def heartRateText(self): return self._value("heart_rate_bpm", " bpm")
    @pyqtProperty(str, notify=stateChanged)
    def spo2Text(self): return self._value("spo2_percent", " %")
    @pyqtProperty(str, notify=stateChanged)
    def temperatureText(self): return self._value("temperature_c", " ℃", 1)
    @pyqtProperty(str, notify=stateChanged)
    def batteryText(self): return self._value("battery_percent", " %")
    @pyqtProperty(str, notify=stateChanged)
    def heightText(self): return self._value("height", " cm", 0)
    @pyqtProperty(str, notify=stateChanged)
    def weightText(self): return self._value("weight", " kg")
    @pyqtProperty(str, notify=stateChanged)
    def bmiText(self): return self._value("bmi", "")
    @pyqtProperty(str, notify=stateChanged)
    def bodySourceText(self): return self._state.get("body_source", "未录入")
    @pyqtProperty(str, notify=stateChanged)
    def scaleStatusText(self): return self._scale_status
    @pyqtProperty(str, notify=stateChanged)
    def massageRecommendationText(self):
        recommendation = self._state.get("massage_recommendation") or {}
        if not recommendation.get("available"):
            return ""
        if recommendation.get("scheduled"):
            return (f"根据 BMI 结果，您属于{recommendation['category']}体型；"
                    f"下一次按摩力度将按基准力度的 {recommendation['factor']:.2f} 倍执行。")
        return (f"根据 BMI 结果，您属于{recommendation['category']}体型；"
                f"本次倍率为 {recommendation['factor']:.2f}，下次按摩请重新录入资料。")
    @pyqtProperty(str, notify=stateChanged)
    def receiverText(self):
        return self._state.get("receiver", {}).get("text", "接收器状态未知")
    @pyqtProperty(bool, notify=stateChanged)
    def receiverRunning(self): return bool(self._state.get("receiver", {}).get("running"))
    @pyqtProperty(int, notify=stateChanged)
    def bucketSeconds(self): return self._bucket_seconds
    @pyqtProperty("QVariantList", notify=stateChanged)
    def heartRateTrend(self): return self._state.get("trends", {}).get("heart_rate_bpm", [])
    @pyqtProperty("QVariantList", notify=stateChanged)
    def spo2Trend(self): return self._state.get("trends", {}).get("spo2_percent", [])
    @pyqtProperty("QVariantList", notify=stateChanged)
    def temperatureTrend(self): return self._state.get("trends", {}).get("temperature_c", [])
