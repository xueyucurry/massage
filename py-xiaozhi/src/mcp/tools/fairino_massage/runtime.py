"""Runtime session manager for FAIRINO trajectory-based massage."""

import asyncio
import json
import os
import signal
import subprocess
import threading
import time
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional

from src.utils.logging_config import get_logger

logger = get_logger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parents[5]
FAIRINO_DIR = PROJECT_ROOT / "robots" / "fairino"
STATE_DIR = FAIRINO_DIR / "ft_agent_state"
STATE_PATH = STATE_DIR / "current_session.json"
CONTROL_PATH = STATE_DIR / "current_control.json"
RESULT_DIR = STATE_DIR / "results"
LOG_DIR = STATE_DIR / "logs"
DETECT_RUNNER = FAIRINO_DIR / "run_ft_agent_process_env.sh"
EXECUTE_RUNNER = FAIRINO_DIR / "run_ft_agent_process_ros2.sh"
MASSAGE_CLI = PROJECT_ROOT / "massage"
MASSAGE_ACTION_SEQUENCE = ("dian_jin", "fen_jin", "shun_jin")
FORCE_ADJUST_STEP_N = float(os.environ.get("FAIRINO_MASSAGE_FORCE_ADJUST_STEP_N", "5.0"))
LIVE_FORCE_TARGET_MIN_N = float(os.environ.get("FT_LIVE_FORCE_TARGET_MIN_N", "1.0"))
LIVE_FORCE_TARGET_MAX_N = float(os.environ.get("FT_LIVE_FORCE_TARGET_MAX_N", "80.0"))
FORCE_INCREASE_DIRECTIONS = {
    "stronger",
    "increase",
    "up",
    "more",
    "harder",
    "heavier",
    "大力",
    "大力一些",
    "加力",
    "加大",
    "增大",
    "增加",
    "重一点",
    "用力一点",
}
FORCE_DECREASE_DIRECTIONS = {
    "softer",
    "decrease",
    "down",
    "less",
    "lighter",
    "weaker",
    "小力",
    "小力一些",
    "减力",
    "减小",
    "减轻",
    "降低",
    "轻一点",
    "弱一点",
}
BLADDER_MERIDIAN_SPEECH_TEXT = os.environ.get(
    "FAIRINO_MASSAGE_BLADDER_MERIDIAN_SPEECH_TEXT",
    "旁光经",
)

_runtime = None


def _now_text() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def _new_session_id() -> str:
    return time.strftime("ft-%Y%m%d-%H%M%S")


def _pid_alive(pid) -> bool:
    try:
        pid = int(pid)
    except (TypeError, ValueError):
        return False
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _env_enabled(name: str, default: bool = True) -> bool:
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _speech_safe_label(label: str) -> str:
    text = str(label or "")
    if not _env_enabled("FAIRINO_MASSAGE_TTS_SAFE_BLADDER_MERIDIAN", True):
        return text
    return text.replace("膀胱经", BLADDER_MERIDIAN_SPEECH_TEXT).replace("膀胱", "旁光")


def _schedule_xiaozhi_detect_status_check(target_label: str) -> bool:
    try:
        from src.application import Application
    except Exception as exc:
        logger.warning(f"无法导入小智Application，跳过自动检测状态查询: {exc}")
        return False

    app = getattr(Application, "_instance", None)
    if app is None or not getattr(app, "running", False):
        logger.warning("小智Application未运行，跳过自动检测状态查询")
        return False

    async def _request_status_check():
        try:
            wait_s = float(os.environ.get("FAIRINO_MASSAGE_STATUS_CHECK_DELAY_S", "0.8"))
            if wait_s > 0:
                await asyncio.sleep(wait_s)

            deadline = time.monotonic() + float(
                os.environ.get("FAIRINO_MASSAGE_STATUS_CHECK_WAIT_IDLE_S", "8.0")
            )
            while getattr(app, "device_state", None) == "speaking" and time.monotonic() < deadline:
                await asyncio.sleep(0.2)

            if not getattr(app, "protocol", None):
                return
            try:
                if not app.is_audio_channel_opened():
                    await app.connect_protocol()
            except Exception:
                pass

            label = target_label or "经络"
            speech_label = _speech_safe_label(label)
            prompt = (
                "请检查当前FAIRINO检测状态，必须调用 self.fairino_massage.status。"
                "语音播报时涉及背部经络，一律口播“旁光经”，不要改成其他写法。"
                f"如果工具返回 status=detected 且 trajectory_saved=true，请播报“{speech_label}检测已完成，轨迹已保存”。"
                "不要开始按摩，不要调用开始、暂停、继续或停止工具。"
            )
            await app.protocol.send_wake_word_detected(prompt)
            logger.info(f"已请求小智自动检查检测状态: {label}")
        except Exception as exc:
            logger.error(f"请求小智自动检查检测状态失败: {exc}", exc_info=True)
            try:
                if hasattr(app, "set_chat_message"):
                    app.set_chat_message(
                        "assistant",
                        f"{_speech_safe_label(target_label or '经络')}检测已完成，轨迹已保存",
                    )
            except Exception:
                pass

    try:
        if hasattr(app, "schedule_command_nowait"):
            app.schedule_command_nowait(_request_status_check)
            logger.info(f"已调度小智自动检测状态查询: {target_label or '经络'}")
            return True
    except Exception as exc:
        logger.error(f"调度小智自动检测状态查询失败: {exc}", exc_info=True)
    return False


def _jsonable(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return str(value)


def _normalize_target(value: str) -> Optional[str]:
    text = str(value or "").strip().lower()
    if text in {"", "auto", "current", "当前"}:
        return None
    if text in {"1", "back", "spine", "bladder", "bladder_meridian", "背部", "膀胱经"}:
        return "back"
    if text in {"2", "leg", "thigh", "outer_thigh", "腿部", "大腿", "大腿外侧", "外侧"}:
        return "leg"
    if text in {"3", "leg_inner", "inner_leg", "inner_thigh", "腿内侧", "大腿内侧", "内侧"}:
        return "leg_inner"
    return None


def _normalize_massage_action(value: str) -> str:
    text = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    if text in {"dian", "dianjin", "dian_jin", "point", "point_press", "press", "点筋", "点压"}:
        return "dian_jin"
    if text in {"fen", "fenjin", "fen_jin", "split", "split_press", "separate", "分筋"}:
        return "fen_jin"
    if text in {"shun", "shunjin", "shun_jin", "stroke", "follow", "line_stroke", "顺筋"}:
        return "shun_jin"
    raise ValueError(f"未知按摩动作: {value!r}; 支持 dian_jin/fen_jin/shun_jin")


def _normalize_massage_actions(actions=None) -> List[str]:
    if actions is None:
        return list(MASSAGE_ACTION_SEQUENCE)

    if isinstance(actions, str):
        text = actions.strip()
        if text.lower().replace("-", "_") in {"", "all", "full", "sequence", "full_sequence", "全部", "全套"}:
            return list(MASSAGE_ACTION_SEQUENCE)
        raw_items = [item.strip() for item in text.replace("，", ",").split(",") if item.strip()]
    else:
        raw_items = list(actions)

    requested = {_normalize_massage_action(item) for item in raw_items}
    if not requested:
        return list(MASSAGE_ACTION_SEQUENCE)
    return [action for action in MASSAGE_ACTION_SEQUENCE if action in requested]


def _normalize_force_direction(direction: str = "") -> Optional[int]:
    text = str(direction or "").strip().lower().replace("-", "_").replace(" ", "_")
    if text in FORCE_INCREASE_DIRECTIONS:
        return 1
    if text in FORCE_DECREASE_DIRECTIONS:
        return -1
    return None


def _normalize_force_delta(direction: str = "", delta_n: Optional[float] = None) -> float:
    direction_sign = _normalize_force_direction(direction)
    if delta_n not in (None, "", 0, "0"):
        value = float(delta_n)
        if direction_sign is not None:
            return float(direction_sign) * abs(value)
        return value

    if str(direction or "").strip() == "":
        return abs(float(FORCE_ADJUST_STEP_N))
    if direction_sign == 1:
        return abs(float(FORCE_ADJUST_STEP_N))
    if direction_sign == -1:
        return -abs(float(FORCE_ADJUST_STEP_N))
    raise ValueError("direction 必须是 stronger/increase 或 softer/decrease")


def _clamp_force_target_n(value: float) -> float:
    low = min(float(LIVE_FORCE_TARGET_MIN_N), float(LIVE_FORCE_TARGET_MAX_N))
    high = max(float(LIVE_FORCE_TARGET_MIN_N), float(LIVE_FORCE_TARGET_MAX_N))
    return max(low, min(high, float(value)))


def _force_target_from_state(state: Dict[str, Any]) -> Optional[float]:
    last_result = state.get("last_result")
    candidates: List[Any] = [
        state.get("force_target_n"),
        (state.get("last_force_adjustment") or {}).get("force_target_n")
        if isinstance(state.get("last_force_adjustment"), dict)
        else None,
    ]
    if isinstance(last_result, dict):
        for path in (
            ("execute", "trajectory", "force_target_n"),
            ("load", "trajectory", "force_target_n"),
            ("trajectory", "force_target_n"),
        ):
            value: Any = last_result
            for key in path:
                if not isinstance(value, dict):
                    value = None
                    break
                value = value.get(key)
            candidates.append(value)

    for candidate in candidates:
        if candidate in (None, ""):
            continue
        try:
            return float(candidate)
        except (TypeError, ValueError):
            continue
    return None


def _infer_massage_target_from_trajectory(path: str) -> str:
    with Path(path).open("r", encoding="utf-8") as f:
        data = json.load(f)
    text = f"{data.get('trajectory_type', '')} {data.get('target', '')}".lower()
    if "inner" in text:
        return "leg_inner"
    if "thigh" in text or "leg" in text:
        return "leg"
    return "back"


def _stage_for_actions(actions: List[str]) -> str:
    if any(action in {"dian_jin", "fen_jin"} for action in actions):
        return "point_actions"
    if "shun_jin" in actions:
        return "shun_jin"
    return "completed"


def _last_checkpoint(state: Dict[str, Any]) -> Dict[str, Any]:
    last_result = state.get("last_result") if isinstance(state.get("last_result"), dict) else {}
    execute = last_result.get("execute") if isinstance(last_result.get("execute"), dict) else {}
    report = execute.get("report") if isinstance(execute.get("report"), dict) else {}
    checkpoint = report.get("checkpoint") if isinstance(report.get("checkpoint"), dict) else {}
    return checkpoint or {}


def _infer_resume_action(state: Dict[str, Any]) -> str:
    if state.get("resume_action"):
        return str(state.get("resume_action"))
    checkpoint = _last_checkpoint(state)
    action = checkpoint.get("current_action") or state.get("current_action") or ""
    message = str(checkpoint.get("message") or state.get("message") or "")
    if action == "dian_jin" and "点筋结束回悬空位" in message and "3/3" in message:
        return "fen_jin"
    if action == "fen_jin" and "分筋结束回悬空位" in message:
        return "move_to_hover_next"
    return str(action or "")


def _int_value(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _infer_resume_repeat_index(state: Dict[str, Any]) -> int:
    if state.get("resume_repeat_index") is not None:
        return _int_value(state.get("resume_repeat_index"))
    checkpoint = _last_checkpoint(state)
    if checkpoint.get("next_repeat_index") is not None:
        return _int_value(checkpoint.get("next_repeat_index"))
    return _int_value(state.get("current_repeat_index"))


def _infer_resume_step_index(state: Dict[str, Any]) -> int:
    if state.get("resume_step_index") is not None:
        return _int_value(state.get("resume_step_index"))
    checkpoint = _last_checkpoint(state)
    if checkpoint.get("next_step_index") is not None:
        return _int_value(checkpoint.get("next_step_index"))
    return _int_value(state.get("current_step_index"))


def _massage_progress_percent(state: Dict[str, Any]) -> Optional[int]:
    status = state.get("status")
    if status == "completed" or (status == "stopped" and state.get("stage") == "completed"):
        return 100
    if status not in {"running", "pausing", "paused", "stopping"}:
        return None

    point_count = max(1, int(state.get("point_count") or 0))
    point_index = max(0, min(point_count, int(state.get("current_point_index") or 0)))
    stage = state.get("stage")
    if stage == "shun_jin":
        return min(99, int(round(70 + (point_index / point_count) * 30)))
    if stage == "point_actions":
        return min(69, int(round((point_index / point_count) * 70)))
    return 0


def _status_summary(state: Dict[str, Any]) -> str:
    status = state.get("status") or "idle"
    label = state.get("target_label") or state.get("target") or "当前部位"
    speech_label = _speech_safe_label(label)
    trajectory_path = state.get("trajectory_path")
    point_count = int(state.get("point_count") or 0)
    progress = _massage_progress_percent(state)

    if status == "detecting":
        return f"{speech_label}正在检测中，轨迹尚未保存；请查看检测画面，检测稳定后会自动保存轨迹。"
    if status == "detected":
        force_text = ""
        if state.get("force_target_n") is not None:
            force_text = f"目标力度={float(state.get('force_target_n')):.1f}N；"
        return f"{speech_label}检测已完成，轨迹已保存，共 {point_count} 个点；{force_text}尚未开始按摩。轨迹文件：{trajectory_path}"
    if status == "running":
        force_text = ""
        if state.get("force_target_n") is not None:
            force_text = f"，目标力度={float(state.get('force_target_n')):.1f}N"
        return (
            f"{speech_label}按摩正在执行，阶段={state.get('stage')}，动作={state.get('current_action')}，"
            f"点位={int(state.get('current_point_index') or 0)}/{point_count}，进度={progress}%{force_text}"
        )
    if status == "pausing":
        return f"{speech_label}已收到暂停请求，正在等待最近安全检查点；当前动作={state.get('current_action')}。"
    if status == "paused":
        force_text = ""
        if state.get("force_target_n") is not None:
            force_text = f"，目标力度={float(state.get('force_target_n')):.1f}N"
        return (
            f"{speech_label}按摩已暂停，可继续；恢复点 stage={state.get('resume_stage')}，"
            f"action={_infer_resume_action(state)}，point_index={state.get('resume_point_index')}，"
            f"repeat_index={_infer_resume_repeat_index(state)}，step_index={_infer_resume_step_index(state)}{force_text}。"
        )
    if status == "stopping":
        if state.get("stage") == "completed" and state.get("current_action") == "return_home":
            return f"{speech_label}按摩已完成，机械臂正在回到起始位置。"
        return f"{speech_label}正在停止，等待执行器退出。"
    if status == "stopped":
        if state.get("stage") == "completed" or state.get("current_action") == "completed":
            home_result = state.get("stop_home_result") or {}
            if home_result.get("ok"):
                return f"{speech_label}按摩已完成并停止，机械臂已回到起始位置，进度=100%。"
            return f"{speech_label}按摩已完成并停止，进度=100%。"
        return f"{speech_label}按摩已停止。"
    if status == "completed":
        return f"{speech_label}按摩已完成，进度=100%。"
    if status == "error":
        return f"{speech_label}任务异常：{state.get('message') or '未知错误'}。"
    return "当前没有正在执行的检测或按摩任务。"


def _empty_state() -> Dict[str, Any]:
    return {
        "session_id": None,
        "status": "idle",
        "target": None,
        "target_label": None,
        "trajectory_path": None,
        "point_count": 0,
        "actions": [],
        "stage": None,
        "current_action": None,
        "current_point_index": 0,
        "current_repeat_index": 0,
        "current_step_index": 0,
        "resume_stage": None,
        "resume_point_index": 0,
        "resume_action": None,
        "resume_repeat_index": 0,
        "resume_step_index": 0,
        "robot_tcp_pose": None,
        "robot_joints_deg": None,
        "force_target_n": None,
        "last_force_adjustment": None,
        "last_force_adjust_seq": None,
        "pending_force_adjustment": None,
        "message": None,
        "last_result": None,
        "stop_home_result": None,
        "worker_pid": None,
        "updated_at": _now_text(),
    }


class FairinoMassageRuntime:
    def __init__(self, state_path: Path = STATE_PATH):
        self.state_path = Path(state_path)
        self.control_path = CONTROL_PATH
        self._lock = threading.RLock()
        self._worker_thread: Optional[threading.Thread] = None
        self._worker_process: Optional[subprocess.Popen] = None
        self._detect_display_process: Optional[subprocess.Popen] = None
        self._state = self._load_state()

    def _load_state(self) -> Dict[str, Any]:
        if not self.state_path.exists():
            return _empty_state()
        try:
            with self.state_path.open("r", encoding="utf-8") as f:
                data = json.load(f)
            state = _empty_state()
            state.update(data)
            return state
        except Exception as exc:
            logger.warning(f"[FairinoMassage] 状态文件读取失败: {exc}")
            return _empty_state()

    def _save_state_unlocked(self):
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        self._state["updated_at"] = _now_text()
        tmp_path = self.state_path.with_suffix(".tmp")
        with tmp_path.open("w", encoding="utf-8") as f:
            json.dump(_jsonable(self._state), f, ensure_ascii=False, indent=2)
        tmp_path.replace(self.state_path)

    def _refresh_state_from_disk_unlocked(self):
        self._state = self._load_state()

    def _state_copy_unlocked(self) -> Dict[str, Any]:
        state = dict(self._state)
        worker_pid = self._worker_process.pid if self._worker_process is not None else state.get("worker_pid")
        if worker_pid is not None and not _pid_alive(worker_pid) and self._worker_process is None:
            worker_pid = None
        state["worker_alive"] = self._is_worker_alive_unlocked()
        state["worker_pid"] = worker_pid
        state["state_path"] = str(self.state_path)
        state["control_path"] = str(self.control_path)
        return _jsonable(state)

    def _update_state(self, **updates) -> Dict[str, Any]:
        with self._lock:
            self._state.update(_jsonable(updates))
            self._save_state_unlocked()
            return self._state_copy_unlocked()

    def _write_control_unlocked(self, request: str, **extra):
        self.control_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "session_id": self._state.get("session_id"),
            "request": request,
            "updated_at": _now_text(),
        }
        payload.update(_jsonable(extra))
        tmp_path = self.control_path.with_suffix(".tmp")
        with tmp_path.open("w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        tmp_path.replace(self.control_path)

    def _result_path(self, session_id: str, operation: str) -> Path:
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        return RESULT_DIR / f"{session_id}-{operation}.json"

    def _log_path(self, session_id: str, operation: str) -> Path:
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        return LOG_DIR / f"{session_id}-{operation}.log"

    def _read_result_file(self, path: Path) -> Dict[str, Any]:
        with Path(path).open("r", encoding="utf-8") as f:
            return _jsonable(json.load(f))

    def _read_log_tail(self, path: Path, max_chars: int = 4000) -> str:
        try:
            data = Path(path).read_text(encoding="utf-8", errors="replace")
        except Exception:
            return ""
        return data[-max_chars:]

    def _is_worker_alive_unlocked(self) -> bool:
        if self._worker_thread is not None and self._worker_thread.is_alive():
            return True
        if self._worker_process is not None and self._worker_process.poll() is None:
            return True
        return _pid_alive(self._state.get("worker_pid"))

    def _cleanup_detect_display_process_unlocked(self):
        proc = self._detect_display_process
        if proc is None:
            return
        if proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            except Exception as exc:
                logger.warning(f"[FairinoMassage] failed to stop detect display process: {exc}")
        self._detect_display_process = None

    def _wait_worker_stopped(self, timeout_s: float) -> bool:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            with self._lock:
                proc = self._worker_process
                thread_alive = self._worker_thread is not None and self._worker_thread.is_alive()
                pid = proc.pid if proc is not None else self._state.get("worker_pid")
                proc_alive = proc is not None and proc.poll() is None
                pid_alive = proc_alive or (proc is None and _pid_alive(pid))
            if not thread_alive and not pid_alive:
                return True
            time.sleep(0.2)

        with self._lock:
            proc = self._worker_process
            pid = proc.pid if proc is not None else self._state.get("worker_pid")

        if proc is not None and proc.poll() is None:
            logger.warning("[FairinoMassage] stop wait timeout, terminating worker process")
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        elif pid is not None and _pid_alive(pid):
            logger.warning(f"[FairinoMassage] stop wait timeout, terminating worker pid={pid}")
            try:
                os.kill(int(pid), signal.SIGTERM)
                time.sleep(1.0)
                if _pid_alive(pid):
                    os.kill(int(pid), signal.SIGKILL)
            except Exception as exc:
                logger.warning(f"[FairinoMassage] failed to terminate worker pid={pid}: {exc}")
        return False

    def _return_robot_home(self) -> Dict[str, Any]:
        if not MASSAGE_CLI.exists():
            return {"ok": False, "message": f"找不到回位脚本: {MASSAGE_CLI}"}

        timeout_s = float(os.environ.get("FAIRINO_MASSAGE_HOME_TIMEOUT_S", "90"))
        completed = subprocess.run(
            ["bash", str(MASSAGE_CLI), "home"],
            cwd=str(PROJECT_ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        output = (completed.stdout or "").strip()
        return {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "message": "机械臂已回到起始位置" if completed.returncode == 0 else "机械臂回起始位置失败",
            "output": output[-4000:],
        }

    async def status(self) -> Dict[str, Any]:
        with self._lock:
            self._refresh_state_from_disk_unlocked()
            state = self._state_copy_unlocked()
            progress = _massage_progress_percent(state)
            return {
                "success": True,
                "message": _status_summary(state),
                "status": state.get("status"),
                "trajectory_saved": bool(state.get("trajectory_path") and state.get("status") in {"detected", "running", "pausing", "paused", "stopped", "completed", "error"}),
                "progress_percent": progress,
                "state": state,
            }

    async def detect(
        self,
        target: str = "back",
        display: bool = True,
        timeout_s: Optional[float] = None,
        stable_frames: Optional[int] = None,
    ) -> Dict[str, Any]:
        normalized_target = _normalize_target(target) or "back"
        with self._lock:
            self._refresh_state_from_disk_unlocked()
            if self._is_worker_alive_unlocked():
                return {
                    "success": False,
                    "message": "已有检测或按摩任务正在执行",
                    "state": self._state_copy_unlocked(),
                }
            self._cleanup_detect_display_process_unlocked()
            session_id = _new_session_id()
            self._state.update(
                _empty_state(),
                session_id=session_id,
                status="detecting",
                target=normalized_target,
                message="正在检测经络并生成轨迹",
            )
            self._save_state_unlocked()
            self._worker_thread = threading.Thread(
                target=self._run_detect_worker,
                args=(
                    session_id,
                    normalized_target,
                    bool(display),
                    timeout_s,
                    stable_frames,
                ),
                daemon=True,
                name="fairino-detect-worker",
            )
            self._worker_thread.start()
            return {
                "success": True,
                "message": "经络检测已启动，完成后会保存轨迹；请稍后查询按摩状态或直接开始按摩",
                "state": self._state_copy_unlocked(),
            }

    def _run_detect_worker(
        self,
        session_id: str,
        target: str,
        display: bool,
        timeout_s: Optional[float],
        stable_frames: Optional[int],
    ):
        self._detect_blocking(session_id, target, display, timeout_s, stable_frames)

    def _detect_blocking(
        self,
        session_id: str,
        target: str,
        display: bool,
        timeout_s: Optional[float],
        stable_frames: Optional[int],
    ) -> Dict[str, Any]:
        try:
            result_path = self._result_path(session_id, "detect")
            log_path = self._log_path(session_id, "detect")
            if result_path.exists():
                result_path.unlink()

            cmd = [
                "bash",
                str(DETECT_RUNNER),
                "detect",
                "--session-id",
                session_id,
                "--target",
                target,
                "--result-path",
                str(result_path),
            ]
            if display:
                cmd.append("--display")
                if _env_enabled("FAIRINO_MASSAGE_KEEP_DETECTION_WINDOW", True):
                    cmd.extend(["--state-path", str(self.state_path), "--keep-display"])
            if timeout_s is not None:
                cmd.extend(["--timeout-s", str(float(timeout_s))])
            if stable_frames is not None:
                cmd.extend(["--stable-frames", str(int(stable_frames))])

            env = os.environ.copy()
            env.setdefault("PYTHONUNBUFFERED", "1")
            timeout = None
            if timeout_s is not None:
                timeout = max(60.0, float(timeout_s) + 60.0)

            with log_path.open("w", encoding="utf-8") as log_file:
                proc = subprocess.Popen(
                    cmd,
                    cwd=str(FAIRINO_DIR),
                    env=env,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    text=True,
                )

                deadline = None if timeout is None else time.monotonic() + float(timeout)
                while not result_path.exists():
                    returncode = proc.poll()
                    if returncode is not None:
                        break
                    if deadline is not None and time.monotonic() > deadline:
                        proc.terminate()
                        try:
                            proc.wait(timeout=3.0)
                        except subprocess.TimeoutExpired:
                            proc.kill()
                        raise subprocess.TimeoutExpired(cmd, timeout)
                    time.sleep(0.2)

                completed_returncode = proc.poll()
                keep_display_alive = bool(display and _env_enabled("FAIRINO_MASSAGE_KEEP_DETECTION_WINDOW", True))
                if result_path.exists() and proc.poll() is None:
                    if keep_display_alive:
                        with self._lock:
                            self._detect_display_process = proc
                    else:
                        try:
                            proc.wait(timeout=2.0)
                            completed_returncode = proc.returncode
                        except subprocess.TimeoutExpired:
                            proc.terminate()
                            completed_returncode = proc.poll()

            if not result_path.exists():
                state = self._update_state(
                    status="error",
                    message="机器人检测进程没有返回结果",
                    last_result={
                        "error": "missing result file",
                        "returncode": completed_returncode,
                        "log_path": str(log_path),
                        "log_tail": self._read_log_tail(log_path),
                    },
                )
                return {
                    "success": False,
                    "message": "机器人检测进程没有返回结果",
                    "state": state,
                }

            result = self._read_result_file(result_path)
            if completed_returncode not in (None, 0) and result.get("ok"):
                result["process_returncode"] = completed_returncode
                result["log_path"] = str(log_path)
            result = _jsonable(result)
            if result.get("ok"):
                trajectory = result.get("trajectory") or {}
                state = self._update_state(
                    session_id=session_id,
                    status="detected",
                    target=result.get("target"),
                    target_label=result.get("target_label"),
                    trajectory_path=trajectory.get("trajectory_path"),
                    point_count=trajectory.get("point_count") or 0,
                    force_target_n=trajectory.get("force_target_n"),
                    actions=[],
                    stage=None,
                    current_action=None,
                    current_point_index=0,
                    resume_stage=None,
                    resume_point_index=0,
                    message="经络检测完成，轨迹已保存",
                    last_result=result,
                )
                if _env_enabled("FAIRINO_MASSAGE_ANNOUNCE_DETECT_DONE", True):
                    target_label = state.get("target_label") or result.get("target_label") or "经络"
                    _schedule_xiaozhi_detect_status_check(target_label)
                return {
                    "success": True,
                    "message": "经络检测完成，轨迹已保存",
                    "result": result,
                    "state": state,
                }

            state = self._update_state(
                status="error",
                message=result.get("error") or "检测未成功",
                last_result=result,
            )
            return {
                "success": False,
                "message": result.get("error") or "检测未成功",
                "result": result,
                "state": state,
            }
        except subprocess.TimeoutExpired as exc:
            state = self._update_state(
                status="error",
                message="经络检测进程超时",
                last_result={"error": "detect process timeout", "timeout_s": exc.timeout},
            )
            return {"success": False, "message": "经络检测进程超时", "state": state}
        except Exception as exc:
            state = self._update_state(
                status="error",
                message=str(exc),
                last_result={"error": str(exc), "traceback": traceback.format_exc()},
            )
            return {"success": False, "message": str(exc), "state": state}

    async def start(
        self,
        actions: str = "all",
        target: str = "auto",
        trajectory_path: str = "",
    ) -> Dict[str, Any]:
        with self._lock:
            self._refresh_state_from_disk_unlocked()
            resume = self._state.get("status") in {"paused", "pausing"} and not trajectory_path
        return self._start_worker(
            actions=actions,
            target=target,
            trajectory_path=trajectory_path,
            resume=resume,
        )

    async def resume(self) -> Dict[str, Any]:
        return self._start_worker(actions="", target="auto", trajectory_path="", resume=True)

    async def adjust_force(
        self,
        direction: str = "stronger",
        delta_n: Optional[float] = None,
    ) -> Dict[str, Any]:
        try:
            delta = _normalize_force_delta(direction, delta_n)
        except Exception as exc:
            with self._lock:
                self._refresh_state_from_disk_unlocked()
                state = self._state_copy_unlocked()
            return {"success": False, "message": str(exc), "state": state}

        with self._lock:
            self._refresh_state_from_disk_unlocked()
            worker_alive = self._is_worker_alive_unlocked()
            status = str(self._state.get("status") or "").strip().lower()

            if status == "pausing" and worker_alive:
                return {
                    "success": False,
                    "message": "正在进入暂停状态，请等机械臂退回贴近前悬空位后再调整力度",
                    "state": self._state_copy_unlocked(),
                }

            if status == "paused" or (status == "pausing" and not worker_alive):
                current_force = _force_target_from_state(self._state)
                if current_force is None:
                    return {
                        "success": False,
                        "message": "当前暂停会话没有可用目标力度，请先完成检测或开始按摩",
                        "state": self._state_copy_unlocked(),
                    }

                target_force = _clamp_force_target_n(float(current_force) + float(delta))
                applied_delta = float(target_force) - float(current_force)
                seq = f"force-{time.time_ns()}"
                direction_text = "增大" if applied_delta > 0 else "减小" if applied_delta < 0 else "保持"
                adjustment = {
                    "seq": seq,
                    "delta_n": float(applied_delta),
                    "requested_delta_n": float(delta),
                    "previous_force_target_n": float(current_force),
                    "force_target_n": float(target_force),
                    "source": "小智语音暂停期间力度调整",
                    "mode": "paused",
                    "created_at": _now_text(),
                }
                if abs(applied_delta) < 1e-9:
                    message = f"暂停期间目标力度已在边界，保持 {target_force:.1f}N，继续按摩后生效"
                else:
                    message = (
                        f"暂停期间已将按摩力度{direction_text} "
                        f"{abs(applied_delta):.1f}N，目标力度 {target_force:.1f}N，继续按摩后生效"
                    )
                self._state.update(
                    status="paused",
                    force_target_n=float(target_force),
                    last_force_adjustment=adjustment,
                    last_force_adjust_seq=seq,
                    pending_force_adjustment=None,
                    message=message,
                )
                self._save_state_unlocked()
                return {
                    "success": True,
                    "message": message,
                    "delta_n": float(applied_delta),
                    "force_target_n": float(target_force),
                    "state": self._state_copy_unlocked(),
                }

            if not worker_alive or status != "running":
                return {
                    "success": False,
                    "message": "当前没有正在执行或已暂停的按摩任务，不能调整力度",
                    "state": self._state_copy_unlocked(),
                }

            existing_control = {}
            if self.control_path.exists():
                try:
                    with self.control_path.open("r", encoding="utf-8") as f:
                        existing_control = json.load(f) or {}
                except Exception:
                    existing_control = {}
            existing_request = str(existing_control.get("request") or "continue").strip().lower()
            if existing_request in {"pause", "paused", "stop", "stopped"}:
                return {
                    "success": False,
                    "message": "当前已有暂停或停止请求，暂不调整力度",
                    "state": self._state_copy_unlocked(),
                }

            pending_adjust = existing_control.get("force_adjust")
            if isinstance(pending_adjust, dict):
                pending_seq = str(pending_adjust.get("seq") or "")
                last_applied_seq = str(self._state.get("last_force_adjust_seq") or "")
                if pending_seq and pending_seq != last_applied_seq:
                    try:
                        delta += float(pending_adjust.get("delta_n") or 0.0)
                    except (TypeError, ValueError):
                        pass

            if abs(float(delta)) < 1e-9:
                self._write_control_unlocked("continue")
                self._state.update(
                    pending_force_adjustment=None,
                    message="力度调整已抵消，保持当前目标力度",
                )
                self._save_state_unlocked()
                return {
                    "success": True,
                    "message": "力度调整已抵消，保持当前目标力度",
                    "delta_n": 0.0,
                    "state": self._state_copy_unlocked(),
                }

            seq = f"force-{time.time_ns()}"
            adjustment = {
                "seq": seq,
                "delta_n": float(delta),
                "source": "小智语音力度调整",
                "created_at": _now_text(),
            }
            self._write_control_unlocked("continue", force_adjust=adjustment)
            pending = dict(adjustment)
            self._state.update(
                pending_force_adjustment=pending,
                message=(
                    f"已请求按摩力度{'增大' if delta > 0 else '减小'} "
                    f"{abs(float(delta)):.1f}N"
                ),
            )
            self._save_state_unlocked()
            return {
                "success": True,
                "message": (
                    f"已请求按摩力度{'增大' if delta > 0 else '减小'} "
                    f"{abs(float(delta)):.1f}N，将在当前动作检查周期生效"
                ),
                "delta_n": float(delta),
                "state": self._state_copy_unlocked(),
            }

    def _start_worker(
        self,
        actions: str,
        target: str,
        trajectory_path: str,
        resume: bool,
    ) -> Dict[str, Any]:
        with self._lock:
            self._refresh_state_from_disk_unlocked()
            if self._is_worker_alive_unlocked():
                return {
                    "success": False,
                    "message": "已有检测或按摩任务正在执行",
                    "state": self._state_copy_unlocked(),
                }

            state = self._state
            path = trajectory_path or state.get("trajectory_path")
            if not path:
                return {
                    "success": False,
                    "message": "没有可用轨迹，请先进行经络检测或指定 trajectory_path",
                    "state": self._state_copy_unlocked(),
                }

            normalized_target = _normalize_target(target)
            if normalized_target is None:
                try:
                    normalized_target = _infer_massage_target_from_trajectory(path)
                except Exception:
                    normalized_target = state.get("target") or "back"

            if resume:
                if state.get("status") not in {"paused", "pausing"}:
                    return {
                        "success": False,
                        "message": "当前没有可继续的暂停按摩任务",
                        "state": self._state_copy_unlocked(),
                    }
                normalized_actions = list(state.get("actions") or _normalize_massage_actions(None))
                start_stage = state.get("resume_stage") or state.get("stage") or _stage_for_actions(normalized_actions)
                start_point_index = int(state.get("resume_point_index") or 0)
                start_action = _infer_resume_action(state)
                start_repeat_index = _infer_resume_repeat_index(state)
                start_step_index = _infer_resume_step_index(state)
                if start_action == "move_to_hover_next":
                    start_point_index += 1
                    start_action = "move_to_hover"
                    start_repeat_index = 0
                    start_step_index = 0
                session_id = state.get("session_id") or _new_session_id()
            else:
                try:
                    normalized_actions = _normalize_massage_actions(actions or "all")
                except Exception as exc:
                    return {
                        "success": False,
                        "message": str(exc),
                        "state": self._state_copy_unlocked(),
                    }
                start_stage = _stage_for_actions(normalized_actions)
                start_point_index = 0
                start_action = ""
                start_repeat_index = 0
                start_step_index = 0
                session_id = state.get("session_id") or _new_session_id()

            self._write_control_unlocked("continue")
            self._state.update(
                session_id=session_id,
                status="running",
                target=normalized_target,
                trajectory_path=str(path),
                actions=normalized_actions,
                force_target_n=state.get("force_target_n"),
                stage=start_stage,
                current_action="start",
                current_point_index=start_point_index,
                current_repeat_index=start_repeat_index,
                current_step_index=start_step_index,
                resume_stage=start_stage,
                resume_point_index=start_point_index,
                resume_action=start_action or None,
                resume_repeat_index=start_repeat_index,
                resume_step_index=start_step_index,
                message="正在启动按摩动作",
            )
            self._save_state_unlocked()

            self._worker_thread = threading.Thread(
                target=self._run_actions_worker,
                args=(
                    session_id,
                    normalized_target,
                    str(path),
                    list(normalized_actions),
                    state.get("force_target_n"),
                    start_stage,
                    start_point_index,
                    start_action,
                    start_repeat_index,
                    start_step_index,
                ),
                daemon=True,
                name="fairino-massage-worker",
            )
            self._worker_thread.start()
            return {
                "success": True,
                "message": "按摩任务已启动" if not resume else "按摩任务已继续",
                "state": self._state_copy_unlocked(),
            }

    def _run_actions_worker(
        self,
        session_id: str,
        target: str,
        trajectory_path: str,
        actions: List[str],
        force_target_n: Optional[float],
        start_stage: str,
        start_point_index: int,
        start_action: str,
        start_repeat_index: int,
        start_step_index: int,
    ):
        try:
            result_path = self._result_path(session_id, "execute")
            log_path = self._log_path(session_id, "execute")
            if result_path.exists():
                result_path.unlink()

            cmd = [
                "bash",
                str(EXECUTE_RUNNER),
                "execute",
                "--session-id",
                session_id,
                "--target",
                target,
                "--trajectory-path",
                trajectory_path,
                "--actions",
                ",".join(actions),
                "--start-stage",
                start_stage,
                "--start-point-index",
                str(int(start_point_index or 0)),
                "--start-action",
                str(start_action or ""),
                "--start-repeat-index",
                str(int(start_repeat_index or 0)),
                "--start-step-index",
                str(int(start_step_index or 0)),
                "--state-path",
                str(self.state_path),
                "--control-path",
                str(self.control_path),
                "--result-path",
                str(result_path),
            ]
            if force_target_n not in (None, ""):
                cmd.extend(["--force-target-n", str(float(force_target_n))])
            env = os.environ.copy()
            env.setdefault("PYTHONUNBUFFERED", "1")

            with log_path.open("w", encoding="utf-8") as log_file:
                proc = subprocess.Popen(
                    cmd,
                    cwd=str(FAIRINO_DIR),
                    env=env,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                with self._lock:
                    self._worker_process = proc
                    self._state["worker_pid"] = proc.pid
                    self._save_state_unlocked()
                returncode = proc.wait()

            if not result_path.exists():
                self._update_state(
                    status="error",
                    message="机器人按摩进程没有返回结果",
                    last_result={
                        "error": "missing result file",
                        "returncode": returncode,
                        "log_path": str(log_path),
                        "log_tail": self._read_log_tail(log_path),
                    },
                )
                return

            result = self._read_result_file(result_path)
            if returncode != 0:
                result.update(
                    process_returncode=returncode,
                    log_path=str(log_path),
                    log_tail=self._read_log_tail(log_path),
                )

            with self._lock:
                self._refresh_state_from_disk_unlocked()
                self._state["last_result"] = _jsonable(result)
                self._save_state_unlocked()

            execute_result = result.get("execute") if isinstance(result.get("execute"), dict) else result
            report = execute_result.get("report") or {}
            report_status = report.get("status")

            if returncode != 0:
                self._update_state(
                    status="error",
                    message=report.get("error") or result.get("error") or "机器人按摩进程异常退出",
                    last_result=result,
                )
            elif report_status == "paused":
                self._update_state(
                    status="paused",
                    message="按摩已暂停",
                    last_result=result,
                )
            elif report_status == "stopped":
                self._update_state(
                    status="stopped",
                    message="按摩已停止",
                    last_result=result,
                )
            elif result.get("ok"):
                home_result = None
                result_for_state = _jsonable(result)
                if _env_enabled("FAIRINO_MASSAGE_RETURN_HOME_ON_COMPLETE", True):
                    self._update_state(
                        status="stopping",
                        stage="completed",
                        current_action="return_home",
                        current_point_index=0,
                        current_repeat_index=0,
                        current_step_index=0,
                        resume_stage=None,
                        resume_point_index=0,
                        resume_action=None,
                        resume_repeat_index=0,
                        resume_step_index=0,
                        message="按摩动作执行完成，正在回到起始位置",
                        last_result=result_for_state,
                    )
                    try:
                        home_result = self._return_robot_home()
                    except Exception as exc:
                        home_result = {
                            "ok": False,
                            "message": f"机械臂回起始位置异常: {exc}",
                            "traceback": traceback.format_exc(),
                        }
                    result_for_state["completion_home_result"] = _jsonable(home_result)

                home_ok = home_result is None or bool(home_result.get("ok"))
                message = (
                    "按摩动作执行完成，机械臂已回到起始位置"
                    if home_result is not None and home_ok
                    else "按摩动作执行完成，任务已停止"
                )
                if home_result is not None and not home_ok:
                    message = home_result.get("message") or "按摩动作执行完成，但机械臂回起始位置失败"
                self._update_state(
                    status="stopped" if home_ok else "error",
                    stage="completed",
                    current_action="completed",
                    current_point_index=0,
                    current_repeat_index=0,
                    current_step_index=0,
                    resume_stage=None,
                    resume_point_index=0,
                    resume_action=None,
                    resume_repeat_index=0,
                    resume_step_index=0,
                    message=message,
                    last_result=result_for_state,
                    stop_home_result=home_result,
                )
            else:
                self._update_state(
                    status="error",
                    message=report.get("error") or result.get("error") or "按摩动作执行失败",
                    last_result=result,
                )
        except Exception as exc:
            logger.error(f"[FairinoMassage] worker failed: {exc}", exc_info=True)
            self._update_state(
                status="error",
                message=str(exc),
                last_result={"error": str(exc), "traceback": traceback.format_exc()},
            )
        finally:
            with self._lock:
                self._worker_process = None
                self._refresh_state_from_disk_unlocked()
                self._state["worker_pid"] = None
                self._save_state_unlocked()

    async def pause(self) -> Dict[str, Any]:
        with self._lock:
            self._refresh_state_from_disk_unlocked()
            if not self._is_worker_alive_unlocked():
                return {
                    "success": False,
                    "message": "当前没有正在执行的按摩任务",
                    "state": self._state_copy_unlocked(),
                }
            self._state.update(status="pausing", message="已请求暂停，等待到达安全检查点")
            self._write_control_unlocked("pause")
            self._save_state_unlocked()
            return {
                "success": True,
                "message": "已请求暂停，机械臂会在最近安全检查点暂停",
                "state": self._state_copy_unlocked(),
            }

    async def stop(self) -> Dict[str, Any]:
        with self._lock:
            self._refresh_state_from_disk_unlocked()
            worker_alive = self._is_worker_alive_unlocked()
            self._state.update(
                status="stopping" if worker_alive else "stopped",
                message="已请求停止，先回当前点悬空位，再回起始位置" if worker_alive else "按摩已停止，准备回到起始位置",
            )
            self._write_control_unlocked("stop")
            self._save_state_unlocked()

        stopped_cleanly = True
        if worker_alive:
            wait_s = float(os.environ.get("FAIRINO_MASSAGE_STOP_WAIT_S", "20"))
            stopped_cleanly = await asyncio.to_thread(self._wait_worker_stopped, wait_s)

        home_result = None
        if stopped_cleanly and _env_enabled("FAIRINO_MASSAGE_RETURN_HOME_ON_STOP", True):
            self._update_state(status="stopping", message="按摩已停止，正在回到起始位置")
            try:
                home_result = await asyncio.to_thread(self._return_robot_home)
            except Exception as exc:
                home_result = {
                    "ok": False,
                    "message": f"机械臂回起始位置异常: {exc}",
                    "error": str(exc),
                    "traceback": traceback.format_exc(),
                }

        home_ok = home_result is None or bool(home_result.get("ok"))
        if home_ok:
            message = "按摩已停止，机械臂已回到起始位置"
        else:
            message = home_result.get("message") or "按摩已停止，但机械臂回起始位置失败"
        if not stopped_cleanly:
            message = "按摩执行进程未及时退出，未确认已回当前点悬空位，已跳过回起始位置"

        state = self._update_state(
            status="stopped" if (stopped_cleanly and home_ok) else "error",
            message=message,
            worker_pid=None,
            stop_home_result=home_result,
        )
        return {
            "success": bool(stopped_cleanly and home_ok),
            "message": message,
            "home_result": home_result,
            "state": state,
        }

def get_runtime() -> FairinoMassageRuntime:
    global _runtime
    if _runtime is None:
        _runtime = FairinoMassageRuntime()
    return _runtime
