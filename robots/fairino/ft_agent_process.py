#!/usr/bin/env python3
"""Robot-side subprocess entrypoint for the voice-control massage agent.

This file must run in the original FAIRINO/vision Python environment, not in
py-xiaozhi's lightweight voice client venv.
"""

import argparse
import json
import traceback
import time
from pathlib import Path

import ft_agent_api


def _now_text():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def _empty_state():
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
        "session_force_override_active": False,
        "message": None,
        "last_result": None,
        "worker_pid": None,
        "updated_at": _now_text(),
    }


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


def _int_value(value, default=0):
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _read_json(path, default=None):
    path = Path(path)
    if not path.exists():
        return default
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def _atomic_write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    with tmp_path.open("w", encoding="utf-8") as f:
        json.dump(_jsonable(data), f, ensure_ascii=False, indent=2)
    tmp_path.replace(path)


def _update_state(state_path, session_id, **updates):
    state = _read_json(state_path, _empty_state()) or _empty_state()
    base = _empty_state()
    base.update(state)
    base.update(_jsonable(updates))
    base["session_id"] = session_id
    base["updated_at"] = _now_text()
    _atomic_write_json(state_path, base)
    return base


def _write_result(result_path, result):
    if result_path:
        _atomic_write_json(result_path, result)
    print("FT_AGENT_RESULT_JSON=" + json.dumps(_jsonable(result), ensure_ascii=False), flush=True)


class FileExecutionControl:
    def __init__(self, session_id, state_path, control_path):
        self.session_id = session_id
        self.state_path = Path(state_path)
        self.control_path = Path(control_path)
        self._last_force_adjust_seq = None

    def _read_control(self):
        control = _read_json(self.control_path, {}) or {}
        if control.get("session_id") not in {None, "", self.session_id}:
            return {}
        return control

    def _request_from_control(self, control):
        return str(control.get("request") or "continue").strip().lower()

    def _force_adjustment_from_control(self, control, state):
        adjustment = control.get("force_adjust") if isinstance(control, dict) else None
        if not isinstance(adjustment, dict):
            return None

        seq = str(adjustment.get("seq") or adjustment.get("id") or "").strip()
        if not seq:
            return None
        if seq == str(self._last_force_adjust_seq or ""):
            return None
        if seq == str(state.get("last_force_adjust_seq") or ""):
            self._last_force_adjust_seq = seq
            return None

        try:
            delta_n = float(adjustment.get("delta_n") or 0.0)
        except (TypeError, ValueError):
            return None
        if abs(delta_n) < 1e-9 and adjustment.get("target_force_n") in (None, ""):
            return None

        force_adjustment = dict(adjustment)
        force_adjustment["seq"] = seq
        force_adjustment["delta_n"] = delta_n
        return force_adjustment

    def force_target_changed(self, force_update, checkpoint=None):
        force_update = _jsonable(force_update or {})
        checkpoint = _jsonable(checkpoint or {})
        seq = force_update.get("seq")
        if seq:
            self._last_force_adjust_seq = str(seq)
        force_target_n = force_update.get("force_target_n")
        message = None
        if force_target_n is not None:
            delta = float(force_update.get("applied_delta_n") or 0.0)
            direction = "增大" if delta > 0 else "减小" if delta < 0 else "保持"
            message = f"按摩力度已{direction}到 {float(force_target_n):.1f}N"
        updates = {
            "force_target_n": force_target_n,
            "last_force_adjustment": force_update,
            "last_force_adjust_seq": seq,
            "pending_force_adjustment": None,
            "message": message or checkpoint.get("message"),
        }
        if checkpoint:
            updates.update(
                stage=checkpoint.get("stage"),
                current_action=checkpoint.get("current_action"),
                current_point_index=_int_value(checkpoint.get("current_point_index")),
                current_repeat_index=_int_value(checkpoint.get("current_repeat_index")),
                current_step_index=_int_value(checkpoint.get("current_step_index")),
                robot_tcp_pose=(checkpoint.get("robot_state") or {}).get("tcp_pose"),
                robot_joints_deg=(checkpoint.get("robot_state") or {}).get("joints_deg"),
            )
        _update_state(self.state_path, self.session_id, **updates)

    def checkpoint(self, checkpoint):
        checkpoint = _jsonable(checkpoint or {})
        state = _read_json(self.state_path, _empty_state()) or _empty_state()
        if state.get("session_id") not in {None, "", self.session_id}:
            return "stop"

        robot_state = checkpoint.get("robot_state") or {}
        current_repeat_index = _int_value(checkpoint.get("current_repeat_index"))
        current_step_index = _int_value(checkpoint.get("current_step_index"))
        updates = {
            "stage": checkpoint.get("stage"),
            "current_action": checkpoint.get("current_action"),
            "current_point_index": _int_value(checkpoint.get("current_point_index")),
            "current_repeat_index": current_repeat_index,
            "current_step_index": current_step_index,
            "resume_stage": checkpoint.get("next_stage") or checkpoint.get("stage"),
            "resume_point_index": _int_value(checkpoint.get("next_point_index")),
            "resume_action": checkpoint.get("next_action") or checkpoint.get("current_action"),
            "resume_repeat_index": _int_value(
                checkpoint.get("next_repeat_index"),
                current_repeat_index,
            ),
            "resume_step_index": _int_value(
                checkpoint.get("next_step_index"),
                current_step_index,
            ),
            "robot_tcp_pose": robot_state.get("tcp_pose"),
            "robot_joints_deg": robot_state.get("joints_deg"),
            "force_target_n": checkpoint.get("force_target_n", state.get("force_target_n")),
            "message": checkpoint.get("message"),
        }

        control = self._read_control()
        request = self._request_from_control(control)
        if request in {"stop", "stopped"}:
            if not checkpoint.get("safe_to_pause", True):
                updates.update(status="stopping", message="已收到停止请求，先回当前点悬空位")
                _update_state(self.state_path, self.session_id, **updates)
                return "stop_pending"
            updates.update(status="stopped", message="按摩已停止")
            _update_state(self.state_path, self.session_id, **updates)
            return "stop"

        if request in {"pause", "paused"}:
            if checkpoint.get("safe_to_pause", True):
                updates.update(status="paused", message="按摩已暂停")
                _update_state(self.state_path, self.session_id, **updates)
                return "pause"
            updates.update(status="pausing", message="已收到暂停请求，等待安全检查点")
            _update_state(self.state_path, self.session_id, **updates)
            return "pause_pending"

        force_adjustment = self._force_adjustment_from_control(control, state)
        updates.update(status="running")
        _update_state(self.state_path, self.session_id, **updates)
        if force_adjustment:
            return {"request": "continue", "force_adjustment": force_adjustment}
        return "continue"


def run_detect(args):
    api = ft_agent_api.FTMassageAgentInterface(args.target)
    try:
        result = api.detect_meridian(
            save=not args.no_save,
            display=bool(args.display),
            timeout_s=args.timeout_s,
            stable_frames=args.stable_frames,
        )
        return _jsonable(result)
    except Exception as exc:
        return {
            "ok": False,
            "operation": "detect_meridian",
            "target": args.target,
            "error": str(exc),
            "exception_type": exc.__class__.__name__,
            "traceback": traceback.format_exc(),
        }
    finally:
        try:
            api.close_vision()
        except Exception:
            pass


def _trajectory_debug_image_path(result):
    trajectory = result.get("trajectory") if isinstance(result, dict) else {}
    trajectory_path = trajectory.get("trajectory_path") if isinstance(trajectory, dict) else None
    if not trajectory_path:
        return None

    trajectory_json = Path(trajectory_path)
    data = _read_json(trajectory_json, {}) or {}
    debug_image = data.get("debug_image")
    if debug_image:
        return Path(debug_image)

    candidate = trajectory_json.with_suffix(".png")
    return candidate if candidate.exists() else None


def _state_says_preview_done(state_path, session_id):
    if not state_path:
        return False
    state = _read_json(state_path, {}) or {}
    state_session_id = state.get("session_id")
    if state_session_id and state_session_id != session_id:
        return True
    status = str(state.get("status") or "").strip().lower()
    return status in {"idle", "stopped", "completed", "error"}


def hold_detection_preview(args, result):
    if not (args.display and args.keep_display and result.get("ok")):
        return
    if not args.state_path:
        return

    try:
        import cv2
        import numpy as np
    except Exception as exc:
        print(f"[Preview] OpenCV not available, skip persistent detection preview: {exc}", flush=True)
        return

    image_path = _trajectory_debug_image_path(result)
    image = None
    if image_path is not None:
        image = cv2.imread(str(image_path))

    if image is None:
        image = np.full((720, 960, 3), 245, dtype=np.uint8)
        cv2.putText(
            image,
            "Trajectory saved",
            (40, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (20, 80, 20),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            str((result.get("trajectory") or {}).get("trajectory_path") or ""),
            (40, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (60, 60, 60),
            1,
            cv2.LINE_AA,
        )

    window_name = "Detection"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.moveWindow(
            window_name,
            ft_agent_api.DETECTION_WINDOW_X,
            ft_agent_api.DETECTION_WINDOW_Y,
        )
    except Exception:
        pass

    print("[Preview] 检测轨迹窗口保持显示，按摩结束/停止/异常后自动关闭。", flush=True)
    try:
        while True:
            frame = image.copy()
            state = _read_json(args.state_path, {}) or {}
            status = str(state.get("status") or "detecting")
            message = str(state.get("message") or "")
            banner = f"status={status}  press q to close"
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 42), (245, 245, 245), -1)
            cv2.putText(
                frame,
                banner[:120],
                (16, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 120, 255),
                2,
                cv2.LINE_AA,
            )
            if message:
                cv2.putText(
                    frame,
                    message[:80],
                    (16, frame.shape[0] - 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (60, 60, 60),
                    1,
                    cv2.LINE_AA,
                )
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(250) & 0xFF
            if key == ord("q"):
                break
            if _state_says_preview_done(args.state_path, args.session_id):
                break
    finally:
        try:
            cv2.destroyWindow(window_name)
        except Exception:
            pass


def run_execute(args):
    target = args.target
    if target == "auto":
        target = ft_agent_api.infer_massage_target_from_trajectory(args.trajectory_path)

    api = ft_agent_api.FTMassageAgentInterface(target)
    try:
        load_result = api.load_trajectory(args.trajectory_path)
        if not load_result.get("ok"):
            result = {
                "ok": False,
                "operation": "execute_loaded_trajectory",
                "load": load_result,
            }
            _update_state(
                args.state_path,
                args.session_id,
                status="error",
                message=load_result.get("error") or "轨迹加载失败",
                last_result=result,
            )
            return _jsonable(result)

        trajectory = load_result.get("trajectory") or {}
        if args.force_target_n is not None:
            api.demo.set_force_target_n(args.force_target_n, context="会话力度恢复")
            if args.force_target_override:
                api.demo.session_force_target_n = float(api.demo.force_target_n)
            trajectory["force_target_n"] = float(api.demo.force_target_n)
        _update_state(
            args.state_path,
            args.session_id,
            status="running",
            target=target,
            target_label=load_result.get("target_label"),
            trajectory_path=args.trajectory_path,
            point_count=trajectory.get("point_count") or 0,
            force_target_n=trajectory.get("force_target_n"),
            actions=ft_agent_api.normalize_massage_actions(args.actions),
            stage=args.start_stage,
            current_action="start",
            current_point_index=_int_value(args.start_point_index),
            current_repeat_index=_int_value(args.start_repeat_index),
            current_step_index=_int_value(args.start_step_index),
            resume_stage=args.start_stage,
            resume_point_index=_int_value(args.start_point_index),
            resume_action=args.start_action or None,
            resume_repeat_index=_int_value(args.start_repeat_index),
            resume_step_index=_int_value(args.start_step_index),
            message="正在执行按摩动作",
        )

        control = FileExecutionControl(args.session_id, args.state_path, args.control_path)
        execute_result = api.execute_actions(
            actions=args.actions,
            control=control,
            start_stage=args.start_stage,
            start_point_index=args.start_point_index,
            start_action=args.start_action,
            start_repeat_index=args.start_repeat_index,
            start_step_index=args.start_step_index,
        )
        result = {
            "ok": bool(execute_result.get("ok")),
            "operation": "execute_loaded_trajectory",
            "load": load_result,
            "execute": execute_result,
        }

        report = execute_result.get("report") or {}
        report_status = report.get("status")
        if report_status == "paused":
            _update_state(
                args.state_path,
                args.session_id,
                status="paused",
                message="按摩已暂停",
                last_result=result,
            )
        elif report_status == "stopped":
            _update_state(
                args.state_path,
                args.session_id,
                status="stopped",
                message="按摩已停止",
                last_result=result,
            )
        elif execute_result.get("ok"):
            _update_state(
                args.state_path,
                args.session_id,
                status="stopped",
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
                message="按摩动作执行完成，任务已停止",
                last_result=result,
            )
        else:
            _update_state(
                args.state_path,
                args.session_id,
                status="error",
                message=report.get("error") or execute_result.get("error") or "按摩动作执行失败",
                last_result=result,
            )
        return _jsonable(result)
    except Exception as exc:
        result = {
            "ok": False,
            "operation": "execute_loaded_trajectory",
            "target": target,
            "error": str(exc),
            "exception_type": exc.__class__.__name__,
            "traceback": traceback.format_exc(),
        }
        _update_state(
            args.state_path,
            args.session_id,
            status="error",
            message=str(exc),
            last_result=result,
        )
        return _jsonable(result)
    finally:
        try:
            api.close_robot()
        except Exception:
            pass


def build_parser():
    parser = argparse.ArgumentParser(description="Robot-side FAIRINO massage agent process")
    sub = parser.add_subparsers(dest="command", required=True)

    detect = sub.add_parser("detect")
    detect.add_argument("--session-id", required=True)
    detect.add_argument("--target", default="back")
    detect.add_argument("--display", action="store_true")
    detect.add_argument("--timeout-s", type=float, default=None)
    detect.add_argument("--stable-frames", type=int, default=None)
    detect.add_argument("--no-save", action="store_true")
    detect.add_argument("--result-path", required=True)
    detect.add_argument("--state-path", default="")
    detect.add_argument("--keep-display", action="store_true")

    execute = sub.add_parser("execute")
    execute.add_argument("--session-id", required=True)
    execute.add_argument("--target", default="auto")
    execute.add_argument("--trajectory-path", required=True)
    execute.add_argument("--actions", default="all")
    execute.add_argument("--start-stage", default="point_actions")
    execute.add_argument("--start-point-index", type=int, default=0)
    execute.add_argument("--start-action", default="")
    execute.add_argument("--start-repeat-index", type=int, default=0)
    execute.add_argument("--start-step-index", type=int, default=0)
    execute.add_argument("--state-path", required=True)
    execute.add_argument("--control-path", required=True)
    execute.add_argument("--result-path", required=True)
    execute.add_argument("--force-target-n", type=float, default=None)
    execute.add_argument("--force-target-override", action="store_true")

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "detect":
        result = run_detect(args)
        _write_result(args.result_path, result)
        hold_detection_preview(args, result)
        return 0
    elif args.command == "execute":
        result = run_execute(args)
    else:
        parser.error(f"unknown command: {args.command}")

    _write_result(args.result_path, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
