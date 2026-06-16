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

    def _read_request(self):
        control = _read_json(self.control_path, {}) or {}
        if control.get("session_id") not in {None, "", self.session_id}:
            return "continue"
        return str(control.get("request") or "continue").strip().lower()

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
            "message": checkpoint.get("message"),
        }

        request = self._read_request()
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

        updates.update(status="running")
        _update_state(self.state_path, self.session_id, **updates)
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
        _update_state(
            args.state_path,
            args.session_id,
            status="running",
            target=target,
            target_label=load_result.get("target_label"),
            trajectory_path=args.trajectory_path,
            point_count=trajectory.get("point_count") or 0,
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
                status="completed",
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
                message="按摩动作执行完成",
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

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "detect":
        result = run_detect(args)
    elif args.command == "execute":
        result = run_execute(args)
    else:
        parser.error(f"unknown command: {args.command}")

    _write_result(args.result_path, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
