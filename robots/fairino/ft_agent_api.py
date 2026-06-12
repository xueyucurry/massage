#!/usr/bin/env python3
"""
Programmatic agent interface for ft.py.

This module intentionally keeps ft.py unchanged. It wraps the existing
LastTimeRos2Demo implementation with importable methods and a JSON CLI.
"""

import argparse
import json
import os
import time
from contextlib import contextmanager
from pathlib import Path

import cv2
import numpy as np

import ft


BACK_DETECTION_TIMEOUT_S = float(os.environ.get("BACK_DETECTION_TIMEOUT_S", "30.0"))
BACK_STABLE_FRAMES = int(os.environ.get("BACK_STABLE_FRAMES", "5"))
MASSAGE_ACTION_SEQUENCE = ("dian_jin", "fen_jin", "shun_jin")


def normalize_massage_action(value):
    text = str(value or "").strip().lower().replace("-", "_").replace(" ", "_")
    if text in {"dian", "dianjin", "dian_jin", "point", "point_press", "press", "点筋", "点压"}:
        return "dian_jin"
    if text in {"fen", "fenjin", "fen_jin", "split", "split_press", "separate", "分筋"}:
        return "fen_jin"
    if text in {"shun", "shunjin", "shun_jin", "stroke", "follow", "line_stroke", "顺筋"}:
        return "shun_jin"
    raise ValueError(f"未知按摩动作: {value!r}; 支持 dian_jin/fen_jin/shun_jin")


def normalize_massage_actions(actions=None):
    if actions is None:
        return list(MASSAGE_ACTION_SEQUENCE)

    if isinstance(actions, str):
        text = actions.strip()
        if text.lower().replace("-", "_") in {"", "all", "full", "sequence", "full_sequence", "全部", "全套"}:
            return list(MASSAGE_ACTION_SEQUENCE)
        raw_items = [item.strip() for item in text.replace("，", ",").split(",") if item.strip()]
    else:
        raw_items = list(actions)

    requested = {normalize_massage_action(item) for item in raw_items}
    if not requested:
        return list(MASSAGE_ACTION_SEQUENCE)
    return [action for action in MASSAGE_ACTION_SEQUENCE if action in requested]


def infer_massage_target_from_trajectory(path):
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    text = f"{data.get('trajectory_type', '')} {data.get('target', '')}".lower()
    if "inner" in text:
        return "leg_inner"
    if "thigh" in text or "leg" in text:
        return "leg"
    return "back"


@contextmanager
def temporary_ft_globals(**overrides):
    previous = {}
    try:
        for name, value in overrides.items():
            previous[name] = getattr(ft, name)
            setattr(ft, name, value)
        yield
    finally:
        for name, value in previous.items():
            setattr(ft, name, value)


def json_default(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    return str(value)


class AgentFTMassageDemo(ft.LastTimeRos2Demo):
    """Small extension layer over ft.LastTimeRos2Demo for agent calls."""

    def __init__(self, massage_target="back"):
        super().__init__(massage_target=massage_target)
        self.last_raw_confirmation_path = None
        self.last_trajectory_path = None
        self.last_action_report = {}

    def capture_back_trajectory(self, timeout_s=None, stable_frames=None, display=False):
        timeout_s = BACK_DETECTION_TIMEOUT_S if timeout_s is None else float(timeout_s)
        stable_frames = BACK_STABLE_FRAMES if stable_frames is None else max(1, int(stable_frames))
        display = bool(display)
        print("等待背部膀胱经检测稳定...")
        print(
            f"背部参数: stable_frames={stable_frames}, "
            f"timeout={timeout_s:.1f}s, min_depth_ratio={ft.BACK_MIN_DEPTH_RATIO:.2f}"
        )

        if self.detector is None:
            self.init_vision()

        stable_count = 0
        locked = None
        start = time.time()

        try:
            while time.time() - start < timeout_s:
                frames = self.detector.pipeline.wait_for_frames()
                frames = self.detector.align.process(frames)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    stable_count = 0
                    continue

                img = np.asanyarray(color_frame.get_data())
                self.frame_idx += 1
                analysis = self._analyze_visual_frame(img)
                analysis = self._attach_back_depth_samples(analysis, depth_frame, require_depth=True)
                ready = bool(analysis.get("visual_motion_ready", False))
                stable_count = stable_count + 1 if ready else 0
                tracking_label = "READY" if ready else str(analysis.get("visual_status", "search")).upper()
                self._set_preview_tracking_state(
                    spine_line=analysis.get("spine_line"),
                    meridian_lines=analysis.get("meridian_lines"),
                    outer_meridian_lines=analysis.get("outer_meridian_lines"),
                    tracking_label=tracking_label,
                )

                if display:
                    detection = self._draw_detection_overlay(img, analysis)
                    self._draw_command_banner(
                        detection,
                        [
                            f"BACK | status={tracking_label} | stable={stable_count}/{stable_frames}",
                            "s lock now | q quit",
                        ],
                    )
                    cv2.imshow("Detection", detection)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        return False
                    if key == ord("s") and ready:
                        locked = (img.copy(), depth_frame, analysis)
                        print("手动锁定当前背部膀胱经")
                        break

                if stable_count >= stable_frames:
                    locked = (img.copy(), depth_frame, analysis)
                    print("背部膀胱经检测稳定！")
                    break

            if locked is None:
                print("背部膀胱经检测超时")
                return False

            img, depth_frame, analysis = locked
            try:
                depth_frame.keep()
            except Exception:
                pass
            self.stable_depth_frame = depth_frame
            self.locked_color_frame = img.copy()
            self.spine_line = analysis["spine_line"]
            self.meridian_lines = analysis["meridian_lines"]
            self.outer_meridian_lines = analysis["outer_meridian_lines"]
            self._set_preview_tracking_state(
                spine_line=self.spine_line,
                meridian_lines=self.meridian_lines,
                outer_meridian_lines=self.outer_meridian_lines,
                tracking_label="LOCKED",
            )
            if not self.capture_trajectory():
                return False
            self._annotate_back_depth_diagnostics()
            return True
        finally:
            if display:
                try:
                    cv2.destroyWindow("Detection")
                except Exception:
                    pass

    def capture_thigh_trajectory_for_agent(self, timeout_s=None, stable_frames=None, display=False):
        if getattr(self, "thigh_pose_detector", None) is None:
            self.init_leg_vision()
        overrides = {"THIGH_DISPLAY": bool(display)}
        if timeout_s is not None:
            overrides["THIGH_DETECTION_TIMEOUT_S"] = float(timeout_s)
        if stable_frames is not None:
            overrides["THIGH_STABLE_FRAMES"] = max(1, int(stable_frames))
        with temporary_ft_globals(**overrides):
            return self.capture_thigh_trajectory()

    def save_current_trajectory(self):
        if not self.massage_frames:
            raise RuntimeError("当前没有已锁定轨迹，无法保存")

        if self.massage_target == "back":
            label = "back"
            extra = {"trajectory_type": "bladder_meridian"}
        else:
            label = self._thigh_trajectory_type()
            thigh_offset_mm = ft._thigh_offset_for_massage_target(self.massage_target)
            thigh_line_shift_mm = ft._thigh_line_shift_for_massage_target(self.massage_target)
            extra = {
                "trajectory_type": self._thigh_trajectory_type(),
                "raw_confirmation_json": str(self.last_raw_confirmation_path)
                if self.last_raw_confirmation_path is not None
                else None,
                "thigh_side": ft.THIGH_SIDE,
                "thigh_offset_mm": float(thigh_offset_mm),
                "thigh_line_shift_mm": float(thigh_line_shift_mm),
                "thigh_direction": ft.THIGH_DIRECTION,
                "thigh_inner_skip_points": int(ft.THIGH_INNER_SKIP_POINTS)
                if self.massage_target == "leg_inner"
                else 0,
                "thigh_inner_tail_skip_points": int(ft.THIGH_INNER_TAIL_SKIP_POINTS)
                if self.massage_target == "leg_inner"
                else 0,
            }

        self.last_trajectory_path = self._save_locked_trajectory(label, extra=extra)
        return self.last_trajectory_path

    def trajectory_summary(self, trajectory_path=None):
        path = trajectory_path if trajectory_path is not None else self.last_trajectory_path
        return {
            "target": self.massage_target,
            "target_label": ft._massage_target_label(self.massage_target),
            "trajectory_path": str(path) if path is not None else None,
            "point_count": len(self.massage_frames),
            "hover_height_mm": float(self.hover_height_mm),
            "force_target_n": float(self.force_target_n),
            "tool_tip_length_mm": float(ft.TOOL_TIP_LENGTH_MM),
        }

    def execute_massage_actions(self, actions=None, frames=None):
        actions = normalize_massage_actions(actions)
        use_current_frames = frames is None
        frames = list(frames) if frames is not None else list(self.massage_frames)
        if not frames:
            raise RuntimeError("没有可执行按摩轨迹，请先完成经络检测/轨迹锁定")

        print("\n开始执行按摩动作接口...")
        print(f"动作列表: {', '.join(actions)}")
        self.last_action_report = {
            "actions": actions,
            "point_count": len(frames),
            "point_failures": [],
            "shun_ok": None,
            "success": False,
        }

        point_actions = [action for action in actions if action in {"dian_jin", "fen_jin"}]
        run_shun = "shun_jin" in actions
        point_failures = []
        shun_candidate_frames = []
        shun_ok = True

        try:
            print("移动到安全高度...")
            self.update_preview_status("移动到安全高度")
            safe_pose, should_move_to_safe = self._build_session_safe_pose()
            if not self._move_to_initial_safe_pose(safe_pose, should_move_to_safe):
                return False

            if not self._adjust_leg_frames_for_reachability():
                return False
            if use_current_frames:
                frames = list(self.massage_frames)
                self.last_action_report["point_count"] = len(frames)

            if ft.LASTTIME_ROS2_FORCE:
                print(f"初始化 {self.force_target_n:.1f}N 恒力控制（请确认末端悬空无接触）...")
                self.init_force_controller()

            first_frame = frames[0]
            self.update_preview_status("移动到起始位置", first_frame.get("index", 0))
            first_pose = self._pose_from_frame_offset(first_frame, -self.hover_height_mm)
            if not self._move_to_work_pose(first_pose, "移动到起始位置", ft.TRANSIT_MOVE_VEL_FAST):
                return False

            if point_actions:
                print("\n执行点位动作...")
                for i, frame in enumerate(frames):
                    point_no = int(frame.get("index", i)) + 1
                    self.update_preview_status("到达悬空位", i)
                    print(f"\n处理点 {point_no}/{len(frames)}...")
                    hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
                    if not self._move_to_hover_with_fallback(hover_pose, "移动到悬空位"):
                        point_failures.append((point_no, "悬空位"))
                        print(f"    警告：点{point_no}悬空位不可达，跳过该点")
                        if not ft.FT_CONTINUE_ON_POINT_ERROR:
                            return False
                        continue

                    shun_candidate_frames.append(frame)
                    skip_remaining_point_actions = False

                    if "dian_jin" in point_actions:
                        print("  点筋...")
                        if not self.execute_dian_jin(frame):
                            point_failures.append((point_no, "点筋"))
                            print(f"    警告：点{point_no}点筋失败")
                            if not ft.FT_CONTINUE_ON_POINT_ERROR:
                                return False
                            skip_remaining_point_actions = True

                    if "fen_jin" in point_actions and not skip_remaining_point_actions:
                        print("  分筋...")
                        if not self.execute_fen_jin(frame):
                            point_failures.append((point_no, "分筋"))
                            print(f"    警告：点{point_no}分筋失败")
                            if not ft.FT_CONTINUE_ON_POINT_ERROR:
                                return False
            else:
                shun_candidate_frames = list(frames)

            if point_failures:
                summary = ", ".join(f"点{point}:{stage}" for point, stage in point_failures)
                print(f"\n[容错] 点位动作阶段跳过: {summary}")

            if run_shun:
                shun_frames = shun_candidate_frames or list(frames)
                if not shun_candidate_frames:
                    print("[容错] 没有已确认可达悬空点，顺筋将尝试原始轨迹")

                print("\n回到顺筋起点...")
                shun_first_frame = shun_frames[0]
                shun_first_pose = self._pose_from_frame_offset(shun_first_frame, -self.hover_height_mm)
                self.update_preview_status("回到顺筋起点", shun_first_frame.get("index", 0))
                with ft._RobotMotionSpeedScaleOverride(ft.SHUN_JIN_MOTION_SPEED_SCALE):
                    if not self._move_to_work_pose(shun_first_pose, "回到顺筋起点", ft.MOVE_VEL_FAST):
                        print("    警告：回到顺筋起点失败，仍将尝试顺筋")
                        if not ft.FT_CONTINUE_ON_POINT_ERROR:
                            return False

                print("\n执行顺筋动作...")
                shun_ok = self.execute_shun_jin(shun_frames)
                if not shun_ok:
                    print("    警告：顺筋阶段未完全成功，继续返回安全位置")
                    if not ft.FT_CONTINUE_ON_POINT_ERROR:
                        return False

            print("\n返回安全位置...")
            self.update_preview_status("返回安全位置")
            if not self._move_to_work_pose(safe_pose, "返回安全位置", ft.TRANSIT_MOVE_VEL_FAST):
                return False

            success = (not point_failures or ft.FT_CONTINUE_ON_POINT_ERROR) and (
                shun_ok or ft.FT_CONTINUE_ON_POINT_ERROR
            )
            self.last_action_report = {
                "actions": actions,
                "point_count": len(frames),
                "point_failures": [
                    {"point": int(point), "stage": str(stage)}
                    for point, stage in point_failures
                ],
                "shun_ok": bool(shun_ok) if run_shun else None,
                "success": bool(success),
            }
            print("\n按摩动作接口执行完成" if success else "\n按摩动作接口执行完成（存在失败）")
            return bool(success)
        except Exception as exc:
            self.last_action_report = {
                "actions": actions,
                "point_count": len(frames),
                "point_failures": [
                    {"point": int(point), "stage": str(stage)}
                    for point, stage in point_failures
                ],
                "shun_ok": bool(shun_ok) if run_shun else None,
                "success": False,
                "error": str(exc),
            }
            print(f"\n错误：{exc}")
            return False
        finally:
            self.close_force_controller()


class FTMassageAgentInterface:
    """Importable facade for agent and script usage."""

    def __init__(self, massage_target="back", demo=None):
        self.demo = demo if demo is not None else AgentFTMassageDemo(massage_target=massage_target)
        self.robot_initialized = self.demo.robot is not None

    def _result(self, ok, operation, **extra):
        data = {
            "ok": bool(ok),
            "operation": operation,
            "target": self.demo.massage_target,
            "target_label": ft._massage_target_label(self.demo.massage_target),
        }
        data.update(extra)
        return data

    def _error_result(self, operation, exc, raise_on_error):
        if raise_on_error:
            raise exc
        return self._result(
            False,
            operation,
            error=str(exc),
            exception_type=exc.__class__.__name__,
        )

    def close_vision(self):
        detector = getattr(self.demo, "detector", None)
        pipeline = getattr(detector, "pipeline", None)
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass

    def close_robot(self):
        self.demo.close_force_controller()
        robot = getattr(self.demo, "robot", None)
        if robot is not None and hasattr(robot, "close"):
            try:
                robot.close()
            except Exception:
                pass
        self.demo.robot = None
        self.robot_initialized = False

    def detect_meridian(
        self,
        save=True,
        display=False,
        timeout_s=None,
        stable_frames=None,
        raise_on_error=False,
    ):
        try:
            if ft._is_thigh_target(self.demo.massage_target):
                ok = self.demo.capture_thigh_trajectory_for_agent(
                    timeout_s=timeout_s,
                    stable_frames=stable_frames,
                    display=display,
                )
            else:
                ok = self.demo.capture_back_trajectory(
                    timeout_s=timeout_s,
                    stable_frames=stable_frames,
                    display=display,
                )
            if not ok:
                return self._result(False, "detect_meridian", error="检测未成功或超时")

            trajectory_path = None
            if save:
                trajectory_path = self.demo.save_current_trajectory()
            return self._result(
                True,
                "detect_meridian",
                trajectory=self.demo.trajectory_summary(trajectory_path),
            )
        except Exception as exc:
            return self._error_result("detect_meridian", exc, raise_on_error)

    def load_trajectory(self, path, raise_on_error=False):
        try:
            path = Path(path)
            with path.open("r", encoding="utf-8") as f:
                data = json.load(f)

            frames = data.get("frames") or []
            if not frames:
                raise RuntimeError(f"轨迹文件没有 frames: {path}")
            self.demo.massage_frames = frames
            self.demo.massage_pixels = data.get("pixels") or [
                frame.get("pixel") for frame in frames if frame.get("pixel") is not None
            ]
            self.demo.massage_points_mm = data.get("points_mm") or [
                frame.get("point_mm") for frame in frames if frame.get("point_mm") is not None
            ]
            if "hover_height_mm" in data:
                self.demo.hover_height_mm = float(data["hover_height_mm"])
            if "force_target_n" in data:
                self.demo.force_target_n = float(data["force_target_n"])
            self.demo.last_trajectory_path = path
            return self._result(
                True,
                "load_trajectory",
                trajectory=self.demo.trajectory_summary(path),
            )
        except Exception as exc:
            return self._error_result("load_trajectory", exc, raise_on_error)

    def init_robot(self, raise_on_error=False):
        try:
            if self.demo.robot is None:
                self.demo.init_robot()
            self.robot_initialized = True
            return self._result(True, "init_robot")
        except Exception as exc:
            return self._error_result("init_robot", exc, raise_on_error)

    def execute_actions(self, actions=None, raise_on_error=False):
        try:
            normalized_actions = normalize_massage_actions(actions)
            if not self.demo.massage_frames:
                raise RuntimeError("没有已加载/已检测的轨迹，不能执行按摩动作")
            if self.demo.robot is None:
                self.demo.init_robot()
            self.robot_initialized = True
            ok = self.demo.execute_massage_actions(normalized_actions)
            return self._result(
                ok,
                "execute_actions",
                actions=normalized_actions,
                report=self.demo.last_action_report,
                trajectory=self.demo.trajectory_summary(),
            )
        except Exception as exc:
            return self._error_result("execute_actions", exc, raise_on_error)

    def run_workflow(
        self,
        actions=None,
        save_trajectory=True,
        display=False,
        timeout_s=None,
        stable_frames=None,
        close_vision=True,
        raise_on_error=False,
    ):
        try:
            detect_result = self.detect_meridian(
                save=save_trajectory,
                display=display,
                timeout_s=timeout_s,
                stable_frames=stable_frames,
                raise_on_error=True,
            )
            if close_vision:
                self.close_vision()
            if not detect_result.get("ok"):
                return self._result(False, "run_workflow", detect=detect_result)

            execute_result = self.execute_actions(actions=actions, raise_on_error=True)
            return self._result(
                bool(detect_result.get("ok")) and bool(execute_result.get("ok")),
                "run_workflow",
                detect=detect_result,
                execute=execute_result,
            )
        except Exception as exc:
            return self._error_result("run_workflow", exc, raise_on_error)


def create_agent_interface(massage_target="back"):
    return FTMassageAgentInterface(massage_target=massage_target)


def run_agent_workflow(
    massage_target="back",
    actions=None,
    save_trajectory=True,
    display=False,
    timeout_s=None,
    stable_frames=None,
    raise_on_error=False,
):
    api = FTMassageAgentInterface(massage_target=massage_target)
    return api.run_workflow(
        actions=actions,
        save_trajectory=save_trajectory,
        display=display,
        timeout_s=timeout_s,
        stable_frames=stable_frames,
        raise_on_error=raise_on_error,
    )


def build_parser():
    parser = argparse.ArgumentParser(description="Agent API for ft.py meridian detection and massage actions")
    sub = parser.add_subparsers(dest="command", required=True)

    def add_common_detect_args(p):
        p.add_argument("--target", default="back", help="back, leg, leg_inner")
        p.add_argument("--display", action="store_true", help="show detection window")
        p.add_argument("--timeout-s", type=float, default=None)
        p.add_argument("--stable-frames", type=int, default=None)

    detect = sub.add_parser("detect", help="detect meridian and optionally save trajectory")
    add_common_detect_args(detect)
    detect.add_argument("--no-save", action="store_true")

    run = sub.add_parser("run", help="detect meridian, save trajectory, then execute actions")
    add_common_detect_args(run)
    run.add_argument("--actions", default="all", help="comma-separated actions, e.g. dian_jin,fen_jin,shun_jin")
    run.add_argument("--no-save", action="store_true")

    execute = sub.add_parser("execute", help="load saved trajectory and execute actions")
    execute.add_argument("--target", default="auto", help="auto, back, leg, leg_inner")
    execute.add_argument("--trajectory", required=True)
    execute.add_argument("--actions", default="all")

    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "detect":
        api = FTMassageAgentInterface(args.target)
        result = api.detect_meridian(
            save=not args.no_save,
            display=args.display,
            timeout_s=args.timeout_s,
            stable_frames=args.stable_frames,
        )
    elif args.command == "run":
        result = run_agent_workflow(
            massage_target=args.target,
            actions=args.actions,
            save_trajectory=not args.no_save,
            display=args.display,
            timeout_s=args.timeout_s,
            stable_frames=args.stable_frames,
        )
    elif args.command == "execute":
        target = infer_massage_target_from_trajectory(args.trajectory) if args.target == "auto" else args.target
        api = FTMassageAgentInterface(target)
        load_result = api.load_trajectory(args.trajectory)
        if load_result.get("ok"):
            execute_result = api.execute_actions(args.actions)
            result = {
                "ok": bool(execute_result.get("ok")),
                "operation": "execute_loaded_trajectory",
                "load": load_result,
                "execute": execute_result,
            }
        else:
            result = {
                "ok": False,
                "operation": "execute_loaded_trajectory",
                "load": load_result,
            }
    else:
        parser.error(f"unknown command: {args.command}")

    print(json.dumps(result, ensure_ascii=False, indent=2, default=json_default))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
