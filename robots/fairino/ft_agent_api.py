#!/usr/bin/env python3
"""
Programmatic agent interface for ft.py.

This module intentionally keeps ft.py unchanged. It wraps the existing
LastTimeRos2Demo implementation with importable methods and a JSON CLI.
"""

import argparse
import json
import math
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
AGENT_FORCE_READING_MAX_ABS_N = float(os.environ.get("FT_AGENT_FORCE_READING_MAX_ABS_N", "1000.0"))
AGENT_FORCE_READING_MAX_ABS_NM = float(os.environ.get("FT_AGENT_FORCE_READING_MAX_ABS_NM", "1000.0"))
AGENT_FORCE_READING_RETRIES = max(1, int(os.environ.get("FT_AGENT_FORCE_READING_RETRIES", "2")))


class MassageExecutionInterrupted(RuntimeError):
    """Raised internally when an external controller pauses or stops execution."""

    def __init__(self, status, checkpoint):
        super().__init__(status)
        self.status = status
        self.checkpoint = dict(checkpoint or {})


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


def _stage_for_actions(actions):
    point_actions = [action for action in actions if action in {"dian_jin", "fen_jin"}]
    if point_actions:
        return "point_actions"
    if "shun_jin" in actions:
        return "shun_jin"
    return "completed"


class AgentFTMassageDemo(ft.LastTimeRos2Demo):
    """Small extension layer over ft.LastTimeRos2Demo for agent calls."""

    def __init__(self, massage_target="back"):
        super().__init__(massage_target=massage_target)
        self.last_raw_confirmation_path = None
        self.last_trajectory_path = None
        self.last_action_report = {}
        self._agent_control = None
        self._agent_control_stage = None
        self._agent_control_action = None
        self._agent_control_point_index = 0
        self._agent_pause_pending = False
        self._agent_stop_pending = False
        self._agent_last_checkpoint = {}
        self._agent_point_actions = []
        self._agent_run_shun = False
        self._agent_point_count = 0
        self._agent_control_repeat_index = 0
        self._agent_control_step_index = 0
        self._agent_applied_force_adjust_seqs = set()

    def _agent_force_reading_valid(self, data):
        if not isinstance(data, (list, tuple)) or len(data) < 6:
            return False
        try:
            values = [float(v) for v in data[:6]]
        except (TypeError, ValueError):
            return False
        if not all(math.isfinite(v) for v in values):
            return False
        force_max = max(abs(v) for v in values[:3])
        torque_max = max(abs(v) for v in values[3:6])
        return (
            force_max <= AGENT_FORCE_READING_MAX_ABS_N
            and torque_max <= AGENT_FORCE_READING_MAX_ABS_NM
        )

    def _install_force_read_guard(self):
        controller = getattr(self, "force_controller", None)
        if controller is None or getattr(controller, "_agent_read_guard_installed", False):
            return

        raw_read = controller.read

        def guarded_read():
            last_invalid = None
            for attempt in range(AGENT_FORCE_READING_RETRIES):
                data = raw_read()
                if self._agent_force_reading_valid(data):
                    return [float(v) for v in data[:6]]
                last_invalid = data
                if data is not None:
                    print(f"[Force] 丢弃异常六维力读数 attempt={attempt + 1}: {data}")
                time.sleep(0.02)
            if last_invalid is not None:
                print("[Force] 连续读到异常六维力数据，按传感器读数异常处理")
            return None

        controller.read = guarded_read
        controller._agent_read_guard_installed = True

    def init_force_controller(self):
        result = super().init_force_controller()
        self._install_force_read_guard()
        return result

    def _apply_live_force_adjustment(self, adjustment):
        if not isinstance(adjustment, dict):
            return None

        seq = str(adjustment.get("seq") or adjustment.get("id") or "").strip()
        if seq and seq in self._agent_applied_force_adjust_seqs:
            return None

        previous = self._current_force_target_n()
        if adjustment.get("target_force_n") not in (None, ""):
            requested = abs(float(adjustment["target_force_n"]))
        else:
            delta_n = float(adjustment.get("delta_n") or 0.0)
            requested = previous + delta_n

        old_force, new_force = self.set_force_target_n(
            requested,
            context=str(adjustment.get("source") or "语音力度调整"),
        )
        if seq:
            self._agent_applied_force_adjust_seqs.add(seq)

        applied_delta = float(new_force) - float(old_force)
        result = {
            "seq": seq or None,
            "requested_delta_n": float(adjustment.get("delta_n") or applied_delta),
            "applied_delta_n": applied_delta,
            "previous_force_target_n": float(old_force),
            "force_target_n": float(new_force),
            "changed": abs(applied_delta) > 1e-6,
            "created_at": adjustment.get("created_at"),
        }
        if result["changed"]:
            print(
                f"[Agent] 语音力度调整: {old_force:.1f}N -> {new_force:.1f}N "
                f"(delta={applied_delta:+.1f}N)"
            )
        else:
            print(f"[Agent] 语音力度调整已到边界: {new_force:.1f}N")
        return result

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

    def robot_state_snapshot(self):
        snapshot = {}
        robot = getattr(self, "robot", None)
        if robot is None:
            return snapshot
        try:
            snapshot["tcp_pose"] = robot.get_actual_tcp_pose()
        except Exception:
            pass
        try:
            snapshot["joints_deg"] = robot.get_actual_joint_positions_deg()
        except Exception:
            pass
        return snapshot

    def _control_checkpoint(
        self,
        control,
        stage,
        current_action=None,
        current_point_index=None,
        next_stage=None,
        next_point_index=None,
        message=None,
        safe_to_pause=True,
        extra=None,
    ):
        if control is None:
            return "continue"
        checkpoint = {
            "stage": stage,
            "current_action": current_action,
            "current_point_index": current_point_index,
            "next_stage": next_stage if next_stage is not None else stage,
            "next_point_index": next_point_index
            if next_point_index is not None
            else current_point_index,
            "message": message,
            "safe_to_pause": bool(safe_to_pause),
            "robot_state": self.robot_state_snapshot(),
            "force_target_n": float(self.force_target_n),
        }
        if extra:
            checkpoint.update(extra)
        self._agent_last_checkpoint = dict(checkpoint)

        raw_request = None
        if hasattr(control, "checkpoint"):
            raw_request = control.checkpoint(checkpoint)
        elif callable(control):
            raw_request = control(checkpoint)

        force_update = None
        if isinstance(raw_request, dict):
            force_adjustment = raw_request.get("force_adjustment") or raw_request.get("force_adjust")
            if force_adjustment:
                force_update = self._apply_live_force_adjustment(force_adjustment)
            request = str(raw_request.get("request") or "continue").strip().lower()
        else:
            request = str(raw_request or "continue").strip().lower()

        if force_update:
            checkpoint["force_target_n"] = float(self.force_target_n)
            checkpoint["last_force_adjustment"] = force_update
            self._agent_last_checkpoint = dict(checkpoint)
            if hasattr(control, "force_target_changed"):
                control.force_target_changed(force_update, checkpoint)

        if request in {"pause", "paused", "stop", "stopped"}:
            status = "paused" if request in {"pause", "paused"} else "stopped"
            raise MassageExecutionInterrupted(status, checkpoint)
        return request

    def _set_agent_control_context(
        self,
        stage=None,
        action=None,
        point_index=None,
        repeat_index=None,
        step_index=None,
    ):
        if stage is not None:
            self._agent_control_stage = stage
        if action is not None:
            self._agent_control_action = action
        if point_index is not None:
            self._agent_control_point_index = int(point_index or 0)
        if repeat_index is not None:
            self._agent_control_repeat_index = int(repeat_index or 0)
        if step_index is not None:
            self._agent_control_step_index = int(step_index or 0)

    def _agent_control_checkpoint(
        self,
        message=None,
        safe_to_pause=False,
        stage=None,
        action=None,
        point_index=None,
        repeat_index=None,
        step_index=None,
        next_stage=None,
        next_point_index=None,
        next_action=None,
        next_repeat_index=None,
        next_step_index=None,
    ):
        control = self._agent_control
        if control is None:
            return "continue"

        if (
            stage is not None
            or action is not None
            or point_index is not None
            or repeat_index is not None
            or step_index is not None
        ):
            self._set_agent_control_context(
                stage,
                action,
                point_index,
                repeat_index,
                step_index,
            )

        stage = self._agent_control_stage or "point_actions"
        action = self._agent_control_action
        point_index = self._agent_control_point_index
        repeat_index = self._agent_control_repeat_index
        step_index = self._agent_control_step_index
        extra = {
            "current_repeat_index": repeat_index,
            "current_step_index": step_index,
            "next_repeat_index": repeat_index if next_repeat_index is None else int(next_repeat_index or 0),
            "next_step_index": step_index if next_step_index is None else int(next_step_index or 0),
        }
        if next_action is not None:
            extra["next_action"] = next_action
        request = self._control_checkpoint(
            control,
            stage=stage,
            current_action=action,
            current_point_index=point_index,
            next_stage=next_stage if next_stage is not None else stage,
            next_point_index=next_point_index if next_point_index is not None else point_index,
            message=message,
            safe_to_pause=safe_to_pause,
            extra=extra,
        )
        if request == "pause_pending":
            self._agent_pause_pending = True
        elif request == "stop_pending":
            self._agent_stop_pending = True
        return request

    def _is_hover_pause_context(self, context):
        text = str(context or "")
        return "悬空位" in text or "回悬空位" in text or "卸力" in text

    def _next_resume_after_point_action(self, action_name):
        current_index = int(self._agent_control_point_index or 0)
        if action_name == "dian_jin" and "fen_jin" in self._agent_point_actions:
            return "point_actions", "fen_jin", current_index
        next_index = current_index + 1
        if next_index < int(self._agent_point_count or 0) and self._agent_point_actions:
            return "point_actions", "move_to_hover", next_index
        if self._agent_run_shun:
            return "shun_jin", "move_to_shun_start", 0
        return "completed", "completed", 0

    def _next_resume_after_repeat(self, action_name, repeat_idx):
        current_index = int(self._agent_control_point_index or 0)
        if action_name == "dian_jin":
            repeat_count = int(ft.DIAN_JIN_REPEAT_COUNT)
        elif action_name == "fen_jin":
            repeat_count = int(ft.FEN_JIN_REPEAT_COUNT)
        else:
            repeat_count = 1

        next_repeat = int(repeat_idx) + 1
        if next_repeat < repeat_count:
            return "point_actions", action_name, current_index, next_repeat, 0

        next_stage, next_action, next_point = self._next_resume_after_point_action(action_name)
        return next_stage, next_action, next_point, 0, 0

    def _move_force_pose_checked(
        self,
        pose,
        context,
        vel=ft.MOVE_VEL_SLOW,
        close_success=False,
        close_pos_tol=ft.ROS2_MOTION_DONE_POSE_TOL_MM,
        close_ori_tol=ft.ROS2_MOTION_DONE_ORI_TOL_DEG,
    ):
        request = self._agent_control_checkpoint(
            message=f"{context}: 准备运动",
            safe_to_pause=False,
        )
        if request in {"pause_pending", "stop_pending"} and not self._is_hover_pause_context(context):
            return False
        ok = super()._move_force_pose_checked(
            pose,
            context,
            vel=vel,
            close_success=close_success,
            close_pos_tol=close_pos_tol,
            close_ori_tol=close_ori_tol,
        )
        if ok:
            request = self._agent_control_checkpoint(
                message=f"{context}: 运动完成",
                safe_to_pause=False,
            )
            if request in {"pause_pending", "stop_pending"} and not self._is_hover_pause_context(context):
                status = "stopped" if request == "stop_pending" else "paused"
                raise MassageExecutionInterrupted(status, self._agent_last_checkpoint)
        return ok

    def _move_to_hover_for_force(self, frame, context):
        ok = super()._move_to_hover_for_force(frame, context)
        if ok:
            self._agent_control_checkpoint(
                message=f"{context}: 悬空位检查完成，可暂停",
                safe_to_pause=True,
            )
        return ok

    def _hold_target_force(self, frame, split_offset_mm, start_offset_mm, seconds, context):
        offset = float(start_offset_mm)
        max_offset = max(float(self.force_approach_max_offset_mm), float(ft.FORCE_CONTACT_OFFSET_MM))
        min_offset = -float(self.hover_height_mm)
        deadline = time.time() + max(0.0, float(seconds))
        last_print = 0.0

        while time.time() < deadline:
            request = self._agent_control_checkpoint(
                message=f"{context}: 保压中",
                safe_to_pause=False,
            )
            if request in {"pause_pending", "stop_pending"}:
                action_text = "停止" if request == "stop_pending" else "暂停"
                print(f"[Force] {context}: 收到{action_text}请求，提前结束保压并回悬空位")
                return offset, False
            target_n = self._current_force_target_n()
            force_n, data = self._read_force_axis(context)
            err_n = target_n - force_n
            if abs(err_n) > ft.FORCE_TARGET_TOL_N:
                delta = ft.FORCE_HOLD_KP_MM_PER_N * err_n
                delta = max(-ft.FORCE_HOLD_MAX_STEP_MM, min(ft.FORCE_HOLD_MAX_STEP_MM, delta))
                next_offset = max(min_offset, min(max_offset, offset + delta))
                if abs(next_offset - offset) > 1e-4:
                    offset = next_offset
                    pose = self._pose_from_frame_offset(frame, offset, split_offset_mm)
                    if not self._move_force_pose_checked(pose, f"{context} 恒力微调"):
                        return offset, False

            now = time.time()
            if now - last_print > 0.4:
                print(
                    f"[Force] {context}: offset={offset:+.1f}mm "
                    f"Fz={data[2]:.2f}N press={force_n:.2f}N target={target_n:.1f}N"
                )
                last_print = now
            time.sleep(self.force_controller._monitor_period)

        return offset, True

    def _retract_to_hover(
        self,
        frame,
        context,
        next_stage=None,
        next_action=None,
        next_point_index=None,
        next_repeat_index=None,
        next_step_index=None,
    ):
        ok = super()._retract_to_hover(frame, context)
        if ok:
            self._agent_control_checkpoint(
                message=f"{context}: 已回到悬空位，可暂停",
                safe_to_pause=True,
                next_stage=next_stage,
                next_action=next_action,
                next_point_index=next_point_index,
                next_repeat_index=next_repeat_index,
                next_step_index=next_step_index,
            )
        return ok

    def execute_dian_jin(self, frame, start_repeat_index=0):
        hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
        dian_jin_pose = self._pose_from_frame_offset(
            frame,
            -(self.hover_height_mm - ft.DIAN_JIN_DEPTH_MM),
        )
        use_small_fen = ft.DIAN_JIN_MODE in {"small_fen", "small-fen", "small_split", "split", "fen"}
        small_fen_lateral_mm = abs(float(ft.DIAN_AS_SMALL_FEN_LATERAL_MM))
        repeat_count = int(ft.DIAN_JIN_REPEAT_COUNT)
        start_repeat_index = max(0, min(repeat_count, int(start_repeat_index or 0)))
        if start_repeat_index >= repeat_count:
            return True

        for repeat_idx in range(start_repeat_index, repeat_count):
            round_text = f"{repeat_idx + 1}/{repeat_count}"
            self._set_agent_control_context(
                "point_actions",
                "dian_jin",
                self._agent_control_point_index,
                repeat_idx,
                0,
            )
            action_label = "点筋小幅分筋" if use_small_fen else "点筋"
            self.update_preview_status(f"{action_label} {round_text}", frame.get("index"))

            if ft.LASTTIME_ROS2_FORCE:
                ok = False
                repeat_completed = False
                next_stage = "point_actions"
                next_action = "dian_jin"
                next_point = self._agent_control_point_index
                next_repeat = repeat_idx
                next_step = 0
                try:
                    if not self._move_to_hover_for_force(frame, f"点筋悬空位 {round_text}"):
                        return False
                    offset, reached = self._approach_to_target_force(frame, f"{action_label}中心 {round_text}")
                    if not reached:
                        return False
                    if use_small_fen:
                        offset, ok = self._hold_target_force(
                            frame,
                            0.0,
                            offset,
                            ft.FORCE_FEN_DWELL_S,
                            f"{action_label}中心保压 {round_text}",
                        )
                        if not ok:
                            return False
                        for label, split_offset in (
                            (f"{action_label}偏移+ {round_text}", small_fen_lateral_mm),
                            (f"{action_label}偏移- {round_text}", -small_fen_lateral_mm),
                            (f"{action_label}回中心 {round_text}", 0.0),
                        ):
                            pose = self._pose_from_frame_offset(frame, offset, split_offset)
                            if not self._move_force_pose_checked(pose, label):
                                return False
                            offset, ok = self._hold_target_force(
                                frame,
                                split_offset,
                                offset,
                                ft.FORCE_FEN_DWELL_S,
                                f"{label}保压",
                            )
                            if not ok:
                                return False
                    else:
                        offset, ok = self._hold_target_force(
                            frame,
                            0.0,
                            offset,
                            ft.FORCE_DIAN_DWELL_S,
                            f"点筋保压 {round_text}",
                        )
                    repeat_completed = bool(ok)
                except MassageExecutionInterrupted:
                    raise
                except Exception as exc:
                    print(f"    警告：{action_label}{round_text}力控失败 ({exc})")
                    return False
                finally:
                    if repeat_completed:
                        (
                            next_stage,
                            next_action,
                            next_point,
                            next_repeat,
                            next_step,
                        ) = self._next_resume_after_repeat("dian_jin", repeat_idx)
                    if not self._retract_to_hover(
                        frame,
                        f"点筋结束回悬空位 {round_text}",
                        next_stage=next_stage,
                        next_action=next_action,
                        next_point_index=next_point,
                        next_repeat_index=next_repeat,
                        next_step_index=next_step,
                    ):
                        ok = False
                if not ok:
                    return False
                continue

            if use_small_fen:
                for label, split_offset in (
                    (f"点筋小幅分筋偏移+ {round_text}", small_fen_lateral_mm),
                    (f"点筋小幅分筋偏移- {round_text}", -small_fen_lateral_mm),
                    (f"点筋小幅分筋回中心 {round_text}", 0.0),
                ):
                    self._agent_control_checkpoint(
                        message=f"{label}: 准备运动",
                        safe_to_pause=False,
                    )
                    pose = self._pose_from_frame_offset(
                        frame,
                        -(self.hover_height_mm - ft.DIAN_JIN_DEPTH_MM),
                        split_offset_mm=split_offset,
                    )
                    ret = self.robot.MoveCart(
                        desc_pos=pose,
                        tool=ft.ROS2_TOOL,
                        user=ft.ROS2_USER,
                        vel=ft.MOVE_VEL_SLOW,
                        blendT=ft.BLEND_BLOCKING,
                    )
                    if ret != 0:
                        print(f"    警告：{label}失败 (err={ret})")
                        return False
                    time.sleep(0.2)

                ret = self.robot.MoveCart(
                    desc_pos=hover_pose,
                    tool=ft.ROS2_TOOL,
                    user=ft.ROS2_USER,
                    vel=ft.MOVE_VEL_SLOW,
                    blendT=ft.BLEND_BLOCKING,
                )
                if ret != 0:
                    print(f"    警告：点筋小幅分筋{round_text}回到悬空位失败 (err={ret})")
                    return False
                next_stage, next_action, next_point, next_repeat, next_step = self._next_resume_after_repeat(
                    "dian_jin",
                    repeat_idx,
                )
                self._agent_control_checkpoint(
                    message=f"点筋小幅分筋{round_text}: 已回到悬空位，可暂停",
                    safe_to_pause=True,
                    next_stage=next_stage,
                    next_action=next_action,
                    next_point_index=next_point,
                    next_repeat_index=next_repeat,
                    next_step_index=next_step,
                )
                continue

            self._agent_control_checkpoint(
                message=f"点筋{round_text}: 准备下压",
                safe_to_pause=False,
            )
            ret = self.robot.MoveCart(
                desc_pos=dian_jin_pose,
                tool=ft.ROS2_TOOL,
                user=ft.ROS2_USER,
                vel=ft.MOVE_VEL_SLOW,
                blendT=ft.BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：点筋{round_text}失败 (err={ret})")
                return False
            time.sleep(0.3)

            ret = self.robot.MoveCart(
                desc_pos=hover_pose,
                tool=ft.ROS2_TOOL,
                user=ft.ROS2_USER,
                vel=ft.MOVE_VEL_SLOW,
                blendT=ft.BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：点筋{round_text}回到悬空位失败 (err={ret})")
                return False
            next_stage, next_action, next_point, next_repeat, next_step = self._next_resume_after_repeat(
                "dian_jin",
                repeat_idx,
            )
            self._agent_control_checkpoint(
                message=f"点筋{round_text}: 已回到悬空位，可暂停",
                safe_to_pause=True,
                next_stage=next_stage,
                next_action=next_action,
                next_point_index=next_point,
                next_repeat_index=next_repeat,
                next_step_index=next_step,
            )
        return True

    def execute_fen_jin(self, frame, start_repeat_index=0, start_step_index=0):
        self.update_preview_status("分筋", frame.get("index"))
        hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
        repeat_count = int(ft.FEN_JIN_REPEAT_COUNT)
        force_steps = (
            ("分筋偏移+", ft.FORCE_FEN_LATERAL_MM),
            ("分筋偏移-", -ft.FORCE_FEN_LATERAL_MM),
            ("分筋回中心", 0.0),
        )
        start_repeat_index = max(0, min(repeat_count, int(start_repeat_index or 0)))
        start_step_index = max(0, min(len(force_steps) - 1, int(start_step_index or 0)))
        if start_repeat_index >= repeat_count:
            return True

        if ft.LASTTIME_ROS2_FORCE:
            ok = False
            next_stage = "point_actions"
            next_action = "fen_jin"
            next_point = self._agent_control_point_index
            next_repeat = start_repeat_index
            next_step = start_step_index
            try:
                self._set_agent_control_context(
                    "point_actions",
                    "fen_jin",
                    self._agent_control_point_index,
                    start_repeat_index,
                    start_step_index,
                )
                if not self._move_to_hover_for_force(frame, "分筋悬空位"):
                    return False
                offset, reached = self._approach_to_target_force(frame, "分筋中心")
                if not reached:
                    return False
                offset, ok = self._hold_target_force(
                    frame,
                    0.0,
                    offset,
                    ft.FORCE_FEN_DWELL_S,
                    "分筋中心保压",
                )
                if not ok:
                    return False
                if self._agent_pause_pending:
                    return False

                for repeat_idx in range(start_repeat_index, repeat_count):
                    round_text = f"{repeat_idx + 1}/{repeat_count}"
                    first_step = start_step_index if repeat_idx == start_repeat_index else 0
                    for step_idx in range(first_step, len(force_steps)):
                        label_prefix, split_offset = force_steps[step_idx]
                        label = f"{label_prefix} {round_text}"
                        self._set_agent_control_context(
                            "point_actions",
                            "fen_jin",
                            self._agent_control_point_index,
                            repeat_idx,
                            step_idx,
                        )
                        next_stage = "point_actions"
                        next_action = "fen_jin"
                        next_point = self._agent_control_point_index
                        next_repeat = repeat_idx
                        next_step = step_idx
                        pose = self._pose_from_frame_offset(frame, offset, split_offset)
                        if not self._move_force_pose_checked(pose, label):
                            return False
                        offset, ok = self._hold_target_force(
                            frame,
                            split_offset,
                            offset,
                            ft.FORCE_FEN_DWELL_S,
                            f"{label}保压",
                        )
                        if not ok:
                            return False
                        if self._agent_pause_pending:
                            return False
                        if step_idx + 1 < len(force_steps):
                            next_step = step_idx + 1
                        else:
                            (
                                next_stage,
                                next_action,
                                next_point,
                                next_repeat,
                                next_step,
                            ) = self._next_resume_after_repeat("fen_jin", repeat_idx)
            except MassageExecutionInterrupted:
                raise
            except Exception as exc:
                print(f"    警告：分筋力控失败 ({exc})")
                return False
            finally:
                if not self._retract_to_hover(
                    frame,
                    "分筋结束回悬空位",
                    next_stage=next_stage,
                    next_action=next_action,
                    next_point_index=next_point,
                    next_repeat_index=next_repeat,
                    next_step_index=next_step,
                ):
                    ok = False
            if not ok:
                return False
            return True

        positive_pose = self._pose_from_frame_offset(
            frame,
            -self.hover_height_mm,
            split_offset_mm=ft.FEN_JIN_LATERAL_MM,
        )
        negative_pose = self._pose_from_frame_offset(
            frame,
            -self.hover_height_mm,
            split_offset_mm=-ft.FEN_JIN_LATERAL_MM,
        )

        non_force_steps = (
            ("分筋偏移+", positive_pose, False),
            ("分筋偏移-", negative_pose, False),
            ("分筋回悬空位", hover_pose, True),
        )
        start_step_index = max(0, min(len(non_force_steps) - 1, int(start_step_index or 0)))
        for repeat_idx in range(start_repeat_index, repeat_count):
            round_text = f"{repeat_idx + 1}/{repeat_count}"
            first_step = start_step_index if repeat_idx == start_repeat_index else 0
            for step_idx in range(first_step, len(non_force_steps)):
                label_prefix, pose, safe_to_pause = non_force_steps[step_idx]
                label = f"{label_prefix} {round_text}"
                self._set_agent_control_context(
                    "point_actions",
                    "fen_jin",
                    self._agent_control_point_index,
                    repeat_idx,
                    step_idx,
                )
                if safe_to_pause:
                    next_stage, next_action, next_point, next_repeat, next_step = (
                        self._next_resume_after_repeat("fen_jin", repeat_idx)
                    )
                elif step_idx + 1 < len(non_force_steps):
                    next_stage = "point_actions"
                    next_action = "fen_jin"
                    next_point = self._agent_control_point_index
                    next_repeat = repeat_idx
                    next_step = step_idx + 1
                else:
                    next_stage, next_action, next_point, next_repeat, next_step = (
                        self._next_resume_after_repeat("fen_jin", repeat_idx)
                    )
                self._agent_control_checkpoint(
                    message=f"{label}: 准备运动",
                    safe_to_pause=False,
                    next_stage=next_stage,
                    next_action=next_action,
                    next_point_index=next_point,
                    next_repeat_index=next_repeat,
                    next_step_index=next_step,
                )
                ret = self.robot.MoveCart(
                    desc_pos=pose,
                    tool=ft.ROS2_TOOL,
                    user=ft.ROS2_USER,
                    vel=ft.MOVE_VEL_SLOW,
                    blendT=ft.BLEND_BLOCKING,
                )
                if ret != 0:
                    print(f"    警告：{label}失败 (err={ret})")
                    return False
                self._agent_control_checkpoint(
                    message=f"{label}: 运动完成",
                    safe_to_pause=safe_to_pause,
                    next_stage=next_stage,
                    next_action=next_action,
                    next_point_index=next_point,
                    next_repeat_index=next_repeat,
                    next_step_index=next_step,
                )
                time.sleep(0.2)
        return True

    def execute_shun_jin(self, frames=None, start_index=0, control=None):
        with ft._RobotMotionSpeedScaleOverride(ft.SHUN_JIN_MOTION_SPEED_SCALE):
            return self._execute_shun_jin(frames, start_index=start_index, control=control)

    def _execute_shun_jin(self, frames=None, start_index=0, control=None):
        print("顺筋动作...")
        frames = list(frames) if frames is not None else list(self.massage_frames)
        if not frames:
            print("    警告：顺筋没有可用点")
            return ft.FT_CONTINUE_ON_POINT_ERROR
        if len(frames) < max(1, ft.FT_SHUN_MIN_POINTS):
            print(f"    警告：顺筋候选点不足 {ft.FT_SHUN_MIN_POINTS} 个，将按可用点继续")

        start_index = min(max(0, int(start_index or 0)), len(frames) - 1)

        if ft.LASTTIME_ROS2_FORCE:
            last_hover_pose = None
            found_start_index = None
            offset = -float(self.hover_height_mm)

            for candidate_index in range(start_index, len(frames)):
                start_frame = frames[candidate_index]
                point_no = int(start_frame.get("index", candidate_index)) + 1
                self._set_agent_control_context("shun_jin", "find_shun_start", candidate_index)
                self._control_checkpoint(
                    control,
                    stage="shun_jin",
                    current_action="find_shun_start",
                    current_point_index=candidate_index,
                    next_stage="shun_jin",
                    next_point_index=candidate_index,
                    message=f"准备寻找顺筋起点 点{point_no}",
                )
                last_hover_pose = self._pose_from_frame_offset(start_frame, -self.hover_height_mm)
                try:
                    if not self._move_to_hover_for_force(start_frame, f"顺筋悬空起点 点{point_no}"):
                        print(f"    警告：顺筋起点 点{point_no} 悬空位失败")
                        if not ft.FT_CONTINUE_ON_POINT_ERROR:
                            return False
                        continue
                    offset, reached = self._approach_to_target_force(start_frame, f"顺筋起点 点{point_no}")
                    if reached:
                        found_start_index = candidate_index
                        break
                    print(f"    警告：顺筋起点 点{point_no} 未达到目标力，尝试下一个候选点")
                    if not self._retract_to_hover(start_frame, f"顺筋起点失败回悬空位 点{point_no}"):
                        print(f"    警告：顺筋起点 点{point_no} 回悬空位失败")
                    if not ft.FT_CONTINUE_ON_POINT_ERROR:
                        return False
                except MassageExecutionInterrupted:
                    raise
                except Exception as exc:
                    print(f"    警告：顺筋起点 点{point_no} 力控失败 ({exc})")
                    return False

            if found_start_index is None:
                print("    警告：顺筋没有找到可贴近的起点")
                return ft.FT_CONTINUE_ON_POINT_ERROR

            ok = True
            moved_count = 0
            skipped_points = []
            try:
                for i in range(found_start_index, len(frames)):
                    frame = frames[i]
                    point_no = int(frame.get("index", i)) + 1
                    self._set_agent_control_context("shun_jin", "shun_jin", i)
                    self._control_checkpoint(
                        control,
                        stage="shun_jin",
                        current_action="shun_jin",
                        current_point_index=i,
                        next_stage="shun_jin",
                        next_point_index=i,
                        message=f"准备顺筋移动 点{point_no}",
                    )
                    self.update_preview_status("顺筋", frame.get("index", point_no - 1))
                    print(f"  移动到点{point_no}...")
                    last_hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
                    pose = self._pose_from_frame_offset(frame, offset)
                    if not self._move_force_pose_checked(pose, f"顺筋移动 点{point_no}"):
                        skipped_points.append(point_no)
                        ok = False
                        if not ft.FT_CONTINUE_ON_POINT_ERROR:
                            break
                        print(f"    警告：顺筋点{point_no}移动失败，跳过该点继续")
                        continue
                    offset, hold_ok = self._hold_target_force(
                        frame,
                        0.0,
                        offset,
                        ft.FORCE_SHUN_DWELL_S,
                        f"顺筋保压 点{point_no}",
                    )
                    if not hold_ok:
                        skipped_points.append(point_no)
                        ok = False
                        if not ft.FT_CONTINUE_ON_POINT_ERROR:
                            break
                        print(f"    警告：顺筋点{point_no}保压失败，跳过该点继续")
                        continue
                    moved_count += 1
                    next_index = i + 1
                    request = self._control_checkpoint(
                        control,
                        stage="shun_jin",
                        current_action="shun_complete",
                        current_point_index=i,
                        next_stage="shun_jin" if next_index < len(frames) else "completed",
                        next_point_index=next_index if next_index < len(frames) else 0,
                        message=f"顺筋点{point_no}完成",
                        safe_to_pause=False,
                    )
                    if request == "pause_pending":
                        pause_hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
                        if not self._move_force_pose_checked(pause_hover_pose, "顺筋暂停回悬空位"):
                            ok = False
                            if not ft.FT_CONTINUE_ON_POINT_ERROR:
                                break
                        self._set_agent_control_context("shun_jin", "pause_hover", i)
                        self._control_checkpoint(
                            control,
                            stage="shun_jin",
                            current_action="pause_hover",
                            current_point_index=i,
                            next_stage="shun_jin" if next_index < len(frames) else "completed",
                            next_point_index=next_index if next_index < len(frames) else 0,
                            message=f"顺筋点{point_no}已回到悬空位，可暂停",
                            safe_to_pause=True,
                        )
                if skipped_points:
                    print(f"    警告：顺筋跳过点: {skipped_points}")
                if moved_count <= 0:
                    print("    警告：顺筋没有完成任何候选点")
                    ok = False
                if ft.FT_CONTINUE_ON_POINT_ERROR and skipped_points:
                    ok = True
            except MassageExecutionInterrupted:
                raise
            except Exception as exc:
                print(f"    警告：顺筋力控失败 ({exc})")
                ok = False
            finally:
                if last_hover_pose is not None:
                    if not self._move_force_pose_checked(last_hover_pose, "顺筋结束回悬空位"):
                        if ft.FT_CONTINUE_ON_POINT_ERROR:
                            print("    警告：顺筋结束回悬空位失败，将继续返回安全位置")
                        else:
                            ok = False
            return ok

        moved_count = 0
        skipped_points = []
        for i in range(start_index, len(frames)):
            frame = frames[i]
            point_no = int(frame.get("index", i)) + 1
            self._set_agent_control_context("shun_jin", "shun_jin", i)
            self._control_checkpoint(
                control,
                stage="shun_jin",
                current_action="shun_jin",
                current_point_index=i,
                next_stage="shun_jin",
                next_point_index=i,
                message=f"准备顺筋移动 点{point_no}",
            )
            self.update_preview_status("顺筋", frame.get("index", i))
            print(f"  移动到点{point_no}...")
            pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
            ret = self.robot.MoveCart(
                desc_pos=pose,
                tool=ft.ROS2_TOOL,
                user=ft.ROS2_USER,
                vel=ft.MOVE_VEL_SLOW,
                blendT=ft.BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：顺筋点{point_no}移动失败 (err={ret})")
                skipped_points.append(point_no)
                if not ft.FT_CONTINUE_ON_POINT_ERROR:
                    return False
                continue
            moved_count += 1
            next_index = i + 1
            self._control_checkpoint(
                control,
                stage="shun_jin",
                current_action="shun_complete",
                current_point_index=i,
                next_stage="shun_jin" if next_index < len(frames) else "completed",
                next_point_index=next_index if next_index < len(frames) else 0,
                message=f"顺筋点{point_no}完成",
            )
        if skipped_points:
            print(f"    警告：顺筋跳过点: {skipped_points}")
        return moved_count > 0 or ft.FT_CONTINUE_ON_POINT_ERROR

    def execute_massage_actions(
        self,
        actions=None,
        frames=None,
        control=None,
        start_stage=None,
        start_point_index=0,
        start_action=None,
        start_repeat_index=0,
        start_step_index=0,
    ):
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
            "status": "running",
        }

        point_actions = [action for action in actions if action in {"dian_jin", "fen_jin"}]
        run_shun = "shun_jin" in actions
        dian_action_text = (
            "点筋小幅分筋"
            if ft.DIAN_JIN_MODE in {"small_fen", "small-fen", "small_split", "split", "fen"}
            else "点筋"
        )
        point_failures = []
        shun_candidate_frames = []
        shun_ok = True
        start_stage = start_stage or _stage_for_actions(actions)
        start_point_index = max(0, int(start_point_index or 0))
        start_repeat_index = max(0, int(start_repeat_index or 0))
        start_step_index = max(0, int(start_step_index or 0))
        previous_agent_control = self._agent_control
        previous_agent_stage = self._agent_control_stage
        previous_agent_action = self._agent_control_action
        previous_agent_point_index = self._agent_control_point_index
        previous_agent_repeat_index = self._agent_control_repeat_index
        previous_agent_step_index = self._agent_control_step_index
        previous_agent_pause_pending = self._agent_pause_pending
        previous_agent_stop_pending = self._agent_stop_pending
        previous_agent_last_checkpoint = self._agent_last_checkpoint
        previous_agent_point_actions = self._agent_point_actions
        previous_agent_run_shun = self._agent_run_shun
        previous_agent_point_count = self._agent_point_count
        self._agent_control = control
        self._agent_pause_pending = False
        self._agent_stop_pending = False
        self._agent_last_checkpoint = {}
        self._agent_point_actions = list(point_actions)
        self._agent_run_shun = bool(run_shun)
        self._agent_point_count = len(frames)
        self._set_agent_control_context(
            start_stage,
            "prepare",
            start_point_index,
            start_repeat_index,
            start_step_index,
        )
        resume_from_local_hover = bool(
            start_action
            or start_repeat_index > 0
            or start_step_index > 0
            or (start_stage == "shun_jin" and start_point_index > 0)
        )

        try:
            if resume_from_local_hover:
                print("从暂停悬空位继续，跳过安全高度抬升...")
                self.update_preview_status("从暂停悬空位继续")
                self._control_checkpoint(
                    control,
                    stage=start_stage,
                    current_action="prepare",
                    current_point_index=start_point_index,
                    next_stage=start_stage,
                    next_point_index=start_point_index,
                    message="从暂停悬空位继续，跳过安全高度抬升",
                )
                current_pose = self.robot.get_actual_tcp_pose()
                safe_pose = [float(v) for v in current_pose]
                self.motion_orientation = [safe_pose[3], safe_pose[4], safe_pose[5]]
                safe_z = float(getattr(ft, "ROS2_LIFT_SAFE_Z_MM", 300.0))
                if safe_pose[2] < safe_z:
                    safe_pose[2] = safe_z
            else:
                print("移动到安全高度...")
                self.update_preview_status("移动到安全高度")
                self._control_checkpoint(
                    control,
                    stage=start_stage,
                    current_action="prepare",
                    current_point_index=start_point_index,
                    next_stage=start_stage,
                    next_point_index=start_point_index,
                    message="准备移动到安全高度",
                )
                safe_pose, should_move_to_safe = self._build_session_safe_pose()
                if not self._move_to_initial_safe_pose(safe_pose, should_move_to_safe):
                    return False

            if not self._adjust_leg_frames_for_reachability():
                return False
            if use_current_frames:
                frames = list(self.massage_frames)
                self.last_action_report["point_count"] = len(frames)
                self._agent_point_count = len(frames)

            if ft.LASTTIME_ROS2_FORCE:
                print(f"初始化 {self.force_target_n:.1f}N 恒力控制（请确认末端悬空无接触）...")
                self.init_force_controller()

            if start_stage == "point_actions" and point_actions:
                first_index = min(start_point_index, len(frames) - 1)
            elif start_stage == "shun_jin" and run_shun:
                first_index = min(start_point_index, len(frames) - 1)
            else:
                first_index = 0

            first_frame = frames[first_index]
            self.update_preview_status("移动到起始位置", first_frame.get("index", 0))
            first_pose = self._pose_from_frame_offset(first_frame, -self.hover_height_mm)
            if resume_from_local_hover:
                if not self._move_cart_checked(
                    first_pose,
                    "确认暂停悬空位",
                    getattr(ft, "TRANSIT_MOVE_VEL_SLOW", ft.MOVE_VEL_SLOW),
                    required=False,
                ):
                    print("确认暂停悬空位失败，停止继续，避免安全高度抬升转场")
                    return False
            else:
                if not self._move_to_work_pose(first_pose, "移动到起始位置", ft.TRANSIT_MOVE_VEL_FAST):
                    return False

            if point_actions and start_stage == "point_actions" and start_point_index < len(frames):
                print("\n执行点位动作...")
                for i in range(start_point_index, len(frames)):
                    frame = frames[i]
                    point_no = int(frame.get("index", i)) + 1
                    self._set_agent_control_context("point_actions", "move_to_hover", i)
                    self._control_checkpoint(
                        control,
                        stage="point_actions",
                        current_action="move_to_hover",
                        current_point_index=i,
                        next_stage="point_actions",
                        next_point_index=i,
                        message=f"准备处理点{point_no}",
                    )
                    self.update_preview_status("到达悬空位", i)
                    print(f"\n处理点 {point_no}/{len(frames)}...")
                    hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
                    if resume_from_local_hover and i == start_point_index:
                        hover_ok = self._move_cart_checked(
                            hover_pose,
                            "确认当前点悬空位",
                            getattr(ft, "TRANSIT_MOVE_VEL_SLOW", ft.MOVE_VEL_SLOW),
                            required=False,
                        )
                    else:
                        hover_ok = self._move_to_hover_with_fallback(hover_pose, "移动到悬空位")
                    if not hover_ok:
                        point_failures.append((point_no, "悬空位"))
                        print(f"    警告：点{point_no}悬空位不可达，跳过该点")
                        if not ft.FT_CONTINUE_ON_POINT_ERROR:
                            return False
                        continue

                    shun_candidate_frames.append(frame)
                    skip_remaining_point_actions = False
                    resume_action = start_action if i == start_point_index else None
                    resume_repeat_index = start_repeat_index if i == start_point_index else 0
                    resume_step_index = start_step_index if i == start_point_index else 0
                    skip_dian_jin = resume_action in {
                        "fen_jin",
                        "point_complete",
                        "move_to_next",
                        "move_to_hover_next",
                        "move_to_shun_start",
                        "shun_jin",
                        "completed",
                    }
                    skip_fen_jin = resume_action in {
                        "point_complete",
                        "move_to_next",
                        "move_to_hover_next",
                        "move_to_shun_start",
                        "shun_jin",
                        "completed",
                    }

                    if "dian_jin" in point_actions and not skip_dian_jin:
                        print(f"  {dian_action_text}...")
                        dian_start_repeat = resume_repeat_index if resume_action == "dian_jin" else 0
                        self._set_agent_control_context("point_actions", "dian_jin", i, dian_start_repeat, 0)
                        self._control_checkpoint(
                            control,
                            stage="point_actions",
                            current_action="dian_jin",
                            current_point_index=i,
                            next_stage="point_actions",
                            next_point_index=i,
                            message=f"准备执行点{point_no}{dian_action_text}",
                            extra={
                                "current_repeat_index": dian_start_repeat,
                                "current_step_index": 0,
                                "next_repeat_index": dian_start_repeat,
                                "next_step_index": 0,
                            },
                        )
                        if not self.execute_dian_jin(
                            frame,
                            start_repeat_index=dian_start_repeat,
                        ):
                            point_failures.append((point_no, dian_action_text))
                            print(f"    警告：点{point_no}{dian_action_text}失败")
                            if not ft.FT_CONTINUE_ON_POINT_ERROR:
                                return False
                            skip_remaining_point_actions = True

                    if "fen_jin" in point_actions and not skip_remaining_point_actions and not skip_fen_jin:
                        print("  分筋...")
                        fen_start_repeat = resume_repeat_index if resume_action == "fen_jin" else 0
                        fen_start_step = resume_step_index if resume_action == "fen_jin" else 0
                        self._set_agent_control_context(
                            "point_actions",
                            "fen_jin",
                            i,
                            fen_start_repeat,
                            fen_start_step,
                        )
                        self._control_checkpoint(
                            control,
                            stage="point_actions",
                            current_action="fen_jin",
                            current_point_index=i,
                            next_stage="point_actions",
                            next_point_index=i,
                            message=f"准备执行点{point_no}分筋",
                            extra={
                                "current_repeat_index": fen_start_repeat,
                                "current_step_index": fen_start_step,
                                "next_repeat_index": fen_start_repeat,
                                "next_step_index": fen_start_step,
                            },
                        )
                        if not self.execute_fen_jin(
                            frame,
                            start_repeat_index=fen_start_repeat,
                            start_step_index=fen_start_step,
                        ):
                            point_failures.append((point_no, "分筋"))
                            print(f"    警告：点{point_no}分筋失败")
                            if not ft.FT_CONTINUE_ON_POINT_ERROR:
                                return False

                    next_stage = "point_actions"
                    next_point_index = i + 1
                    if next_point_index >= len(frames):
                        next_stage = "shun_jin" if run_shun else "completed"
                        next_point_index = 0
                    self._set_agent_control_context("point_actions", "point_complete", i)
                    self._control_checkpoint(
                        control,
                        stage="point_actions",
                        current_action="point_complete",
                        current_point_index=i,
                        next_stage=next_stage,
                        next_point_index=next_point_index,
                        message=f"点{point_no}点位动作完成",
                    )
            else:
                shun_candidate_frames = list(frames)

            if point_failures:
                summary = ", ".join(f"点{point}:{stage}" for point, stage in point_failures)
                print(f"\n[容错] 点位动作阶段跳过: {summary}")

            if point_actions and start_stage == "point_actions" and start_point_index > 0 and run_shun:
                shun_candidate_frames = list(frames)

            if run_shun:
                shun_frames = shun_candidate_frames or list(frames)
                if not shun_candidate_frames:
                    print("[容错] 没有已确认可达悬空点，顺筋将尝试原始轨迹")

                print("\n回到顺筋起点...")
                shun_first_frame = shun_frames[0]
                shun_start_index = start_point_index if start_stage == "shun_jin" else 0
                shun_start_index = min(max(0, shun_start_index), len(shun_frames) - 1)
                shun_first_frame = shun_frames[shun_start_index]
                shun_first_pose = self._pose_from_frame_offset(shun_first_frame, -self.hover_height_mm)
                self.update_preview_status("回到顺筋起点", shun_first_frame.get("index", 0))
                self._set_agent_control_context("shun_jin", "move_to_shun_start", shun_start_index)
                self._control_checkpoint(
                    control,
                    stage="shun_jin",
                    current_action="move_to_shun_start",
                    current_point_index=shun_start_index,
                    next_stage="shun_jin",
                    next_point_index=shun_start_index,
                    message="准备回到顺筋起点",
                )
                with ft._RobotMotionSpeedScaleOverride(ft.SHUN_JIN_MOTION_SPEED_SCALE):
                    if not self._move_to_work_pose(shun_first_pose, "回到顺筋起点", ft.MOVE_VEL_FAST):
                        print("    警告：回到顺筋起点失败，仍将尝试顺筋")
                        if not ft.FT_CONTINUE_ON_POINT_ERROR:
                            return False

                print("\n执行顺筋动作...")
                shun_ok = self.execute_shun_jin(
                    shun_frames,
                    start_index=shun_start_index,
                    control=control,
                )
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
                "status": "completed" if success else "failed",
            }
            print("\n按摩动作接口执行完成" if success else "\n按摩动作接口执行完成（存在失败）")
            return bool(success)
        except MassageExecutionInterrupted as exc:
            self.last_action_report = {
                "actions": actions,
                "point_count": len(frames),
                "point_failures": [
                    {"point": int(point), "stage": str(stage)}
                    for point, stage in point_failures
                ],
                "shun_ok": bool(shun_ok) if run_shun else None,
                "success": False,
                "status": exc.status,
                "checkpoint": exc.checkpoint,
            }
            print(f"\n按摩动作已{('暂停' if exc.status == 'paused' else '停止')}")
            return False
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
                "status": "error",
                "error": str(exc),
            }
            print(f"\n错误：{exc}")
            return False
        finally:
            self._agent_control = previous_agent_control
            self._agent_control_stage = previous_agent_stage
            self._agent_control_action = previous_agent_action
            self._agent_control_point_index = previous_agent_point_index
            self._agent_control_repeat_index = previous_agent_repeat_index
            self._agent_control_step_index = previous_agent_step_index
            self._agent_pause_pending = previous_agent_pause_pending
            self._agent_stop_pending = previous_agent_stop_pending
            self._agent_last_checkpoint = previous_agent_last_checkpoint
            self._agent_point_actions = previous_agent_point_actions
            self._agent_run_shun = previous_agent_run_shun
            self._agent_point_count = previous_agent_point_count
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
        if robot is not None:
            try:
                if hasattr(robot, "close"):
                    robot.close()
                elif hasattr(robot, "CloseRPC"):
                    robot.CloseRPC()
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

    def execute_actions(
        self,
        actions=None,
        raise_on_error=False,
        control=None,
        start_stage=None,
        start_point_index=0,
        start_action=None,
        start_repeat_index=0,
        start_step_index=0,
    ):
        try:
            normalized_actions = normalize_massage_actions(actions)
            if not self.demo.massage_frames:
                raise RuntimeError("没有已加载/已检测的轨迹，不能执行按摩动作")
            if self.demo.robot is None:
                self.demo.init_robot()
            self.robot_initialized = True
            ok = self.demo.execute_massage_actions(
                normalized_actions,
                control=control,
                start_stage=start_stage,
                start_point_index=start_point_index,
                start_action=start_action,
                start_repeat_index=start_repeat_index,
                start_step_index=start_step_index,
            )
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
