from __future__ import annotations

import datetime
import glob
import json
import os
import re
import sys
from typing import Any

from .config import DEFAULT_CAPTURE_PREPARE_SECTION, ROS_VENDOR_PYTHON_DIR
from .rm_ros import create_arm_backend
from .rm_speed import normalize_motion_speed


def _import_ros_modules():
    try:
        import rospy  # type: ignore
        from rm_healthcare_robot_msgs.srv import MoveCameraAbovePerson  # type: ignore
    except Exception:
        candidates = []
        candidates.extend(glob.glob("/opt/ros/*/lib/python3/dist-packages"))
        candidates.append(ROS_VENDOR_PYTHON_DIR)
        candidates.append("/home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/lib/python3/dist-packages")
        for candidate in candidates:
            if candidate and os.path.isdir(candidate) and candidate not in sys.path:
                sys.path.append(candidate)
        import rospy  # type: ignore
        from rm_healthcare_robot_msgs.srv import MoveCameraAbovePerson  # type: ignore

    return rospy, MoveCameraAbovePerson


def load_prepare_joints(config_path: str, section_name: str = DEFAULT_CAPTURE_PREPARE_SECTION) -> list[float] | None:
    if not config_path or not os.path.isfile(config_path):
        return None
    section_name = (section_name or DEFAULT_CAPTURE_PREPARE_SECTION).strip()
    if not section_name:
        section_name = DEFAULT_CAPTURE_PREPARE_SECTION
    with open(config_path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()

    in_section = False
    joints: dict[int, float] = {}
    section_pattern = re.compile(rf"^\s*{re.escape(section_name)}\s*:\s*$")
    for raw_line in lines:
        line = raw_line.rstrip()
        if not in_section:
            if section_pattern.match(line):
                in_section = True
            continue
        if line and not line.startswith((" ", "\t")):
            break
        match = re.match(r"^\s*joint(\d+)\s*:\s*([-+]?\d+(?:\.\d+)?)\s*$", line)
        if match:
            joints[int(match.group(1))] = float(match.group(2))

    if len(joints) < 6:
        return None
    return [float(joints[idx]) for idx in range(6)]


def _normalize_prepare_joints(prepare_joints: list[float] | tuple[float, ...] | None) -> list[float] | None:
    if prepare_joints is None:
        return None
    if len(prepare_joints) < 6:
        raise RuntimeError(f"capture prepare joints must contain 6 values, got {len(prepare_joints)}")
    return [float(v) for v in list(prepare_joints)[:6]]


def _format_joint_value(value: float) -> str:
    text = f"{float(value):.6f}".rstrip("0").rstrip(".")
    return text if text and text != "-0" else "0"


def _render_prepare_section(section_name: str, joints: list[float]) -> list[str]:
    return [f"{section_name}:"] + [f"  joint{i}: {_format_joint_value(joints[i])}" for i in range(6)]


def save_prepare_joints(
    config_path: str,
    section_name: str,
    joints: list[float] | tuple[float, ...],
    *,
    backup: bool = True,
) -> dict[str, Any]:
    normalized = _normalize_prepare_joints(joints)
    if normalized is None:
        raise RuntimeError("no joints supplied")
    section_name = (section_name or DEFAULT_CAPTURE_PREPARE_SECTION).strip() or DEFAULT_CAPTURE_PREPARE_SECTION
    config_path = os.path.abspath(config_path)
    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    old_text = ""
    if os.path.isfile(config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            old_text = f.read()
    old_lines = old_text.splitlines()
    new_block = _render_prepare_section(section_name, normalized)

    section_pattern = re.compile(rf"^\s*{re.escape(section_name)}\s*:\s*$")
    start_idx: int | None = None
    for idx, line in enumerate(old_lines):
        if section_pattern.match(line.rstrip()):
            start_idx = idx
            break

    replaced = False
    if start_idx is None:
        new_lines = list(old_lines)
        if new_lines and new_lines[-1].strip():
            new_lines.append("")
        new_lines.extend(new_block)
    else:
        end_idx = start_idx + 1
        while end_idx < len(old_lines):
            line = old_lines[end_idx]
            if line and not line.startswith((" ", "\t")):
                break
            end_idx += 1
        new_lines = old_lines[:start_idx] + new_block + old_lines[end_idx:]
        replaced = True

    if backup and os.path.isfile(config_path):
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = f"{config_path}.bak_{stamp}"
        with open(backup_path, "w", encoding="utf-8") as f:
            f.write(old_text)
            if old_text and not old_text.endswith("\n"):
                f.write("\n")
    else:
        backup_path = ""

    with open(config_path, "w", encoding="utf-8") as f:
        f.write("\n".join(new_lines))
        f.write("\n")

    return {
        "config_path": config_path,
        "section_name": section_name,
        "replaced": replaced,
        "backup_path": backup_path,
        "joints_deg": [round(float(v), 6) for v in normalized],
    }


def save_current_arm_as_prepare_pose(
    host: str,
    config_path: str,
    section_name: str = DEFAULT_CAPTURE_PREPARE_SECTION,
    *,
    control_backend: str = "ros",
    timeout: float = 3.0,
    metadata_path: str | None = None,
) -> dict[str, Any]:
    arm = create_arm_backend(control_backend)
    try:
        joints, pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(host=host, timeout=float(timeout))
    except TypeError:
        joints, pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(host)
    saved = save_prepare_joints(config_path, section_name, joints)
    result = {
        "step": "save_current_capture_pose",
        "control_backend": str(control_backend),
        "host": str(host),
        "saved_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "arm_err": int(arm_err),
        "sys_err": int(sys_err),
        "inverse_km_err": int(inverse_km_err),
        "pose_m_rad": [round(float(v), 9) for v in pose[:6]],
        **saved,
    }
    if metadata_path:
        metadata_path = os.path.abspath(metadata_path)
        os.makedirs(os.path.dirname(metadata_path), exist_ok=True)
        with open(metadata_path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
        result["metadata_path"] = metadata_path
    return result


def move_to_prepare_pose(
    host: str,
    config_path: str,
    speed: int,
    control_backend: str = "json",
    prepare_section: str = DEFAULT_CAPTURE_PREPARE_SECTION,
    prepare_joints: list[float] | tuple[float, ...] | None = None,
) -> dict[str, Any]:
    joints = _normalize_prepare_joints(prepare_joints)
    joint_source = "cli"
    if joints is None:
        joints = load_prepare_joints(config_path, section_name=prepare_section)
        joint_source = "yaml"
    if joints is None:
        raise RuntimeError(f"{prepare_section} not found in trajectory config: {config_path}")
    arm = create_arm_backend(control_backend)
    arm.recover_if_needed(host)
    arm.movej(
        host,
        joints,
        speed=normalize_motion_speed(control_backend, float(speed), ros_default=0.2),
        timeout=40.0,
    )
    return {
        "step": "prepare",
        "control_backend": str(control_backend),
        "config_path": os.path.abspath(config_path),
        "prepare_section": str(prepare_section),
        "joint_source": joint_source,
        "joints_deg": [round(float(v), 3) for v in joints],
    }


def call_move_camera_above_person(
    tool_name_camera: str,
    tool_name_restore: str,
    shifting_number: int,
    timeout: float = 20.0,
) -> dict[str, Any]:
    rospy, MoveCameraAbovePerson = _import_ros_modules()
    if not rospy.core.is_initialized():
        rospy.init_node("rm_demo_positioning", anonymous=True, disable_signals=True)
    rospy.wait_for_service("/ai_service/move_camera_above_person", timeout=timeout)
    proxy = rospy.ServiceProxy("/ai_service/move_camera_above_person", MoveCameraAbovePerson)
    response = proxy(tool_name_camera, tool_name_restore, int(shifting_number))
    return {
        "step": "service",
        "tool_name_camera": str(tool_name_camera),
        "tool_name_restore": str(tool_name_restore),
        "shifting_number": int(shifting_number),
        "state": bool(response.state),
        "error_code": int(response.error_code),
        "msg": str(response.msg),
    }


def position_for_capture(
    host: str,
    mode: str,
    trajectory_config: str,
    speed: int,
    tool_name_camera: str,
    tool_name_restore: str,
    shifting_number: int,
    control_backend: str = "json",
    prepare_section: str = DEFAULT_CAPTURE_PREPARE_SECTION,
    prepare_joints: list[float] | tuple[float, ...] | None = None,
) -> list[dict[str, Any]]:
    mode = mode.strip().lower()
    if mode == "none":
        return []
    if mode not in ("prepare", "service", "prepare_then_service"):
        raise ValueError(f"unsupported capture positioning mode: {mode}")

    events: list[dict[str, Any]] = []
    if mode in ("prepare", "prepare_then_service"):
        events.append(
            move_to_prepare_pose(
                host=host,
                config_path=trajectory_config,
                speed=speed,
                control_backend=control_backend,
                prepare_section=prepare_section,
                prepare_joints=prepare_joints,
            )
        )
    if mode in ("service", "prepare_then_service"):
        result = call_move_camera_above_person(
            tool_name_camera=tool_name_camera,
            tool_name_restore=tool_name_restore,
            shifting_number=shifting_number,
        )
        events.append(result)
        if not bool(result["state"]) and mode == "service":
            raise RuntimeError(
                "move_camera_above_person failed: "
                f"error_code={result['error_code']} msg={result['msg']}"
            )
    return events
