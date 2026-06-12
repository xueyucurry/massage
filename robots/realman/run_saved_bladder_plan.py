#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
from pathlib import Path

from rm_demo.rm_bladder import (
    BladderMassageFrame,
    BladderMassagePlan,
    build_aligned_contact_preview,
    execute_bladder_constant_force_plan,
    execute_bladder_hover_path,
    execute_bladder_plan,
    execute_bladder_tool_axis_probe_remote,
    execute_bladder_touch_probe_plan,
    preview_bladder_plan,
)


DEFAULT_PLAN = "rm_demo_output/bladder_demo_20260421_220540_plan.json"
PROJECT_DIR = Path(__file__).resolve().parent


def _docker_command(argv: list[str]) -> list[str]:
    try:
        subprocess.run(["docker", "version"], check=True, capture_output=True, timeout=3)
        return ["docker", *argv]
    except Exception:
        return ["sg", "docker", "-c", " ".join(shlex.quote(part) for part in ["docker", *argv])]


def _can_import_rospy() -> bool:
    try:
        import rospy  # type: ignore  # noqa: F401

        return True
    except Exception:
        return False


def _delegate_ros_run_to_docker(args: argparse.Namespace, plan_path: Path) -> int | None:
    if os.environ.get("RUN_SAVED_PLAN_IN_DOCKER") == "1":
        return None
    needs_ros = (
        args.execution_mode in ("touch_probe", "constant_force")
        or (args.control_backend == "ros" and args.execution_mode != "tool_axis_probe")
    )
    if not args.run or not needs_ros or _can_import_rospy():
        return None

    subprocess.run(_docker_command(["start", "noetic"]), check=True)
    inner_args = [
        "python3",
        str(PROJECT_DIR / "run_saved_bladder_plan.py"),
        "--plan-json",
        str(plan_path),
        "--arm-host",
        str(args.arm_host),
        "--control-backend",
        str(args.control_backend),
        "--execution-mode",
        str(args.execution_mode),
        "--speed",
        str(args.speed),
        "--dwell-s",
        str(args.dwell_s),
        "--max-step-m",
        str(args.max_step_m),
        "--frame-start",
        str(args.frame_start),
        "--frame-count",
        str(args.frame_count),
        "--target-force-n",
        str(args.target_force_n),
        "--max-force-n",
        str(args.max_force_n),
        "--touch-step-mm",
        str(args.touch_step_mm),
        "--max-press-mm",
        str(args.max_press_mm),
        "--force-direction",
        str(args.force_direction),
        "--tool-contact-axis",
        str(args.tool_contact_axis),
        "--contact-motion-axis",
        str(args.contact_motion_axis),
        "--probe-depth-mm",
        str(args.probe_depth_mm),
        "--touch-entry-motion",
        str(args.touch_entry_motion),
        "--hover-entry-motion",
        str(args.hover_entry_motion),
        "--run",
    ]
    if args.use_plan_orientation:
        inner_args.append("--use-plan-orientation")
    elif args.keep_current_orientation:
        inner_args.append("--keep-current-orientation")
    if args.tool_name:
        inner_args.extend(["--tool-name", str(args.tool_name)])
    if args.work_frame:
        inner_args.extend(["--work-frame", str(args.work_frame)])
    if args.use_global_safe_z:
        inner_args.append("--use-global-safe-z")
    if args.allow_experimental_contact:
        inner_args.append("--allow-experimental-contact")
    inner_cmd = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        "export RUN_SAVED_PLAN_IN_DOCKER=1; "
        f"export PYTHONPATH={shlex.quote(str(PROJECT_DIR / 'ros_vendor' / 'python'))}:$PYTHONPATH; "
        f"cd {shlex.quote(str(PROJECT_DIR))}; "
        + " ".join(shlex.quote(part) for part in inner_args)
    )
    docker_args = [
        "exec",
        "-i",
        "-e",
        "ROS_MASTER_URI=http://192.168.1.11:11311",
        "-e",
        "ROS_IP=192.168.1.250",
        "-e",
        "ROS_HOSTNAME=192.168.1.250",
        "noetic",
        "bash",
        "-lc",
        inner_cmd,
    ]
    print("local Python has no rospy; delegating ROS execution to Docker container noetic")
    return subprocess.run(_docker_command(docker_args)).returncode


def _run_ros_frame_setup_in_docker(args: argparse.Namespace) -> None:
    if os.environ.get("RUN_SAVED_PLAN_IN_DOCKER") == "1":
        return
    if args.control_backend == "ros" or (not args.work_frame and not args.tool_name):
        return
    if _can_import_rospy():
        return

    subprocess.run(_docker_command(["start", "noetic"]), check=True)
    setup_lines = [
        "from rm_demo.rm_ros import create_arm_backend",
        "arm = create_arm_backend('ros')",
    ]
    if args.work_frame:
        setup_lines.append(f"print('switching work frame to {args.work_frame}')")
        setup_lines.append(f"print(arm.change_work_frame({args.work_frame!r}))")
    if args.tool_name:
        setup_lines.append(f"print('switching tool frame to {args.tool_name}')")
        setup_lines.append(f"print(arm.change_tool({args.tool_name!r}))")
    inner_cmd = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        f"export PYTHONPATH={shlex.quote(str(PROJECT_DIR / 'ros_vendor' / 'python'))}:$PYTHONPATH; "
        f"cd {shlex.quote(str(PROJECT_DIR))}; "
        "python3 - <<'PY'\n"
        + "\n".join(setup_lines)
        + "\nPY"
    )
    docker_args = [
        "exec",
        "-i",
        "-e",
        "ROS_MASTER_URI=http://192.168.1.11:11311",
        "-e",
        "ROS_HOSTNAME=192.168.1.250",
        "noetic",
        "bash",
        "-lc",
        inner_cmd,
    ]
    subprocess.run(_docker_command(docker_args), check=True)


def load_plan(path: str) -> BladderMassagePlan:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    frame_fields = set(BladderMassageFrame.__dataclass_fields__.keys())
    frames = [
        BladderMassageFrame(**{key: value for key, value in frame.items() if key in frame_fields})
        for frame in data["frames"]
    ]
    return BladderMassagePlan(
        side=str(data["side"]),
        line_type=str(data["line_type"]),
        point_count=int(data["point_count"]),
        hover_m=float(data["hover_m"]),
        dian_jin_depth_m=float(data["dian_jin_depth_m"]),
        fen_jin_lateral_m=float(data["fen_jin_lateral_m"]),
        safe_z_m=float(data["safe_z_m"]),
        anchor_pose_m=[float(v) for v in data["anchor_pose_m"]],
        frames=frames,
        hover_offset_mode=str(data.get("hover_offset_mode", "normal")),
    )


def select_plan_frames(plan: BladderMassagePlan, frame_start: int, frame_count: int) -> BladderMassagePlan:
    if frame_start <= 0 and frame_count <= 0:
        return plan
    start = max(1, int(frame_start)) if frame_start > 0 else 1
    start_idx = start - 1
    if start_idx >= len(plan.frames):
        raise RuntimeError(f"frame_start {start} is outside plan frame count {len(plan.frames)}")
    if frame_count > 0:
        frames = plan.frames[start_idx : start_idx + int(frame_count)]
    else:
        frames = plan.frames[start_idx:]
    if not frames:
        raise RuntimeError("selected plan frame range is empty")
    return BladderMassagePlan(
        side=plan.side,
        line_type=plan.line_type,
        point_count=len(frames),
        hover_m=plan.hover_m,
        dian_jin_depth_m=plan.dian_jin_depth_m,
        fen_jin_lateral_m=plan.fen_jin_lateral_m,
        safe_z_m=plan.safe_z_m,
        anchor_pose_m=plan.anchor_pose_m,
        frames=frames,
        hover_offset_mode=plan.hover_offset_mode,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Preview or execute a saved bladder meridian plan.")
    parser.add_argument("--plan-json", default=DEFAULT_PLAN, help="saved *_plan.json to load")
    parser.add_argument("--arm-host", default="192.168.1.18", help="unused for ROS backend; kept for compatibility")
    parser.add_argument("--control-backend", choices=("ros", "json"), default="ros")
    parser.add_argument(
        "--execution-mode",
        choices=("hover_path", "massage", "touch_probe", "constant_force", "tool_axis_probe"),
        default="hover_path",
    )
    parser.add_argument("--speed", type=float, default=0.3)
    parser.add_argument("--dwell-s", type=float, default=0.0, help="dwell per hover point for hover_path")
    parser.add_argument(
        "--max-step-m",
        type=float,
        default=0.03,
        help="maximum Cartesian step for direct hover_path interpolation",
    )
    parser.add_argument(
        "--keep-current-orientation",
        action="store_true",
        default=True,
        help="for hover_path, keep the current end-effector RPY and only follow saved XYZ points",
    )
    parser.add_argument(
        "--use-plan-orientation",
        action="store_true",
        help="for hover_path, use saved plan RPY so the tool follows the product surface normal",
    )
    parser.add_argument(
        "--tool-name",
        default="",
        help="optional tool frame to switch to before executing, e.g. mas_rub or mas_palm",
    )
    parser.add_argument(
        "--work-frame",
        default="",
        help="optional work frame to switch to before executing, e.g. Base",
    )
    parser.add_argument(
        "--use-global-safe-z",
        action="store_true",
        help="lift via plan.safe_z_m between points; off by default for saved product_ros hover plans",
    )
    parser.add_argument("--frame-start", type=int, default=0, help="1-based first saved plan frame to execute")
    parser.add_argument("--frame-count", type=int, default=0, help="number of saved plan frames to execute")
    parser.add_argument("--target-force-n", type=float, default=1.0)
    parser.add_argument("--max-force-n", type=float, default=4.0)
    parser.add_argument("--touch-step-mm", type=float, default=1.0)
    parser.add_argument("--max-press-mm", type=float, default=5.0)
    parser.add_argument(
        "--force-direction",
        type=int,
        default=2,
        help="RM force axis enum; 2 is tool Z for the current side-lying setup",
    )
    parser.add_argument(
        "--tool-contact-axis",
        default="pos_z",
        help="tool-frame axis that should align with the back normal for touch probing",
    )
    parser.add_argument(
        "--contact-motion-axis",
        default="",
        help="tool-frame axis used for contact approach; empty means same as --tool-contact-axis",
    )
    parser.add_argument(
        "--probe-depth-mm",
        type=float,
        default=0.0,
        help="override total probing distance from hover; <=0 uses hover distance plus max press",
    )
    parser.add_argument(
        "--allow-experimental-contact",
        action="store_true",
        help="allow contact execution modes that still need hardware validation",
    )
    parser.add_argument("--touch-entry-motion", choices=("movel", "movej_p"), default="movej_p")
    parser.add_argument("--hover-entry-motion", choices=("movel", "movej_p"), default="movej_p")
    parser.add_argument(
        "--force-container",
        default="noetic",
        help="Docker container used only for force samples in tool_axis_probe",
    )
    parser.add_argument("--remote-sdk-ssh", default="rm@192.168.1.11")
    parser.add_argument(
        "--remote-sdk-dir",
        default="/home/rm/rm_healthcare_robot/collection/data_collection_d435_arm",
    )
    parser.add_argument("--sdk-code", type=int, default=65)
    parser.add_argument(
        "--entry-tolerance-m",
        type=float,
        default=0.015,
        help="tool_axis_probe requires the TCP to already be this close to the saved hover point",
    )
    parser.add_argument("--run", action="store_true", help="actually move the robot; otherwise preview only")
    args = parser.parse_args()
    if args.use_plan_orientation:
        args.keep_current_orientation = False

    plan_path = Path(args.plan_json).resolve()
    delegated_rc = _delegate_ros_run_to_docker(args, plan_path)
    if delegated_rc is not None:
        return delegated_rc

    plan = select_plan_frames(load_plan(str(plan_path)), args.frame_start, args.frame_count)
    print(f"plan_json={plan_path}")
    aligned_contact_preview = build_aligned_contact_preview(
        plan,
        tool_contact_axis=args.tool_contact_axis,
        contact_motion_axis=args.contact_motion_axis or None,
        max_press_m=args.max_press_mm / 1000.0,
        touch_step_m=args.touch_step_mm / 1000.0,
        probe_depth_m=args.probe_depth_mm / 1000.0 if args.probe_depth_mm > 0.0 else None,
    )
    print(
        "aligned_contact_preview "
        f"tool_axis={aligned_contact_preview['tool_contact_axis']} "
        f"motion_axis={aligned_contact_preview['contact_motion_axis']} "
        f"min_dot={float(aligned_contact_preview['min_tool_axis_dot_press']):.4f} "
        f"hover_m={float(aligned_contact_preview['hover_m']):.4f} "
        f"max_press_m={float(aligned_contact_preview['max_press_m']):.4f} "
        f"probe_depth_m={aligned_contact_preview['probe_depth_m']}"
    )
    preview_bladder_plan(plan)
    if not args.run:
        print("preview only; add --run to execute this saved plan")
        return 0

    if args.execution_mode == "hover_path":
        _run_ros_frame_setup_in_docker(args)
        if (args.tool_name or args.work_frame) and _can_import_rospy():
            from rm_demo.rm_ros import create_arm_backend

            arm = create_arm_backend("ros")
            if args.work_frame:
                print(f"switching work frame to {args.work_frame}")
                arm.change_work_frame(args.work_frame)
            if args.tool_name:
                print(f"switching tool frame to {args.tool_name}")
                arm.change_tool(args.tool_name)
        execute_bladder_hover_path(
            host=args.arm_host,
            plan=plan,
            speed=args.speed,
            control_backend=args.control_backend,
            dwell_s=args.dwell_s,
            use_global_safe_z=args.use_global_safe_z,
            keep_current_orientation=args.keep_current_orientation,
            max_step_m=args.max_step_m,
            entry_motion=args.hover_entry_motion,
        )
    elif args.execution_mode == "massage":
        execute_bladder_plan(
            host=args.arm_host,
            plan=plan,
            speed=args.speed,
            control_backend=args.control_backend,
        )
    elif args.execution_mode == "touch_probe":
        if not args.allow_experimental_contact:
            raise RuntimeError("touch_probe is disabled until the side-lying contact backend is validated")
        _run_ros_frame_setup_in_docker(args)
        if (args.tool_name or args.work_frame) and _can_import_rospy():
            from rm_demo.rm_ros import create_arm_backend

            arm = create_arm_backend("ros")
            if args.work_frame:
                print(f"switching work frame to {args.work_frame}")
                arm.change_work_frame(args.work_frame)
            if args.tool_name:
                print(f"switching tool frame to {args.tool_name}")
                arm.change_tool(args.tool_name)
        execute_bladder_touch_probe_plan(
            host=args.arm_host,
            plan=plan,
            speed=args.speed,
            control_backend=args.control_backend,
            target_force_n=args.target_force_n,
            max_force_n=args.max_force_n,
            touch_step_m=args.touch_step_mm / 1000.0,
            max_press_m=args.max_press_mm / 1000.0,
            dwell_s=args.dwell_s,
            max_step_m=args.max_step_m,
            keep_current_orientation=False,
            entry_motion=args.touch_entry_motion,
            tool_contact_axis=args.tool_contact_axis,
            contact_motion_axis=args.contact_motion_axis or None,
            probe_depth_m=args.probe_depth_mm / 1000.0 if args.probe_depth_mm > 0.0 else None,
        )
    elif args.execution_mode == "tool_axis_probe":
        if not args.allow_experimental_contact:
            raise RuntimeError("tool_axis_probe is disabled until the side-lying contact backend is validated")
        execute_bladder_tool_axis_probe_remote(
            host=args.arm_host,
            plan=plan,
            speed=max(1, int(round(float(args.speed)))),
            target_force_n=args.target_force_n,
            max_force_n=args.max_force_n,
            touch_step_m=args.touch_step_mm / 1000.0,
            probe_depth_m=args.probe_depth_mm / 1000.0 if args.probe_depth_mm > 0.0 else args.max_press_mm / 1000.0,
            dwell_s=args.dwell_s,
            tool_contact_axis=args.tool_contact_axis,
            contact_motion_axis=args.contact_motion_axis or "neg_z",
            entry_tolerance_m=args.entry_tolerance_m,
            force_container=args.force_container,
            remote_ssh=args.remote_sdk_ssh,
            remote_sdk_dir=args.remote_sdk_dir,
            sdk_code=args.sdk_code,
        )
    elif args.execution_mode == "constant_force":
        if not args.allow_experimental_contact:
            raise RuntimeError("constant_force is disabled until the side-lying contact backend is validated")
        if args.control_backend != "ros":
            raise RuntimeError("constant_force saved-plan execution now requires --control-backend ros")
        _run_ros_frame_setup_in_docker(args)
        if (args.tool_name or args.work_frame) and _can_import_rospy():
            from rm_demo.rm_ros import create_arm_backend

            arm = create_arm_backend("ros")
            if args.work_frame:
                print(f"switching work frame to {args.work_frame}")
                arm.change_work_frame(args.work_frame)
            if args.tool_name:
                print(f"switching tool frame to {args.tool_name}")
                arm.change_tool(args.tool_name)
        execute_bladder_constant_force_plan(
            host=args.arm_host,
            plan=plan,
            speed=args.speed,
            target_force_n=args.target_force_n,
            max_force_n=args.max_force_n,
            force_direction=args.force_direction,
            touch_step_m=args.touch_step_mm / 1000.0,
            max_press_m=args.max_press_mm / 1000.0,
            dwell_s=args.dwell_s,
            point_limit=plan.point_count,
            entry_motion=args.touch_entry_motion,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
