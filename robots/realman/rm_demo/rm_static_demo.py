#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import os
import shlex
import subprocess
import sys
import time
from typing import Any

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from rm_demo.config import (
        DEFAULT_CAMERA_TOOL_NAME,
        DEFAULT_CAPTURE_PREPARE_SECTION,
        DEFAULT_CAPTURE_POSITIONING,
        DEFAULT_CAPTURE_SETTLE_S,
        DEFAULT_CONTROL_BACKEND,
        DEFAULT_DETECTOR_BACKEND,
        DEFAULT_DWELL_S,
        DEFAULT_FINGER_WIDTH_MM,
        DEFAULT_HOST,
        DEFAULT_HOVER_MM,
        DEFAULT_INSTALL_ANG,
        DEFAULT_MAX_FORCE_N,
        DEFAULT_MAX_PRESS_MM,
        DEFAULT_MATRIX_PATH,
        DEFAULT_MODEL_PATH,
        DEFAULT_OUTPUT_DIR,
        DEFAULT_PLAN_POINTS,
        DEFAULT_POSITION_SPEED,
        DEFAULT_REMOTE_DIR,
        DEFAULT_REMOTE_SSH,
        DEFAULT_RESTORE_TOOL_NAME,
        DEFAULT_SAFE_LIFT_MM,
        DEFAULT_SAMPLE_POINTS,
        DEFAULT_SIDE,
        DEFAULT_SHIFTING_NUMBER,
        DEFAULT_SPEED,
        DEFAULT_TARGET_FORCE_N,
        DEFAULT_TOUCH_STEP_MM,
        DEFAULT_TRANSFORM_BACKEND,
        DEFAULT_TRAJECTORY_CONFIG,
    )
    from rm_demo.rm_capture import capture_single_frame, has_realsense_device, load_captured_frame
    from rm_demo.rm_detect import detect_static_meridians, save_detection_artifacts, select_side
    from rm_demo.rm_execute import execute_plan, preview_plan
    from rm_demo.rm_plan import build_static_plan, plan_to_dict
    from rm_demo.rm_positioning import position_for_capture
    from rm_demo.rm_product_ros import attach_robot_points_via_product_services
    from rm_demo.rm_ros import create_arm_backend
    from rm_demo.rm_transform import attach_robot_points, load_transform_matrix
else:
    from .config import (
        DEFAULT_CAMERA_TOOL_NAME,
        DEFAULT_CAPTURE_PREPARE_SECTION,
        DEFAULT_CAPTURE_POSITIONING,
        DEFAULT_CAPTURE_SETTLE_S,
        DEFAULT_CONTROL_BACKEND,
        DEFAULT_DETECTOR_BACKEND,
        DEFAULT_DWELL_S,
        DEFAULT_FINGER_WIDTH_MM,
        DEFAULT_HOST,
        DEFAULT_HOVER_MM,
        DEFAULT_INSTALL_ANG,
        DEFAULT_MAX_FORCE_N,
        DEFAULT_MAX_PRESS_MM,
        DEFAULT_MATRIX_PATH,
        DEFAULT_MODEL_PATH,
        DEFAULT_OUTPUT_DIR,
        DEFAULT_PLAN_POINTS,
        DEFAULT_POSITION_SPEED,
        DEFAULT_REMOTE_DIR,
        DEFAULT_REMOTE_SSH,
        DEFAULT_RESTORE_TOOL_NAME,
        DEFAULT_SAFE_LIFT_MM,
        DEFAULT_SAMPLE_POINTS,
        DEFAULT_SIDE,
        DEFAULT_SHIFTING_NUMBER,
        DEFAULT_SPEED,
        DEFAULT_TARGET_FORCE_N,
        DEFAULT_TOUCH_STEP_MM,
        DEFAULT_TRANSFORM_BACKEND,
        DEFAULT_TRAJECTORY_CONFIG,
    )
    from .rm_capture import capture_single_frame, has_realsense_device, load_captured_frame
    from .rm_detect import detect_static_meridians, save_detection_artifacts, select_side
    from .rm_execute import execute_plan, preview_plan
    from .rm_plan import build_static_plan, plan_to_dict
    from .rm_positioning import position_for_capture
    from .rm_product_ros import attach_robot_points_via_product_services
    from .rm_ros import create_arm_backend
    from .rm_transform import attach_robot_points, load_transform_matrix


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Static RealMan back massage demo")
    parser.add_argument("--host", default=DEFAULT_HOST, help="RM controller IP")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="artifact directory")
    parser.add_argument("--model-path", default=DEFAULT_MODEL_PATH, help="pose model path")
    parser.add_argument("--matrix-path", default=DEFAULT_MATRIX_PATH, help="camera->robot 4x4 matrix JSON")
    parser.add_argument(
        "--transform-backend",
        choices=("auto", "product_ros", "static"),
        default=DEFAULT_TRANSFORM_BACKEND,
        help="camera->robot transform backend; product_ros uses RM services, static uses matrix file",
    )
    parser.add_argument(
        "--install-ang",
        nargs=3,
        type=float,
        default=DEFAULT_INSTALL_ANG,
        metavar=("RX", "RY", "RZ"),
        help="robot install angles for RM product calc_poses service, default 0 0 0",
    )
    parser.add_argument(
        "--detector-backend",
        choices=("auto", "pose", "area"),
        default=DEFAULT_DETECTOR_BACKEND,
        help="vision backend: pose, RM area_detection, or auto fallback",
    )
    parser.add_argument("--side", choices=("left", "right"), default=DEFAULT_SIDE, help="meridian side to execute")
    parser.add_argument("--finger-width", type=float, default=DEFAULT_FINGER_WIDTH_MM, help="physical offset in mm")
    parser.add_argument("--sample-points", type=int, default=DEFAULT_SAMPLE_POINTS, help="points sampled along the meridian line")
    parser.add_argument("--plan-points", type=int, default=DEFAULT_PLAN_POINTS, help="massage points executed along one side")
    parser.add_argument("--hover-mm", type=float, default=DEFAULT_HOVER_MM, help="hover height above the body")
    parser.add_argument("--safe-lift-mm", type=float, default=DEFAULT_SAFE_LIFT_MM, help="extra safe lift before lateral moves")
    parser.add_argument("--dwell-s", type=float, default=DEFAULT_DWELL_S, help="stay time at each massage point")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="RM motion speed 1..100")
    parser.add_argument("--target-force-n", type=int, default=DEFAULT_TARGET_FORCE_N, help="target force for logging/config")
    parser.add_argument("--control-backend", choices=("json", "ros"), default=DEFAULT_CONTROL_BACKEND, help="arm control backend")
    parser.add_argument(
        "--mode",
        choices=("preview", "hover", "monitor", "touch_monitor", "ros_force_pose"),
        default="preview",
        help="execution mode",
    )
    parser.add_argument("--transport", choices=("auto", "local", "remote"), default="auto", help="where to run capture + control")
    parser.add_argument(
        "--capture-positioning",
        choices=("none", "prepare", "service", "prepare_then_service"),
        default=DEFAULT_CAPTURE_POSITIONING,
        help="pre-capture positioning using RM official prepare pose and/or move_camera_above_person service",
    )
    parser.add_argument("--trajectory-config", default=DEFAULT_TRAJECTORY_CONFIG, help="trajectory_generate.yaml path")
    parser.add_argument(
        "--capture-prepare-section",
        default=DEFAULT_CAPTURE_PREPARE_SECTION,
        help="trajectory config section used by --capture-positioning prepare",
    )
    parser.add_argument(
        "--capture-joints",
        nargs=6,
        type=float,
        default=None,
        metavar=("J0", "J1", "J2", "J3", "J4", "J5"),
        help="override prepare section with 6 capture-pose joint angles in degrees",
    )
    parser.add_argument("--position-speed", type=int, default=DEFAULT_POSITION_SPEED, help="speed for prepare joint move")
    parser.add_argument("--camera-tool-name", default=DEFAULT_CAMERA_TOOL_NAME, help="tool frame name for camera")
    parser.add_argument(
        "--restore-tool-name",
        default=DEFAULT_RESTORE_TOOL_NAME,
        help="tool frame restored after move_camera_above_person; set this to the product's massage tool",
    )
    parser.add_argument("--shifting-number", type=int, default=DEFAULT_SHIFTING_NUMBER, help="RM move_camera_above_person shifting_number")
    parser.add_argument("--capture-settle-s", type=float, default=DEFAULT_CAPTURE_SETTLE_S, help="settle time after positioning")
    parser.add_argument("--rgb-path", default="", help="use an existing rgb png instead of live capture")
    parser.add_argument("--depth-path", default="", help="use an existing depth npy instead of live capture")
    parser.add_argument("--intrinsics-path", default="", help="intrinsics json for offline input")
    parser.add_argument("--force-direction", type=int, default=2, help="RM ROS force direction enum for ros_force_pose")
    parser.add_argument("--force-mode", type=int, default=0, help="RM ROS force mode enum for ros_force_pose")
    parser.add_argument("--max-force-n", type=int, default=DEFAULT_MAX_FORCE_N, help="safety stop threshold for touch_monitor")
    parser.add_argument("--touch-step-mm", type=float, default=DEFAULT_TOUCH_STEP_MM, help="probe step for touch_monitor")
    parser.add_argument("--max-press-mm", type=float, default=DEFAULT_MAX_PRESS_MM, help="extra press distance limit for touch_monitor")
    parser.add_argument("--run", action="store_true", help="execute robot motion")
    return parser.parse_args()


def _has_local_capture_input(args: argparse.Namespace) -> bool:
    return bool(args.rgb_path and args.depth_path and args.intrinsics_path)


def maybe_exec_remote(args: argparse.Namespace) -> None:
    if args.transport == "local":
        return
    should_remote = args.transport == "remote"
    if args.transport == "auto":
        should_remote = not has_realsense_device()
    if not should_remote:
        return

    remote_cmd = (
        "source /opt/ros/*/setup.bash >/dev/null 2>&1 || true; "
        "source /home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/setup.bash >/dev/null 2>&1 || true; "
        "cd {remote_dir} && RM_DEMO_ROS_ENV_READY=1 python3 rm_static_demo.py --transport local {args}"
    ).format(
        remote_dir=shlex.quote(DEFAULT_REMOTE_DIR),
        args=" ".join(shlex.quote(arg) for arg in sys.argv[1:]),
    ).strip()
    print(f"local capture unavailable; running remotely via SSH on {DEFAULT_REMOTE_SSH}", file=sys.stderr)
    raise SystemExit(
        subprocess.run(
            [
                "ssh",
                "-o",
                "ConnectTimeout=5",
                "-o",
                "StrictHostKeyChecking=no",
                "-o",
                "UserKnownHostsFile=/dev/null",
                DEFAULT_REMOTE_SSH,
                remote_cmd,
            ],
            check=False,
        ).returncode
    )


def maybe_reexec_with_ros_env(args: argparse.Namespace) -> None:
    if os.environ.get("RM_DEMO_ROS_ENV_READY") == "1":
        return
    if args.transport != "local":
        return
    if _has_local_capture_input(args):
        return
    try:
        import pyrealsense2  # type: ignore  # noqa: F401

        return
    except Exception:
        pass

    ros_setups = sorted(glob.glob("/opt/ros/*/setup.bash"))
    ws_setup = "/home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/setup.bash"
    if not ros_setups and not os.path.isfile(ws_setup):
        return
    sources = []
    if ros_setups:
        sources.append(f"source {shlex.quote(ros_setups[-1])} >/dev/null 2>&1 || true")
    if os.path.isfile(ws_setup):
        sources.append(f"source {shlex.quote(ws_setup)} >/dev/null 2>&1 || true")
    cmd = (
        "{sources}; "
        "cd {cwd} && RM_DEMO_ROS_ENV_READY=1 python3 {script} {args}"
    ).format(
        sources="; ".join(sources),
        cwd=shlex.quote(os.getcwd()),
        script=shlex.quote(os.path.abspath(__file__)),
        args=" ".join(shlex.quote(arg) for arg in sys.argv[1:]),
    )
    raise SystemExit(subprocess.run(["bash", "-lc", cmd], check=False).returncode)


def _save_json(path: str, data: dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def main() -> None:
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    if not _has_local_capture_input(args):
        maybe_exec_remote(args)
        maybe_reexec_with_ros_env(args)

    if _has_local_capture_input(args):
        capture = load_captured_frame(args.rgb_path, args.depth_path, args.intrinsics_path)
    else:
        if args.capture_positioning != "none":
            position_events = position_for_capture(
                host=args.host,
                mode=args.capture_positioning,
                trajectory_config=args.trajectory_config,
                speed=args.position_speed,
                tool_name_camera=args.camera_tool_name,
                tool_name_restore=args.restore_tool_name,
                shifting_number=args.shifting_number,
                control_backend=args.control_backend,
                prepare_section=args.capture_prepare_section,
                prepare_joints=args.capture_joints,
            )
            for event in position_events:
                print(f"capture_positioning={json.dumps(event, ensure_ascii=False)}")
            if args.capture_settle_s > 0:
                time.sleep(args.capture_settle_s)
        capture = capture_single_frame(args.output_dir)
    print(f"capture_rgb={capture.color_path}")
    print(f"capture_depth={capture.depth_path}")

    try:
        detect_result, overlay = detect_static_meridians(
            color_bgr=capture.color_bgr,
            depth_m=capture.depth_m,
            intrinsics_data=capture.intrinsics,
            finger_width_mm=args.finger_width,
            model_path=args.model_path,
            sample_points=args.sample_points,
            backend=args.detector_backend,
        )
    except Exception as exc:
        raise RuntimeError(
            f"{exc}; latest capture is rgb={capture.color_path} depth={capture.depth_path}"
        ) from exc
    detect_result["capture"] = {
        "rgb_path": capture.color_path,
        "depth_path": capture.depth_path,
        "intrinsics_path": capture.intrinsics_path,
    }
    detect_result = select_side(detect_result, args.side)
    prefix = f"static_demo_{detect_result['timestamp']}"
    overlay_path, detect_json_path = save_detection_artifacts(args.output_dir, detect_result, overlay, prefix=prefix)
    print(f"detector_backend={detect_result.get('detector_backend', 'unknown')}")
    print(f"detection_overlay={overlay_path}")
    print(f"detection_json={detect_json_path}")

    if args.transform_backend == "product_ros":
        detect_result = attach_robot_points_via_product_services(
            color_bgr=capture.color_bgr,
            depth_m=capture.depth_m,
            detection_result=detect_result,
            host=args.host,
            install_ang=list(args.install_ang),
            control_backend=args.control_backend,
        )
    elif args.transform_backend == "static":
        matrix = load_transform_matrix(args.matrix_path)
        if matrix is None:
            raise RuntimeError(f"camera->robot matrix not found or invalid: {args.matrix_path}")
        detect_result = attach_robot_points(detect_result, matrix)
        detect_result["transform_backend"] = "static"
    else:
        try:
            detect_result = attach_robot_points_via_product_services(
                color_bgr=capture.color_bgr,
                depth_m=capture.depth_m,
                detection_result=detect_result,
                host=args.host,
                install_ang=list(args.install_ang),
                control_backend=args.control_backend,
            )
        except Exception as exc:
            matrix = load_transform_matrix(args.matrix_path)
            if matrix is None:
                raise RuntimeError(f"product_ros transform failed and static matrix is unavailable: {exc}") from exc
            detect_result = attach_robot_points(detect_result, matrix)
            detect_result["transform_backend"] = "static"
            detect_result["transform_backend_fallback_reason"] = str(exc)
    transformed_json_path = os.path.join(args.output_dir, f"{prefix}_transform.json")
    _save_json(transformed_json_path, detect_result)
    print(f"transform_backend={detect_result.get('transform_backend', 'unknown')}")
    print(f"transform_json={transformed_json_path}")

    arm = create_arm_backend(args.control_backend)
    if not arm.can_connect(args.host):
        raise RuntimeError(f"{args.control_backend} arm backend is not reachable")
    _, current_pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(args.host)
    print(
        f"anchor_pose={[round(v, 6) for v in current_pose]} "
        f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
    )

    selected_points = list(detect_result.get("selected_meridian_robot", []))
    if len(selected_points) < 2:
        raise RuntimeError("selected_meridian_robot has insufficient valid points")
    plan = build_static_plan(
        side=args.side,
        meridian_points_robot_m=selected_points,
        anchor_pose_m=current_pose,
        point_count=args.plan_points,
        hover_m=args.hover_mm / 1000.0,
        dwell_s=args.dwell_s,
        safe_lift_m=args.safe_lift_mm / 1000.0,
        meridian_pose_quat=list(detect_result.get("selected_meridian_robot_pose_quat", [])),
    )
    plan_json = os.path.join(args.output_dir, f"{prefix}_plan.json")
    _save_json(plan_json, plan_to_dict(plan))
    print(f"plan_json={plan_json}")
    preview_plan(plan)

    if not args.run or args.mode == "preview":
        print("preview only; pass --run with --mode hover|monitor|ros_force_pose to execute")
        return

    execute_plan(
        host=args.host,
        plan=plan,
        speed=args.speed,
        monitor_force=args.mode in ("monitor", "touch_monitor", "ros_force_pose"),
        target_force_n=args.target_force_n,
        mode=args.mode,
        control_backend=args.control_backend,
        force_direction=args.force_direction,
        force_mode=args.force_mode,
        max_force_n=args.max_force_n,
        touch_step_m=args.touch_step_mm / 1000.0,
        max_press_m=args.max_press_mm / 1000.0,
    )


if __name__ == "__main__":
    main()
