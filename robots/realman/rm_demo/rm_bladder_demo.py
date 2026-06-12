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
        DEFAULT_FINGER_WIDTH_MM,
        DEFAULT_HOST,
        DEFAULT_HOVER_MM,
        DEFAULT_INSTALL_ANG,
        DEFAULT_MATRIX_PATH,
        DEFAULT_MODEL_PATH,
        DEFAULT_OUTPUT_DIR,
        DEFAULT_PLAN_POINTS,
        DEFAULT_POSITION_SPEED,
        DEFAULT_RESTORE_TOOL_NAME,
        DEFAULT_SAFE_LIFT_MM,
        DEFAULT_SAMPLE_POINTS,
        DEFAULT_SIDE,
        DEFAULT_SHIFTING_NUMBER,
        DEFAULT_SPEED,
        DEFAULT_TRAJECTORY_CONFIG,
    )
    from rm_demo.rm_bladder import (
        attach_selected_robot_points_static,
        bladder_plan_to_dict,
        build_bladder_massage_plan,
        detect_bladder_lines,
        execute_bladder_plan,
        preview_bladder_plan,
        save_bladder_artifacts,
        select_bladder_line,
    )
    from rm_demo.rm_capture import capture_single_frame, has_realsense_device, load_captured_frame
    from rm_demo.rm_positioning import position_for_capture
    from rm_demo.rm_product_ros import attach_robot_points_via_product_services
    from rm_demo.rm_ros import create_arm_backend
    from rm_demo.rm_transform import load_transform_matrix
else:
    from .config import (
        DEFAULT_CAMERA_TOOL_NAME,
        DEFAULT_CAPTURE_PREPARE_SECTION,
        DEFAULT_CAPTURE_POSITIONING,
        DEFAULT_CAPTURE_SETTLE_S,
        DEFAULT_CONTROL_BACKEND,
        DEFAULT_FINGER_WIDTH_MM,
        DEFAULT_HOST,
        DEFAULT_HOVER_MM,
        DEFAULT_INSTALL_ANG,
        DEFAULT_MATRIX_PATH,
        DEFAULT_MODEL_PATH,
        DEFAULT_OUTPUT_DIR,
        DEFAULT_PLAN_POINTS,
        DEFAULT_POSITION_SPEED,
        DEFAULT_RESTORE_TOOL_NAME,
        DEFAULT_SAFE_LIFT_MM,
        DEFAULT_SAMPLE_POINTS,
        DEFAULT_SIDE,
        DEFAULT_SHIFTING_NUMBER,
        DEFAULT_SPEED,
        DEFAULT_TRAJECTORY_CONFIG,
    )
    from .rm_bladder import (
        attach_selected_robot_points_static,
        bladder_plan_to_dict,
        build_bladder_massage_plan,
        detect_bladder_lines,
        execute_bladder_plan,
        preview_bladder_plan,
        save_bladder_artifacts,
        select_bladder_line,
    )
    from .rm_capture import capture_single_frame, has_realsense_device, load_captured_frame
    from .rm_positioning import position_for_capture
    from .rm_product_ros import attach_robot_points_via_product_services
    from .rm_ros import create_arm_backend
    from .rm_transform import load_transform_matrix


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RealMan bladder meridian massage demo")
    parser.add_argument("--host", default=DEFAULT_HOST, help="RM controller IP")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="artifact directory")
    parser.add_argument("--model-path", default=DEFAULT_MODEL_PATH, help="pose model path")
    parser.add_argument("--matrix-path", default=DEFAULT_MATRIX_PATH, help="camera->robot 4x4 matrix JSON")
    parser.add_argument(
        "--transform-backend",
        choices=("auto", "product_ros", "static"),
        default="auto",
        help="camera->robot transform backend",
    )
    parser.add_argument(
        "--install-ang",
        nargs=3,
        type=float,
        default=DEFAULT_INSTALL_ANG,
        metavar=("RX", "RY", "RZ"),
        help="robot install angles for calc_poses service",
    )
    parser.add_argument("--side", choices=("left", "right"), default=DEFAULT_SIDE, help="meridian side to execute")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer", help="bladder meridian layer")
    parser.add_argument("--finger-width", type=float, default=DEFAULT_FINGER_WIDTH_MM, help="base offset in mm")
    parser.add_argument("--sample-points", type=int, default=DEFAULT_SAMPLE_POINTS, help="points sampled along the selected line")
    parser.add_argument("--plan-points", type=int, default=DEFAULT_PLAN_POINTS, help="massage points executed along one side")
    parser.add_argument("--hover-mm", type=float, default=DEFAULT_HOVER_MM, help="hover height above the body")
    parser.add_argument("--dian-jin-depth-mm", type=float, default=10.0, help="press depth for dian jin")
    parser.add_argument("--fen-jin-lateral-mm", type=float, default=20.0, help="lateral split offset for fen jin")
    parser.add_argument("--safe-lift-mm", type=float, default=DEFAULT_SAFE_LIFT_MM, help="extra safe lift before lateral moves")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="motion speed; ROS backend uses 0..1")
    parser.add_argument("--control-backend", choices=("json", "ros"), default=DEFAULT_CONTROL_BACKEND, help="arm control backend")
    parser.add_argument(
        "--transport",
        choices=("auto", "local"),
        default="local",
        help="always run on workstation; auto only auto-detects local capture backend",
    )
    parser.add_argument(
        "--capture-positioning",
        choices=("none", "prepare", "service", "prepare_then_service"),
        default=DEFAULT_CAPTURE_POSITIONING,
        help="pre-capture positioning",
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
    parser.add_argument("--position-speed", type=float, default=DEFAULT_POSITION_SPEED, help="speed for prepare joint move")
    parser.add_argument("--camera-tool-name", default=DEFAULT_CAMERA_TOOL_NAME, help="tool frame name for camera")
    parser.add_argument("--restore-tool-name", default=DEFAULT_RESTORE_TOOL_NAME, help="tool frame restored after service move")
    parser.add_argument("--shifting-number", type=int, default=DEFAULT_SHIFTING_NUMBER, help="move_camera_above_person shifting_number")
    parser.add_argument("--capture-settle-s", type=float, default=DEFAULT_CAPTURE_SETTLE_S, help="settle time after positioning")
    parser.add_argument("--rgb-path", default="", help="use an existing rgb png instead of live capture")
    parser.add_argument("--depth-path", default="", help="use an existing depth npy instead of live capture")
    parser.add_argument("--intrinsics-path", default="", help="intrinsics json for offline input")
    parser.add_argument("--dian-jin-dwell-s", type=float, default=0.5, help="dwell after the dian jin press")
    parser.add_argument("--fen-jin-dwell-s", type=float, default=0.3, help="dwell at each fen jin side pose")
    parser.add_argument("--shun-jin-dwell-s", type=float, default=0.0, help="optional dwell on shun jin path points")
    parser.add_argument("--run", action="store_true", help="execute robot motion")
    return parser.parse_args()


def _has_local_capture_input(args: argparse.Namespace) -> bool:
    return bool(args.rgb_path and args.depth_path and args.intrinsics_path)


def maybe_reexec_with_ros_env(args: argparse.Namespace) -> None:
    if os.environ.get("RM_DEMO_ROS_ENV_READY") == "1":
        return
    if args.transport not in ("local", "auto"):
        return
    if _has_local_capture_input(args):
        return
    try:
        import pyrealsense2  # type: ignore  # noqa: F401

        return
    except Exception:
        pass

    ros_setups = sorted(glob.glob("/opt/ros/*/setup.bash"))
    extra_setup = os.environ.get("RM_DEMO_LOCAL_ROS_SETUP", "").strip()
    if not ros_setups and not (extra_setup and os.path.isfile(extra_setup)):
        return
    sources = []
    if ros_setups:
        sources.append(f"source {shlex.quote(ros_setups[-1])} >/dev/null 2>&1 || true")
    if extra_setup and os.path.isfile(extra_setup):
        sources.append(f"source {shlex.quote(extra_setup)} >/dev/null 2>&1 || true")
    cmd = (
        "{sources}; "
        "cd {cwd} && RM_DEMO_ROS_ENV_READY=1 {python} {script} {args}"
    ).format(
        sources="; ".join(sources),
        cwd=shlex.quote(os.getcwd()),
        python=shlex.quote(sys.executable or "python3"),
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

    detect_result, overlay = detect_bladder_lines(
        color_bgr=capture.color_bgr,
        depth_m=capture.depth_m,
        intrinsics_data=capture.intrinsics,
        finger_width_mm=args.finger_width,
        model_path=args.model_path,
        sample_points=args.sample_points,
    )
    detect_result["capture"] = {
        "rgb_path": capture.color_path,
        "depth_path": capture.depth_path,
        "intrinsics_path": capture.intrinsics_path,
    }
    detect_result = select_bladder_line(detect_result, args.side, args.line_type)
    prefix = f"bladder_demo_{detect_result['timestamp']}"
    overlay_path, detect_json_path = save_bladder_artifacts(args.output_dir, detect_result, overlay, prefix=prefix)
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
        detect_result = attach_selected_robot_points_static(detect_result, matrix)
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
            detect_result = attach_selected_robot_points_static(detect_result, matrix)
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
    selected_pixels = list(detect_result.get("selected_meridian_pixel", []))
    if len(selected_points) < 2:
        raise RuntimeError("selected_meridian_robot has insufficient valid points")
    plan = build_bladder_massage_plan(
        side=args.side,
        line_type=args.line_type,
        meridian_points_robot_m=selected_points,
        meridian_pixels=selected_pixels,
        anchor_pose_m=current_pose,
        point_count=args.plan_points,
        hover_m=args.hover_mm / 1000.0,
        dian_jin_depth_m=args.dian_jin_depth_mm / 1000.0,
        fen_jin_lateral_m=args.fen_jin_lateral_mm / 1000.0,
        safe_lift_m=args.safe_lift_mm / 1000.0,
        meridian_pose_quat=list(detect_result.get("selected_meridian_robot_pose_quat", [])),
    )
    plan_json = os.path.join(args.output_dir, f"{prefix}_plan.json")
    _save_json(plan_json, bladder_plan_to_dict(plan))
    print(f"plan_json={plan_json}")
    preview_bladder_plan(plan)

    if not args.run:
        print("preview only; pass --run to execute point/fen/shun massage sequence")
        return

    execute_bladder_plan(
        host=args.host,
        plan=plan,
        speed=args.speed,
        control_backend=args.control_backend,
        dian_jin_dwell_s=args.dian_jin_dwell_s,
        fen_jin_dwell_s=args.fen_jin_dwell_s,
        shun_jin_dwell_s=args.shun_jin_dwell_s,
    )


if __name__ == "__main__":
    main()
