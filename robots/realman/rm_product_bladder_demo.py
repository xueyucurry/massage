#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from dataclasses import asdict
from typing import Any

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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
    DEFAULT_OUTPUT_DIR,
    DEFAULT_PLAN_POINTS,
    DEFAULT_POSITION_SPEED,
    DEFAULT_RESTORE_TOOL_NAME,
    DEFAULT_SAFE_LIFT_MM,
    DEFAULT_SAMPLE_POINTS,
    DEFAULT_SHIFTING_NUMBER,
    DEFAULT_SPEED,
    DEFAULT_TRAJECTORY_CONFIG,
)
from rm_demo.rm_bladder import (
    bladder_plan_to_dict,
    build_bladder_massage_plan,
    detect_bladder_lines,
    execute_bladder_plan,
    preview_bladder_plan,
    save_bladder_artifacts,
    select_bladder_line,
)
from rm_demo.rm_capture import _import_ros_capture_modules, ensure_output_dir
from rm_demo.rm_positioning import position_for_capture
from rm_demo.rm_product_ros import attach_robot_points_via_product_services
from rm_demo.rm_ros import create_arm_backend


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RealMan product bladder demo: live preview + save trajectory + simulate dian/fen/shun massage"
    )
    parser.add_argument("--host", default=os.environ.get("RM_ARM_HOST", DEFAULT_HOST), help="RM controller IP")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="artifact directory")
    parser.add_argument("--side", choices=("left", "right"), default="left", help="meridian side to execute")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer", help="bladder meridian layer")
    parser.add_argument("--finger-width", type=float, default=DEFAULT_FINGER_WIDTH_MM, help="base offset in mm")
    parser.add_argument("--sample-points", type=int, default=DEFAULT_SAMPLE_POINTS, help="points sampled along the selected line")
    parser.add_argument("--plan-points", type=int, default=DEFAULT_PLAN_POINTS, help="massage points executed along one side")
    parser.add_argument("--hover-mm", type=float, default=DEFAULT_HOVER_MM, help="hover height above body surface")
    parser.add_argument("--dian-jin-depth-mm", type=float, default=8.0, help="press depth for dian jin")
    parser.add_argument("--fen-jin-lateral-mm", type=float, default=15.0, help="lateral split offset for fen jin")
    parser.add_argument("--safe-lift-mm", type=float, default=DEFAULT_SAFE_LIFT_MM, help="extra safe lift before transit")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="motion speed")
    parser.add_argument("--control-backend", choices=("json", "ros"), default=DEFAULT_CONTROL_BACKEND, help="arm control backend")
    parser.add_argument(
        "--capture-positioning",
        choices=("none", "prepare", "service", "prepare_then_service"),
        default=DEFAULT_CAPTURE_POSITIONING,
        help="pre-position camera above body using product flow",
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
    parser.add_argument(
        "--install-ang",
        nargs=3,
        type=float,
        default=DEFAULT_INSTALL_ANG,
        metavar=("RX", "RY", "RZ"),
        help="robot install angles for calc_poses service",
    )
    parser.add_argument("--dian-jin-dwell-s", type=float, default=0.5, help="dwell after the dian jin press")
    parser.add_argument("--fen-jin-dwell-s", type=float, default=0.3, help="dwell at each fen jin side pose")
    parser.add_argument("--shun-jin-dwell-s", type=float, default=0.0, help="optional dwell on shun jin path points")
    return parser.parse_args()


def _save_json(path: str, data: dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def _camera_info_to_intrinsics(cam_info) -> dict[str, object]:
    distortion_model = str(getattr(cam_info, "distortion_model", "none")).lower().strip()
    if distortion_model in ("plumb_bob", "brown_conrady"):
        model_name = "brown_conrady"
    elif distortion_model in ("inverse_brown_conrady",):
        model_name = "inverse_brown_conrady"
    else:
        model_name = "none"
    return {
        "width": int(cam_info.width),
        "height": int(cam_info.height),
        "ppx": float(cam_info.K[2]),
        "ppy": float(cam_info.K[5]),
        "fx": float(cam_info.K[0]),
        "fy": float(cam_info.K[4]),
        "model_name": model_name,
        "coeffs": [float(v) for v in cam_info.D[:5]] + [0.0] * max(0, 5 - len(cam_info.D[:5])),
        "depth_scale": 0.001,
    }


class RosFrameCollector:
    def __init__(self) -> None:
        rospy, CvBridge, CameraInfo, Image = _import_ros_capture_modules()
        self.rospy = rospy
        self.CvBridge = CvBridge
        self.CameraInfo = CameraInfo
        self.Image = Image
        if not self.rospy.core.is_initialized():
            self.rospy.init_node("rm_product_bladder_demo", anonymous=True, disable_signals=True)

        from message_filters import ApproximateTimeSynchronizer, Subscriber  # type: ignore

        self.bridge = self.CvBridge()
        self.lock = threading.Lock()
        self.latest_frame: dict[str, Any] | None = None
        self.latest_seq = 0
        self.intrinsics: dict[str, object] | None = None

        self.color_sub = Subscriber("/camera/color/image_raw", self.Image)
        self.depth_sub = Subscriber("/camera/aligned_depth_to_color/image_raw", self.Image)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=5, slop=0.1)
        self.sync.registerCallback(self._on_frames)
        self.cam_info_sub = self.rospy.Subscriber("/camera/color/camera_info", self.CameraInfo, self._on_camera_info, queue_size=1)

    def _on_camera_info(self, msg) -> None:
        intrinsics = _camera_info_to_intrinsics(msg)
        with self.lock:
            self.intrinsics = intrinsics

    def _on_frames(self, color_msg, depth_msg) -> None:
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception:
            return

        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) * 0.001
        else:
            depth_m = depth_raw.astype(np.float32)

        with self.lock:
            self.latest_seq += 1
            self.latest_frame = {
                "seq": self.latest_seq,
                "stamp": time.time(),
                "color_bgr": color_bgr.copy(),
                "depth_m": depth_m.copy(),
            }

    def wait_until_ready(self, timeout_s: float = 10.0) -> None:
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            with self.lock:
                ready = self.latest_frame is not None and self.intrinsics is not None
            if ready:
                return
            time.sleep(0.05)
        raise RuntimeError("camera frames or camera_info not ready")

    def get_latest(self) -> tuple[dict[str, Any] | None, dict[str, object] | None]:
        with self.lock:
            frame = None if self.latest_frame is None else {
                "seq": int(self.latest_frame["seq"]),
                "stamp": float(self.latest_frame["stamp"]),
                "color_bgr": self.latest_frame["color_bgr"].copy(),
                "depth_m": self.latest_frame["depth_m"].copy(),
            }
            intrinsics = None if self.intrinsics is None else dict(self.intrinsics)
        return frame, intrinsics


def _annotate_preview(image_bgr: np.ndarray, lines: list[str], color: tuple[int, int, int]) -> np.ndarray:
    out = image_bgr.copy()
    y = 28
    for line in lines:
        cv2.putText(out, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        y += 26
    return out


def _run_positioning_once(args: argparse.Namespace) -> None:
    if args.capture_positioning == "none":
        return
    events = position_for_capture(
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
    for event in events:
        print(f"capture_positioning={json.dumps(event, ensure_ascii=False)}")
    if args.capture_settle_s > 0:
        time.sleep(args.capture_settle_s)


def _prepare_plan_from_detection(
    args: argparse.Namespace,
    frame: dict[str, Any],
    detect_result: dict[str, Any],
    overlay: np.ndarray,
) -> tuple[Any, dict[str, Any]]:
    ensure_output_dir(args.output_dir)
    prefix = f"bladder_product_{detect_result['timestamp']}"

    overlay_path, detect_json_path = save_bladder_artifacts(
        args.output_dir,
        detect_result,
        overlay,
        prefix=prefix,
    )

    transformed = attach_robot_points_via_product_services(
        color_bgr=frame["color_bgr"],
        depth_m=frame["depth_m"],
        detection_result=detect_result,
        host=args.host,
        install_ang=list(args.install_ang),
        control_backend=args.control_backend,
    )
    transform_json_path = os.path.join(args.output_dir, f"{prefix}_transform.json")
    _save_json(transform_json_path, transformed)

    arm = create_arm_backend(args.control_backend)
    if not arm.can_connect(args.host):
        raise RuntimeError(f"{args.control_backend} arm backend is not reachable")
    _, current_pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(args.host)
    print(
        f"anchor_pose={[round(v, 6) for v in current_pose]} "
        f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}"
    )

    selected_points = list(transformed.get("selected_meridian_robot", []))
    selected_pixels = list(transformed.get("selected_meridian_pixel", []))
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
        meridian_pose_quat=list(transformed.get("selected_meridian_robot_pose_quat", [])),
    )
    plan_json_path = os.path.join(args.output_dir, f"{prefix}_plan.json")
    _save_json(plan_json_path, bladder_plan_to_dict(plan))

    summary = {
        "prefix": prefix,
        "overlay_path": overlay_path,
        "detect_json_path": detect_json_path,
        "transform_json_path": transform_json_path,
        "plan_json_path": plan_json_path,
        "transform_backend": str(transformed.get("transform_backend", "unknown")),
        "point_count": int(plan.point_count),
        "side": str(args.side),
        "line_type": str(args.line_type),
        "timestamp": str(detect_result["timestamp"]),
    }
    return plan, summary


def main() -> None:
    args = parse_args()
    ensure_output_dir(args.output_dir)

    collector = RosFrameCollector()
    collector.wait_until_ready(timeout_s=12.0)
    print("camera_ready=1")

    try:
        _run_positioning_once(args)
    except Exception as exc:
        print(f"capture_positioning_failed={exc}")

    last_ok: tuple[dict[str, Any], dict[str, Any], np.ndarray] | None = None
    last_saved_plan_json = ""

    print("keys: c=reposition  s=save trajectory+plan  r=run simulate massage  q=quit")
    while True:
        frame, intrinsics = collector.get_latest()
        if frame is None or intrinsics is None:
            time.sleep(0.03)
            continue

        try:
            detect_result, overlay = detect_bladder_lines(
                color_bgr=frame["color_bgr"],
                depth_m=frame["depth_m"],
                intrinsics_data=intrinsics,
                finger_width_mm=args.finger_width,
                sample_points=args.sample_points,
            )
            detect_result["capture"] = {
                "backend": "product_ros_live",
                "stamp": float(frame["stamp"]),
            }
            detect_result = select_bladder_line(detect_result, args.side, args.line_type)
            overlay = _annotate_preview(
                overlay,
                [
                    f"side={args.side} line={args.line_type} sample={args.sample_points} plan={args.plan_points}",
                    f"keys: c reposition | s save | r run | q quit",
                    f"saved_plan={os.path.basename(last_saved_plan_json) if last_saved_plan_json else 'none'}",
                ],
                (0, 255, 255),
            )
            last_ok = (frame, detect_result, overlay)
        except Exception as exc:
            overlay = _annotate_preview(
                frame["color_bgr"],
                [
                    f"detect failed: {exc}",
                    "keep full back / shoulders / hips in frame",
                    "keys: c reposition | q quit",
                ],
                (0, 0, 255),
            )

        cv2.imshow("RM Product Bladder Demo", overlay)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        if key == ord("c"):
            try:
                _run_positioning_once(args)
                print("capture_positioning=ok")
            except Exception as exc:
                print(f"capture_positioning_failed={exc}")
            continue

        if key == ord("s"):
            if last_ok is None:
                print("当前没有可用检测结果，未保存")
                continue
            try:
                plan, summary = _prepare_plan_from_detection(args, *last_ok)
                preview_bladder_plan(plan)
                last_saved_plan_json = str(summary["plan_json_path"])
                print(f"saved_overlay={summary['overlay_path']}")
                print(f"saved_detect={summary['detect_json_path']}")
                print(f"saved_transform={summary['transform_json_path']}")
                print(f"saved_plan={summary['plan_json_path']}")
                print(f"transform_backend={summary['transform_backend']}")
            except Exception as exc:
                print(f"save_failed={exc}")
            continue

        if key == ord("r"):
            if last_ok is None:
                print("当前没有可用检测结果，无法执行")
                continue
            try:
                plan, summary = _prepare_plan_from_detection(args, *last_ok)
                last_saved_plan_json = str(summary["plan_json_path"])
                preview_bladder_plan(plan)
                print(
                    f"execute_plan side={summary['side']} line={summary['line_type']} "
                    f"points={summary['point_count']} backend={summary['transform_backend']}"
                )
                execute_bladder_plan(
                    host=args.host,
                    plan=plan,
                    speed=args.speed,
                    control_backend=args.control_backend,
                    dian_jin_dwell_s=args.dian_jin_dwell_s,
                    fen_jin_dwell_s=args.fen_jin_dwell_s,
                    shun_jin_dwell_s=args.shun_jin_dwell_s,
                )
            except Exception as exc:
                print(f"execute_failed={exc}")
            continue

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
