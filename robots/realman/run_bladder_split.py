#!/usr/bin/env python3
"""End-to-end orchestrator for bladder meridian demo using the hybrid link:

1. noetic docker on THIS workstation acts as a ROS1 client against the board's
   rosmaster (http://192.168.1.11:11311). It handles all ROS I/O -- capturing
   one aligned RGB-D frame from the board camera, calling the product ROS
   services (calc_position_normal + calc_poses) for the eye-on-hand transform,
   and publishing MoveL/MoveJ commands to /rm_driver/*.
2. Host venv (.venv, Python 3.10) runs the ultralytics YOLO pose detection and
   builds the bladder meridian plan geometry.

Artifacts (rgb.png, depth.npy, intrinsics.json, detect.json, transform.json,
plan.json) are written under rm_demo_output/ and are visible to both the host
and the docker thanks to the /home/franka bind mount.

Run examples:
  # Preview only (no arm motion)
  .venv/bin/python run_bladder_split.py

  # Execute point-press + split + smooth sequence (real arm motion)
  .venv/bin/python run_bladder_split.py --run

  # Execute hover-only path traversal (no body contact)
  .venv/bin/python run_bladder_split.py --run --execution-mode hover_path

  # Side-lying capture pose from taught joints, then detect only
  .venv/bin/python run_bladder_split.py --product-flow \
      --product-flow-positioning prepare \
      --capture-joints 0 35 -95 20 -70 10 \
      --transform-backend product_ros
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import textwrap
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_DIR))

from rm_demo.config import (  # noqa: E402
    DEFAULT_CAMERA_TOOL_NAME,
    DEFAULT_CAPTURE_PREPARE_SECTION,
    DEFAULT_RESTORE_TOOL_NAME,
    DEFAULT_SHIFTING_NUMBER,
    DEFAULT_TRAJECTORY_CONFIG,
)
from rm_demo.rm_bladder import (  # noqa: E402
    attach_selected_robot_points_eye_in_hand,
    attach_selected_robot_points_static,
    detect_bladder_lines,
    filter_selected_meridian_continuity,
    offset_selected_meridian_robot,
    rebuild_plan_with_fixed_first_normal,
    rebuild_plan_with_horizontal_press,
    save_bladder_artifacts,
    select_bladder_line,
    select_topmost_bladder_line,
    trim_selected_meridian_ends,
)
from rm_demo.rm_positioning import load_prepare_joints  # noqa: E402
from rm_demo.rm_transform import load_transform_matrix  # noqa: E402

DEFAULT_MASTER_URI = "http://192.168.1.11:11311"
DEFAULT_ROS_IP = "192.168.1.250"
DEFAULT_ARM_HOST = "192.168.1.18"
DEFAULT_OUTPUT_DIR = PROJECT_DIR / "rm_demo_output"
DEFAULT_CONTAINER = "noetic"
DEFAULT_VENDOR_PY = PROJECT_DIR / "ros_vendor" / "python"


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Split bladder demo: docker captures & controls, host detects.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("--container", default=DEFAULT_CONTAINER, help="docker container name")
    ap.add_argument("--master-uri", default=DEFAULT_MASTER_URI, help="board rosmaster URI")
    ap.add_argument("--ros-ip", default=DEFAULT_ROS_IP, help="local ROS_IP on the 192.168.1.0/24 subnet")
    ap.add_argument("--arm-host", default=DEFAULT_ARM_HOST, help="arm host (only used by json backend)")
    ap.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="artifact directory")
    ap.add_argument("--model-path", default=str(PROJECT_DIR / "yolo11l-pose.pt"), help="YOLO pose model")
    ap.add_argument(
        "--transform-backend",
        choices=("static", "product_ros"),
        default="static",
        help="camera->robot transform backend; static loads the eye-on-hand matrix from trajectory config by default",
    )
    ap.add_argument(
        "--matrix-path",
        default=DEFAULT_TRAJECTORY_CONFIG,
        help="static camera->robot matrix source (.json matrix file or trajectory_generate.yaml eye_on_hand_calibrate)",
    )
    ap.add_argument(
        "--trajectory-config",
        default="",
        help="trajectory_generate.yaml used for product-flow prepare pose; defaults to --matrix-path when it is yaml, otherwise the local default",
    )
    ap.add_argument("--camera-tool-name", default=DEFAULT_CAMERA_TOOL_NAME, help="tool frame name for camera")
    ap.add_argument("--restore-tool-name", default=DEFAULT_RESTORE_TOOL_NAME, help="tool frame restored after static transform flow")
    ap.add_argument(
        "--product-flow",
        action="store_true",
        help="follow product flow before capture: prepare pose, move_camera_above_person(camera, massage tool), then detect",
    )
    ap.add_argument(
        "--skip-product-scan",
        action="store_true",
        help="when --product-flow is set, skip move_camera_above_person/person-area scan and use the current/cached trajectory flow",
    )
    ap.add_argument(
        "--product-flow-positioning",
        choices=("prepare", "prepare_then_service"),
        default="prepare_then_service",
        help="positioning sequence used by --product-flow before capture",
    )
    ap.add_argument("--massage-tool-name", default="mas_rub", help="product massage tool frame, e.g. mas_rub or mas_palm")
    ap.add_argument(
        "--product-trajectory-mode",
        choices=("generate_only", "upload_only", "execute", "pose_check", "legacy_movel"),
        default="generate_only",
        help="for product_ros, use native product rubbing trajectory flow instead of custom MoveL execution",
    )
    ap.add_argument("--product-force", type=int, default=2, help="force value passed to /generate_trajectory_rubbing")
    ap.add_argument("--product-speed", type=int, default=50, help="speed value passed to /generate_trajectory_rubbing")
    ap.add_argument("--trajectory-type", type=int, default=4, help="trajectory_type passed to /generate_trajectory_rubbing")
    ap.add_argument(
        "--allow-controller-write",
        action="store_true",
        help="allow upload_only/execute modes to write the trajectory to the controller",
    )
    ap.add_argument(
        "--side-lying-product-correction",
        action="store_true",
        help="rewrite native product rubbing entry poses so hover/down offsets follow the side-lying back normal",
    )
    ap.add_argument(
        "--product-down-3cm-mm",
        type=float,
        default=10.0,
        help="side-lying product correction overtravel for the product p1_down_3cm point",
    )
    ap.add_argument(
        "--product-down-1cm-mm",
        type=float,
        default=3.0,
        help="side-lying product correction overtravel for the product p1_down_1cm point",
    )
    ap.add_argument(
        "--tool-contact-axis",
        choices=("neg_z", "pos_z", "pos_x", "neg_x", "pos_y", "neg_y"),
        default="pos_z",
        help="which mas_rub tool axis is the real contact normal; pose_check can be used to find the correct axis",
    )
    ap.add_argument(
        "--legacy-tool-contact-axis",
        choices=("neg_z", "pos_z", "pos_x", "neg_x", "pos_y", "neg_y"),
        default="",
        help="optional tool contact-axis correction for legacy_movel plans; empty means use product calc_poses orientation unchanged",
    )
    ap.add_argument(
        "--pose-check-move-prepare",
        action="store_true",
        help="in product pose_check mode, also move through the product-generated prepare joint before p1_above_2cm",
    )
    ap.add_argument(
        "--pose-check-entry-motion",
        choices=("movel", "movej_p"),
        default="movej_p",
        help="motion primitive used for product pose_check p1_above_2cm after optional prepare",
    )
    ap.add_argument("--shifting-number", type=int, default=DEFAULT_SHIFTING_NUMBER)
    ap.add_argument("--position-speed", type=float, default=5.0, help="speed for product-flow prepare joint move")
    ap.add_argument(
        "--capture-prepare-section",
        default=DEFAULT_CAPTURE_PREPARE_SECTION,
        help="trajectory config section used by product-flow prepare",
    )
    ap.add_argument(
        "--capture-joints",
        nargs=6,
        type=float,
        default=None,
        metavar=("J0", "J1", "J2", "J3", "J4", "J5"),
        help="override prepare section with 6 side-lying capture-pose joint angles in degrees",
    )
    ap.add_argument(
        "--save-current-capture-pose",
        action="store_true",
        help="read current arm state and save its 6 joints into --capture-prepare-section before capture/execution",
    )
    ap.add_argument(
        "--save-current-capture-pose-only",
        action="store_true",
        help="save current arm state as the capture prepare pose and exit without capturing or moving",
    )
    ap.add_argument("--finger-width-mm", type=float, default=45.0)
    ap.add_argument("--sample-points", type=int, default=30)
    ap.add_argument("--plan-points", type=int, default=6)
    ap.add_argument("--side", choices=("left", "right"), default="left")
    ap.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    ap.add_argument(
        "--auto-top-line",
        action="store_true",
        help="ignore --side/--line-type and select the line with the smallest average pixel y",
    )
    ap.add_argument("--hover-mm", type=float, default=20.0, help="hover height above body")
    ap.add_argument(
        "--hover-offset-mode",
        choices=("normal", "base_z"),
        default="normal",
        help="how to offset from the transformed meridian point for hover targets; base_z matches product pose_check localization",
    )
    ap.add_argument("--dian-jin-depth-mm", type=float, default=10.0, help="press depth for dian jin")
    ap.add_argument("--fen-jin-lateral-mm", type=float, default=20.0, help="lateral split offset")
    ap.add_argument("--safe-lift-mm", type=float, default=40.0, help="safe lift over the body between points")
    ap.add_argument("--speed", type=float, default=0.3, help="motion speed (ROS backend 0..1, or 1..100)")
    ap.add_argument(
        "--max-step-m",
        type=float,
        default=0.05,
        help="maximum Cartesian step for hover_path direct execution; larger is faster but less conservative",
    )
    ap.add_argument(
        "--max-meridian-step-m",
        type=float,
        default=0.08,
        help="drop robot-space meridian samples after product transform when adjacent points jump farther than this; set <=0 to disable",
    )
    ap.add_argument(
        "--trim-meridian-ends",
        type=int,
        default=0,
        help="drop this many selected meridian samples from each end before planning",
    )
    ap.add_argument(
        "--robot-offset-mm",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.0],
        metavar=("DX", "DY", "DZ"),
        help="manual robot-frame correction applied to transformed meridian points before planning",
    )
    ap.add_argument(
        "--product-normal-axis",
        choices=("local_x", "local_y", "local_z", "neg_local_x", "neg_local_y", "neg_local_z"),
        default="local_z",
        help="product pose axis used as the body normal for hover/press offsets",
    )
    ap.add_argument(
        "--keep-current-orientation",
        action="store_true",
        help="for hover_path, keep the current TCP orientation while traversing hover positions",
    )
    ap.add_argument(
        "--start-nearest",
        action="store_true",
        help="reverse the selected meridian if its last sampled point is closer to the current anchor pose",
    )
    ap.add_argument(
        "--hover-entry-motion",
        choices=("movel", "movej_p"),
        default="movel",
        help="motion primitive used only for entering the first hover point in direct hover_path mode",
    )
    ap.add_argument(
        "--fixed-first-normal",
        action="store_true",
        help="use the first sampled surface normal and tool RPY for all custom side-lying hover/touch points",
    )
    ap.add_argument(
        "--project-press-to-horizontal",
        action="store_true",
        help="project custom side-lying contact motion onto Base-XY so probing never moves upward/downward",
    )
    ap.add_argument("--execution-work-frame", default="Base", help="work frame to use before executing arm motion")
    ap.add_argument("--install-ang", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                    metavar=("RX", "RY", "RZ"), help="robot install angles for calc_poses")
    ap.add_argument("--dian-jin-dwell-s", type=float, default=0.5)
    ap.add_argument("--fen-jin-dwell-s", type=float, default=0.3)
    ap.add_argument("--shun-jin-dwell-s", type=float, default=0.0)
    ap.add_argument("--target-force-n", type=float, default=2.0, help="touch_probe target force delta")
    ap.add_argument("--max-force-n", type=float, default=6.0, help="touch_probe abort force delta")
    ap.add_argument("--touch-step-mm", type=float, default=2.0, help="touch_probe step along body normal")
    ap.add_argument("--max-press-mm", type=float, default=10.0, help="touch_probe extra travel after visual surface")
    ap.add_argument(
        "--contact-motion-axis",
        default="",
        help="tool-frame axis used for touch_probe approach; empty means same as the contact/orientation axis",
    )
    ap.add_argument(
        "--probe-depth-mm",
        type=float,
        default=0.0,
        help="override touch_probe total probing distance from hover; <=0 uses hover distance plus max press",
    )
    ap.add_argument("--touch-dwell-s", type=float, default=0.2, help="touch_probe dwell after target force is reached")
    ap.add_argument(
        "--temperature-c",
        type=float,
        default=0.0,
        help="optional massage-head temperature setpoint; <=0 leaves temperature unchanged",
    )
    ap.add_argument("--temperature-wait-s", type=float, default=0.0, help="wait for temperature to approach target")
    ap.add_argument("--temperature-tolerance-c", type=float, default=2.0, help="temperature wait tolerance")
    ap.add_argument(
        "--execution-mode",
        choices=("massage", "hover_path", "touch_probe"),
        default="massage",
        help="robot execution mode when --run is set",
    )
    ap.add_argument("--run", action="store_true", help="actually move the arm; otherwise preview only")
    ap.add_argument("--reuse-capture", default="",
                    help="skip docker capture step and reuse the given capture prefix "
                         "(e.g. rm_demo_output/capture_20260421_...)")
    return ap.parse_args()


def docker_group_prefix() -> list[str]:
    """Return the command prefix that ensures the docker group is active.

    When the Cursor shell was opened before `franka` joined the docker group,
    plain `docker` still fails with "permission denied". `sg docker -c ...`
    forces the group, and is a no-op otherwise.
    """
    try:
        subprocess.run(["docker", "version"], check=True, capture_output=True, timeout=3)
        return []
    except Exception:
        return ["sg", "docker", "-c"]


def run_in_docker(container: str, env: dict[str, str], stdin_text: str, timeout_s: float) -> str:
    """Run `python3 -` inside the container with ROS sourced, feeding stdin_text.

    Returns combined stdout. Raises on non-zero exit.
    """
    env_args = []
    for k, v in env.items():
        env_args += ["-e", f"{k}={v}"]
    inner_cmd = (
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; "
        "python3 -"
    )
    argv = (
        ["docker", "exec", "-i"] + env_args + [container, "bash", "-lc", inner_cmd]
    )
    prefix = docker_group_prefix()
    if prefix:
        argv = prefix + [" ".join(shlex.quote(a) for a in argv)]

    proc = subprocess.run(
        argv,
        input=stdin_text,
        text=True,
        capture_output=True,
        timeout=timeout_s,
        check=False,
    )
    if proc.returncode != 0:
        if proc.stdout.strip():
            sys.stderr.write("[docker stdout]\n" + proc.stdout + "\n")
        sys.stderr.write("[docker stderr]\n" + (proc.stderr or "") + "\n")
        raise RuntimeError(f"docker exec failed (rc={proc.returncode})")
    if proc.stderr.strip():
        sys.stderr.write("[docker stderr]\n" + proc.stderr + "\n")
    return proc.stdout


def trajectory_config_path(args: argparse.Namespace) -> Path:
    if getattr(args, "trajectory_config", ""):
        return Path(args.trajectory_config).resolve()
    matrix_path = Path(args.matrix_path)
    if matrix_path.suffix.lower() in (".yaml", ".yml"):
        return matrix_path.resolve()
    return Path(DEFAULT_TRAJECTORY_CONFIG).resolve()


def _safe_section_name(section: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in str(section))


def _load_json_file(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise RuntimeError(f"expected JSON object: {path}")
    return data


def _pick_capture_arm_state(data: dict) -> dict | None:
    for key in ("arm_state_after", "arm_state_before"):
        state = data.get(key)
        if not isinstance(state, dict):
            continue
        if state.get("error"):
            continue
        joints = state.get("joints_deg")
        pose = state.get("pose_m_rad")
        if isinstance(joints, list) and len(joints) >= 6 and isinstance(pose, list) and len(pose) >= 6:
            return {
                "source": key,
                "joints_deg": [float(v) for v in joints[:6]],
                "pose_m_rad": [float(v) for v in pose[:6]],
            }
    return None


def load_capture_pose_context(args: argparse.Namespace, capture_info: dict[str, str]) -> dict[str, object]:
    candidates: list[tuple[str, Path]] = []
    arm_state_path = str(capture_info.get("arm_state_path", "") or "").strip()
    if arm_state_path:
        candidates.append(("capture_arm_state", Path(arm_state_path)))
    rgb_path = str(capture_info.get("rgb_path", "") or "").strip()
    if rgb_path.endswith("_rgb.png"):
        candidates.append(("capture_arm_state", Path(rgb_path[:-8] + "_arm_state.json")))
    section = str(args.capture_prepare_section)
    candidates.append(
        (
            "current_capture_pose",
            Path(args.output_dir).resolve() / f"current_capture_pose_{_safe_section_name(section)}.json",
        )
    )

    seen: set[Path] = set()
    for source_name, path in candidates:
        path = path.resolve()
        if path in seen or not path.is_file():
            continue
        seen.add(path)
        try:
            data = _load_json_file(path)
        except Exception as exc:
            print(f"[warn] capture pose context unreadable: {path}: {type(exc).__name__}: {exc}")
            continue
        state = _pick_capture_arm_state(data)
        if state is None:
            joints = data.get("joints_deg")
            pose = data.get("pose_m_rad")
            if isinstance(joints, list) and len(joints) >= 6 and isinstance(pose, list) and len(pose) >= 6:
                state = {
                    "source": source_name,
                    "joints_deg": [float(v) for v in joints[:6]],
                    "pose_m_rad": [float(v) for v in pose[:6]],
                }
        if state is None:
            continue
        state["path"] = str(path)
        return state
    return {}


# ---------------------------------------------------------------------------
# Step 1: capture in docker
# ---------------------------------------------------------------------------
CAPTURE_PY = r"""
import json, os, sys, time
sys.path.insert(0, '/home/franka/massage')
sys.path.insert(0, '/home/franka/massage/robots/realman/ros_vendor/python')

from rm_demo.rm_capture import _capture_single_frame_via_ros
from rm_demo.rm_ros import create_arm_backend

out_dir = os.environ['OUTPUT_DIR']
os.makedirs(out_dir, exist_ok=True)

def read_arm_state():
    host = os.environ.get('ARM_HOST', '').strip()
    if not host:
        return None
    try:
        arm = create_arm_backend('ros')
        joints, pose, arm_err, sys_err, inverse_km_err = arm.get_current_arm_state(
            host=host,
            timeout=float(os.environ.get('ARM_STATE_TIMEOUT', '4.0')),
        )
        return {
            'host': host,
            'joints_deg': [float(v) for v in joints[:6]],
            'pose_m_rad': [float(v) for v in pose[:6]],
            'arm_err': int(arm_err),
            'sys_err': int(sys_err),
            'inverse_km_err': int(inverse_km_err),
        }
    except Exception as exc:
        return {'error': type(exc).__name__ + ': ' + str(exc)}

arm_state_before = read_arm_state()
frame = _capture_single_frame_via_ros(out_dir)
arm_state_after = read_arm_state()
arm_state_path = os.path.join(out_dir, 'capture_%s_arm_state.json' % frame.timestamp)
with open(arm_state_path, 'w', encoding='utf-8') as f:
    json.dump({
        'timestamp': frame.timestamp,
        'rgb_path': frame.color_path,
        'depth_path': frame.depth_path,
        'intrinsics_path': frame.intrinsics_path,
        'arm_state_before': arm_state_before,
        'arm_state_after': arm_state_after,
    }, f, ensure_ascii=False, indent=2)
print('##CAPTURE_JSON##')
print(json.dumps({
    'rgb_path': frame.color_path,
    'depth_path': frame.depth_path,
    'intrinsics_path': frame.intrinsics_path,
    'timestamp': frame.timestamp,
    'arm_state_path': arm_state_path,
}, ensure_ascii=False))
"""


SAVE_CURRENT_CAPTURE_POSE_PY = r"""
import json, os, sys
sys.path.insert(0, '/home/franka/massage')
sys.path.insert(0, '/home/franka/massage/robots/realman/ros_vendor/python')

from rm_demo.rm_positioning import save_current_arm_as_prepare_pose

metadata_path = os.environ.get('CAPTURE_POSE_METADATA', '').strip() or None
result = save_current_arm_as_prepare_pose(
    host=os.environ.get('ARM_HOST', ''),
    config_path=os.environ['TRAJECTORY_CONFIG'],
    section_name=os.environ.get('CAPTURE_PREPARE_SECTION', 'arm_massage_prepare'),
    control_backend='ros',
    timeout=float(os.environ.get('ARM_STATE_TIMEOUT', '4.0')),
    metadata_path=metadata_path,
)
print('##SAVE_CAPTURE_POSE_JSON##')
print(json.dumps(result, ensure_ascii=False))
"""


def save_current_capture_pose(args: argparse.Namespace) -> dict:
    section = str(args.capture_prepare_section)
    safe_section = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in section)
    metadata_path = Path(args.output_dir).resolve() / f"current_capture_pose_{safe_section}.json"
    config_path = trajectory_config_path(args)
    env = {
        "ROS_MASTER_URI": args.master_uri,
        "ROS_IP": args.ros_ip,
        "PYTHONPATH": str(DEFAULT_VENDOR_PY),
        "ARM_HOST": args.arm_host,
        "TRAJECTORY_CONFIG": str(config_path),
        "CAPTURE_PREPARE_SECTION": section,
        "CAPTURE_POSE_METADATA": str(metadata_path),
        "ARM_STATE_TIMEOUT": "4.0",
    }
    print(f"[teach] saving current arm pose -> {config_path}:{section}")
    stdout = run_in_docker(args.container, env, SAVE_CURRENT_CAPTURE_POSE_PY, timeout_s=20.0)
    marker = "##SAVE_CAPTURE_POSE_JSON##"
    if marker not in stdout:
        sys.stderr.write(stdout)
        raise RuntimeError("save current capture pose: missing SAVE_CAPTURE_POSE_JSON marker")
    payload = stdout.rsplit(marker, 1)[-1].strip().splitlines()[-1]
    result = json.loads(payload)
    print("[teach] saved capture joints_deg=" + json.dumps(result.get("joints_deg", []), ensure_ascii=False))
    print("[teach] saved capture tcp_pose_m_rad=" + json.dumps(result.get("pose_m_rad", []), ensure_ascii=False))
    print("[teach] capture section=" + str(result.get("section_name", section)))
    if result.get("metadata_path"):
        print("[teach] metadata=" + str(result["metadata_path"]))
    if result.get("backup_path"):
        print("[teach] backup=" + str(result["backup_path"]))
    return result


def capture_frame(args: argparse.Namespace) -> dict[str, str]:
    env = {
        "ROS_MASTER_URI": args.master_uri,
        "ROS_IP": args.ros_ip,
        "OUTPUT_DIR": str(Path(args.output_dir).resolve()),
        "PYTHONPATH": str(DEFAULT_VENDOR_PY),
        "ARM_HOST": str(args.arm_host),
        "ARM_STATE_TIMEOUT": "4.0",
    }
    print(f"[step1] capturing aligned RGB-D from board via docker -> master={args.master_uri}")
    t0 = time.time()
    stdout = run_in_docker(args.container, env, CAPTURE_PY, timeout_s=30.0)
    print(f"[step1] docker capture done in {time.time() - t0:.1f}s")
    marker = "##CAPTURE_JSON##"
    if marker not in stdout:
        sys.stderr.write(stdout)
        raise RuntimeError("capture: missing CAPTURE_JSON marker in docker output")
    payload = stdout.rsplit(marker, 1)[-1].strip().splitlines()[-1]
    info = json.loads(payload)
    print(f"[step1] rgb={info['rgb_path']}")
    print(f"[step1] depth={info['depth_path']}")
    print(f"[step1] intr={info['intrinsics_path']}")
    if info.get("arm_state_path"):
        print(f"[step1] arm_state={info['arm_state_path']}")
    return info


POSITION_PRODUCT_FLOW_PY = r"""
import json, os, sys
sys.path.insert(0, '/home/franka/massage')
sys.path.insert(0, '/home/franka/massage/robots/realman/ros_vendor/python')

from rm_demo.rm_positioning import position_for_capture

capture_joints_text = os.environ.get('CAPTURE_JOINTS', '').strip()
capture_joints = [float(v) for v in capture_joints_text.split()] if capture_joints_text else None
events = position_for_capture(
    host=os.environ['ARM_HOST'],
    mode=os.environ.get('PRODUCT_FLOW_POSITIONING', 'prepare_then_service'),
    trajectory_config=os.environ['TRAJECTORY_CONFIG'],
    speed=float(os.environ.get('POSITION_SPEED', '5')),
    tool_name_camera=os.environ.get('CAMERA_TOOL_NAME', 'camera'),
    tool_name_restore=os.environ.get('MASSAGE_TOOL_NAME', 'mas_rub'),
    shifting_number=int(os.environ.get('SHIFTING_NUMBER', '0')),
    control_backend='json',
    prepare_section=os.environ.get('CAPTURE_PREPARE_SECTION', 'arm_massage_prepare'),
    prepare_joints=capture_joints,
)
print('##POSITION_JSON##')
print(json.dumps(events, ensure_ascii=False))
"""


def run_product_positioning(args: argparse.Namespace) -> list[dict]:
    config_path = trajectory_config_path(args)
    env = {
        "ROS_MASTER_URI": args.master_uri,
        "ROS_IP": args.ros_ip,
        "PYTHONPATH": str(DEFAULT_VENDOR_PY),
        "ARM_HOST": args.arm_host,
        "TRAJECTORY_CONFIG": str(config_path),
        "POSITION_SPEED": f"{args.position_speed}",
        "CAMERA_TOOL_NAME": str(args.camera_tool_name),
        "MASSAGE_TOOL_NAME": str(args.massage_tool_name),
        "SHIFTING_NUMBER": str(args.shifting_number),
        "PRODUCT_FLOW_POSITIONING": str(args.product_flow_positioning),
        "CAPTURE_PREPARE_SECTION": str(args.capture_prepare_section),
        "CAPTURE_JOINTS": "" if args.capture_joints is None else " ".join(str(v) for v in args.capture_joints),
    }
    print(
        "[step0] product flow positioning: "
        f"prepare[{args.capture_prepare_section}] -> "
        f"{args.product_flow_positioning} config={config_path}"
    )
    stdout = run_in_docker(args.container, env, POSITION_PRODUCT_FLOW_PY, timeout_s=180.0)
    marker = "##POSITION_JSON##"
    if marker not in stdout:
        sys.stderr.write(stdout)
        raise RuntimeError("product positioning: missing POSITION_JSON marker")
    payload = stdout.rsplit(marker, 1)[-1].strip().splitlines()[-1]
    events = json.loads(payload)
    print(textwrap.indent(stdout.rstrip(), "  "))
    return events


# ---------------------------------------------------------------------------
# Step 2: detect on host
# ---------------------------------------------------------------------------
def detect_on_host(args: argparse.Namespace, capture_info: dict[str, str]) -> tuple[str, dict]:
    import cv2
    import numpy as np

    rgb_path = capture_info["rgb_path"]
    depth_path = capture_info["depth_path"]
    intrinsics_path = capture_info["intrinsics_path"]

    print(f"[step2] host YOLO pose detection ({args.model_path})")
    color_bgr = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
    if color_bgr is None:
        raise FileNotFoundError(f"rgb not readable: {rgb_path}")
    depth_m = np.load(depth_path)
    with open(intrinsics_path, "r", encoding="utf-8") as f:
        intrinsics = json.load(f)

    t0 = time.time()
    detect_result, overlay = detect_bladder_lines(
        color_bgr=color_bgr,
        depth_m=depth_m,
        intrinsics_data=intrinsics,
        finger_width_mm=args.finger_width_mm,
        model_path=args.model_path,
        sample_points=args.sample_points,
    )
    print(f"[step2] detection done in {time.time() - t0:.1f}s")
    detect_result["capture"] = {
        "rgb_path": os.path.abspath(rgb_path),
        "depth_path": os.path.abspath(depth_path),
        "intrinsics_path": os.path.abspath(intrinsics_path),
    }
    if args.auto_top_line:
        detect_result = select_topmost_bladder_line(detect_result)
        print(
            "[step2] auto top line selected: "
            f"{detect_result.get('selected_side')} {detect_result.get('selected_line_type')} "
            f"reason={detect_result.get('selected_line_reason')}"
        )
    else:
        detect_result = select_bladder_line(detect_result, args.side, args.line_type)
    prefix = f"bladder_demo_{detect_result['timestamp']}"
    overlay_path, detect_json_path = save_bladder_artifacts(
        args.output_dir, detect_result, overlay, prefix=prefix
    )
    print(f"[step2] overlay={overlay_path}")
    print(f"[step2] detect_json={detect_json_path}")
    return detect_json_path, detect_result


# ---------------------------------------------------------------------------
# Step 3: transform + plan + (optional) execute in docker
# ---------------------------------------------------------------------------
PLAN_RUN_PY = r"""
import json, math, os, sys, time
sys.path.insert(0, '/home/franka/massage')
sys.path.insert(0, '/home/franka/massage/robots/realman/ros_vendor/python')

import cv2
import numpy as np

from rm_demo.rm_product_ros import attach_robot_points_via_product_services
from rm_demo.rm_product_executor import upload_product_trajectory
from rm_demo.rm_product_trajectory import generate_rubbing_trajectory
from rm_demo.rm_ros import create_arm_backend
from rm_demo.rm_side_lying import (
    correct_product_trajectory_for_side_lying,
    validate_product_trajectory_side_lying,
)
from rm_demo.rm_temperature import set_temperature
from rm_demo.rm_bladder import (
    attach_selected_robot_points_eye_in_hand,
    attach_selected_robot_points_static,
    filter_selected_meridian_continuity,
    offset_selected_meridian_robot,
    rebuild_plan_with_fixed_first_normal,
    rebuild_plan_with_horizontal_press,
    trim_selected_meridian_ends,
    build_aligned_contact_preview,
    build_bladder_massage_plan, bladder_plan_to_dict,
    preview_bladder_plan, execute_bladder_plan, execute_bladder_hover_path,
    execute_bladder_touch_probe_plan,
)
from rm_demo.rm_transform import load_transform_matrix


def build_pose_check_trajectory(trajectory_content):
    # Keep only the product-generated hover pose needed for tool-orientation inspection.
    keep_lines = []
    for raw_line in trajectory_content.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        obj = json.loads(line)
        if 'file' in obj:
            keep_lines.append(json.dumps(obj, ensure_ascii=False, separators=(',', ':')))
            continue
        name = str(obj.get('name', ''))
        num = int(obj.get('num', -1))
        if name in {'Stop_Force', 'MOVEJ', 'prepare', 'p1_above_2cm'}:
            keep_lines.append(json.dumps(obj, ensure_ascii=False, separators=(',', ':')))
        elif name == 'MOVEL' and num == 4:
            keep_lines.append(json.dumps(obj, ensure_ascii=False, separators=(',', ':')))
    if not any('"p1_above_2cm"' in line for line in keep_lines):
        raise RuntimeError('product trajectory does not contain p1_above_2cm for pose_check')
    if any('"Force"' in line for line in keep_lines if '"Stop_Force"' not in line):
        raise RuntimeError('pose_check trajectory unexpectedly contains Force command')
    return '\n'.join(keep_lines) + '\n'


def extract_pose_check_targets(trajectory_content):
    prepare_joint = None
    p1_above_pose = None
    for raw_line in trajectory_content.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        obj = json.loads(line)
        name = str(obj.get('name', ''))
        if name == 'prepare':
            joint = list(obj.get('joint', []))
            if len(joint) >= 6:
                prepare_joint = [float(v) / 1000.0 for v in joint[:6]]
        elif name == 'p1_above_2cm':
            pose = list(obj.get('pose', []))
            if len(pose) >= 6:
                p1_above_pose = [float(v) / 1000000.0 for v in pose[:3]] + [float(v) / 1000.0 for v in pose[3:6]]
    if prepare_joint is None:
        raise RuntimeError('pose_check could not find product prepare joint target')
    if p1_above_pose is None:
        raise RuntimeError('pose_check could not find product p1_above_2cm pose target')
    return prepare_joint, p1_above_pose


def _axis_vector(axis_name):
    axes = {
        'pos_x': [1.0, 0.0, 0.0],
        'neg_x': [-1.0, 0.0, 0.0],
        'pos_y': [0.0, 1.0, 0.0],
        'neg_y': [0.0, -1.0, 0.0],
        'pos_z': [0.0, 0.0, 1.0],
        'neg_z': [0.0, 0.0, -1.0],
    }
    if axis_name not in axes:
        raise RuntimeError('unsupported tool_contact_axis: ' + str(axis_name))
    return axes[axis_name]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return _matmul(_matmul(rz, ry), rx)


def _matrix_to_rpy(m):
    sy = -float(m[2][0])
    sy = max(-1.0, min(1.0, sy))
    p = math.asin(sy)
    cp = math.cos(p)
    if abs(cp) > 1e-6:
        r = math.atan2(m[2][1], m[2][2])
        y = math.atan2(m[1][0], m[0][0])
    else:
        r = 0.0
        y = math.atan2(-m[0][1], m[1][1])
    return [r, p, y]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _dot(a, b):
    return sum(float(x) * float(y) for x, y in zip(a, b))


def _norm(v):
    return math.sqrt(max(0.0, _dot(v, v)))


def _normalize(v):
    n = _norm(v)
    if n < 1e-9:
        raise RuntimeError('zero vector in contact-axis correction')
    return [float(x) / n for x in v]


def _align_vector_rotation(source, target):
    source = _normalize(source)
    target = _normalize(target)
    v = _cross(source, target)
    c = max(-1.0, min(1.0, _dot(source, target)))
    s = _norm(v)
    if s < 1e-9:
        if c > 0.0:
            return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        helper = [1.0, 0.0, 0.0] if abs(source[0]) < 0.9 else [0.0, 1.0, 0.0]
        v = _normalize(_cross(source, helper))
        s = 1.0
        c = -1.0
    vx = [[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]
    vx2 = _matmul(vx, vx)
    factor = (1.0 - c) / (s * s)
    return [
        [
            (1.0 if i == j else 0.0) + vx[i][j] + vx2[i][j] * factor
            for j in range(3)
        ]
        for i in range(3)
    ]


def correct_tool_contact_axis_in_trajectory(trajectory_content, contact_axis):
    if contact_axis == 'neg_z':
        return trajectory_content, 0
    line_sep = '\r\n' if '\r\n' in trajectory_content else '\n'
    # Product-generated poses assume the tool -Z axis is the contact normal.
    # Rotate each pose so the configured physical contact axis points where
    # product -Z originally pointed.
    correction = _align_vector_rotation(_axis_vector(contact_axis), [0.0, 0.0, -1.0])
    out_lines = []
    changed = 0
    for raw_line in trajectory_content.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        obj = json.loads(line)
        pose = obj.get('pose')
        joint = obj.get('joint')
        if isinstance(pose, list) and len(pose) >= 6 and not any(float(v) for v in list(joint or [])[:6]):
            rpy = [float(v) / 1000.0 for v in pose[3:6]]
            rot = _rpy_to_matrix(rpy[0], rpy[1], rpy[2])
            corrected = _matmul(rot, correction)
            new_rpy = _matrix_to_rpy(corrected)
            obj['pose'] = list(pose[:3]) + [int(round(v * 1000.0)) for v in new_rpy]
            changed += 1
        out_lines.append(json.dumps(obj, ensure_ascii=False, separators=(',', ':')))
    return line_sep.join(out_lines) + line_sep, changed


detect_json = os.environ['DETECT_JSON']
with open(detect_json, 'r', encoding='utf-8') as f:
    detect_result = json.load(f)

rgb_path = detect_result['capture']['rgb_path']
depth_path = detect_result['capture']['depth_path']
color_bgr = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
if color_bgr is None:
    raise FileNotFoundError('rgb not readable: ' + rgb_path)
depth_m = np.load(depth_path)

host = os.environ['ARM_HOST']
install_ang = [float(v) for v in os.environ.get('INSTALL_ANG', '0 0 0').split()]
calc_poses_joints_text = os.environ.get('CALC_POSES_JOINTS', '').strip()
calc_poses_joints = [float(v) for v in calc_poses_joints_text.split()] if calc_poses_joints_text else None
calc_poses_joints_source = os.environ.get('CALC_POSES_JOINTS_SOURCE', 'current_arm_state')
transform_backend = os.environ.get('TRANSFORM_BACKEND', 'static').strip().lower()
matrix_path = os.environ.get('MATRIX_PATH', '').strip()
camera_tool_name = os.environ.get('CAMERA_TOOL_NAME', 'camera').strip()
restore_tool_name = os.environ.get('RESTORE_TOOL_NAME', camera_tool_name).strip()
side = os.environ.get('SIDE', 'left')
line_type = os.environ.get('LINE_TYPE', 'outer')
plan_points = int(os.environ.get('PLAN_POINTS', '6'))
hover_mm = float(os.environ.get('HOVER_MM', '20'))
hover_offset_mode = os.environ.get('HOVER_OFFSET_MODE', 'normal').strip().lower()
dian_jin_depth_mm = float(os.environ.get('DIAN_JIN_DEPTH_MM', '10'))
fen_jin_lateral_mm = float(os.environ.get('FEN_JIN_LATERAL_MM', '20'))
safe_lift_mm = float(os.environ.get('SAFE_LIFT_MM', '40'))
speed = float(os.environ.get('SPEED', '0.3'))
max_step_m = float(os.environ.get('MAX_STEP_M', '0.05'))
max_meridian_step_m = float(os.environ.get('MAX_MERIDIAN_STEP_M', '0.08'))
trim_meridian_ends = int(os.environ.get('TRIM_MERIDIAN_ENDS', '0'))
robot_offset_m = [float(v) / 1000.0 for v in os.environ.get('ROBOT_OFFSET_MM', '0 0 0').split()[:3]]
while len(robot_offset_m) < 3:
    robot_offset_m.append(0.0)
product_normal_axis = os.environ.get('PRODUCT_NORMAL_AXIS', 'local_x').strip().lower()
keep_current_orientation = os.environ.get('KEEP_CURRENT_ORIENTATION', '0') == '1'
start_nearest = os.environ.get('START_NEAREST', '0') == '1'
hover_entry_motion = os.environ.get('HOVER_ENTRY_MOTION', 'movel').strip().lower()
fixed_first_normal = os.environ.get('FIXED_FIRST_NORMAL', '0') == '1'
project_press_to_horizontal = os.environ.get('PROJECT_PRESS_TO_HORIZONTAL', '0') == '1'
execution_work_frame = os.environ.get('EXECUTION_WORK_FRAME', 'Base').strip()
massage_tool_name = os.environ.get('MASSAGE_TOOL_NAME', 'mas_rub').strip()
product_trajectory_mode = os.environ.get('PRODUCT_TRAJECTORY_MODE', 'generate_only').strip().lower()
allow_controller_write = os.environ.get('ALLOW_CONTROLLER_WRITE', '0') == '1'
product_force = int(os.environ.get('PRODUCT_FORCE', '2'))
product_speed = int(os.environ.get('PRODUCT_SPEED', '50'))
trajectory_type = int(os.environ.get('TRAJECTORY_TYPE', '4'))
side_lying_product_correction = os.environ.get('SIDE_LYING_PRODUCT_CORRECTION', '0') == '1'
product_down_3cm_m = float(os.environ.get('PRODUCT_DOWN_3CM_MM', '30.0')) / 1000.0
product_down_1cm_m = float(os.environ.get('PRODUCT_DOWN_1CM_MM', '10.0')) / 1000.0
tool_contact_axis = os.environ.get('TOOL_CONTACT_AXIS', 'neg_z').strip().lower()
legacy_tool_contact_axis = os.environ.get('LEGACY_TOOL_CONTACT_AXIS', '').strip().lower()
pose_check_move_prepare = os.environ.get('POSE_CHECK_MOVE_PREPARE', '0') == '1'
pose_check_entry_motion = os.environ.get('POSE_CHECK_ENTRY_MOTION', 'movej_p').strip().lower()
do_run = os.environ.get('DO_RUN', '0') == '1'
dian_jin_dwell_s = float(os.environ.get('DIAN_JIN_DWELL_S', '0.5'))
fen_jin_dwell_s = float(os.environ.get('FEN_JIN_DWELL_S', '0.3'))
shun_jin_dwell_s = float(os.environ.get('SHUN_JIN_DWELL_S', '0.0'))
target_force_n = float(os.environ.get('TARGET_FORCE_N', '2.0'))
max_force_n = float(os.environ.get('MAX_FORCE_N', '6.0'))
touch_step_m = float(os.environ.get('TOUCH_STEP_MM', '2.0')) / 1000.0
max_press_m = float(os.environ.get('MAX_PRESS_MM', '10.0')) / 1000.0
contact_motion_axis = os.environ.get('CONTACT_MOTION_AXIS', '').strip().lower()
probe_depth_m = float(os.environ.get('PROBE_DEPTH_MM', '0.0')) / 1000.0
touch_dwell_s = float(os.environ.get('TOUCH_DWELL_S', '0.2'))
temperature_c = float(os.environ.get('TEMPERATURE_C', '0.0'))
temperature_wait_s = float(os.environ.get('TEMPERATURE_WAIT_S', '0.0'))
temperature_tolerance_c = float(os.environ.get('TEMPERATURE_TOLERANCE_C', '2.0'))
execution_mode = os.environ.get('EXECUTION_MODE', 'massage').strip().lower()

arm = create_arm_backend('ros')
restore_tool_after = ''
try:
    if transform_backend == 'static':
        print('[step3] switching to World + camera tool for static eye-in-hand transform')
        arm.change_work_frame('World')
        arm.change_tool(camera_tool_name)
        restore_tool_after = restore_tool_name
    current_joints, current_pose, arm_err, sys_err, _ = arm.get_current_arm_state(host)
    anchor_pose_override_text = os.environ.get('ANCHOR_POSE_M_RAD', '').strip()
    anchor_pose_source = os.environ.get('ANCHOR_POSE_SOURCE', 'current_arm_state').strip() or 'current_arm_state'
    if anchor_pose_override_text:
        anchor_pose_override = [float(v) for v in anchor_pose_override_text.split()[:6]]
        if len(anchor_pose_override) == 6:
            current_pose = anchor_pose_override
        else:
            raise RuntimeError('ANCHOR_POSE_M_RAD must contain 6 values')
    print('[step3] anchor_pose=' + str([round(v, 6) for v in current_pose]) +
          ' source=' + str(anchor_pose_source) +
          ' arm_err=' + str(arm_err) + ' sys_err=' + str(sys_err))
    transform_joints = list(calc_poses_joints or current_joints)
    print(
        '[step3] calc_poses_joints_source=%s joints=%s'
        % (calc_poses_joints_source if calc_poses_joints else 'current_arm_state',
           str([round(float(v), 6) for v in transform_joints[:6]]))
    )

    t0 = time.time()
    if transform_backend == 'static':
        matrix = load_transform_matrix(matrix_path)
        if matrix is None:
            raise RuntimeError('static transform matrix not found or invalid: ' + matrix_path)
        if matrix_path.lower().endswith(('.yaml', '.yml')):
            detect_result = attach_selected_robot_points_eye_in_hand(
                detect_result,
                tool_from_camera_matrix=matrix,
                tool_pose_m=current_pose,
            )
        else:
            detect_result = attach_selected_robot_points_static(detect_result, matrix)
        detect_result['static_matrix_path'] = matrix_path
        print('[step3] static matrix transform done in %.1fs path=%s' % (time.time() - t0, matrix_path))
    else:
        detect_result = attach_robot_points_via_product_services(
            color_bgr=color_bgr,
            depth_m=depth_m,
            detection_result=detect_result,
            host=host,
            install_ang=install_ang,
            control_backend='ros',
            joints_deg_override=transform_joints,
        )
        print('[step3] product ROS transform done in %.1fs' % (time.time() - t0))

    before_filter_count = len(list(detect_result.get('selected_meridian_robot', [])))
    detect_result = filter_selected_meridian_continuity(
        detect_result,
        max_step_m=max_meridian_step_m,
    )
    after_filter_count = len(list(detect_result.get('selected_meridian_robot', [])))
    if after_filter_count != before_filter_count:
        print(
            '[step3] meridian continuity filter: kept %d/%d removed=%s max_step_m=%.3f'
            % (
                after_filter_count,
                before_filter_count,
                detect_result.get('selected_meridian_filter', {}).get('removed_indices_1based', []),
                max_meridian_step_m,
            )
        )
    before_trim_count = len(list(detect_result.get('selected_meridian_robot', [])))
    detect_result = trim_selected_meridian_ends(
        detect_result,
        trim_count=trim_meridian_ends,
    )
    after_trim_count = len(list(detect_result.get('selected_meridian_robot', [])))
    if after_trim_count != before_trim_count:
        print(
            '[step3] meridian end trim: kept %d/%d trim_each_end=%d'
            % (after_trim_count, before_trim_count, trim_meridian_ends)
        )
    detect_result = offset_selected_meridian_robot(
        detect_result,
        offset_m=robot_offset_m,
    )
    if any(abs(v) > 1e-12 for v in robot_offset_m):
        print(
            '[step3] meridian robot offset applied m=%s'
            % str([round(float(v), 6) for v in robot_offset_m])
        )

    out_dir = os.path.dirname(detect_json)
    prefix = os.path.splitext(os.path.basename(detect_json))[0].replace('_detect', '')
    transform_json = os.path.join(out_dir, prefix + '_transform.json')
    with open(transform_json, 'w', encoding='utf-8') as f:
        json.dump(detect_result, f, ensure_ascii=False, indent=2)
    print('[step3] transform_json=' + transform_json)

    if transform_backend == 'product_ros' and product_trajectory_mode != 'legacy_movel':
        product_points = list(detect_result.get('product_camera_waypoints', []))
        product_vectors = list(detect_result.get('product_camera_vectors', []))
        if len(product_points) < 2 or len(product_vectors) < 2:
            raise RuntimeError('product camera waypoint/vector data missing before rubbing trajectory generation')
        print(
            '[step3] generating native product rubbing trajectory: '
            'tool=%s force=%d speed=%d trajectory_type=%d waypoints=%d'
            % (massage_tool_name, product_force, product_speed, trajectory_type, len(product_points))
        )
        rubbing = generate_rubbing_trajectory(
            joints_deg=list(transform_joints),
            install_ang=install_ang,
            tool_name=massage_tool_name,
            waypoint_points=product_points,
            waypoint_vectors=product_vectors,
            force=product_force,
            speed=product_speed,
            trajectory_type=trajectory_type,
        )
        side_lying_correction = None
        side_lying_validation = None
        trajectory_content = rubbing.trajectory_content
        if side_lying_product_correction:
            trajectory_content, side_lying_correction = correct_product_trajectory_for_side_lying(
                trajectory_content=trajectory_content,
                selected_pose_quat=list(detect_result.get('selected_meridian_robot_pose_quat', [])),
                anchor_pose_m=current_pose,
                product_normal_axis=product_normal_axis,
                hover_m=hover_mm / 1000.0,
                tool_contact_axis=tool_contact_axis,
                prepare_joints_deg=list(transform_joints),
                down_3cm_m=product_down_3cm_m,
                down_1cm_m=product_down_1cm_m,
                fixed_first_normal=fixed_first_normal,
                project_press_to_horizontal=project_press_to_horizontal,
            )
            side_lying_validation = validate_product_trajectory_side_lying(
                trajectory_content=trajectory_content,
                tool_contact_axis=tool_contact_axis,
                prepare_joints_deg=list(transform_joints),
                require_horizontal_press=project_press_to_horizontal,
            )
            if not side_lying_validation.ok:
                raise RuntimeError(
                    'side-lying product trajectory validation failed: '
                    + '; '.join(side_lying_validation.messages)
                )
            pose_axis_correction_count = 0
        else:
            trajectory_content, pose_axis_correction_count = correct_tool_contact_axis_in_trajectory(
                trajectory_content,
                tool_contact_axis,
            )
        trajectory_artifact = 'product_trajectory'
        if product_trajectory_mode == 'pose_check':
            trajectory_content = build_pose_check_trajectory(trajectory_content)
            trajectory_artifact = 'product_pose_check_trajectory'

        trajectory_txt = os.path.join(out_dir, prefix + '_' + trajectory_artifact + '.txt')
        with open(trajectory_txt, 'w', encoding='utf-8') as f:
            f.write(trajectory_content)
            if trajectory_content and not trajectory_content.endswith('\n'):
                f.write('\n')
        metadata = rubbing.to_dict()
        metadata['trajectory_content'] = trajectory_content
        metadata.update({
            'transform_json': transform_json,
            'detect_json': detect_json,
            'rgb_path': rgb_path,
            'depth_path': depth_path,
            'product_trajectory_mode': product_trajectory_mode,
            'tool_contact_axis': tool_contact_axis,
            'pose_axis_correction_count': pose_axis_correction_count,
            'side_lying_product_correction': None if side_lying_correction is None else side_lying_correction.to_dict(),
            'side_lying_product_validation': None if side_lying_validation is None else side_lying_validation.to_dict(),
            'trajectory_artifact': trajectory_artifact,
            'source_trajectory_line_count': len(rubbing.trajectory_content.splitlines()),
            'trajectory_line_count': len(trajectory_content.splitlines()),
            'trajectory_preview_lines': trajectory_content.splitlines()[:12],
            'product_camera_waypoint_count': len(product_points),
            'run_requested': bool(do_run),
            'executed': False,
            'note': 'native product trajectory generated; execution depends on product_trajectory_mode',
        })
        product_json = os.path.join(out_dir, prefix + '_product_trajectory.json')
        with open(product_json, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, ensure_ascii=False, indent=2)
        print('[step3] product_trajectory_txt=' + trajectory_txt)
        print('[step3] product_trajectory_json=' + product_json)
        print('[step3] product_trajectory_lines=%d' % len(trajectory_content.splitlines()))
        for preview_idx, preview_line in enumerate(trajectory_content.splitlines()[:8], start=1):
            print('[step3] product_trajectory_preview_%02d=%s' % (preview_idx, preview_line[:240]))
        print('[step3] open_force_num_list=' + str(rubbing.open_force_num_list))
        print('[step3] stop_force_num_list=' + str(rubbing.stop_force_num_list))
        print('[step3] tool_contact_axis=%s corrected_poses=%d' % (tool_contact_axis, pose_axis_correction_count))
        if side_lying_correction is not None:
            print('[step3] side_lying_product_correction=' + str(side_lying_correction.to_dict()))
        if side_lying_validation is not None:
            print('[step3] side_lying_product_validation=' + str(side_lying_validation.to_dict()))
        if product_trajectory_mode == 'generate_only':
            if do_run:
                print('[step3] --run was requested, but generate_only never uploads or executes arm motion')
        elif product_trajectory_mode == 'upload_only':
            if not allow_controller_write:
                raise RuntimeError(
                    'controller write refused: upload_only requires --allow-controller-write'
                )
            upload = upload_product_trajectory(
                trajectory_content=trajectory_content,
                project_name=os.path.basename(trajectory_txt),
                plan_speed=product_speed,
                clear_existing=True,
                execute=False,
            )
            metadata['upload_result'] = upload.to_dict()
            with open(product_json, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, ensure_ascii=False, indent=2)
            print('[step3] product trajectory uploaded but not executed: ' + str(upload.to_dict()))
        elif product_trajectory_mode == 'pose_check':
            if not do_run:
                print('[step3] pose_check generated hover-only tool orientation trajectory; add --run to move to p1_above_2cm')
            else:
                prepare_joint, p1_above_pose = extract_pose_check_targets(trajectory_content)
                print('[step3] pose_check direct ROS motion; no Force/down/line trajectory will be executed')
                print('[step3] pose_check_prepare_joint=' + str(prepare_joint))
                print('[step3] pose_check_p1_above_pose=' + str(p1_above_pose))
                arm.change_work_frame(execution_work_frame)
                arm.change_tool(massage_tool_name)
                # Product ROS MoveJ/MoveL topics use fractional speed values
                # (the product AI scripts publish 0.15-0.2), not trajectory-file
                # speed values like 30/50.
                if pose_check_move_prepare:
                    arm.movej(host, prepare_joint, speed=0.2, timeout=60.0)
                else:
                    print('[step3] pose_check skipping product prepare joint; add --pose-check-move-prepare to enable it')
                if pose_check_entry_motion == 'movej_p':
                    if not hasattr(arm, 'movej_p'):
                        raise RuntimeError('selected arm backend does not support movej_p')
                    arm.movej_p(host, p1_above_pose, speed=0.15, timeout=60.0)
                elif pose_check_entry_motion == 'movel':
                    arm.movel(host, p1_above_pose, speed=0.15, timeout=60.0)
                else:
                    raise RuntimeError('unsupported pose_check_entry_motion: ' + str(pose_check_entry_motion))
                metadata['executed'] = True
                metadata['pose_check_direct_motion'] = {
                    'prepare_joint_deg': prepare_joint,
                    'p1_above_pose_m_rad': p1_above_pose,
                    'work_frame': execution_work_frame,
                    'tool_name': massage_tool_name,
                }
                metadata['note'] = 'pose_check: direct ROS motion to p1_above_2cm; no Force/down/line motion'
                with open(product_json, 'w', encoding='utf-8') as f:
                    json.dump(metadata, f, ensure_ascii=False, indent=2)
                print('[step3] pose_check reached p1_above_2cm')
        elif product_trajectory_mode == 'execute':
            if not do_run:
                print('[step3] execute mode requires --run; generated trajectory was not uploaded or executed')
            else:
                if not allow_controller_write:
                    raise RuntimeError(
                        'controller write refused: execute mode with --run requires --allow-controller-write'
                    )
                if temperature_c > 0.0:
                    temp_result = set_temperature(
                        target_c=temperature_c,
                        wait_s=temperature_wait_s,
                        tolerance_c=temperature_tolerance_c,
                    )
                    metadata['temperature_result'] = temp_result.to_dict()
                    print('[step3] temperature_result=' + str(temp_result.to_dict()))
                upload = upload_product_trajectory(
                    trajectory_content=trajectory_content,
                    project_name=os.path.basename(trajectory_txt),
                    plan_speed=product_speed,
                    clear_existing=True,
                    execute=True,
                )
                metadata['executed'] = True
                metadata['upload_result'] = upload.to_dict()
                with open(product_json, 'w', encoding='utf-8') as f:
                    json.dump(metadata, f, ensure_ascii=False, indent=2)
                print('[step3] product trajectory uploaded and execution requested: ' + str(upload.to_dict()))
        else:
            raise RuntimeError('unsupported product trajectory mode: ' + product_trajectory_mode)
    else:
        selected_points = list(detect_result.get('selected_meridian_robot', []))
        selected_pixels = list(detect_result.get('selected_meridian_pixel', []))
        if len(selected_points) < 2:
            raise RuntimeError('selected_meridian_robot insufficient (%d)' % len(selected_points))

        plan = build_bladder_massage_plan(
            side=side,
            line_type=line_type,
            meridian_points_robot_m=selected_points,
            meridian_pixels=selected_pixels,
            anchor_pose_m=current_pose,
            point_count=plan_points,
            hover_m=hover_mm / 1000.0,
            dian_jin_depth_m=dian_jin_depth_mm / 1000.0,
            fen_jin_lateral_m=fen_jin_lateral_mm / 1000.0,
            safe_lift_m=safe_lift_mm / 1000.0,
            meridian_pose_quat=list(detect_result.get('selected_meridian_robot_pose_quat', [])),
            product_normal_axis=product_normal_axis,
            tool_contact_axis=legacy_tool_contact_axis,
            start_nearest_anchor=start_nearest,
            hover_offset_mode=hover_offset_mode,
        )
        if fixed_first_normal:
            plan = rebuild_plan_with_fixed_first_normal(
                plan,
                tool_contact_axis=legacy_tool_contact_axis or tool_contact_axis,
            )
            print('[step3] fixed_first_normal applied: using point 1 press_direction/RPY for all plan frames')
        if project_press_to_horizontal:
            plan = rebuild_plan_with_horizontal_press(
                plan,
                tool_contact_axis=legacy_tool_contact_axis or tool_contact_axis,
            )
            print('[step3] project_press_to_horizontal applied: contact/hover offsets use Base-XY direction only')
        contact_axis_for_plan = legacy_tool_contact_axis or tool_contact_axis
        aligned_contact_preview = build_aligned_contact_preview(
            plan,
            tool_contact_axis=contact_axis_for_plan,
            contact_motion_axis=contact_motion_axis or None,
            max_press_m=max_press_m,
            touch_step_m=touch_step_m,
            probe_depth_m=probe_depth_m if probe_depth_m > 0.0 else None,
        )
        plan_data = bladder_plan_to_dict(plan)
        plan_data['aligned_contact_preview'] = aligned_contact_preview
        plan_json = os.path.join(out_dir, prefix + '_plan.json')
        with open(plan_json, 'w', encoding='utf-8') as f:
            json.dump(plan_data, f, ensure_ascii=False, indent=2)
        print('[step3] plan_json=' + plan_json)
        print(
            '[step3] aligned_contact_preview '
            + 'tool_axis=' + str(aligned_contact_preview['tool_contact_axis'])
            + ' motion_axis=' + str(aligned_contact_preview['contact_motion_axis'])
            + ' min_dot=' + ('%.4f' % float(aligned_contact_preview['min_tool_axis_dot_press']))
            + ' hover_m=' + ('%.4f' % float(aligned_contact_preview['hover_m']))
            + ' max_press_m=' + ('%.4f' % float(aligned_contact_preview['max_press_m']))
            + ' probe_depth_m=' + str(aligned_contact_preview['probe_depth_m'])
        )
        preview_bladder_plan(plan)

        if not do_run:
            print('[step3] preview only; rerun with --run to execute the massage sequence')
        else:
            if temperature_c > 0.0:
                temp_result = set_temperature(
                    target_c=temperature_c,
                    wait_s=temperature_wait_s,
                    tolerance_c=temperature_tolerance_c,
                )
                print('[step3] temperature_result=' + str(temp_result.to_dict()))
            if execution_work_frame:
                print('[step3] switching work frame=' + execution_work_frame)
                arm.change_work_frame(execution_work_frame)
            if massage_tool_name:
                print('[step3] switching massage tool=' + massage_tool_name)
                arm.change_tool(massage_tool_name)
            if execution_mode == 'hover_path':
                print('[step3] executing hover-only path traversal (no contact)')
                use_global_safe_z = transform_backend != 'product_ros'
                execute_bladder_hover_path(
                    host=host,
                    plan=plan,
                    speed=speed,
                    control_backend='ros',
                    dwell_s=shun_jin_dwell_s,
                    use_global_safe_z=use_global_safe_z,
                    keep_current_orientation=keep_current_orientation,
                    max_step_m=max_step_m,
                    entry_motion=hover_entry_motion,
                )
            elif execution_mode == 'touch_probe':
                print('[step3] executing aligned side-lying touch probe with force monitoring')
                execute_bladder_touch_probe_plan(
                    host=host,
                    plan=plan,
                    speed=speed,
                    control_backend='ros',
                    target_force_n=target_force_n,
                    max_force_n=max_force_n,
                    touch_step_m=touch_step_m,
                    max_press_m=max_press_m,
                    dwell_s=touch_dwell_s,
                    max_step_m=max_step_m,
                    keep_current_orientation=keep_current_orientation,
                    entry_motion='movej_p',
                    tool_contact_axis=contact_axis_for_plan,
                    contact_motion_axis=contact_motion_axis or None,
                    probe_depth_m=probe_depth_m if probe_depth_m > 0.0 else None,
                )
            else:
                print('[step3] executing point/split/smooth sequence')
                execute_bladder_plan(
                    host=host,
                    plan=plan,
                    speed=speed,
                    control_backend='ros',
                    dian_jin_dwell_s=dian_jin_dwell_s,
                    fen_jin_dwell_s=fen_jin_dwell_s,
                    shun_jin_dwell_s=shun_jin_dwell_s,
                )
            print('[step3] execute done')

finally:
    if restore_tool_after and restore_tool_after != camera_tool_name:
        print('[step3] restoring tool=' + restore_tool_after)
        arm.change_tool(restore_tool_after)
"""


def transform_and_run(args: argparse.Namespace, detect_json_path: str, capture_info: dict[str, str]) -> None:
    capture_context = load_capture_pose_context(args, capture_info)
    if capture_context:
        print(
            "[step3] capture_pose_context=%s path=%s"
            % (capture_context.get("source", ""), capture_context.get("path", ""))
        )
    calc_poses_joints = args.capture_joints
    calc_poses_joints_source = "cli_capture_joints" if calc_poses_joints is not None else ""
    if calc_poses_joints is None and capture_context.get("joints_deg"):
        calc_poses_joints = [float(v) for v in list(capture_context["joints_deg"])[:6]]
        calc_poses_joints_source = str(capture_context.get("source", "capture_pose_context"))
    if calc_poses_joints is None:
        calc_poses_joints = load_prepare_joints(
            str(trajectory_config_path(args)),
            section_name=str(args.capture_prepare_section),
        )
        calc_poses_joints_source = "capture_prepare_section" if calc_poses_joints is not None else "current_arm_state"
    anchor_pose_m_rad = ""
    if capture_context.get("pose_m_rad"):
        anchor_pose_m_rad = " ".join(str(float(v)) for v in list(capture_context["pose_m_rad"])[:6])
    env = {
        "ROS_MASTER_URI": args.master_uri,
        "ROS_IP": args.ros_ip,
        "PYTHONPATH": str(DEFAULT_VENDOR_PY),
        "DETECT_JSON": str(Path(detect_json_path).resolve()),
        "ARM_HOST": args.arm_host,
        "TRANSFORM_BACKEND": str(args.transform_backend),
        "MATRIX_PATH": str(Path(args.matrix_path).resolve()),
        "CAMERA_TOOL_NAME": str(args.camera_tool_name),
        "RESTORE_TOOL_NAME": str(args.restore_tool_name),
        "SIDE": args.side,
        "LINE_TYPE": args.line_type,
        "PLAN_POINTS": str(args.plan_points),
        "HOVER_MM": f"{args.hover_mm}",
        "HOVER_OFFSET_MODE": str(args.hover_offset_mode),
        "DIAN_JIN_DEPTH_MM": f"{args.dian_jin_depth_mm}",
        "FEN_JIN_LATERAL_MM": f"{args.fen_jin_lateral_mm}",
        "SAFE_LIFT_MM": f"{args.safe_lift_mm}",
        "INSTALL_ANG": " ".join(f"{v}" for v in args.install_ang),
        "CALC_POSES_JOINTS": "" if calc_poses_joints is None else " ".join(str(v) for v in calc_poses_joints[:6]),
        "CALC_POSES_JOINTS_SOURCE": calc_poses_joints_source,
        "ANCHOR_POSE_M_RAD": anchor_pose_m_rad,
        "ANCHOR_POSE_SOURCE": str(capture_context.get("source", "")) if anchor_pose_m_rad else "current_arm_state",
        "SPEED": f"{args.speed}",
        "MAX_STEP_M": f"{args.max_step_m}",
        "MAX_MERIDIAN_STEP_M": f"{args.max_meridian_step_m}",
        "TRIM_MERIDIAN_ENDS": str(args.trim_meridian_ends),
        "ROBOT_OFFSET_MM": " ".join(str(v) for v in args.robot_offset_mm[:3]),
        "PRODUCT_NORMAL_AXIS": str(args.product_normal_axis),
        "KEEP_CURRENT_ORIENTATION": "1" if args.keep_current_orientation else "0",
        "START_NEAREST": "1" if args.start_nearest else "0",
        "HOVER_ENTRY_MOTION": str(args.hover_entry_motion),
        "FIXED_FIRST_NORMAL": "1" if args.fixed_first_normal else "0",
        "PROJECT_PRESS_TO_HORIZONTAL": "1" if args.project_press_to_horizontal else "0",
        "EXECUTION_WORK_FRAME": str(args.execution_work_frame),
        "MASSAGE_TOOL_NAME": str(args.massage_tool_name),
        "PRODUCT_TRAJECTORY_MODE": str(args.product_trajectory_mode),
        "ALLOW_CONTROLLER_WRITE": "1" if args.allow_controller_write else "0",
        "PRODUCT_FORCE": str(args.product_force),
        "PRODUCT_SPEED": str(args.product_speed),
        "TRAJECTORY_TYPE": str(args.trajectory_type),
        "SIDE_LYING_PRODUCT_CORRECTION": "1" if args.side_lying_product_correction else "0",
        "PRODUCT_DOWN_3CM_MM": str(args.product_down_3cm_mm),
        "PRODUCT_DOWN_1CM_MM": str(args.product_down_1cm_mm),
        "TOOL_CONTACT_AXIS": str(args.tool_contact_axis),
        "LEGACY_TOOL_CONTACT_AXIS": str(args.legacy_tool_contact_axis),
        "POSE_CHECK_MOVE_PREPARE": "1" if args.pose_check_move_prepare else "0",
        "POSE_CHECK_ENTRY_MOTION": str(args.pose_check_entry_motion),
        "DIAN_JIN_DWELL_S": f"{args.dian_jin_dwell_s}",
        "FEN_JIN_DWELL_S": f"{args.fen_jin_dwell_s}",
        "SHUN_JIN_DWELL_S": f"{args.shun_jin_dwell_s}",
        "TARGET_FORCE_N": f"{args.target_force_n}",
        "MAX_FORCE_N": f"{args.max_force_n}",
        "TOUCH_STEP_MM": f"{args.touch_step_mm}",
        "MAX_PRESS_MM": f"{args.max_press_mm}",
        "CONTACT_MOTION_AXIS": str(args.contact_motion_axis),
        "PROBE_DEPTH_MM": str(args.probe_depth_mm),
        "TOUCH_DWELL_S": f"{args.touch_dwell_s}",
        "TEMPERATURE_C": f"{args.temperature_c}",
        "TEMPERATURE_WAIT_S": f"{args.temperature_wait_s}",
        "TEMPERATURE_TOLERANCE_C": f"{args.temperature_tolerance_c}",
        "EXECUTION_MODE": str(args.execution_mode),
        "DO_RUN": "1" if args.run else "0",
    }
    timeout = 600.0 if args.run else 120.0
    print(
        "[step3] invoking docker for product-ROS transform + product trajectory "
        f"(mode={args.product_trajectory_mode}, run={args.run})"
    )
    out = run_in_docker(args.container, env, PLAN_RUN_PY, timeout_s=timeout)
    print(textwrap.indent(out.rstrip(), "  "))


# ---------------------------------------------------------------------------
# Reuse-capture helper (skip docker step1)
# ---------------------------------------------------------------------------
def reuse_capture(prefix_path: str) -> dict[str, str]:
    base = Path(prefix_path).resolve()
    rgb = Path(str(base) + "_rgb.png")
    depth = Path(str(base) + "_depth.npy")
    intr = Path(str(base) + "_intrinsics.json")
    arm_state = Path(str(base) + "_arm_state.json")
    for p in (rgb, depth, intr):
        if not p.is_file():
            raise FileNotFoundError(f"reuse-capture missing: {p}")
    info = {
        "rgb_path": str(rgb),
        "depth_path": str(depth),
        "intrinsics_path": str(intr),
        "timestamp": base.name.replace("capture_", ""),
    }
    if arm_state.is_file():
        info["arm_state_path"] = str(arm_state)
    return info


def main() -> int:
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    if not Path(args.model_path).is_file():
        sys.stderr.write(f"[warn] model file not found: {args.model_path}\n")

    if args.save_current_capture_pose or args.save_current_capture_pose_only:
        saved_pose = save_current_capture_pose(args)
        args.capture_joints = [float(v) for v in list(saved_pose.get("joints_deg", []))[:6]]
        if args.save_current_capture_pose_only:
            return 0

    if args.reuse_capture:
        capture_info = reuse_capture(args.reuse_capture)
        print(f"[step1] reusing capture {capture_info['timestamp']}")
    else:
        if args.product_flow:
            if args.transform_backend != "product_ros":
                print("[warn] --product-flow is intended for --transform-backend product_ros")
            if args.skip_product_scan:
                print("[step0] skipping product scan/positioning; using current arm/camera state")
            else:
                run_product_positioning(args)
        capture_info = capture_frame(args)

    detect_json_path, _ = detect_on_host(args, capture_info)
    transform_and_run(args, detect_json_path, capture_info)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
