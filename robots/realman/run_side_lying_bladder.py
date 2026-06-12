#!/usr/bin/env python3
"""Side-lying bladder meridian workflow preset.

This is a thin safety-oriented wrapper around `run_bladder_split.py`. It keeps
the long product/ROS command line in one place and makes the intended stages
explicit:

  preview          detect + transform + save plan, no motion
  hover            follow the selected line in the air, no contact
  touch_probe      low-force normal-direction probing at sampled points
  product_generate generate native product rubbing trajectory, no upload
  product_upload   upload native product trajectory, do not execute
  product_execute  upload and execute native product trajectory
  pose_check       move only to product p1_above pose for localization checking
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_TRAJECTORY_CONFIG = PROJECT_DIR / "ros_vendor" / "trajectory_generate.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Side-lying bladder meridian detection and massage preset",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=(
            "preview",
            "hover",
            "touch_probe",
            "product_generate",
            "product_upload",
            "product_execute",
            "pose_check",
        ),
        default="preview",
    )
    parser.add_argument("--run", action="store_true", help="actually move or execute when mode supports motion")
    parser.add_argument(
        "--allow-controller-write",
        action="store_true",
        help="allow product_upload/product_execute to write a trajectory to the controller",
    )
    parser.add_argument("--container", default="noetic")
    parser.add_argument("--master-uri", default="http://192.168.1.11:11311")
    parser.add_argument("--ros-ip", default="192.168.1.250")
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--output-dir", default=str(PROJECT_DIR / "rm_demo_output"))
    parser.add_argument("--model-path", default=str(PROJECT_DIR / "yolo11l-pose.pt"))
    parser.add_argument("--reuse-capture", default="")
    parser.add_argument("--capture-prepare-section", default="arm_side_lying_prepare")
    parser.add_argument("--trajectory-config", default=str(DEFAULT_TRAJECTORY_CONFIG))
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--line-type", choices=("inner", "outer"), default="outer")
    parser.add_argument("--auto-top-line", action="store_true")
    parser.add_argument("--finger-width-mm", type=float, default=45.0)
    parser.add_argument("--sample-points", type=int, default=30)
    parser.add_argument("--plan-points", type=int, default=8)
    parser.add_argument("--hover-mm", type=float, default=30.0)
    parser.add_argument("--safe-lift-mm", type=float, default=40.0)
    parser.add_argument("--speed", type=float, default=0.15)
    parser.add_argument("--max-step-m", type=float, default=0.025)
    parser.add_argument("--target-force-n", type=float, default=2.0)
    parser.add_argument("--max-force-n", type=float, default=6.0)
    parser.add_argument("--touch-step-mm", type=float, default=2.0)
    parser.add_argument("--max-press-mm", type=float, default=10.0)
    parser.add_argument(
        "--contact-motion-axis",
        choices=("", "neg_z", "pos_z", "pos_x", "neg_x", "pos_y", "neg_y"),
        default="neg_z",
        help="tool-frame direction used to move from hover toward the body",
    )
    parser.add_argument(
        "--probe-depth-mm",
        type=float,
        default=0.0,
        help="override touch_probe probing distance from hover; <=0 uses hover distance plus max press",
    )
    parser.add_argument("--touch-dwell-s", type=float, default=0.2)
    parser.add_argument("--temperature-c", type=float, default=0.0)
    parser.add_argument("--temperature-wait-s", type=float, default=0.0)
    parser.add_argument("--product-force", type=int, default=2)
    parser.add_argument("--product-speed", type=int, default=30)
    parser.add_argument("--trajectory-type", type=int, default=4)
    parser.add_argument(
        "--product-down-3cm-mm",
        type=float,
        default=10.0,
        help="side-lying product correction overtravel for p1_down_3cm",
    )
    parser.add_argument(
        "--product-down-1cm-mm",
        type=float,
        default=3.0,
        help="side-lying product correction overtravel for p1_down_1cm",
    )
    parser.add_argument(
        "--disable-fixed-first-normal",
        action="store_true",
        help="do not force all custom hover/touch points to reuse the first detected side-lying back normal",
    )
    parser.add_argument(
        "--disable-horizontal-press",
        action="store_true",
        help="allow custom side-lying touch/hover offsets to keep the product normal Z component",
    )
    parser.add_argument(
        "--project-press-to-horizontal",
        action="store_true",
        help="legacy option: flatten side-lying contact motion onto Base-XY",
    )
    parser.add_argument(
        "--product-normal-axis",
        choices=("local_x", "local_y", "local_z", "neg_local_x", "neg_local_y", "neg_local_z"),
        default="local_z",
        help="axis from calc_poses pose treated as outward body normal",
    )
    parser.add_argument(
        "--legacy-tool-contact-axis",
        choices=("", "neg_z", "pos_z", "pos_x", "neg_x", "pos_y", "neg_y"),
        default="pos_z",
        help="optional pose correction for custom hover/touch plans",
    )
    parser.add_argument(
        "--tool-contact-axis",
        choices=("neg_z", "pos_z", "pos_x", "neg_x", "pos_y", "neg_y"),
        default="pos_z",
        help="physical contact axis for native product trajectory pose correction",
    )
    parser.add_argument("--skip-product-correction", action="store_true")
    parser.add_argument("extra", nargs=argparse.REMAINDER, help="extra args passed after -- to run_bladder_split.py")
    return parser.parse_args()


def build_split_args(args: argparse.Namespace) -> list[str]:
    cmd = [
        sys.executable,
        str(PROJECT_DIR / "run_bladder_split.py"),
        "--container",
        args.container,
        "--master-uri",
        args.master_uri,
        "--ros-ip",
        args.ros_ip,
        "--arm-host",
        args.arm_host,
        "--output-dir",
        args.output_dir,
        "--model-path",
        args.model_path,
        "--product-flow",
        "--product-flow-positioning",
        "prepare",
        "--capture-prepare-section",
        args.capture_prepare_section,
        "--trajectory-config",
        args.trajectory_config,
        "--matrix-path",
        args.trajectory_config,
        "--transform-backend",
        "product_ros",
        "--side",
        args.side,
        "--line-type",
        args.line_type,
        "--finger-width-mm",
        str(args.finger_width_mm),
        "--sample-points",
        str(args.sample_points),
        "--plan-points",
        str(args.plan_points),
        "--hover-mm",
        str(args.hover_mm),
        "--safe-lift-mm",
        str(args.safe_lift_mm),
        "--speed",
        str(args.speed),
        "--max-step-m",
        str(args.max_step_m),
        "--hover-offset-mode",
        "normal",
        "--product-normal-axis",
        args.product_normal_axis,
        "--massage-tool-name",
        "mas_rub",
        "--execution-work-frame",
        "Base",
        "--hover-entry-motion",
        "movej_p",
        "--target-force-n",
        str(args.target_force_n),
        "--max-force-n",
        str(args.max_force_n),
        "--touch-step-mm",
        str(args.touch_step_mm),
        "--max-press-mm",
        str(args.max_press_mm),
        "--contact-motion-axis",
        args.contact_motion_axis,
        "--probe-depth-mm",
        str(args.probe_depth_mm),
        "--touch-dwell-s",
        str(args.touch_dwell_s),
        "--temperature-c",
        str(args.temperature_c),
        "--temperature-wait-s",
        str(args.temperature_wait_s),
        "--product-force",
        str(args.product_force),
        "--product-speed",
        str(args.product_speed),
        "--trajectory-type",
        str(args.trajectory_type),
        "--product-down-3cm-mm",
        str(args.product_down_3cm_mm),
        "--product-down-1cm-mm",
        str(args.product_down_1cm_mm),
        "--tool-contact-axis",
        args.tool_contact_axis,
    ]
    if args.reuse_capture:
        cmd.extend(["--reuse-capture", args.reuse_capture])
    if args.auto_top_line:
        cmd.append("--auto-top-line")
    if not args.disable_fixed_first_normal:
        cmd.append("--fixed-first-normal")
    if args.project_press_to_horizontal:
        cmd.append("--project-press-to-horizontal")
    if args.legacy_tool_contact_axis:
        cmd.extend(["--legacy-tool-contact-axis", args.legacy_tool_contact_axis])

    if args.mode == "preview":
        cmd.extend(["--product-trajectory-mode", "legacy_movel"])
    elif args.mode == "hover":
        cmd.extend(["--product-trajectory-mode", "legacy_movel", "--execution-mode", "hover_path"])
    elif args.mode == "touch_probe":
        cmd.extend(["--product-trajectory-mode", "legacy_movel", "--execution-mode", "touch_probe"])
    elif args.mode == "product_generate":
        cmd.extend(["--product-trajectory-mode", "generate_only"])
    elif args.mode == "product_upload":
        cmd.extend(["--product-trajectory-mode", "upload_only"])
    elif args.mode == "product_execute":
        cmd.extend(["--product-trajectory-mode", "execute"])
    elif args.mode == "pose_check":
        cmd.extend(["--product-trajectory-mode", "pose_check"])
    else:
        raise RuntimeError(f"unsupported mode: {args.mode}")

    if args.mode in ("product_generate", "product_upload", "product_execute", "pose_check") and not args.skip_product_correction:
        cmd.append("--side-lying-product-correction")
    if args.allow_controller_write:
        cmd.append("--allow-controller-write")
    if args.run:
        cmd.append("--run")

    extra = list(args.extra)
    if extra and extra[0] == "--":
        extra = extra[1:]
    cmd.extend(extra)
    return cmd


def main() -> int:
    args = parse_args()
    if args.mode in ("product_upload", "product_execute") and not args.allow_controller_write:
        sys.stderr.write(
            "[side_lying] refusing controller write; use --mode product_generate for local-only output, "
            "or add --allow-controller-write when you explicitly want upload/execute\n"
        )
        return 2
    split_cmd = build_split_args(args)
    env = dict(os.environ)
    env.setdefault("PYTHONUNBUFFERED", "1")
    print("[side_lying] delegated command:")
    print(" ".join(str(part) for part in split_cmd))
    return subprocess.run(split_cmd, env=env, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
