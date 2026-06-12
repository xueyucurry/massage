#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from rm_demo import rm_json
from rm_demo.rm_bladder import bladder_plan_to_dict, rebuild_plan_with_horizontal_press
from rm_demo.rm_execute import RosForceBridge
from rm_demo.rm_ros import create_arm_backend
from run_saved_bladder_plan import load_plan


DEFAULT_PLAN = "rm_demo_output/bladder_demo_20260429_161636_plan.json"
DEFAULT_TRAJ = "ros_vendor/trajectory_generate.yaml"


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def _force_delta_n(sample: dict[str, float] | None, baseline: dict[str, float] | None) -> float | None:
    if sample is None or baseline is None:
        return None
    d = np.asarray(
        [
            float(sample["fx"] - baseline["fx"]),
            float(sample["fy"] - baseline["fy"]),
            float(sample["fz"] - baseline["fz"]),
        ],
        dtype=np.float64,
    )
    return float(np.linalg.norm(d))


def _fmt_force(sample: dict[str, float] | None) -> str:
    if sample is None:
        return "force=n/a"
    return (
        f"Fx={sample['fx']:.2f} Fy={sample['fy']:.2f} Fz={sample['fz']:.2f} "
        f"Mx={sample['mx']:.2f} My={sample['my']:.2f} Mz={sample['mz']:.2f}"
    )


def _pose_distance_m(a: list[float], b: list[float]) -> float:
    return float(np.linalg.norm(np.asarray(a[:3], dtype=np.float64) - np.asarray(b[:3], dtype=np.float64)))


def _parse_directions(text: str) -> list[int]:
    out: list[int] = []
    for part in str(text).replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        out.append(int(part))
    if not out:
        raise RuntimeError("no force directions selected")
    return out


def _load_capture_joints(path: Path, section: str) -> list[float] | None:
    try:
        import yaml  # type: ignore
    except Exception:
        return None
    if not path.is_file():
        return None
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        return None
    value = data.get(section)
    if not isinstance(value, list) or len(value) < 6:
        return None
    return [float(v) for v in value[:6]]


def _print_tool_axes(pose: list[float], press: list[float]) -> None:
    rot = _rpy_to_matrix(float(pose[3]), float(pose[4]), float(pose[5]))
    press_vec = np.asarray(press[:3], dtype=np.float64)
    norm = float(np.linalg.norm(press_vec))
    if norm > 1e-9:
        press_vec = press_vec / norm
    print(f"press_dir_base={[round(float(v), 6) for v in press_vec.tolist()]}")
    for name, axis in (("+X", rot[:, 0]), ("+Y", rot[:, 1]), ("+Z", rot[:, 2])):
        print(
            f"tool_axis {name} base={[round(float(v), 6) for v in axis.tolist()]} "
            f"dot_press={float(np.dot(axis, press_vec)):.4f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Low-force side-lying phantom probe.")
    parser.add_argument("--plan-json", default=DEFAULT_PLAN)
    parser.add_argument("--arm-host", default="192.168.1.18")
    parser.add_argument("--work-frame", default="Base")
    parser.add_argument("--tool-name", default="mas_rub")
    parser.add_argument(
        "--tool-contact-axis",
        default="pos_z",
        choices=("pos_z", "neg_z", "pos_x", "neg_x", "pos_y", "neg_y"),
        help="realign the saved plan so this tool axis points along the side-lying press direction",
    )
    parser.add_argument("--directions", default="2", help="comma-separated Force_Position direction values to test")
    parser.add_argument("--force-mode", type=int, default=1)
    parser.add_argument("--force-coordinate", type=int, default=1, help="1 means tool frame in product docs")
    parser.add_argument("--z-control-mode", type=int, default=1)
    parser.add_argument("--target-force", type=float, default=1.0)
    parser.add_argument("--entry-speed", type=float, default=0.12)
    parser.add_argument("--motion-speed", type=float, default=0.03)
    parser.add_argument("--observe-s", type=float, default=0.8)
    parser.add_argument("--force-start-wait-s", type=float, default=0.02)
    parser.add_argument("--sample-period-s", type=float, default=0.1)
    parser.add_argument("--max-force-delta", type=float, default=5.0)
    parser.add_argument("--max-drift-m", type=float, default=0.004)
    parser.add_argument("--micro-step-m", type=float, default=0.0, help="optional tiny press step after drift test")
    parser.add_argument("--capture-section", default="arm_side_lying_prepare")
    parser.add_argument("--trajectory-yaml", default=DEFAULT_TRAJ)
    parser.add_argument("--no-restore-capture", action="store_true")
    parser.add_argument("--save-realigned-plan", default="")
    args = parser.parse_args()

    plan_path = Path(args.plan_json).resolve()
    plan = load_plan(str(plan_path))
    if not plan.frames:
        raise RuntimeError("plan has no frames")
    plan = rebuild_plan_with_horizontal_press(plan, tool_contact_axis=args.tool_contact_axis)
    frame = plan.frames[0]
    hover_pose = [float(v) for v in frame.hover_pose_m[:6]]
    press = [float(v) for v in frame.press_direction_m[:3]]
    if abs(press[2]) > 1e-5:
        raise RuntimeError(f"press direction is not horizontal: {press}")

    if args.save_realigned_plan:
        out_path = Path(args.save_realigned_plan)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(bladder_plan_to_dict(plan), ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"saved realigned plan: {out_path}")

    print(f"plan_json={plan_path}")
    print(f"hover_pose={[round(v, 6) for v in hover_pose]}")
    _print_tool_axes(hover_pose, press)

    arm = create_arm_backend("ros")
    bridge = RosForceBridge()
    active_force = False
    all_ok = False
    capture_joints = _load_capture_joints(Path(args.trajectory_yaml), str(args.capture_section))

    try:
        print("checking arm state")
        arm.recover_if_needed(args.arm_host)
        print("switching frames")
        if args.work_frame:
            print(arm.change_work_frame(args.work_frame))
        if args.tool_name:
            print(arm.change_tool(args.tool_name))

        print("moving to hover")
        print(arm.movej_p(args.arm_host, hover_pose, speed=args.entry_speed, timeout=60.0))
        bridge.enable_force_sensor()
        baseline = bridge.request_force_sample(timeout=1.0)
        if baseline is None:
            raise RuntimeError("force baseline unavailable")
        print(f"baseline {_fmt_force(baseline)}")

        for direction in _parse_directions(args.directions):
            print(
                f"direction_test direction={direction} force={float(args.target_force):.2f} "
                f"mode={int(args.force_mode)} coordinate={int(args.force_coordinate)} "
                f"z_control_mode={int(args.z_control_mode)}"
            )
            _, before_pose, arm_err, sys_err, _ = arm.get_current_arm_state(args.arm_host, timeout=1.5)
            if arm_err != 0 or sys_err != 0:
                raise RuntimeError(f"arm not clean before force test: arm_err={arm_err} sys_err={sys_err}")

            bridge.configure_force_tracking(
                target_force_n=int(round(float(args.target_force))),
                coordinate=int(args.force_coordinate),
                z_control_mode=int(args.z_control_mode),
                sensor=1,
            )
            bridge.configure_force_position(
                target_force_n=int(round(float(args.target_force))),
                mode=int(args.force_mode),
                direction=int(direction),
                sensor=1,
            )
            bridge.start_force_position(wait_s=float(args.force_start_wait_s))
            active_force = True

            deadline = time.time() + max(0.0, float(args.observe_s))
            sample_idx = 0
            while time.time() < deadline:
                time.sleep(max(0.02, float(args.sample_period_s)))
                sample_idx += 1
                sample = bridge.request_force_sample(timeout=0.6)
                _, pose, arm_err, sys_err, _ = arm.get_current_arm_state(args.arm_host, timeout=0.8)
                delta = _force_delta_n(sample, baseline)
                drift = _pose_distance_m(pose, before_pose)
                delta_text = "n/a" if delta is None else f"{delta:.2f}N"
                state = bridge.last_force_state
                state_text = "" if state is None else f" state_force={float(state.get('force', 0.0)):.2f} state_arm_err={int(state.get('arm_err', 0))}"
                print(
                    f"direction={direction} observe {sample_idx} "
                    f"drift_m={drift:.5f} delta={delta_text} arm_err={arm_err} sys_err={sys_err}"
                    f"{state_text} {_fmt_force(sample)}"
                )
                if arm_err != 0 or sys_err != 0:
                    raise RuntimeError(f"arm error during force test: arm_err={arm_err} sys_err={sys_err}")
                if delta is not None and delta > float(args.max_force_delta):
                    raise RuntimeError(f"force delta too high during drift test: {delta:.2f}N")
                if drift > float(args.max_drift_m):
                    raise RuntimeError(f"unexpected drift during force test: {drift:.5f}m")

            if float(args.micro_step_m) > 0.0:
                step_pose = list(hover_pose)
                step_m = float(args.micro_step_m)
                for axis in range(3):
                    step_pose[axis] = float(hover_pose[axis]) + float(press[axis]) * step_m
                print(f"direction={direction} micro_step_m={step_m:.5f} target={[round(v, 6) for v in step_pose[:3]]}")
                arm.movel(args.arm_host, step_pose, speed=args.motion_speed, timeout=8.0)
                sample = bridge.request_force_sample(timeout=0.8)
                _, pose, arm_err, sys_err, _ = arm.get_current_arm_state(args.arm_host, timeout=1.0)
                delta = _force_delta_n(sample, baseline)
                drift = _pose_distance_m(pose, hover_pose)
                delta_text = "n/a" if delta is None else f"{delta:.2f}N"
                print(
                    f"direction={direction} micro_done drift_from_hover_m={drift:.5f} "
                    f"delta={delta_text} arm_err={arm_err} sys_err={sys_err} {_fmt_force(sample)}"
                )
                if drift > float(args.micro_step_m) + float(args.max_drift_m):
                    raise RuntimeError(f"unexpected extra drift after micro step: {drift:.5f}m")
                if delta is not None and delta > float(args.max_force_delta):
                    raise RuntimeError(f"force delta too high after micro step: {delta:.2f}N")

            bridge.stop_force_position()
            active_force = False
            rm_json.stop_motion(args.arm_host)
            time.sleep(0.2)
            print("returning to hover")
            print(arm.movej_p(args.arm_host, hover_pose, speed=args.entry_speed, timeout=45.0))

        all_ok = True
        return 0
    except BaseException as exc:
        print(f"probe aborted: {type(exc).__name__}: {exc}")
        try:
            if active_force:
                bridge.stop_force_position()
        except Exception as stop_exc:
            print(f"stop force failed: {stop_exc}")
        try:
            rm_json.stop_motion(args.arm_host)
        except Exception as stop_exc:
            print(f"json stop failed: {stop_exc}")
        try:
            arm.stop_motion(args.arm_host)
        except Exception as stop_exc:
            print(f"ros stop failed: {stop_exc}")
        return 2
    finally:
        if all_ok and not args.no_restore_capture:
            if capture_joints is not None:
                print(f"restoring capture joints {args.capture_section}: {[round(v, 3) for v in capture_joints]}")
                print(arm.movej(args.arm_host, capture_joints, speed=10, timeout=60.0))
            else:
                print("capture joints unavailable; leaving at hover")


if __name__ == "__main__":
    raise SystemExit(main())
