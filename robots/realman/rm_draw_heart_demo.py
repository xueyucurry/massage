#!/usr/bin/env python3
"""
Draw a heart in the air with a RealMan arm using the controller JSON protocol.

Defaults:
- anchor at current TCP pose
- heart lies in the XY plane
- current TCP orientation is kept unchanged
- size/offset are tuned for the currently validated heart path
- preview only unless --run is provided
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import socket
import subprocess
import sys
import time
from typing import Iterable, Sequence


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_HOST = os.environ.get("RM_ARM_HOST", "192.168.1.18")
DEFAULT_REMOTE_SSH = os.environ.get("RM_ARM_REMOTE_SSH", "rm@192.168.1.11")
DEFAULT_REMOTE_SCRIPT = os.environ.get(
    "RM_ARM_HEART_REMOTE_SCRIPT",
    "/home/rm/rm_healthcare_robot/collection/data_collection_d435_arm/rm_draw_heart_demo.py",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RM JSON heart drawing demo")
    parser.add_argument("--host", default=DEFAULT_HOST, help="controller IP")
    parser.add_argument("--plane", choices=("xy", "xz", "yz"), default="xy", help="heart plane")
    parser.add_argument("--width", type=float, default=0.02, help="heart width in meters")
    parser.add_argument("--height", type=float, default=0.018, help="heart height in meters")
    parser.add_argument("--samples", type=int, default=60, help="number of heart points")
    parser.add_argument("--speed", type=int, default=3, help="JSON motion speed 1..100")
    parser.add_argument("--x-offset", type=float, default=0.015, help="additional X offset in meters")
    parser.add_argument("--y-offset", type=float, default=-0.005, help="additional Y offset in meters")
    parser.add_argument("--z-offset", type=float, default=0.0, help="additional Z offset in meters")
    parser.add_argument("--sleep", type=float, default=0.02, help="extra wait between segments")
    parser.add_argument("--frame", default="Base", help="reserved for compatibility")
    parser.add_argument("--transport", choices=("auto", "local", "remote"), default="auto", help="how to run")
    parser.add_argument("--dump-points", action="store_true", help="print all waypoints")
    parser.add_argument("--run", action="store_true", help="execute the heart path")
    return parser.parse_args()


def can_connect(host: str, port: int, timeout: float = 1.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def maybe_exec_remote(args: argparse.Namespace) -> None:
    if args.transport == "local":
        return
    if args.transport == "auto" and can_connect(args.host, 8080):
        return
    remote_dir = os.path.dirname(DEFAULT_REMOTE_SCRIPT)
    remote_cmd = "cd {remote_dir} && python3 {script} --transport local {args}".format(
        remote_dir=shlex.quote(remote_dir),
        script=shlex.quote(DEFAULT_REMOTE_SCRIPT),
        args=" ".join(shlex.quote(arg) for arg in sys.argv[1:]),
    ).strip()
    print(f"controller {args.host}:8080 not reachable locally; running remote script via SSH on {DEFAULT_REMOTE_SSH}", file=sys.stderr)
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


def query_json(host: str, command: dict[str, object], timeout: float = 3.0) -> dict[str, object]:
    payload = json.dumps(command, separators=(",", ":")).encode("utf-8") + b"\r\n"
    with socket.create_connection((host, 8080), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(payload)
        chunks: list[bytes] = []
        while True:
            try:
                part = sock.recv(4096)
            except socket.timeout:
                break
            if not part:
                break
            chunks.append(part)
            if b"}\n" in part or b"}\r\n" in part:
                break
    raw = b"".join(chunks).decode("utf-8", "ignore").strip()
    if not raw:
        raise RuntimeError(f"empty JSON reply for command {command}")
    return json.loads(raw)


def send_motion_and_wait(host: str, command: dict[str, object], timeout: float = 12.0) -> dict[str, object]:
    payload = json.dumps(command, separators=(",", ":")).encode("utf-8") + b"\r\n"
    with socket.create_connection((host, 8080), timeout=3.0) as sock:
        sock.settimeout(0.5)
        sock.sendall(payload)
        start = time.time()
        receive_reply: dict[str, object] | None = None
        traj_reply: dict[str, object] | None = None
        while time.time() - start < timeout:
            try:
                part = sock.recv(4096)
            except socket.timeout:
                continue
            if not part:
                break
            for line in part.decode("utf-8", "ignore").splitlines():
                line = line.strip()
                if not line:
                    continue
                msg = json.loads(line)
                if "receive_state" in msg:
                    receive_reply = msg
                    if not bool(msg["receive_state"]):
                        raise RuntimeError(f"command rejected: {msg}")
                if msg.get("state") == "current_trajectory_state":
                    traj_reply = msg
                    if msg.get("trajectory_state") is False:
                        raise RuntimeError(f"trajectory failed: {msg}")
                    if msg.get("trajectory_state") is True:
                        return msg
        if receive_reply is None:
            raise RuntimeError(f"no receive_state returned for command {command}")
        if traj_reply is None:
            raise RuntimeError(f"no current_trajectory_state returned for command {command}")
        return traj_reply


def get_current_arm_state(host: str) -> tuple[list[float], list[float], int, int, int]:
    data = query_json(host, {"command": "get_current_arm_state"})
    arm_state = data["arm_state"]
    joints = [float(v) / 1000.0 for v in arm_state["joint"]]
    pose_raw = arm_state["pose"]
    pose = [float(pose_raw[i]) / 1000000.0 for i in range(3)] + [float(pose_raw[i]) / 1000.0 for i in range(3, 6)]
    arm_err = int(arm_state.get("arm_err", 0))
    sys_err = int(arm_state.get("sys_err", 0))
    inverse_km_err = int(arm_state.get("inverse_km_err", 0))
    return joints, pose, arm_err, sys_err, inverse_km_err


def get_joint_status(host: str) -> tuple[list[int], list[int], list[int]]:
    en = query_json(host, {"command": "get_joint_en_state"})
    flags = query_json(host, {"command": "get_joint_err_flag"})
    en_state = [int(v) for v in en["en_state"]]
    err_flag = [int(v) for v in flags["err_flag"]]
    brake_state = [int(v) for v in flags["brake_state"]]
    return en_state, err_flag, brake_state


def recover_if_needed(host: str) -> None:
    joints, pose, arm_err, sys_err, inverse_km_err = get_current_arm_state(host)
    en_state, err_flag, brake_state = get_joint_status(host)
    bad_joints = [idx + 1 for idx, v in enumerate(en_state) if v == 0 or err_flag[idx] != 0]

    if arm_err == 0 and sys_err == 0 and inverse_km_err in (0, -1) and not bad_joints:
        return

    print(
        "recovering controller state: "
        f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err} "
        f"bad_joints={bad_joints} brake_state={brake_state}"
    )
    reply = query_json(host, {"command": "clear_system_err"})
    if not bool(reply.get("clear_state", False)):
        raise RuntimeError(f"clear_system_err failed: {reply}")

    for joint_id in bad_joints:
        reply = query_json(host, {"command": "set_joint_clear_err", "joint_clear_err": int(joint_id)})
        if not bool(reply.get("joint_clear_err", False)):
            raise RuntimeError(f"set_joint_clear_err({joint_id}) failed: {reply}")
        reply = query_json(host, {"command": "set_joint_en_state", "joint_en_state": [int(joint_id), 1]})
        if not bool(reply.get("joint_en_state", False)):
            raise RuntimeError(f"set_joint_en_state({joint_id},1) failed: {reply}")

    time.sleep(0.6)
    joints, pose, arm_err, sys_err, inverse_km_err = get_current_arm_state(host)
    en_state, err_flag, brake_state = get_joint_status(host)
    bad_joints = [idx + 1 for idx, v in enumerate(en_state) if v == 0 or err_flag[idx] != 0]
    if arm_err != 0 or sys_err != 0 or bad_joints:
        raise RuntimeError(
            "controller recovery incomplete: "
            f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err} "
            f"en_state={en_state} err_flag={err_flag} brake_state={brake_state}"
        )


def pose_to_json_units(pose: Sequence[float]) -> list[int]:
    xyz = [int(round(float(v) * 1_000_000.0)) for v in pose[:3]]
    rpy = [int(round(float(v) * 1_000.0)) for v in pose[3:6]]
    return xyz + rpy


def heart_curve(samples: int, width_m: float, height_m: float) -> list[tuple[float, float]]:
    if samples < 12:
        raise ValueError("samples must be at least 12")
    if width_m <= 0.0 or height_m <= 0.0:
        raise ValueError("width and height must be positive")

    raw: list[tuple[float, float]] = []
    for idx in range(samples):
        t = math.pi + (2.0 * math.pi * idx / (samples - 1))
        x = 16.0 * (math.sin(t) ** 3)
        y = (
            13.0 * math.cos(t)
            - 5.0 * math.cos(2.0 * t)
            - 2.0 * math.cos(3.0 * t)
            - math.cos(4.0 * t)
        )
        raw.append((x, y))

    xs = [p[0] for p in raw]
    ys = [p[1] for p in raw]
    x_mid = 0.5 * (min(xs) + max(xs))
    y_min = min(ys)
    x_span = max(xs) - min(xs)
    y_span = max(ys) - min(ys)

    return [
        (
            ((x - x_mid) / x_span) * width_m,
            ((y - y_min) / y_span) * height_m,
        )
        for x, y in raw
    ]


def build_poses(
    anchor_pose: Sequence[float],
    plane: str,
    width_m: float,
    height_m: float,
    samples: int,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
    z_offset: float = 0.0,
) -> list[list[float]]:
    ax, ay, az, rx, ry, rz = [float(v) for v in anchor_pose]
    ax += float(x_offset)
    ay += float(y_offset)
    az += float(z_offset)
    curve = heart_curve(samples=samples, width_m=width_m, height_m=height_m)
    poses: list[list[float]] = []
    for u, v in curve:
        if plane == "xy":
            pos = [ax + u, ay + v, az]
        elif plane == "xz":
            pos = [ax + u, ay, az + v]
        elif plane == "yz":
            pos = [ax, ay + u, az + v]
        else:
            raise ValueError(f"unsupported plane: {plane}")
        poses.append(pos + [rx, ry, rz])
    return poses


def expand_axis_aligned(poses: Sequence[Sequence[float]], plane: str) -> list[list[float]]:
    if not poses:
        return []
    expanded: list[list[float]] = [list(poses[0])]
    for target in poses[1:]:
        prev = expanded[-1]
        mid = list(prev)
        if plane == "xy":
            mid[0] = float(target[0])
            if mid[:3] != prev[:3]:
                expanded.append(mid)
            end = list(mid)
            end[1] = float(target[1])
        elif plane == "xz":
            mid[0] = float(target[0])
            if mid[:3] != prev[:3]:
                expanded.append(mid)
            end = list(mid)
            end[2] = float(target[2])
        elif plane == "yz":
            mid[1] = float(target[1])
            if mid[:3] != prev[:3]:
                expanded.append(mid)
            end = list(mid)
            end[2] = float(target[2])
        else:
            raise ValueError(f"unsupported plane: {plane}")
        if end[:3] != expanded[-1][:3]:
            expanded.append(end)
    return expanded


def bbox(points: Iterable[Sequence[float]]) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    pts = list(points)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def preview(args: argparse.Namespace, joints: Sequence[float], anchor_pose: Sequence[float], poses: Sequence[Sequence[float]], arm_err: int, sys_err: int, inverse_km_err: int) -> None:
    xr, yr, zr = bbox(poses)
    print(f"controller={args.host} frame={args.frame} transport={args.transport}")
    print(f"current_joints={[round(v, 3) for v in joints]}")
    print(f"current_pose={[round(v, 6) for v in anchor_pose]}")
    print(f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}")
    print(
        f"plane={args.plane} width={args.width:.3f}m height={args.height:.3f}m "
        f"samples={args.samples} speed={args.speed}"
    )
    print(f"offsets x={args.x_offset:.3f}m y={args.y_offset:.3f}m z={args.z_offset:.3f}m")
    print(f"expanded_segments={len(poses) - 1}")
    print(f"bbox X[{xr[0]:.4f}, {xr[1]:.4f}] Y[{yr[0]:.4f}, {yr[1]:.4f}] Z[{zr[0]:.4f}, {zr[1]:.4f}]")
    if args.dump_points:
        for idx, pose in enumerate(poses):
            print(f"{idx:02d}: {[round(v, 6) for v in pose]}")


def run_demo(args: argparse.Namespace) -> None:
    maybe_exec_remote(args)
    recover_if_needed(args.host)
    joints, anchor_pose, arm_err, sys_err, inverse_km_err = get_current_arm_state(args.host)
    heart_poses = build_poses(
        anchor_pose,
        args.plane,
        args.width,
        args.height,
        args.samples,
        x_offset=args.x_offset,
        y_offset=args.y_offset,
        z_offset=args.z_offset,
    )
    poses = expand_axis_aligned(heart_poses, args.plane)
    preview(args, joints, anchor_pose, poses, arm_err, sys_err, inverse_km_err)

    if not args.run:
        print("preview only; pass --run to execute")
        return

    if arm_err != 0 or sys_err != 0 or inverse_km_err not in (0, -1):
        raise RuntimeError(
            f"controller not ready: arm_err={arm_err}, sys_err={sys_err}, inverse_km_err={inverse_km_err}"
        )

    print("executing heart path...")
    for idx, pose in enumerate(poses[1:], start=1):
        send_motion_and_wait(
            args.host,
            {
                "command": "movel",
                "pose": pose_to_json_units(pose),
                "v": int(args.speed),
                "r": 0,
                "trajectory_connect": 0,
            },
        )
        if args.sleep > 0:
            time.sleep(args.sleep)
        if idx % 5 == 0 or idx == len(poses) - 1:
            print(f"segment {idx}/{len(poses) - 1} done")

    joints2, pose2, arm_err2, sys_err2, inverse_km_err2 = get_current_arm_state(args.host)
    print(f"final_joints={[round(v, 3) for v in joints2]}")
    print(f"final_pose={[round(v, 6) for v in pose2]}")
    print(f"final_arm_err={arm_err2} final_sys_err={sys_err2} final_inverse_km_err={inverse_km_err2}")


if __name__ == "__main__":
    run_demo(parse_args())
