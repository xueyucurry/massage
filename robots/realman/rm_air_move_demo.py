#!/usr/bin/env python3
"""
Conservative RealMan air-motion demo.

Default mode is joint-space because the current arm pose may enter a
singularity-like state where linear planning is rejected.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import shlex
import socket
import subprocess
import sys
import time
from typing import Iterable, Sequence


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RM_PKG_DIR = "/home/rm/rm_healthcare_robot/collection/data_collection_d435_arm"
REMOTE_SSH_TARGET = os.environ.get("RM_ARM_REMOTE_SSH", "rm@192.168.1.11")
REMOTE_SCRIPT_PATH = os.environ.get(
    "RM_ARM_REMOTE_SCRIPT",
    f"{DEFAULT_RM_PKG_DIR}/rm_air_move_demo.py",
)


def find_local_rm_pkg() -> str | None:
    candidates = []
    env_pkg = os.environ.get("RM_ARM_PKG_DIR")
    if env_pkg:
        candidates.append(env_pkg)
    candidates.extend(
        [
            SCRIPT_DIR,
            DEFAULT_RM_PKG_DIR,
        ]
    )
    for candidate in candidates:
        if not candidate:
            continue
        config_path = os.path.join(candidate, "config.py")
        arm_path = os.path.join(candidate, "robotic_arm.py")
        if os.path.isfile(config_path) and os.path.isfile(arm_path):
            return candidate
    return None


def maybe_exec_remote() -> None:
    if os.environ.get("RM_AIR_MOVE_FORCE_LOCAL") == "1":
        return
    remote_dir = os.path.dirname(REMOTE_SCRIPT_PATH)
    remote_cmd = "cd {remote_dir} && python3 {script} {args}".format(
        remote_dir=shlex.quote(remote_dir),
        script=shlex.quote(REMOTE_SCRIPT_PATH),
        args=" ".join(shlex.quote(arg) for arg in sys.argv[1:]),
    ).strip()
    print(
        f"local RM package not found; running remote script via SSH on {REMOTE_SSH_TARGET}",
        file=sys.stderr,
    )
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
                REMOTE_SSH_TARGET,
                remote_cmd,
            ],
            check=False,
        ).returncode
    )


RM_PKG_DIR = find_local_rm_pkg()
if RM_PKG_DIR is None:
    maybe_exec_remote()
    raise ModuleNotFoundError("RealMan package not found locally and remote execution did not start")
if RM_PKG_DIR not in sys.path:
    sys.path.insert(0, RM_PKG_DIR)

from config import CODE_fi, HOST_fi  # type: ignore  # noqa: E402
from robotic_arm import Arm, POSE  # type: ignore  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Conservative RM air-motion demo")
    parser.add_argument("--host", default=HOST_fi, help="controller IP")
    parser.add_argument("--code", type=int, default=CODE_fi, help="arm model code")
    parser.add_argument("--frame", default="Base", help="work frame name")
    parser.add_argument("--mode", choices=("joint", "linear"), default="joint", help="motion mode")
    parser.add_argument("--plane", choices=("xy", "xz", "yz"), default="xz", help="linear motion plane")
    parser.add_argument("--lift", type=float, default=0.02, help="linear upward offset in meters")
    parser.add_argument("--span", type=float, default=0.015, help="linear side offset in meters")
    parser.add_argument("--joint-index", type=int, default=6, help="1-based joint index for joint mode")
    parser.add_argument("--joint-delta", type=float, default=10.0, help="joint mode offset in degrees")
    parser.add_argument("--speed", type=int, default=5, help="speed ratio, 1..100")
    parser.add_argument("--blend-radius", type=float, default=0.0, help="Movel blend radius")
    parser.add_argument("--settle", type=float, default=0.0, help="extra wait after each segment")
    parser.add_argument("--dump-points", action="store_true", help="print all waypoints")
    parser.add_argument("--run", action="store_true", help="send motion commands")
    return parser.parse_args()


def require_ok(name: str, ret: int) -> None:
    if ret != 0:
        raise RuntimeError(f"{name} failed: {ret}")


def close_arm(arm: Arm) -> None:
    try:
        arm.pDll.Arm_Socket_Close.argtypes = (ctypes.c_int,)
        arm.pDll.Arm_Socket_Close.restype = None
        arm.pDll.Arm_Socket_Close(arm.nSocket)
    finally:
        arm.pDll.RM_API_UnInit.argtypes = ()
        arm.pDll.RM_API_UnInit.restype = ctypes.c_int
        arm.pDll.RM_API_UnInit()


def change_work_frame(arm: Arm, name: str) -> None:
    arm.pDll.Change_Work_Frame.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_bool]
    arm.pDll.Change_Work_Frame.restype = ctypes.c_int
    require_ok("Change_Work_Frame", arm.pDll.Change_Work_Frame(arm.nSocket, name.encode("utf-8"), True))
    time.sleep(1.0)


def get_sdk_state(arm: Arm) -> tuple[list[float], list[float], int]:
    arm.pDll.Get_Current_Arm_State.argtypes = (
        ctypes.c_int,
        ctypes.c_float * 6,
        ctypes.POINTER(POSE),
        ctypes.POINTER(ctypes.c_uint16),
        ctypes.POINTER(ctypes.c_uint16),
    )
    arm.pDll.Get_Current_Arm_State.restype = ctypes.c_int

    joints = (ctypes.c_float * 6)()
    pose = POSE()
    arm_err = ctypes.c_uint16()
    sys_err = ctypes.c_uint16()
    ret = arm.pDll.Get_Current_Arm_State(
        arm.nSocket,
        joints,
        ctypes.pointer(pose),
        ctypes.pointer(arm_err),
        ctypes.pointer(sys_err),
    )
    joint_list = [float(joints[i]) for i in range(6)]
    pose_list = [float(v) for v in (pose.px, pose.py, pose.pz, pose.rx, pose.ry, pose.rz)]
    return joint_list, pose_list, int(ret)


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


def get_json_arm_state(host: str) -> tuple[list[float], list[float], int, int, int]:
    data = query_json(host, {"command": "get_current_arm_state"})
    arm_state = data["arm_state"]
    joints = [float(v) / 1000.0 for v in arm_state["joint"]]
    pose_raw = arm_state["pose"]
    pose = [float(pose_raw[i]) / 1000000.0 for i in range(3)] + [float(pose_raw[i]) / 1000.0 for i in range(3, 6)]
    arm_err = int(arm_state.get("arm_err", 0))
    sys_err = int(arm_state.get("sys_err", 0))
    inverse_km_err = int(arm_state.get("inverse_km_err", 0))
    return joints, pose, arm_err, sys_err, inverse_km_err


def movel(arm: Arm, pose: Sequence[float], speed: int, blend_radius: float, block: bool = True) -> None:
    target = POSE()
    target.px, target.py, target.pz, target.rx, target.ry, target.rz = [float(v) for v in pose]
    arm.pDll.Movel_Cmd.argtypes = (
        ctypes.c_int,
        POSE,
        ctypes.c_byte,
        ctypes.c_float,
        ctypes.c_int,
    )
    arm.pDll.Movel_Cmd.restype = ctypes.c_int
    ret = arm.pDll.Movel_Cmd(
        arm.nSocket,
        target,
        int(speed),
        float(blend_radius),
        int(block),
    )
    require_ok("Movel_Cmd", ret)


def movej(arm: Arm, joints: Sequence[float], speed: int, blend_radius: float, block: bool = True) -> None:
    joint_array_t = ctypes.c_float * 6
    joint_array = joint_array_t(*[float(v) for v in joints])
    arm.pDll.Movej_Cmd.argtypes = (
        ctypes.c_int,
        joint_array_t,
        ctypes.c_byte,
        ctypes.c_float,
        ctypes.c_bool,
    )
    arm.pDll.Movej_Cmd.restype = ctypes.c_int
    ret = arm.pDll.Movej_Cmd(
        arm.nSocket,
        joint_array,
        int(speed),
        float(blend_radius),
        bool(block),
    )
    require_ok("Movej_Cmd", ret)


def offset_pose(anchor_pose: Sequence[float], plane: str, du: float, dv: float) -> list[float]:
    x, y, z, rx, ry, rz = [float(v) for v in anchor_pose]
    if plane == "xy":
        return [x + du, y + dv, z, rx, ry, rz]
    if plane == "xz":
        return [x + du, y, z + dv, rx, ry, rz]
    if plane == "yz":
        return [x, y + du, z + dv, rx, ry, rz]
    raise ValueError(f"unsupported plane: {plane}")


def build_linear_path(anchor_pose: Sequence[float], plane: str, lift: float, span: float) -> list[list[float]]:
    if lift <= 0.0:
        raise ValueError("lift must be > 0")
    if span < 0.0:
        raise ValueError("span must be >= 0")
    return [
        list(anchor_pose),
        offset_pose(anchor_pose, plane, 0.0, lift),
        offset_pose(anchor_pose, plane, span, lift),
        offset_pose(anchor_pose, plane, -span, lift),
        offset_pose(anchor_pose, plane, 0.0, lift),
        list(anchor_pose),
    ]


def build_joint_path(anchor_joints: Sequence[float], joint_index: int, joint_delta: float) -> list[list[float]]:
    if not 1 <= joint_index <= 6:
        raise ValueError("joint-index must be in [1, 6]")
    if joint_delta == 0.0:
        raise ValueError("joint-delta must be non-zero")

    idx = joint_index - 1
    start = [float(v) for v in anchor_joints]
    step_pos = list(start)
    step_neg = list(start)
    step_pos[idx] += joint_delta
    step_neg[idx] -= joint_delta
    return [start, step_pos, step_neg, start]


def bbox(points: Iterable[Sequence[float]]) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    pts = list(points)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def preview_joint(args: argparse.Namespace, sdk_joints: Sequence[float], sdk_pose: Sequence[float], json_joints: Sequence[float], json_pose: Sequence[float], arm_err: int, sys_err: int, inverse_km_err: int, points: Sequence[Sequence[float]]) -> None:
    print(f"controller={args.host} code={args.code} frame={args.frame} mode={args.mode}")
    print(f"sdk_joints={[round(v, 3) for v in sdk_joints]}")
    print(f"sdk_pose={[round(v, 6) for v in sdk_pose]}")
    print(f"json_joints={[round(v, 3) for v in json_joints]}")
    print(f"json_pose={[round(v, 6) for v in json_pose]}")
    print(f"json_arm_err={arm_err} json_sys_err={sys_err} inverse_km_err={inverse_km_err}")
    print(f"joint_index={args.joint_index} joint_delta={args.joint_delta:.3f}deg speed={args.speed}")
    if args.dump_points:
        for idx, point in enumerate(points):
            print(f"{idx:02d}: {[round(v, 3) for v in point]}")


def preview_linear(args: argparse.Namespace, sdk_joints: Sequence[float], sdk_pose: Sequence[float], json_joints: Sequence[float], json_pose: Sequence[float], arm_err: int, sys_err: int, inverse_km_err: int, points: Sequence[Sequence[float]]) -> None:
    xr, yr, zr = bbox(points)
    print(f"controller={args.host} code={args.code} frame={args.frame} mode={args.mode}")
    print(f"sdk_joints={[round(v, 3) for v in sdk_joints]}")
    print(f"sdk_pose={[round(v, 6) for v in sdk_pose]}")
    print(f"json_joints={[round(v, 3) for v in json_joints]}")
    print(f"json_pose={[round(v, 6) for v in json_pose]}")
    print(f"json_arm_err={arm_err} json_sys_err={sys_err} inverse_km_err={inverse_km_err}")
    print(f"plane={args.plane} lift={args.lift:.3f}m span={args.span:.3f}m speed={args.speed}")
    print(f"bbox X[{xr[0]:.4f}, {xr[1]:.4f}] Y[{yr[0]:.4f}, {yr[1]:.4f}] Z[{zr[0]:.4f}, {zr[1]:.4f}]")
    if args.dump_points:
        for idx, point in enumerate(points):
            print(f"{idx:02d}: {[round(v, 6) for v in point]}")


def run_demo(args: argparse.Namespace) -> None:
    arm = Arm(args.code, args.host)
    try:
        change_work_frame(arm, args.frame)
        sdk_joints, sdk_pose, sdk_ret = get_sdk_state(arm)
        require_ok("Get_Current_Arm_State", sdk_ret)
        json_joints, json_pose, arm_err, sys_err, inverse_km_err = get_json_arm_state(args.host)

        if args.mode == "joint":
            points = build_joint_path(sdk_joints, args.joint_index, args.joint_delta)
            preview_joint(args, sdk_joints, sdk_pose, json_joints, json_pose, arm_err, sys_err, inverse_km_err, points)
            if not args.run:
                print("preview only; pass --run to execute")
                return
            print("executing joint path...")
            for idx, point in enumerate(points[1:], start=1):
                movej(arm, point, speed=args.speed, blend_radius=args.blend_radius, block=True)
                if args.settle > 0:
                    time.sleep(args.settle)
                print(f"segment {idx}/{len(points) - 1} done")
        else:
            points = build_linear_path(sdk_pose, args.plane, args.lift, args.span)
            preview_linear(args, sdk_joints, sdk_pose, json_joints, json_pose, arm_err, sys_err, inverse_km_err, points)
            if arm_err != 0 or inverse_km_err != 0:
                raise RuntimeError(
                    f"linear mode refused because json_arm_err={arm_err}, inverse_km_err={inverse_km_err}; "
                    "recover with joint mode first"
                )
            if not args.run:
                print("preview only; pass --run to execute")
                return
            print("executing linear path...")
            for idx, point in enumerate(points[1:], start=1):
                movel(arm, point, speed=args.speed, blend_radius=args.blend_radius, block=True)
                if args.settle > 0:
                    time.sleep(args.settle)
                print(f"segment {idx}/{len(points) - 1} done")

        final_json_joints, final_json_pose, final_arm_err, final_sys_err, final_inverse_km_err = get_json_arm_state(args.host)
        print(f"final_json_joints={[round(v, 3) for v in final_json_joints]}")
        print(f"final_json_pose={[round(v, 6) for v in final_json_pose]}")
        print(f"final_json_arm_err={final_arm_err} final_json_sys_err={final_sys_err} final_inverse_km_err={final_inverse_km_err}")
    finally:
        close_arm(arm)


if __name__ == "__main__":
    run_demo(parse_args())
