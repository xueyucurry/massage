#!/usr/bin/env python3
"""
Move the RealMan arm to a more open pose before drawing a heart.

This script uses the controller JSON protocol directly and can run locally.
It clears recoverable errors, re-enables affected joints, and then performs
one low-speed joint move to a known-good preparation pose.
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import socket
import subprocess
import sys
import time
from typing import Sequence


DEFAULT_HOST = os.environ.get("RM_ARM_HOST", "192.168.1.18")
DEFAULT_REMOTE_SSH = os.environ.get("RM_ARM_REMOTE_SSH", "rm@192.168.1.11")
DEFAULT_REMOTE_SCRIPT = os.environ.get(
    "RM_ARM_PREP_REMOTE_SCRIPT",
    "/home/rm/rm_healthcare_robot/collection/data_collection_d435_arm/rm_prepare_heart_pose.py",
)

# This joint pose is from a previously verified state where XY-plane linear
# motions succeeded. Units are 0.001 degree in controller JSON format.
PRESETS = {
    "heart_xy": {
        "joint": [-1365, -14473, -17942, 659, -117885, 14289],
        "note": "open pose validated for small XY moves",
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prepare a safer pose for heart drawing")
    parser.add_argument("--host", default=DEFAULT_HOST, help="controller IP")
    parser.add_argument("--preset", choices=sorted(PRESETS), default="heart_xy", help="target preparation preset")
    parser.add_argument("--speed", type=int, default=5, help="JSON movej speed 1..100")
    parser.add_argument("--transport", choices=("auto", "local", "remote"), default="auto", help="how to run")
    parser.add_argument("--run", action="store_true", help="execute the preparation move")
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
    print(
        f"controller {args.host}:8080 not reachable locally; running remote script via SSH on {DEFAULT_REMOTE_SSH}",
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


def send_motion_and_wait(host: str, command: dict[str, object], timeout: float = 18.0) -> dict[str, object]:
    payload = json.dumps(command, separators=(",", ":")).encode("utf-8") + b"\r\n"
    with socket.create_connection((host, 8080), timeout=3.0) as sock:
        sock.settimeout(0.5)
        sock.sendall(payload)
        start = time.time()
        receive_reply: dict[str, object] | None = None
        last_traj: dict[str, object] | None = None
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
                    last_traj = msg
                    if msg.get("trajectory_state") is True:
                        return msg
        if receive_reply is None:
            raise RuntimeError(f"no receive_state returned for command {command}")
        if last_traj is None:
            raise RuntimeError(f"no current_trajectory_state returned for command {command}")
        raise RuntimeError(f"trajectory did not succeed: {last_traj}")


def get_current_arm_state(host: str) -> tuple[list[float], list[float], int, int, int]:
    data = query_json(host, {"command": "get_current_arm_state"})
    arm_state = data["arm_state"]
    joints = [float(v) / 1000.0 for v in arm_state["joint"]]
    pose_raw = arm_state["pose"]
    pose = [float(pose_raw[i]) / 1_000_000.0 for i in range(3)] + [float(pose_raw[i]) / 1000.0 for i in range(3, 6)]
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
    bad_joints = [idx + 1 for idx, value in enumerate(en_state) if value == 0 or err_flag[idx] != 0]

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
    _, _, arm_err, sys_err, inverse_km_err = get_current_arm_state(host)
    en_state, err_flag, brake_state = get_joint_status(host)
    bad_joints = [idx + 1 for idx, value in enumerate(en_state) if value == 0 or err_flag[idx] != 0]
    if arm_err != 0 or sys_err != 0 or bad_joints:
        raise RuntimeError(
            "controller recovery incomplete: "
            f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err} "
            f"en_state={en_state} err_flag={err_flag} brake_state={brake_state}"
        )


def max_joint_delta_deg(current: Sequence[float], target_json: Sequence[int]) -> float:
    target_deg = [float(v) / 1000.0 for v in target_json]
    return max(abs(a - b) for a, b in zip(current, target_deg))


def run(args: argparse.Namespace) -> None:
    maybe_exec_remote(args)
    recover_if_needed(args.host)

    target = PRESETS[args.preset]
    current_joints, current_pose, arm_err, sys_err, inverse_km_err = get_current_arm_state(args.host)
    en_state, err_flag, brake_state = get_joint_status(args.host)
    target_deg = [round(v / 1000.0, 3) for v in target["joint"]]
    current_deg = [round(v, 3) for v in current_joints]
    max_delta = max_joint_delta_deg(current_joints, target["joint"])

    print(f"preset={args.preset} note={target['note']}")
    print(f"current_joints={current_deg}")
    print(f"target_joints={target_deg}")
    print(f"current_pose={[round(v, 6) for v in current_pose]}")
    print(f"arm_err={arm_err} sys_err={sys_err} inverse_km_err={inverse_km_err}")
    print(f"en_state={en_state} err_flag={err_flag} brake_state={brake_state}")
    print(f"max_joint_delta={max_delta:.3f} deg speed={args.speed}")

    if not args.run:
        print("preview only; pass --run to execute")
        return

    send_motion_and_wait(
        args.host,
        {
            "command": "movej",
            "joint": list(target["joint"]),
            "v": int(args.speed),
            "r": 0,
            "trajectory_connect": 0,
        },
    )
    time.sleep(0.5)
    final_joints, final_pose, final_arm_err, final_sys_err, final_inverse_km_err = get_current_arm_state(args.host)
    print(f"final_joints={[round(v, 3) for v in final_joints]}")
    print(f"final_pose={[round(v, 6) for v in final_pose]}")
    print(
        f"final_arm_err={final_arm_err} final_sys_err={final_sys_err} "
        f"final_inverse_km_err={final_inverse_km_err}"
    )


if __name__ == "__main__":
    run(parse_args())
