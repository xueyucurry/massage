from __future__ import annotations

import json
import socket
import time
from typing import Callable


def can_connect(host: str, port: int = 8080, timeout: float = 1.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


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
                    if msg.get("trajectory_state") is False:
                        raise RuntimeError(f"trajectory failed: {msg}")
                    if msg.get("trajectory_state") is True:
                        return msg
        if receive_reply is None:
            raise RuntimeError(f"no receive_state returned for command {command}")
        if last_traj is None:
            raise RuntimeError(f"no current_trajectory_state returned for command {command}")
        raise RuntimeError(f"trajectory did not succeed: {last_traj}")


def _pose_close(current_pose: list[float], target_pose: list[float], pos_tol_m: float = 0.003, ang_tol_rad: float = 0.02) -> bool:
    pos_ok = all(abs(float(current_pose[i]) - float(target_pose[i])) <= float(pos_tol_m) for i in range(3))
    ang_ok = all(abs(float(current_pose[i]) - float(target_pose[i])) <= float(ang_tol_rad) for i in range(3, 6))
    return pos_ok and ang_ok


def _joint_close(current_joint_deg: list[float], target_joint_deg: list[float], tol_deg: float = 0.5) -> bool:
    return all(abs(float(curr) - float(tgt)) <= float(tol_deg) for curr, tgt in zip(current_joint_deg, target_joint_deg))


def _is_planning_residue(arm_err: int, sys_err: int) -> bool:
    # RM reports 4116 after an IK/planning failure. It can remain visible in
    # state even when the controller and all joints are otherwise healthy.
    return int(arm_err) == 4116 and int(sys_err) == 0


def _poll_until(
    predicate: Callable[[], bool],
    timeout: float,
    interval_s: float = 0.15,
) -> bool:
    deadline = time.time() + float(timeout)
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(float(interval_s))
    return False


def pose_to_json_units(pose: list[float]) -> list[int]:
    xyz = [int(round(float(v) * 1_000_000.0)) for v in pose[:3]]
    rpy = [int(round(float(v) * 1_000.0)) for v in pose[3:6]]
    return xyz + rpy


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


def get_power_state(host: str) -> int:
    data = query_json(host, {"command": "get_arm_power_state"})
    return int(data.get("power_state", 0))


def recover_if_needed(host: str) -> None:
    _, _, arm_err, sys_err, inverse_km_err = get_current_arm_state(host)
    en_state, err_flag, brake_state = get_joint_status(host)
    bad_joints = [idx + 1 for idx, value in enumerate(en_state) if value == 0 or err_flag[idx] != 0]
    if arm_err == 0 and sys_err == 0 and inverse_km_err in (0, -1) and not bad_joints:
        return

    clear_reply = None
    for _ in range(3):
        clear_reply = query_json(host, {"command": "clear_system_err"})
        if not bool(clear_reply.get("clear_state", False)):
            raise RuntimeError(f"clear_system_err failed: {clear_reply}")
        time.sleep(0.35)
        _, _, arm_err, sys_err, inverse_km_err = get_current_arm_state(host)
        if arm_err == 0 and sys_err == 0 and inverse_km_err in (0, -1):
            break
    for joint_id in bad_joints:
        reply = query_json(host, {"command": "set_joint_clear_err", "joint_clear_err": int(joint_id)})
        if not bool(reply.get("joint_clear_err", False)):
            raise RuntimeError(f"set_joint_clear_err({joint_id}) failed: {reply}")
        reply = query_json(host, {"command": "set_joint_en_state", "joint_en_state": [int(joint_id), 1]})
        if not bool(reply.get("joint_en_state", False)):
            raise RuntimeError(f"set_joint_en_state({joint_id},1) failed: {reply}")
    time.sleep(0.6)
    _, _, arm_err, sys_err, _ = get_current_arm_state(host)
    en_state, err_flag, brake_state = get_joint_status(host)
    bad_joints = [idx + 1 for idx, value in enumerate(en_state) if value == 0 or err_flag[idx] != 0]
    if arm_err != 0 or sys_err != 0 or bad_joints:
        raise RuntimeError(
            "controller recovery incomplete: "
            f"arm_err={arm_err} sys_err={sys_err} en_state={en_state} err_flag={err_flag} "
            f"brake_state={brake_state} clear_reply={clear_reply}"
        )


def movel(host: str, pose: list[float], speed: int, blend_radius: float = 0.0, timeout: float = 18.0) -> dict[str, object]:
    command = {
        "command": "movel",
        "pose": pose_to_json_units(pose),
        "v": int(speed),
        "r": float(blend_radius),
        "trajectory_connect": 0,
    }
    try:
        return send_motion_and_wait(host, command, timeout=timeout)
    except RuntimeError as exc:
        if "no current_trajectory_state returned" not in str(exc):
            raise
        reached = _poll_until(
            lambda: _pose_close(get_current_arm_state(host)[1], pose),
            timeout=max(10.0, float(timeout) * 4.0),
        )
        if not reached:
            raise
        return {
            "device": 0,
            "state": "polled_trajectory_state",
            "trajectory_connect": 0,
            "trajectory_state": True,
        }


def movej_p(host: str, pose: list[float], speed: int, blend_radius: float = 0.0, timeout: float = 24.0) -> dict[str, object]:
    command = {
        "command": "movej_p",
        "pose": pose_to_json_units(pose),
        "v": int(speed),
        "r": float(blend_radius),
        "trajectory_connect": 0,
    }
    try:
        return send_motion_and_wait(host, command, timeout=timeout)
    except RuntimeError as exc:
        if "no current_trajectory_state returned" not in str(exc):
            raise
        reached = _poll_until(
            lambda: _pose_close(get_current_arm_state(host)[1], pose),
            timeout=max(10.0, float(timeout) * 4.0),
        )
        if not reached:
            raise
        return {
            "device": 0,
            "state": "polled_trajectory_state_movej_p",
            "trajectory_connect": 0,
            "trajectory_state": True,
        }


def movej(host: str, joint_deg: list[float], speed: int, timeout: float = 18.0) -> dict[str, object]:
    command = {
        "command": "movej",
        "joint": [int(round(float(v) * 1000.0)) for v in joint_deg],
        "v": int(speed),
        "r": 0,
        "trajectory_connect": 0,
    }
    try:
        return send_motion_and_wait(host, command, timeout=timeout)
    except RuntimeError as exc:
        if "no current_trajectory_state returned" not in str(exc):
            raise
        reached = _poll_until(
            lambda: _joint_close(get_current_arm_state(host)[0], joint_deg),
            timeout=max(10.0, float(timeout) * 4.0),
        )
        if not reached:
            raise
        return {
            "device": 0,
            "state": "polled_trajectory_state",
            "trajectory_connect": 0,
            "trajectory_state": True,
        }


def stop_motion(host: str) -> dict[str, object]:
    return query_json(host, {"command": "set_arm_stop"})


def set_pos_teach(host: str, teach_type: str, direction: str, speed: int, timeout: float = 2.0) -> dict[str, object]:
    axis = str(teach_type).strip().lower()
    if axis not in ("x", "y", "z"):
        raise ValueError(f"unsupported position teach axis: {teach_type}")
    direction_key = str(direction).strip().lower()
    if direction_key not in ("pos", "neg"):
        raise ValueError(f"unsupported position teach direction: {direction}")
    return query_json(
        host,
        {
            "command": "set_pos_teach",
            "teach_type": axis,
            "direction": direction_key,
            "v": int(speed),
        },
        timeout=timeout,
    )


def stop_teach(host: str, timeout: float = 2.0) -> dict[str, object]:
    return query_json(host, {"command": "set_stop_teach"}, timeout=timeout)
