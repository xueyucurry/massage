from __future__ import annotations

import json
import os
import shlex
import subprocess
from typing import Any


DEFAULT_REMOTE_SSH = os.environ.get("RM_REMOTE_SDK_SSH", "rm@192.168.1.11")
DEFAULT_REMOTE_DIR = os.environ.get(
    "RM_REMOTE_SDK_DIR",
    "/home/rm/rm_healthcare_robot/collection/data_collection_d435_arm",
)


def _run_remote_python(
    code: str,
    *,
    payload: dict[str, Any],
    remote_ssh: str = DEFAULT_REMOTE_SSH,
    remote_dir: str = DEFAULT_REMOTE_DIR,
    timeout: float = 20.0,
) -> dict[str, Any]:
    env_payload = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    remote_cmd = (
        f"cd {shlex.quote(str(remote_dir))} && "
        f"RM_REMOTE_SDK_PAYLOAD={shlex.quote(env_payload)} python3 - <<'PY'\n"
        f"{code}\n"
        "PY"
    )
    proc = subprocess.run(
        [
            "ssh",
            "-o",
            "ConnectTimeout=5",
            "-o",
            "StrictHostKeyChecking=no",
            str(remote_ssh),
            remote_cmd,
        ],
        text=True,
        capture_output=True,
        timeout=max(8.0, float(timeout)),
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            "remote SDK command failed: "
            f"returncode={proc.returncode} stdout={proc.stdout.strip()} stderr={proc.stderr.strip()}"
        )
    marker = "##RM_REMOTE_SDK_JSON##"
    if marker not in proc.stdout:
        raise RuntimeError(f"remote SDK command returned no marker: {proc.stdout.strip()} {proc.stderr.strip()}")
    text = proc.stdout.rsplit(marker, 1)[-1].strip().splitlines()[0]
    data = json.loads(text)
    if not isinstance(data, dict):
        raise RuntimeError(f"remote SDK command returned invalid payload: {text}")
    return data


def move_cartesian_tool(
    *,
    host: str,
    joints_deg: list[float],
    dx_m: float = 0.0,
    dy_m: float = 0.0,
    dz_m: float = 0.0,
    speed: int = 3,
    code: int = 65,
    remote_ssh: str = DEFAULT_REMOTE_SSH,
    remote_dir: str = DEFAULT_REMOTE_DIR,
    timeout: float = 30.0,
) -> dict[str, Any]:
    """Move a small distance in the current tool frame using the board SDK.

    The SDK library on the board is ARM-only, so this function executes a
    short Python snippet over SSH. It does not write to the board filesystem.
    """
    payload = {
        "host": str(host),
        "joints_deg": [float(v) for v in joints_deg[:6]],
        "dx_m": float(dx_m),
        "dy_m": float(dy_m),
        "dz_m": float(dz_m),
        "speed": int(speed),
        "code": int(code),
    }
    code_text = r'''
import ctypes
import json
import os

payload = json.loads(os.environ["RM_REMOTE_SDK_PAYLOAD"])
host = str(payload["host"])
code = int(payload.get("code") or 65)
lib_path = os.path.join(os.getcwd(), "libRM_Base.so.1.0.0")
dll = ctypes.cdll.LoadLibrary(lib_path)
if int(dll.RM_API_Init(code, 0)) != 0:
    raise RuntimeError("RM_API_Init failed")
sock = dll.Arm_Socket_Start(bytes(host, "gbk"), 8080, 200)
state = int(dll.Arm_Socket_State(sock))
if state != 0:
    raise RuntimeError("Arm_Socket_Start failed: state=%d" % state)
try:
    joint_array_t = ctypes.c_float * 6
    joint_array = joint_array_t(*[float(v) for v in payload["joints_deg"][:6]])
    dll.MoveCartesianTool_Cmd.argtypes = (
        ctypes.c_int,
        joint_array_t,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_int,
        ctypes.c_byte,
        ctypes.c_float,
        ctypes.c_int,
        ctypes.c_bool,
    )
    dll.MoveCartesianTool_Cmd.restype = ctypes.c_int
    ret = dll.MoveCartesianTool_Cmd(
        sock,
        joint_array,
        float(payload["dx_m"]),
        float(payload["dy_m"]),
        float(payload["dz_m"]),
        code,
        int(payload["speed"]),
        0.0,
        0,
        True,
    )
    print("##RM_REMOTE_SDK_JSON##" + json.dumps({"ret": int(ret)}, ensure_ascii=False))
finally:
    try:
        dll.Arm_Socket_Close.argtypes = (ctypes.c_int,)
        dll.Arm_Socket_Close.restype = None
        dll.Arm_Socket_Close(sock)
    except Exception:
        pass
    try:
        dll.RM_API_UnInit.argtypes = ()
        dll.RM_API_UnInit.restype = ctypes.c_int
        dll.RM_API_UnInit()
    except Exception:
        pass
'''
    result = _run_remote_python(
        code_text,
        payload=payload,
        remote_ssh=remote_ssh,
        remote_dir=remote_dir,
        timeout=timeout,
    )
    ret = int(result.get("ret", -1))
    if ret != 0:
        raise RuntimeError(f"MoveCartesianTool_Cmd failed: ret={ret}")
    return result
