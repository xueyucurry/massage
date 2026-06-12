from __future__ import annotations

from typing import Any


FORCE_FIELD_CANDIDATES = {
    "fx": ("force_fx", "force_Fx", "Fx", "fx", "x"),
    "fy": ("force_fy", "force_Fy", "Fy", "fy", "y"),
    "fz": ("force_fz", "force_Fz", "Fz", "fz", "z"),
    "mx": ("force_mx", "force_Mx", "Mx", "mx"),
    "my": ("force_my", "force_My", "My", "my"),
    "mz": ("force_mz", "force_Mz", "Mz", "mz"),
}


def get_message_class(type_name: str):
    from rosidl_runtime_py.utilities import get_message

    return get_message(type_name)


def get_attr_any(msg: Any, names: tuple[str, ...], default: Any = None) -> Any:
    for name in names:
        if hasattr(msg, name):
            return getattr(msg, name)
    return default


def extract_force(msg: Any) -> dict[str, float]:
    out: dict[str, float] = {}
    for key, names in FORCE_FIELD_CANDIDATES.items():
        value = get_attr_any(msg, names)
        if value is None:
            continue
        out[key] = float(value)
    if not out and hasattr(msg, "wrench"):
        wrench = msg.wrench
        out = {
            "fx": float(wrench.force.x),
            "fy": float(wrench.force.y),
            "fz": float(wrench.force.z),
            "mx": float(wrench.torque.x),
            "my": float(wrench.torque.y),
            "mz": float(wrench.torque.z),
        }
    if not out:
        raise AttributeError(f"cannot find force fields in {type(msg).__name__}")
    return out


def make_pose_msg(position: tuple[float, float, float], quaternion_xyzw: tuple[float, float, float, float]):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(quaternion_xyzw[0])
    pose.orientation.y = float(quaternion_xyzw[1])
    pose.orientation.z = float(quaternion_xyzw[2])
    pose.orientation.w = float(quaternion_xyzw[3])
    return pose


def assign_pose_command_fields(msg: Any, pose_msg: Any, speed: float, trajectory_connect: int = 0) -> None:
    assigned = False
    for field in ("pose", "Pose", "target_pose", "target"):
        if hasattr(msg, field):
            setattr(msg, field, pose_msg)
            assigned = True
            break
    if not assigned:
        raise AttributeError(f"cannot find pose field in {type(msg).__name__}")

    for field in ("speed", "velocity", "v"):
        if hasattr(msg, field):
            setattr(msg, field, float(speed))
            break
    if hasattr(msg, "trajectory_connect"):
        setattr(msg, "trajectory_connect", int(trajectory_connect))


def assign_force_position_fields(msg: Any, force_n: float, sensor: int = 1, mode: int = 1, direction: int = 2) -> None:
    if hasattr(msg, "sensor"):
        msg.sensor = int(sensor)
    if hasattr(msg, "mode"):
        msg.mode = int(mode)
    if hasattr(msg, "direction"):
        msg.direction = int(direction)
    if hasattr(msg, "dir"):
        msg.dir = int(direction)
    assigned_force = False
    for field in ("n", "force", "target_force", "force_n"):
        if hasattr(msg, field):
            setattr(msg, field, int(round(float(force_n))))
            assigned_force = True
            break
    if not assigned_force:
        raise AttributeError(f"cannot find force field in {type(msg).__name__}")


def stamp_now(node: Any):
    return node.get_clock().now().to_msg()
