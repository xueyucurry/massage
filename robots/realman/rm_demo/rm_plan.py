from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Iterable


@dataclass
class MassagePointPlan:
    index: int
    robot_point_m: list[float]
    approach_pose_m: list[float]
    work_pose_m: list[float]
    retreat_pose_m: list[float]
    dwell_s: float
    press_direction_m: list[float] = field(default_factory=list)
    source_pose_quat: list[float] = field(default_factory=list)


@dataclass
class StaticMassagePlan:
    side: str
    point_count: int
    hover_m: float
    safe_z_m: float
    anchor_pose_m: list[float]
    points: list[MassagePointPlan]


def _pick_evenly_indices(length: int, count: int) -> list[int]:
    if length < 1:
        raise RuntimeError("not enough meridian points for plan generation")
    if count <= 1:
        return [length // 2]
    if count >= length:
        return list(range(length))
    out: list[int] = []
    for idx in range(count):
        ratio = idx / max(1, count - 1)
        src = int(round(ratio * (length - 1)))
        out.append(src)
    return out


def _pick_evenly(points: list[list[float]], count: int) -> list[list[float]]:
    if len(points) < 1:
        raise RuntimeError("not enough meridian points for plan generation")
    if count <= 1:
        midpoint = len(points) // 2
        return [[float(v) for v in points[midpoint]]]
    return [[float(v) for v in points[src]] for src in _pick_evenly_indices(len(points), count)]


def _quat_to_rpy(quat_xyzw: list[float]) -> list[float]:
    qx, qy, qz, qw = [float(v) for v in quat_xyzw[:4]]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [float(roll), float(pitch), float(yaw)]


def _quat_to_local_z_axis(quat_xyzw: list[float]) -> list[float]:
    qx, qy, qz, qw = [float(v) for v in quat_xyzw[:4]]
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    z_axis = [
        2.0 * (xz + wy),
        2.0 * (yz - wx),
        1.0 - 2.0 * (xx + yy),
    ]
    norm = math.sqrt(sum(v * v for v in z_axis))
    if norm <= 1e-8:
        return [0.0, 0.0, -1.0]
    return [float(v / norm) for v in z_axis]


def _bbox(points: Iterable[list[float]]) -> dict[str, list[float]]:
    pts = list(points)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    return {
        "x": [min(xs), max(xs)],
        "y": [min(ys), max(ys)],
        "z": [min(zs), max(zs)],
    }


def build_static_plan(
    side: str,
    meridian_points_robot_m: list[list[float]],
    anchor_pose_m: list[float],
    point_count: int,
    hover_m: float,
    dwell_s: float,
    safe_lift_m: float,
    meridian_pose_quat: list[list[float]] | None = None,
) -> StaticMassagePlan:
    pick_indices = _pick_evenly_indices(len(meridian_points_robot_m), max(1, int(point_count)))
    selected = [[float(v) for v in meridian_points_robot_m[src]] for src in pick_indices]
    selected_pose_quat: list[list[float]] = []
    if meridian_pose_quat and len(meridian_pose_quat) == len(meridian_points_robot_m):
        selected_pose_quat = [[float(v) for v in meridian_pose_quat[src]] for src in pick_indices]

    rx, ry, rz = [float(v) for v in anchor_pose_m[3:6]]
    safe_candidates = [float(anchor_pose_m[2]) + float(safe_lift_m)]
    points: list[MassagePointPlan] = []
    for idx, point in enumerate(selected, start=1):
        x, y, z = [float(v) for v in point]
        pose_quat = selected_pose_quat[idx - 1] if idx - 1 < len(selected_pose_quat) else []
        if pose_quat and len(pose_quat) >= 7:
            rpy = _quat_to_rpy(pose_quat[3:7])
            press_dir = _quat_to_local_z_axis(pose_quat[3:7])
        else:
            rpy = [rx, ry, rz]
            press_dir = [0.0, 0.0, -1.0]

        approach_xyz = [
            x - press_dir[0] * float(hover_m),
            y - press_dir[1] * float(hover_m),
            z - press_dir[2] * float(hover_m),
        ]
        approach = [approach_xyz[0], approach_xyz[1], approach_xyz[2], rpy[0], rpy[1], rpy[2]]
        work = [x, y, z, rpy[0], rpy[1], rpy[2]]
        retreat = list(approach)
        safe_candidates.append(float(approach_xyz[2]) + 0.01)
        points.append(
            MassagePointPlan(
                index=idx,
                robot_point_m=[x, y, z],
                approach_pose_m=approach,
                work_pose_m=work,
                retreat_pose_m=retreat,
                dwell_s=float(dwell_s),
                press_direction_m=press_dir,
                source_pose_quat=pose_quat,
            )
        )
    safe_z_m = max(safe_candidates)
    return StaticMassagePlan(
        side=side,
        point_count=len(points),
        hover_m=float(hover_m),
        safe_z_m=float(safe_z_m),
        anchor_pose_m=[float(v) for v in anchor_pose_m[:6]],
        points=points,
    )


def plan_to_dict(plan: StaticMassagePlan) -> dict[str, object]:
    data = asdict(plan)
    data["bbox_robot_m"] = _bbox([p.robot_point_m for p in plan.points])
    return data
