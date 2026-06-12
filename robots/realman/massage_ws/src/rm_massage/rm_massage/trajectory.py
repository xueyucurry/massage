from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .transforms import (
    IDENTITY_T,
    Transform,
    Vector3,
    add,
    distance,
    normalize,
    scale,
    sub,
    tool_quat_from_normal_tangent,
    transform_point,
    transform_vector,
    vec3,
)


@dataclass(frozen=True)
class TcpWaypoint:
    index: int
    surface_base: Vector3
    contact_base: Vector3
    pre_base: Vector3
    retreat_base: Vector3
    normal_base: Vector3
    tangent_base: Vector3
    quaternion_xyzw: tuple[float, float, float, float]


def build_tcp_trajectory(
    line: dict[str, Any],
    T_base_line: Transform = IDENTITY_T,
    tcp_radius_m: float = 0.0,
    approach_distance_m: float = 0.03,
    retreat_distance_m: float = 0.03,
    tcp_type: str = "sphere",
) -> list[TcpWaypoint]:
    raw_points = line.get("points_body", [])
    if len(raw_points) < 2:
        raise ValueError("line must contain at least two points")

    points_line = [vec3(p) for p in raw_points]
    normal_line = normalize(vec3(line.get("normal_body", [0.0, 0.0, 1.0])))
    normal_base = normalize(transform_vector(T_base_line, normal_line))
    points_base = [transform_point(T_base_line, p) for p in points_line]

    is_sphere = str(tcp_type).strip().lower() == "sphere"
    radius = float(tcp_radius_m) if is_sphere else 0.0
    out: list[TcpWaypoint] = []
    for idx, surface in enumerate(points_base):
        if idx < len(points_base) - 1:
            tangent = sub(points_base[idx + 1], surface)
        else:
            tangent = sub(surface, points_base[idx - 1])
        tangent = normalize(tangent, fallback=(1.0, 0.0, 0.0))

        contact = add(surface, scale(normal_base, radius))
        pre = add(surface, scale(normal_base, radius + float(approach_distance_m)))
        retreat = add(surface, scale(normal_base, radius + float(retreat_distance_m)))
        quat = tool_quat_from_normal_tangent(normal_base, tangent)
        out.append(
            TcpWaypoint(
                index=idx,
                surface_base=surface,
                contact_base=contact,
                pre_base=pre,
                retreat_base=retreat,
                normal_base=normal_base,
                tangent_base=tangent,
                quaternion_xyzw=quat,
            )
        )
    return out


def check_workspace(waypoints: list[TcpWaypoint], limits: dict[str, float]) -> None:
    for point in waypoints:
        for label, p in (("pre", point.pre_base), ("contact", point.contact_base), ("retreat", point.retreat_base)):
            x, y, z = p
            if not (
                float(limits["x_min"]) <= x <= float(limits["x_max"])
                and float(limits["y_min"]) <= y <= float(limits["y_max"])
                and float(limits["z_min"]) <= z <= float(limits["z_max"])
            ):
                raise ValueError(f"waypoint {point.index} {label} outside workspace: {p}")


def check_step_size(waypoints: list[TcpWaypoint], max_step_m: float) -> None:
    for prev, curr in zip(waypoints, waypoints[1:]):
        step = distance(prev.contact_base, curr.contact_base)
        if step > float(max_step_m):
            raise ValueError(
                f"contact step too large between {prev.index} and {curr.index}: "
                f"{step:.4f} m > {float(max_step_m):.4f} m"
            )


def pose_list(position: Vector3, quaternion_xyzw: tuple[float, float, float, float]) -> list[float]:
    return [
        float(position[0]),
        float(position[1]),
        float(position[2]),
        float(quaternion_xyzw[0]),
        float(quaternion_xyzw[1]),
        float(quaternion_xyzw[2]),
        float(quaternion_xyzw[3]),
    ]
