from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np


@dataclass
class ProductTrajectorySideLyingCorrection:
    enabled: bool
    corrected_pose_count: int
    hover_m: float
    down_3cm_m: float
    down_1cm_m: float
    product_normal_axis: str
    fixed_first_normal: bool
    project_press_to_horizontal: bool
    force_direction: int | None
    prepare_joint_corrected: bool
    note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ProductTrajectorySideLyingValidation:
    ok: bool
    messages: list[str]
    prepare_joint_deg: list[float] | None
    force_direction: int | None
    expected_force_direction: int | None
    contact_unit_base: list[float] | None
    contact_z_abs: float | None
    tool_axis_dot_contact: float | None
    p1_above_pose_m_rad: list[float] | None
    p1_down_3cm_pose_m_rad: list[float] | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _normalize_vec(vec: list[float] | np.ndarray, eps: float = 1e-9) -> np.ndarray | None:
    arr = np.asarray(vec, dtype=np.float64)
    norm = float(np.linalg.norm(arr))
    if norm <= eps:
        return None
    return arr / norm


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


def _quat_to_local_axes(quat_xyzw: list[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    qx, qy, qz, qw = [float(v) for v in quat_xyzw[:4]]
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    rot = np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    return rot[:, 0], rot[:, 1], rot[:, 2]


def _axis_vector(axis_name: str) -> np.ndarray:
    axes = {
        "pos_x": [1.0, 0.0, 0.0],
        "neg_x": [-1.0, 0.0, 0.0],
        "pos_y": [0.0, 1.0, 0.0],
        "neg_y": [0.0, -1.0, 0.0],
        "pos_z": [0.0, 0.0, 1.0],
        "neg_z": [0.0, 0.0, -1.0],
    }
    key = str(axis_name or "").strip().lower()
    if key not in axes:
        raise ValueError(f"unsupported tool_contact_axis: {axis_name}")
    return np.asarray(axes[key], dtype=np.float64)


def _force_direction_for_tool_axis(axis_name: str) -> int | None:
    key = str(axis_name or "").strip().lower()
    if key in ("pos_x", "neg_x"):
        return 0
    if key in ("pos_y", "neg_y"):
        return 1
    if key in ("pos_z", "neg_z"):
        return 2
    return None


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def _matrix_to_rpy(rot: np.ndarray) -> list[float]:
    m = np.asarray(rot, dtype=np.float64)
    sy = max(-1.0, min(1.0, -float(m[2, 0])))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) > 1e-6:
        roll = math.atan2(float(m[2, 1]), float(m[2, 2]))
        yaw = math.atan2(float(m[1, 0]), float(m[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(m[0, 1]), float(m[1, 1]))
    return [float(roll), float(pitch), float(yaw)]


def _align_axis_rotation(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    src = _normalize_vec(np.asarray(source, dtype=np.float64))
    dst = _normalize_vec(np.asarray(target, dtype=np.float64))
    if src is None or dst is None:
        raise RuntimeError("cannot align zero-length axes")
    cross = np.cross(src, dst)
    dot = max(-1.0, min(1.0, float(np.dot(src, dst))))
    sin = float(np.linalg.norm(cross))
    if sin < 1e-9:
        if dot > 0.0:
            return np.eye(3, dtype=np.float64)
        helper = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(src, helper))) > 0.9:
            helper = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        cross = _normalize_vec(np.cross(src, helper))
        if cross is None:
            raise RuntimeError("cannot build 180-degree axis correction")
        sin = 1.0
        dot = -1.0
    vx = np.asarray(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ],
        dtype=np.float64,
    )
    return np.eye(3, dtype=np.float64) + vx + (vx @ vx) * ((1.0 - dot) / (sin * sin))


def _press_direction_from_product_pose(
    pose_xyz_quat: list[float],
    anchor_pose_m: list[float],
    product_normal_axis: str,
) -> np.ndarray:
    local_x, local_y, local_z = _quat_to_local_axes(pose_xyz_quat[3:7])
    axes = {
        "local_x": local_x,
        "local_y": local_y,
        "local_z": local_z,
        "neg_local_x": -local_x,
        "neg_local_y": -local_y,
        "neg_local_z": -local_z,
    }
    axis_key = str(product_normal_axis or "local_x").strip().lower()
    if axis_key not in axes:
        raise ValueError(f"unsupported product_normal_axis: {product_normal_axis}")

    outward = _normalize_vec(axes[axis_key])
    if outward is None:
        raise RuntimeError("product pose normal axis is zero")

    anchor_xyz = np.asarray(anchor_pose_m[:3], dtype=np.float64)
    point_xyz = np.asarray(pose_xyz_quat[:3], dtype=np.float64)
    toward_anchor = anchor_xyz - point_xyz
    if np.linalg.norm(toward_anchor) > 1e-8 and float(np.dot(outward, toward_anchor)) < 0.0:
        outward = -outward
    return -outward


def _pose_units_to_m_rad(pose_units: list[Any]) -> list[float]:
    if len(pose_units) < 6:
        raise RuntimeError("trajectory pose must contain at least 6 values")
    return [float(v) / 1_000_000.0 for v in pose_units[:3]] + [float(v) / 1000.0 for v in pose_units[3:6]]


def _joint_units_to_deg(joint_units: list[Any]) -> list[float]:
    if len(joint_units) < 6:
        raise RuntimeError("trajectory joint must contain at least 6 values")
    return [float(v) / 1000.0 for v in joint_units[:6]]


def _pose_m_rad_to_units(pose_m_rad: list[float], old_pose_units: list[Any]) -> list[int]:
    converted = [int(round(float(v) * 1_000_000.0)) for v in pose_m_rad[:3]]
    converted.extend(int(round(float(v) * 1000.0)) for v in pose_m_rad[3:6])
    if len(old_pose_units) > 6:
        converted.extend(int(v) for v in old_pose_units[6:])
    return converted


def validate_product_trajectory_side_lying(
    *,
    trajectory_content: str,
    tool_contact_axis: str,
    prepare_joints_deg: list[float] | None = None,
    require_horizontal_press: bool = True,
    z_tolerance: float = 1e-4,
    axis_dot_min: float = 0.98,
    prepare_joint_tolerance_deg: float = 0.25,
) -> ProductTrajectorySideLyingValidation:
    """Validate the native product trajectory after side-lying correction."""
    poses: dict[str, list[float]] = {}
    prepare_joint_deg: list[float] | None = None
    force_direction: int | None = None
    messages: list[str] = []

    for raw_line in trajectory_content.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        obj = json.loads(line)
        name = str(obj.get("name", ""))
        if name == "prepare" and isinstance(obj.get("joint"), list):
            prepare_joint_deg = _joint_units_to_deg(obj["joint"])
        if name == "Force" and "direction" in obj:
            force_direction = int(obj["direction"])
        pose_units = obj.get("pose")
        if name in {"p1_above_2cm", "p1_down_3cm"} and isinstance(pose_units, list):
            poses[name] = _pose_units_to_m_rad(pose_units)

    expected_force_direction = _force_direction_for_tool_axis(tool_contact_axis)
    if expected_force_direction is not None and force_direction != expected_force_direction:
        messages.append(
            f"Force.direction={force_direction} does not match {tool_contact_axis} "
            f"(expected {expected_force_direction})"
        )

    if prepare_joints_deg is not None:
        if prepare_joint_deg is None:
            messages.append("prepare joint is missing")
        else:
            max_err = max(
                abs(float(a) - float(b))
                for a, b in zip(prepare_joint_deg[:6], prepare_joints_deg[:6])
            )
            if max_err > float(prepare_joint_tolerance_deg):
                messages.append(
                    f"prepare joint differs from side-lying capture joints by {max_err:.3f} deg"
                )

    contact_unit_base: list[float] | None = None
    contact_z_abs: float | None = None
    tool_axis_dot_contact: float | None = None
    p1_above = poses.get("p1_above_2cm")
    p1_down_3cm = poses.get("p1_down_3cm")
    if p1_above is None or p1_down_3cm is None:
        messages.append("p1_above_2cm or p1_down_3cm pose is missing")
    else:
        delta = np.asarray(p1_down_3cm[:3], dtype=np.float64) - np.asarray(p1_above[:3], dtype=np.float64)
        contact = _normalize_vec(delta)
        if contact is None:
            messages.append("p1 contact direction is zero")
        else:
            contact_unit_base = [float(v) for v in contact.tolist()]
            contact_z_abs = abs(float(contact[2]))
            if require_horizontal_press and contact_z_abs > float(z_tolerance):
                messages.append(f"contact direction has non-horizontal Base-Z component {contact_z_abs:.6f}")

            rot = _rpy_to_matrix(float(p1_above[3]), float(p1_above[4]), float(p1_above[5]))
            tool_axis = rot @ _axis_vector(tool_contact_axis)
            tool_axis = _normalize_vec(tool_axis)
            if tool_axis is None:
                messages.append("tool contact axis is zero")
            else:
                tool_axis_dot_contact = float(np.dot(tool_axis, contact))
                if tool_axis_dot_contact < float(axis_dot_min):
                    messages.append(
                        f"{tool_contact_axis} dot contact={tool_axis_dot_contact:.4f}, "
                        f"expected >= {axis_dot_min:.4f}"
                    )

    return ProductTrajectorySideLyingValidation(
        ok=not messages,
        messages=messages,
        prepare_joint_deg=prepare_joint_deg,
        force_direction=force_direction,
        expected_force_direction=expected_force_direction,
        contact_unit_base=contact_unit_base,
        contact_z_abs=contact_z_abs,
        tool_axis_dot_contact=tool_axis_dot_contact,
        p1_above_pose_m_rad=p1_above,
        p1_down_3cm_pose_m_rad=p1_down_3cm,
    )


def _point_pose_from_waypoint(
    pose_xyz_quat: list[float],
    press_direction: np.ndarray,
    *,
    offset_m: float,
    tool_contact_axis: str = "",
) -> list[float]:
    point = np.asarray(pose_xyz_quat[:3], dtype=np.float64)
    xyz = point + press_direction * float(offset_m)
    rpy = _quat_to_rpy(pose_xyz_quat[3:7])
    if str(tool_contact_axis or "").strip():
        rot = _rpy_to_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        contact_axis = _axis_vector(tool_contact_axis)
        world_contact_axis = rot @ contact_axis
        correction = _align_axis_rotation(world_contact_axis, np.asarray(press_direction, dtype=np.float64))
        rpy = _matrix_to_rpy(correction @ rot)
    return [float(xyz[0]), float(xyz[1]), float(xyz[2]), float(rpy[0]), float(rpy[1]), float(rpy[2])]


def correct_product_trajectory_for_side_lying(
    *,
    trajectory_content: str,
    selected_pose_quat: list[list[float]],
    anchor_pose_m: list[float],
    product_normal_axis: str,
    hover_m: float,
    tool_contact_axis: str = "",
    prepare_joints_deg: list[float] | None = None,
    down_3cm_m: float = 0.03,
    down_1cm_m: float = 0.01,
    fixed_first_normal: bool = False,
    project_press_to_horizontal: bool = False,
    rewrite_path_orientations: bool = False,
) -> tuple[str, ProductTrajectorySideLyingCorrection]:
    """Rewrite product trajectory entry poses so side-lying contact is along the body normal.

    The product rubbing generator still emits `p1_above_2cm`, `p1_down_3cm`,
    and `p1_down_1cm` using Base-Z offsets. In side-lying use, those offsets
    must follow the surface normal from `/calc_poses`; otherwise the arm moves
    vertically instead of moving toward the back.

    The rubbing path poses (`p2..`) are left untouched by default. The product
    generator already uses `/calc_poses` for those path orientations, and
    rewriting them here can make the controller-side trajectory verifier reject
    otherwise valid waypoints.
    """
    line_sep = "\r\n" if "\r\n" in trajectory_content else "\n"

    if not selected_pose_quat:
        return trajectory_content, ProductTrajectorySideLyingCorrection(
            enabled=False,
            corrected_pose_count=0,
            hover_m=float(hover_m),
            down_3cm_m=float(down_3cm_m),
            down_1cm_m=float(down_1cm_m),
            product_normal_axis=str(product_normal_axis),
            fixed_first_normal=bool(fixed_first_normal),
            project_press_to_horizontal=bool(project_press_to_horizontal),
            force_direction=_force_direction_for_tool_axis(tool_contact_axis),
            prepare_joint_corrected=False,
            note="skipped: selected_pose_quat is empty",
        )

    normalized_poses = [[float(v) for v in pose[:7]] for pose in selected_pose_quat if len(pose) >= 7]
    if not normalized_poses:
        return trajectory_content, ProductTrajectorySideLyingCorrection(
            enabled=False,
            corrected_pose_count=0,
            hover_m=float(hover_m),
            down_3cm_m=float(down_3cm_m),
            down_1cm_m=float(down_1cm_m),
            product_normal_axis=str(product_normal_axis),
            fixed_first_normal=bool(fixed_first_normal),
            project_press_to_horizontal=bool(project_press_to_horizontal),
            force_direction=_force_direction_for_tool_axis(tool_contact_axis),
            prepare_joint_corrected=False,
            note="skipped: no valid xyz+quat waypoint poses",
        )

    press_dirs = [
        _press_direction_from_product_pose(pose, anchor_pose_m, product_normal_axis)
        for pose in normalized_poses
    ]
    if project_press_to_horizontal:
        projected_dirs: list[np.ndarray] = []
        for press in press_dirs:
            projected = np.asarray(press, dtype=np.float64)
            projected[2] = 0.0
            normalized = _normalize_vec(projected)
            if normalized is None:
                normalized = _normalize_vec(press)
            if normalized is None:
                raise RuntimeError("cannot project side-lying product press direction")
            projected_dirs.append(normalized)
        press_dirs = projected_dirs
    if fixed_first_normal and press_dirs:
        first = np.asarray(press_dirs[0], dtype=np.float64)
        press_dirs = [first.copy() for _ in press_dirs]

    changed = 0
    force_direction = _force_direction_for_tool_axis(tool_contact_axis)
    corrected_prepare = False
    out_lines: list[str] = []
    for raw_line in trajectory_content.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        obj = json.loads(line)
        name = str(obj.get("name", ""))
        if name == "prepare" and prepare_joints_deg is not None and len(prepare_joints_deg) >= 6:
            obj["joint"] = [int(round(float(v) * 1000.0)) for v in prepare_joints_deg[:6]]
            corrected_prepare = True
        if name == "Force" and force_direction is not None:
            obj["direction"] = int(force_direction)
        pose_units = obj.get("pose")
        if isinstance(pose_units, list) and len(pose_units) >= 6:
            waypoint_idx: int | None = None
            offset_m: float | None = None
            if name == "p1_above_2cm":
                waypoint_idx = 0
                offset_m = -float(hover_m)
            elif name == "p1_down_3cm":
                waypoint_idx = 0
                offset_m = float(down_3cm_m)
            elif name == "p1_down_1cm":
                waypoint_idx = 0
                offset_m = float(down_1cm_m)
            elif rewrite_path_orientations and name.startswith("p") and name[1:].isdigit():
                waypoint_idx = int(name[1:]) - 1
                offset_m = 0.0

            if waypoint_idx is not None and offset_m is not None and 0 <= waypoint_idx < len(normalized_poses):
                corrected_pose = _point_pose_from_waypoint(
                    normalized_poses[waypoint_idx],
                    press_dirs[waypoint_idx],
                    offset_m=offset_m,
                    tool_contact_axis=tool_contact_axis,
                )
                obj["pose"] = _pose_m_rad_to_units(corrected_pose, pose_units)
                changed += 1
            else:
                # Preserve unrelated product poses exactly.
                _pose_units_to_m_rad(pose_units)
        out_lines.append(json.dumps(obj, ensure_ascii=False, separators=(",", ":")))

    note = (
        "side-lying correction applied: p1 hover/down use product surface normal; "
        "p2.. keep product-generated poses"
        if changed
        else "no matching p* trajectory poses found"
    )
    if rewrite_path_orientations:
        note += "; p2.. orientations rewritten"
    if fixed_first_normal:
        note += "; fixed first normal"
    if project_press_to_horizontal:
        note += "; press projected to Base-XY"
    if force_direction is not None:
        note += f"; force direction set to tool axis {force_direction}"
    if corrected_prepare:
        note += "; prepare joint replaced by side-lying capture joints"
    return line_sep.join(out_lines) + line_sep, ProductTrajectorySideLyingCorrection(
        enabled=True,
        corrected_pose_count=changed,
        hover_m=float(hover_m),
        down_3cm_m=float(down_3cm_m),
        down_1cm_m=float(down_1cm_m),
        product_normal_axis=str(product_normal_axis),
        fixed_first_normal=bool(fixed_first_normal),
        project_press_to_horizontal=bool(project_press_to_horizontal),
        force_direction=force_direction,
        prepare_joint_corrected=corrected_prepare,
        note=note,
    )
