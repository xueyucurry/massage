from __future__ import annotations

import math
from typing import Iterable


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]
Transform = tuple[Matrix3, Vector3]


IDENTITY_R: Matrix3 = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
IDENTITY_T: Transform = (IDENTITY_R, (0.0, 0.0, 0.0))


def vec3(values: Iterable[float]) -> Vector3:
    vals = [float(v) for v in values]
    if len(vals) != 3:
        raise ValueError(f"expected 3 values, got {len(vals)}")
    return vals[0], vals[1], vals[2]


def add(a: Vector3, b: Vector3) -> Vector3:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def sub(a: Vector3, b: Vector3) -> Vector3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def scale(v: Vector3, s: float) -> Vector3:
    return v[0] * s, v[1] * s, v[2] * s


def dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(v: Vector3) -> float:
    return math.sqrt(dot(v, v))


def normalize(v: Vector3, fallback: Vector3 | None = None) -> Vector3:
    n = norm(v)
    if n <= 1e-12:
        if fallback is None:
            raise ValueError("zero-length vector")
        return normalize(fallback)
    return v[0] / n, v[1] / n, v[2] / n


def mat_vec(R: Matrix3, v: Vector3) -> Vector3:
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


def mat_mul(A: Matrix3, B: Matrix3) -> Matrix3:
    cols = [
        (B[0][0], B[1][0], B[2][0]),
        (B[0][1], B[1][1], B[2][1]),
        (B[0][2], B[1][2], B[2][2]),
    ]
    rows = []
    for row in A:
        rows.append(tuple(dot(row, col) for col in cols))
    return rows[0], rows[1], rows[2]


def transpose(R: Matrix3) -> Matrix3:
    return (
        (R[0][0], R[1][0], R[2][0]),
        (R[0][1], R[1][1], R[2][1]),
        (R[0][2], R[1][2], R[2][2]),
    )


def transform_point(T: Transform, p: Vector3) -> Vector3:
    R, t = T
    return add(mat_vec(R, p), t)


def transform_vector(T: Transform, v: Vector3) -> Vector3:
    R, _ = T
    return mat_vec(R, v)


def compose(A: Transform, B: Transform) -> Transform:
    Ra, ta = A
    Rb, tb = B
    return mat_mul(Ra, Rb), add(mat_vec(Ra, tb), ta)


def inverse(T: Transform) -> Transform:
    R, t = T
    Rt = transpose(R)
    return Rt, scale(mat_vec(Rt, t), -1.0)


def quat_to_matrix(q: Iterable[float]) -> Matrix3:
    qx, qy, qz, qw = [float(v) for v in q]
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n <= 1e-12:
        raise ValueError("zero-length quaternion")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def matrix_to_quat(R: Matrix3) -> tuple[float, float, float, float]:
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2][1] - R[1][2]) / s
        qy = (R[0][2] - R[2][0]) / s
        qz = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        qw = (R[2][1] - R[1][2]) / s
        qx = 0.25 * s
        qy = (R[0][1] + R[1][0]) / s
        qz = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        qw = (R[0][2] - R[2][0]) / s
        qx = (R[0][1] + R[1][0]) / s
        qy = 0.25 * s
        qz = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        qw = (R[1][0] - R[0][1]) / s
        qx = (R[0][2] + R[2][0]) / s
        qy = (R[1][2] + R[2][1]) / s
        qz = 0.25 * s
    return normalize_quat((qx, qy, qz, qw))


def normalize_quat(q: Iterable[float]) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = [float(v) for v in q]
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n <= 1e-12:
        raise ValueError("zero-length quaternion")
    return qx / n, qy / n, qz / n, qw / n


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(float(roll) * 0.5)
    sr = math.sin(float(roll) * 0.5)
    cp = math.cos(float(pitch) * 0.5)
    sp = math.sin(float(pitch) * 0.5)
    cy = math.cos(float(yaw) * 0.5)
    sy = math.sin(float(yaw) * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return normalize_quat((qx, qy, qz, qw))


def quat_to_rpy(quat_xyzw: Iterable[float]) -> tuple[float, float, float]:
    qx, qy, qz, qw = normalize_quat(quat_xyzw)
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
    return float(roll), float(pitch), float(yaw)


def from_translation_quat(translation: Iterable[float], quaternion: Iterable[float]) -> Transform:
    return quat_to_matrix(quaternion), vec3(translation)


def to_translation_quat(T: Transform) -> tuple[Vector3, tuple[float, float, float, float]]:
    R, t = T
    return t, matrix_to_quat(R)


def transform_from_ros(tf_msg) -> Transform:
    tr = tf_msg.transform.translation
    rot = tf_msg.transform.rotation
    return from_translation_quat((tr.x, tr.y, tr.z), (rot.x, rot.y, rot.z, rot.w))


def axes_to_quat(x_axis: Vector3, y_axis: Vector3, z_axis: Vector3) -> tuple[float, float, float, float]:
    R: Matrix3 = (
        (x_axis[0], y_axis[0], z_axis[0]),
        (x_axis[1], y_axis[1], z_axis[1]),
        (x_axis[2], y_axis[2], z_axis[2]),
    )
    return matrix_to_quat(R)


def tool_quat_from_normal_tangent(normal_base: Vector3, tangent_base: Vector3) -> tuple[float, float, float, float]:
    z_tool = normalize(scale(normal_base, -1.0))
    tangent = sub(tangent_base, scale(z_tool, dot(tangent_base, z_tool)))
    x_tool = normalize(tangent, fallback=(1.0, 0.0, 0.0))
    if abs(dot(x_tool, z_tool)) > 0.95:
        x_tool = normalize(cross((0.0, 0.0, 1.0), z_tool), fallback=(1.0, 0.0, 0.0))
    y_tool = normalize(cross(z_tool, x_tool), fallback=(0.0, 1.0, 0.0))
    x_tool = normalize(cross(y_tool, z_tool), fallback=x_tool)
    return axes_to_quat(x_tool, y_tool, z_tool)


def distance(a: Vector3, b: Vector3) -> float:
    return norm(sub(a, b))
