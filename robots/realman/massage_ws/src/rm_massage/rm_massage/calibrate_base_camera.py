from __future__ import annotations

import argparse
from typing import Any

from .config_io import load_yaml, save_yaml
from .transforms import compose, from_translation_quat, inverse, matrix_to_quat, mat_vec, to_translation_quat


def rigid_transform_3d(board_points: list[list[float]], base_points: list[list[float]]):
    import numpy as np

    A = np.asarray(board_points, dtype=float)
    B = np.asarray(base_points, dtype=float)
    if A.shape != B.shape:
        raise ValueError(f"point arrays must have same shape: {A.shape} vs {B.shape}")
    if A.ndim != 2 or A.shape[1] != 3 or A.shape[0] < 4:
        raise ValueError("need at least four 3D point pairs")

    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    predicted = (R @ A.T).T + t
    errors = np.linalg.norm(predicted - B, axis=1)
    return R, t, errors


def _matrix_tuple(R: Any):
    return tuple(tuple(float(R[i, j]) for j in range(3)) for i in range(3))


def _transform_map(T) -> dict[str, Any]:
    t, q = to_translation_quat(T)
    return {
        "translation": [float(v) for v in t],
        "quaternion": [float(v) for v in q],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Solve T_base_camera from board/base point pairs.")
    parser.add_argument("--input", required=True, help="YAML with board_points, base_points, optional T_camera_board")
    parser.add_argument("--output", default="config/T_base_camera.yaml")
    args = parser.parse_args()

    data = load_yaml(args.input)
    board_points = data.get("board_points") or data.get("points_board")
    base_points = data.get("base_points") or data.get("points_base")
    if not board_points or not base_points:
        raise ValueError("input must contain board_points and base_points")

    R, t, errors = rigid_transform_3d(board_points, base_points)
    T_base_board = (_matrix_tuple(R), (float(t[0]), float(t[1]), float(t[2])))

    output: dict[str, Any] = {
        "T_base_board": _transform_map(T_base_board),
        "registration_error_m": {
            "mean": float(errors.mean()),
            "max": float(errors.max()),
            "per_point": [float(v) for v in errors],
        },
    }

    T_camera_board_data = data.get("T_camera_board")
    if T_camera_board_data:
        T_camera_board = from_translation_quat(
            T_camera_board_data["translation"],
            T_camera_board_data["quaternion"],
        )
        T_base_camera = compose(T_base_board, inverse(T_camera_board))
        output["T_camera_board"] = _transform_map(T_camera_board)
        output["T_base_camera"] = _transform_map(T_base_camera)

    save_yaml(args.output, output)
    print(f"mean_error_m: {float(errors.mean()):.6f}")
    print(f"max_error_m:  {float(errors.max()):.6f}")
    if "T_base_camera" in output:
        tbc = output["T_base_camera"]
        vals = tbc["translation"] + tbc["quaternion"]
        print("static_transform_args:")
        print(" ".join(f"{v:.9f}" for v in vals) + " robot_base camera_color_optical_frame")
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
