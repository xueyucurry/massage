#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        raise RuntimeError("zero-length vector")
    return vec / norm


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def axis_vector(axis: str) -> np.ndarray:
    axes = {
        "pos_x": [1.0, 0.0, 0.0],
        "neg_x": [-1.0, 0.0, 0.0],
        "pos_y": [0.0, 1.0, 0.0],
        "neg_y": [0.0, -1.0, 0.0],
        "pos_z": [0.0, 0.0, 1.0],
        "neg_z": [0.0, 0.0, -1.0],
    }
    if axis not in axes:
        raise ValueError(f"unsupported axis: {axis}")
    return np.asarray(axes[axis], dtype=np.float64)


def build_split_axis(press: np.ndarray, tangent: np.ndarray) -> np.ndarray:
    for candidate in (np.cross(press, tangent), np.cross(press, np.array([0.0, 0.0, 1.0])), np.cross(press, np.array([0.0, 1.0, 0.0]))):
        norm = float(np.linalg.norm(candidate))
        if norm > 1e-9:
            return candidate / norm
    return np.array([0.0, 1.0, 0.0], dtype=np.float64)


def main() -> int:
    parser = argparse.ArgumentParser(description="Retarget a saved bladder plan to use a chosen tool axis as contact normal.")
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--reference-frame", type=int, default=1)
    parser.add_argument("--tool-axis", choices=("pos_x", "neg_x", "pos_y", "neg_y", "pos_z", "neg_z"), default="pos_z")
    parser.add_argument(
        "--sign",
        choices=("auto", "positive", "negative"),
        default="auto",
        help="contact direction sign along selected tool axis; auto keeps hover on the anchor side of the surface",
    )
    args = parser.parse_args()

    src = Path(args.input)
    data = json.loads(src.read_text(encoding="utf-8"))
    frames = list(data["frames"])
    if not frames:
        raise RuntimeError("input plan has no frames")

    ref = frames[max(0, min(len(frames) - 1, int(args.reference_frame) - 1))]
    ref_rot = rpy_to_matrix(*[float(v) for v in ref["hover_pose_m"][3:6]])
    contact = normalize(ref_rot @ axis_vector(args.tool_axis))

    if args.sign == "negative":
        contact = -contact
    elif args.sign == "auto":
        first_point = np.asarray(frames[0]["robot_point_m"][:3], dtype=np.float64)
        anchor = np.asarray(data.get("anchor_pose_m", ref["hover_pose_m"])[:3], dtype=np.float64)
        hover_pos = first_point - contact * float(data["hover_m"])
        hover_neg = first_point + contact * float(data["hover_m"])
        if float(np.linalg.norm(hover_neg - anchor)) < float(np.linalg.norm(hover_pos - anchor)):
            contact = -contact

    hover_m = float(data["hover_m"])
    dian_depth_m = float(data["dian_jin_depth_m"])
    fen_m = float(data["fen_jin_lateral_m"])
    prev_split: np.ndarray | None = None
    new_frames = []
    for frame in frames:
        point = np.asarray(frame["robot_point_m"][:3], dtype=np.float64)
        tangent = normalize(np.asarray(frame.get("tangent_axis_m") or [0.0, 1.0, 0.0], dtype=np.float64))
        split = build_split_axis(contact, tangent)
        if prev_split is not None and float(np.dot(split, prev_split)) < 0.0:
            split = -split
        prev_split = split
        rpy = [float(v) for v in ref["hover_pose_m"][3:6]]
        base = [float(point[0]), float(point[1]), float(point[2]), *rpy]
        hover = point - contact * hover_m
        dian = point - contact * max(0.0, hover_m - dian_depth_m)
        fen_pos = hover + split * fen_m
        fen_neg = hover - split * fen_m
        new_frame = dict(frame)
        new_frame["hover_pose_m"] = [float(hover[0]), float(hover[1]), float(hover[2]), *rpy]
        new_frame["dian_jin_pose_m"] = [float(dian[0]), float(dian[1]), float(dian[2]), *rpy]
        new_frame["fen_positive_pose_m"] = [float(fen_pos[0]), float(fen_pos[1]), float(fen_pos[2]), *rpy]
        new_frame["fen_negative_pose_m"] = [float(fen_neg[0]), float(fen_neg[1]), float(fen_neg[2]), *rpy]
        new_frame["press_direction_m"] = [float(v) for v in contact.tolist()]
        new_frame["split_axis_m"] = [float(v) for v in split.tolist()]
        new_frame["base_pose_m"] = base
        new_frame["normal_source"] = {
            "tool_axis": args.tool_axis,
            "reference_frame": int(args.reference_frame),
            "sign": args.sign,
        }
        new_frames.append(new_frame)

    data["frames"] = new_frames
    data["point_count"] = len(new_frames)
    data["normal_source"] = {
        "type": "saved_plan_tool_axis",
        "tool_axis": args.tool_axis,
        "reference_frame": int(args.reference_frame),
        "sign": args.sign,
        "contact_direction_m": [float(v) for v in contact.tolist()],
        "source_plan": str(src),
    }
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"output={out}")
    print(f"contact_direction={[round(float(v), 6) for v in contact.tolist()]}")
    print(f"first_hover={[round(float(v), 6) for v in new_frames[0]['hover_pose_m'][:3]]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
