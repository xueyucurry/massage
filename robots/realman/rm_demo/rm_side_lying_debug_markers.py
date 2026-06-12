#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-9:
        raise RuntimeError("zero-length vector")
    return vec / norm


def point(xyz: np.ndarray) -> Point:
    msg = Point()
    msg.x = float(xyz[0])
    msg.y = float(xyz[1])
    msg.z = float(xyz[2])
    return msg


def color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = float(r)
    msg.g = float(g)
    msg.b = float(b)
    msg.a = float(a)
    return msg


def arrow(marker_id: int, name: str, origin: np.ndarray, direction: np.ndarray, rgba: ColorRGBA, scale: float) -> Marker:
    msg = Marker()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.ns = name
    msg.id = marker_id
    msg.type = Marker.ARROW
    msg.action = Marker.ADD
    msg.points = [point(origin), point(origin + normalize(direction) * float(scale))]
    msg.scale.x = 0.01
    msg.scale.y = 0.025
    msg.scale.z = 0.035
    msg.color = rgba
    msg.lifetime = rospy.Duration(0.0)
    return msg


def label(marker_id: int, text: str, origin: np.ndarray, rgba: ColorRGBA) -> Marker:
    msg = Marker()
    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    msg.ns = "side_lying_labels"
    msg.id = marker_id
    msg.type = Marker.TEXT_VIEW_FACING
    msg.action = Marker.ADD
    msg.pose.position = point(origin)
    msg.pose.orientation.w = 1.0
    msg.scale.z = 0.035
    msg.color = rgba
    msg.text = text
    msg.lifetime = rospy.Duration(0.0)
    return msg


def build_markers(plan_path: Path, aruco_path: Path, frame_index: int) -> MarkerArray:
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    frames = list(plan["frames"])
    frame = frames[max(0, min(len(frames) - 1, int(frame_index) - 1))]
    hover = np.asarray(frame["hover_pose_m"][:3], dtype=np.float64)
    surface = np.asarray(frame["robot_point_m"][:3], dtype=np.float64)
    press = np.asarray(frame["press_direction_m"][:3], dtype=np.float64)
    tool_rot = rpy_to_matrix(*[float(v) for v in frame["hover_pose_m"][3:6]])

    markers = MarkerArray()
    mid = 1
    markers.markers.append(arrow(mid, "plan_press_direction", hover, press, color(1.0, 0.1, 0.1), 0.16))
    mid += 1
    markers.markers.append(label(mid, "saved plan press", hover + normalize(press) * 0.17, color(1.0, 0.1, 0.1)))
    mid += 1
    markers.markers.append(arrow(mid, "hover_to_surface", hover, surface - hover, color(1.0, 0.5, 0.0), 0.08))
    mid += 1
    markers.markers.append(label(mid, "hover -> visual point", surface, color(1.0, 0.5, 0.0)))
    mid += 1
    markers.markers.append(arrow(mid, "tool_pos_x_at_hover", hover, tool_rot[:, 0], color(1.0, 1.0, 0.0), 0.12))
    mid += 1
    markers.markers.append(label(mid, "tool +X", hover + tool_rot[:, 0] * 0.13, color(1.0, 1.0, 0.0)))
    mid += 1
    markers.markers.append(arrow(mid, "tool_pos_z_at_hover", hover, tool_rot[:, 2], color(0.0, 1.0, 1.0), 0.12))
    mid += 1
    markers.markers.append(label(mid, "tool +Z", hover + tool_rot[:, 2] * 0.13, color(0.0, 1.0, 1.0)))
    mid += 1

    if aruco_path.exists():
        aruco = json.loads(aruco_path.read_text(encoding="utf-8"))
        aruco_pose = aruco.get("aruco_pose_m_rpy", [])
        if len(aruco_pose) >= 6:
            aruco_origin = np.asarray(aruco_pose[:3], dtype=np.float64)
            aruco_rot = rpy_to_matrix(*[float(v) for v in aruco_pose[3:6]])
            markers.markers.append(arrow(mid, "aruco_pos_z", aruco_origin, aruco_rot[:, 2], color(0.1, 0.8, 0.1), 0.16))
            mid += 1
            markers.markers.append(label(mid, "aruco +Z", aruco_origin + aruco_rot[:, 2] * 0.17, color(0.1, 0.8, 0.1)))
            mid += 1
            markers.markers.append(arrow(mid, "aruco_neg_z", aruco_origin, -aruco_rot[:, 2], color(0.2, 0.2, 1.0), 0.16))
            mid += 1
            markers.markers.append(label(mid, "aruco -Z", aruco_origin - aruco_rot[:, 2] * 0.17, color(0.2, 0.2, 1.0)))

    return markers


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan-json", required=True)
    parser.add_argument("--aruco-json", default="rm_demo_output/aruco_latest.json")
    parser.add_argument("--frame-index", type=int, default=1)
    parser.add_argument("--rate", type=float, default=2.0)
    args = parser.parse_args()

    rospy.init_node("rm_side_lying_debug_markers", anonymous=True)
    pub = rospy.Publisher("/rm_demo/side_lying_debug_markers", MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(max(0.2, float(args.rate)))
    plan_path = Path(args.plan_json)
    aruco_path = Path(args.aruco_json)
    while not rospy.is_shutdown():
        pub.publish(build_markers(plan_path, aruco_path, args.frame_index))
        rate.sleep()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
