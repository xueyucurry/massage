from __future__ import annotations

import glob
import os
import sys
import time
from typing import Any

import numpy as np

from .config import ROS_VENDOR_PYTHON_DIR
from .rm_ros import create_arm_backend


def _import_ros_modules():
    try:
        import rospy  # type: ignore
        import actionlib  # type: ignore
        from cv_bridge import CvBridge  # type: ignore
        from rm_healthcare_robot_msgs.msg import CalcPositionVectorAction, CalcPositionVectorGoal, PixelCoor  # type: ignore
        from rm_healthcare_robot_msgs.srv import WaypointPosesCalc  # type: ignore
    except Exception:
        candidates = []
        candidates.extend(glob.glob("/opt/ros/*/lib/python3/dist-packages"))
        candidates.append(ROS_VENDOR_PYTHON_DIR)
        candidates.append("/home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/lib/python3/dist-packages")
        for candidate in candidates:
            if candidate and os.path.isdir(candidate) and candidate not in sys.path:
                sys.path.append(candidate)
        import rospy  # type: ignore
        import actionlib  # type: ignore
        from cv_bridge import CvBridge  # type: ignore
        from rm_healthcare_robot_msgs.msg import CalcPositionVectorAction, CalcPositionVectorGoal, PixelCoor  # type: ignore
        from rm_healthcare_robot_msgs.srv import WaypointPosesCalc  # type: ignore

    return rospy, actionlib, CvBridge, CalcPositionVectorAction, CalcPositionVectorGoal, PixelCoor, WaypointPosesCalc


def _mk_pixel(PixelCoor, x: int, y: int):
    point = PixelCoor()
    point.x = int(x)
    point.y = int(y)
    return point


def _default_bbox_from_points(points: list[list[int]], image_width: int, image_height: int) -> list[list[int]]:
    xs = [int(p[0]) for p in points]
    ys = [int(p[1]) for p in points]
    pad_x = max(20, int((max(xs) - min(xs)) * 0.5))
    pad_y = max(40, int((max(ys) - min(ys)) * 0.6))
    x1 = max(0, min(xs) - pad_x)
    x2 = min(image_width - 1, max(xs) + pad_x)
    y1 = max(0, min(ys) - pad_y)
    y2 = min(image_height - 1, max(ys) + pad_y)
    return [[x1, y1], [x2, y1], [x1, y2], [x2, y2]]


def attach_robot_points_via_product_services(
    *,
    color_bgr,
    depth_m,
    detection_result: dict[str, Any],
    host: str,
    install_ang: list[float],
    control_backend: str = "json",
    joints_deg_override: list[float] | None = None,
) -> dict[str, Any]:
    rospy, actionlib, CvBridge, CalcPositionVectorAction, CalcPositionVectorGoal, PixelCoor, WaypointPosesCalc = _import_ros_modules()
    if not rospy.core.is_initialized():
        rospy.init_node("rm_demo_product_ros", anonymous=True, disable_signals=True)

    selected_side = str(detection_result.get("selected_side", ""))
    if selected_side not in ("left", "right"):
        raise RuntimeError("selected_side missing before product ROS transform")
    selected_pixels = list(detection_result.get("selected_meridian_pixel", []))
    if len(selected_pixels) < 2:
        raise RuntimeError("selected_meridian_pixel is empty before product ROS transform")

    image_h, image_w = color_bgr.shape[:2]
    bbox = detection_result.get("body_bbox_pixel")
    if not isinstance(bbox, list) or len(bbox) != 4:
        bbox = _default_bbox_from_points(selected_pixels, image_w, image_h)

    bridge = CvBridge()
    depth_u16 = np.clip(np.round(depth_m * 1000.0), 0, 65535).astype(np.uint16)

    goal = CalcPositionVectorGoal()
    goal.color_image = bridge.cv2_to_imgmsg(color_bgr, encoding="bgr8")
    goal.depth_image = bridge.cv2_to_imgmsg(depth_u16, encoding="16UC1")
    goal.diagonal_point_coor = [_mk_pixel(PixelCoor, int(x), int(y)) for x, y in bbox]
    goal.waypoints_pixel_coor = [_mk_pixel(PixelCoor, int(x), int(y)) for x, y in selected_pixels]

    client = actionlib.SimpleActionClient("/ai_service/calc_position_normal", CalcPositionVectorAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        raise RuntimeError("calc_position_normal action server is unavailable")
    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(20.0)):
        raise RuntimeError("calc_position_normal action timed out")
    result = client.get_result()
    if result is None:
        raise RuntimeError("calc_position_normal action returned no result")
    camera_waypoints = list(result.waypoints_position_vector)
    if not camera_waypoints:
        raise RuntimeError("calc_position_normal returned zero waypoints")

    joints_deg = None
    if joints_deg_override:
        if len(joints_deg_override) < 6:
            raise RuntimeError(f"joints_deg_override must contain 6 values, got {len(joints_deg_override)}")
        joints_deg = [float(v) for v in joints_deg_override[:6]]
    else:
        arm = create_arm_backend(control_backend)
        last_exc: Exception | None = None
        for timeout_s in (1.5, 3.0, 5.0):
            try:
                joints_deg, _, _, _, _ = arm.get_current_arm_state(host, timeout=timeout_s)
                break
            except Exception as exc:
                last_exc = exc
                time.sleep(0.2)
        if joints_deg is None:
            raise RuntimeError(f"product_ros failed to read current arm state: {last_exc}")
    rospy.wait_for_service("/calc_poses", timeout=10.0)
    calc_poses = rospy.ServiceProxy("/calc_poses", WaypointPosesCalc)
    pose_response = calc_poses(list(joints_deg), [float(v) for v in install_ang[:3]], camera_waypoints)

    world_poses = list(pose_response.waypoint_poses)
    if not world_poses:
        raise RuntimeError("calc_poses returned zero waypoint poses")

    updated = dict(detection_result)
    robot_points = [[float(p.position.x), float(p.position.y), float(p.position.z)] for p in world_poses]
    robot_pose_quat = [
        [
            float(p.position.x),
            float(p.position.y),
            float(p.position.z),
            float(p.orientation.x),
            float(p.orientation.y),
            float(p.orientation.z),
            float(p.orientation.w),
        ]
        for p in world_poses
    ]
    product_camera_points = [
        [float(w.point.x), float(w.point.y), float(w.point.z)]
        for w in camera_waypoints
    ]
    product_camera_vectors = [
        [float(w.vector.x), float(w.vector.y), float(w.vector.z)]
        for w in camera_waypoints
    ]
    updated["body_bbox_pixel"] = bbox
    updated["selected_meridian_robot"] = robot_points
    updated["selected_meridian_robot_pose_quat"] = robot_pose_quat
    updated["product_camera_waypoints"] = product_camera_points
    updated["product_camera_vectors"] = product_camera_vectors
    updated[f"{selected_side}_meridian_robot"] = robot_points
    updated[f"{selected_side}_meridian_robot_pose_quat"] = robot_pose_quat
    updated["robot_frame_unit"] = "meters"
    updated["transform_backend"] = "product_ros"
    updated["control_backend"] = str(control_backend)
    updated["calc_poses_joints_deg"] = [float(v) for v in joints_deg[:6]]
    updated["calc_poses_joints_source"] = "override" if joints_deg_override else "current_arm_state"
    return updated
