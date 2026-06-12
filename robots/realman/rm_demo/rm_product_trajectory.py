from __future__ import annotations

import glob
import os
import sys
from dataclasses import dataclass, asdict
from typing import Any

from .config import ROS_VENDOR_PYTHON_DIR


@dataclass
class ProductRubbingTrajectory:
    trajectory_content: str
    open_force_num_list: list[int]
    stop_force_num_list: list[int]
    service_name: str
    tool_name: str
    force: int
    speed: int
    trajectory_type: int
    waypoint_count: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _append_ros_python_candidates() -> None:
    candidates: list[str] = []
    candidates.extend(glob.glob("/opt/ros/*/lib/python3/dist-packages"))
    candidates.append(ROS_VENDOR_PYTHON_DIR)
    candidates.append("/home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/lib/python3/dist-packages")
    for candidate in candidates:
        if candidate and os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.append(candidate)


def _import_ros_modules():
    try:
        import rospy  # type: ignore
        from geometry_msgs.msg import Point, Vector3  # type: ignore
        from rm_healthcare_robot_msgs.msg import WaypointPositionVector  # type: ignore
        from rm_healthcare_robot_msgs.srv import RubbingTrajectory  # type: ignore
    except Exception:
        _append_ros_python_candidates()
        import rospy  # type: ignore
        from geometry_msgs.msg import Point, Vector3  # type: ignore
        from rm_healthcare_robot_msgs.msg import WaypointPositionVector  # type: ignore
        from rm_healthcare_robot_msgs.srv import RubbingTrajectory  # type: ignore
    return rospy, Point, Vector3, WaypointPositionVector, RubbingTrajectory


def _xyz_from_obj(value: Any, *, key: str) -> list[float]:
    if hasattr(value, key):
        value = getattr(value, key)
    if isinstance(value, dict):
        return [float(value[axis]) for axis in ("x", "y", "z")]
    if isinstance(value, (list, tuple)):
        if len(value) < 3:
            raise RuntimeError(f"expected at least 3 values for {key}, got {len(value)}")
        return [float(value[0]), float(value[1]), float(value[2])]
    return [float(getattr(value, axis)) for axis in ("x", "y", "z")]


def _build_waypoints(
    waypoint_points: list[Any],
    waypoint_vectors: list[Any],
):
    rospy, Point, Vector3, WaypointPositionVector, _ = _import_ros_modules()
    del rospy
    if len(waypoint_points) != len(waypoint_vectors):
        raise RuntimeError(
            "product waypoint points/vectors length mismatch: "
            f"{len(waypoint_points)} != {len(waypoint_vectors)}"
        )
    if len(waypoint_points) < 2:
        raise RuntimeError("product waypoint count is insufficient")

    waypoints = []
    for point_value, vector_value in zip(waypoint_points, waypoint_vectors):
        point_xyz = _xyz_from_obj(point_value, key="point")
        vector_xyz = _xyz_from_obj(vector_value, key="vector")
        waypoint = WaypointPositionVector()
        waypoint.point = Point(*point_xyz)
        waypoint.vector = Vector3(*vector_xyz)
        waypoints.append(waypoint)
    return waypoints


def generate_rubbing_trajectory(
    *,
    joints_deg: list[float],
    install_ang: list[float],
    tool_name: str,
    waypoint_points: list[Any],
    waypoint_vectors: list[Any],
    force: int,
    speed: int,
    trajectory_type: int,
    service_name: str = "/generate_trajectory_rubbing",
    timeout_s: float = 20.0,
) -> ProductRubbingTrajectory:
    """Ask the product trajectory service to generate native massage content."""
    rospy, _, _, _, RubbingTrajectory = _import_ros_modules()
    if not rospy.core.is_initialized():
        rospy.init_node("rm_demo_product_trajectory", anonymous=True, disable_signals=True)

    waypoints = _build_waypoints(waypoint_points, waypoint_vectors)
    rospy.wait_for_service(service_name, timeout=float(timeout_s))
    proxy = rospy.ServiceProxy(service_name, RubbingTrajectory)
    response = proxy(
        [float(v) for v in joints_deg[:6]],
        [float(v) for v in install_ang[:3]],
        str(tool_name),
        waypoints,
        int(force),
        int(speed),
        int(trajectory_type),
    )
    return ProductRubbingTrajectory(
        trajectory_content=str(response.trajectory_content),
        open_force_num_list=[int(v) for v in response.open_force_num_list],
        stop_force_num_list=[int(v) for v in response.stop_force_num_list],
        service_name=str(service_name),
        tool_name=str(tool_name),
        force=int(force),
        speed=int(speed),
        trajectory_type=int(trajectory_type),
        waypoint_count=len(waypoints),
    )
