from __future__ import annotations

import glob
import os
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any

from .config import ROS_VENDOR_PYTHON_DIR


@dataclass
class TemperatureResult:
    target_c: float | None
    set_state: bool | None
    current_c: float | None
    reached: bool | None
    elapsed_s: float

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
        from rm_healthcare_robot_msgs.srv import get_temperature_Srv, set_temperature_Srv  # type: ignore
        from std_msgs.msg import Empty  # type: ignore
    except Exception:
        _append_ros_python_candidates()
        import rospy  # type: ignore
        from rm_healthcare_robot_msgs.srv import get_temperature_Srv, set_temperature_Srv  # type: ignore
        from std_msgs.msg import Empty  # type: ignore
    return rospy, get_temperature_Srv, set_temperature_Srv, Empty


def get_temperature(*, timeout_s: float = 5.0) -> float:
    rospy, get_temperature_Srv, _, Empty = _import_ros_modules()
    if not rospy.core.is_initialized():
        rospy.init_node("rm_demo_temperature", anonymous=True, disable_signals=True)
    rospy.wait_for_service("/temperature_get_srv", timeout=float(timeout_s))
    proxy = rospy.ServiceProxy("/temperature_get_srv", get_temperature_Srv)
    response = proxy(Empty())
    return float(response.temp_value)


def set_temperature(
    *,
    target_c: float,
    wait_s: float = 0.0,
    tolerance_c: float = 2.0,
    poll_s: float = 1.0,
    timeout_s: float = 5.0,
) -> TemperatureResult:
    rospy, get_temperature_Srv, set_temperature_Srv, Empty = _import_ros_modules()
    if not rospy.core.is_initialized():
        rospy.init_node("rm_demo_temperature", anonymous=True, disable_signals=True)

    started = time.time()
    rospy.wait_for_service("/temperature_set_srv", timeout=float(timeout_s))
    set_proxy = rospy.ServiceProxy("/temperature_set_srv", set_temperature_Srv)
    set_response = set_proxy(float(target_c))
    set_state = bool(set_response.state)

    current_c: float | None = None
    reached: bool | None = None
    if wait_s > 0.0:
        rospy.wait_for_service("/temperature_get_srv", timeout=float(timeout_s))
        get_proxy = rospy.ServiceProxy("/temperature_get_srv", get_temperature_Srv)
        deadline = time.time() + float(wait_s)
        reached = False
        while time.time() < deadline:
            current_c = float(get_proxy(Empty()).temp_value)
            if abs(current_c - float(target_c)) <= float(tolerance_c):
                reached = True
                break
            time.sleep(max(0.1, float(poll_s)))
    else:
        try:
            current_c = get_temperature(timeout_s=min(2.0, float(timeout_s)))
        except Exception:
            current_c = None

    return TemperatureResult(
        target_c=float(target_c),
        set_state=set_state,
        current_c=current_c,
        reached=reached,
        elapsed_s=float(time.time() - started),
    )
