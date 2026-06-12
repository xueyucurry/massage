from __future__ import annotations

import glob
import os
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any

from .config import ROS_VENDOR_PYTHON_DIR


@dataclass
class ProductTrajectoryUploadResult:
    project_name: str
    file_size: int
    plan_speed: int
    chunk_count: int
    receive_state: bool | None
    verify_state: bool | None
    executed: bool
    run_state: bool | None

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
        from rm_msgs.msg import RunProject, TrajectoryFile  # type: ignore
        from rm_msgs.srv import Set_Control_Plan_Number, Set_Arm_Trajectory_Srv  # type: ignore
        from std_msgs.msg import Bool, Empty  # type: ignore
    except Exception:
        _append_ros_python_candidates()
        import rospy  # type: ignore
        from rm_msgs.msg import RunProject, TrajectoryFile  # type: ignore
        from rm_msgs.srv import Set_Control_Plan_Number, Set_Arm_Trajectory_Srv  # type: ignore
        from std_msgs.msg import Bool, Empty  # type: ignore
    return rospy, RunProject, TrajectoryFile, Set_Control_Plan_Number, Set_Arm_Trajectory_Srv, Bool, Empty


class ProductTrajectoryExecutor:
    def __init__(self) -> None:
        (
            self.rospy,
            self.RunProject,
            self.TrajectoryFile,
            self.Set_Control_Plan_Number,
            self.Set_Arm_Trajectory_Srv,
            self.Bool,
            self.Empty,
        ) = _import_ros_modules()
        if not self.rospy.core.is_initialized():
            self.rospy.init_node("rm_demo_product_executor", anonymous=True, disable_signals=True)

        self.receive_state: bool | None = None
        self.receive_state_time = 0.0
        self.verify_state: bool | None = None
        self.verify_state_time = 0.0
        self.run_state: bool | None = None
        self.run_state_time = 0.0

        self.pub_prepare = self.rospy.Publisher("/rm_driver/PrepareRunProject", self.RunProject, queue_size=1)
        self.pub_file = self.rospy.Publisher("/rm_driver/SendTrajectoryFile", self.TrajectoryFile, queue_size=10)
        self.sub_receive = self.rospy.Subscriber(
            "/rm_driver/ReceiveTrajectoryFileState", self.Bool, self._on_receive_state, queue_size=5
        )
        self.sub_verify = self.rospy.Subscriber(
            "/rm_driver/TrajectoryFileVerifyState", self.Bool, self._on_verify_state, queue_size=5
        )
        self.sub_run = self.rospy.Subscriber("/rm_driver/RunProjectState", self.Bool, self._on_run_state, queue_size=5)
        self.rospy.sleep(0.2)

    def _on_receive_state(self, msg) -> None:
        self.receive_state = bool(msg.data)
        self.receive_state_time = time.time()

    def _on_verify_state(self, msg) -> None:
        self.verify_state = bool(msg.data)
        self.verify_state_time = time.time()

    def _on_run_state(self, msg) -> None:
        self.run_state = bool(msg.data)
        self.run_state_time = time.time()

    def _wait_for_publishers(self, timeout_s: float) -> None:
        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            if self.pub_prepare.get_num_connections() > 0 and self.pub_file.get_num_connections() > 0:
                return
            self.rospy.sleep(0.05)
        raise RuntimeError("product executor publishers are not connected to rm_driver")

    def _call_empty_service(self, service_name: str, timeout_s: float = 5.0) -> bool:
        self.rospy.wait_for_service(service_name, timeout=float(timeout_s))
        proxy = self.rospy.ServiceProxy(service_name, self.Set_Arm_Trajectory_Srv)
        response = proxy(self.Empty())
        return bool(response.state)

    def clear_controller_trajectory(self, *, timeout_s: float = 5.0) -> dict[str, bool]:
        return {
            "delete_current": self._call_empty_service("/deletecurrenttrajectory", timeout_s=timeout_s),
            "delete_all": self._call_empty_service("/deletetrajectory", timeout_s=timeout_s),
        }

    def upload(
        self,
        *,
        trajectory_content: str,
        project_name: str,
        plan_speed: int,
        clear_existing: bool = True,
        execute: bool = False,
        timeout_s: float = 20.0,
    ) -> ProductTrajectoryUploadResult:
        content_bytes = trajectory_content.encode("utf-8")
        if not content_bytes:
            raise RuntimeError("trajectory_content is empty")

        self._wait_for_publishers(timeout_s=timeout_s)
        if clear_existing:
            self.clear_controller_trajectory(timeout_s=min(5.0, float(timeout_s)))

        before_time = time.time()
        self.receive_state = None
        self.verify_state = None
        self.run_state = None

        prepare = self.RunProject()
        prepare.project_name = str(project_name)
        prepare.file_size = int(len(content_bytes))
        prepare.plan_speed = int(plan_speed)
        self.pub_prepare.publish(prepare)
        self.rospy.sleep(0.25)

        chunk_size = 2048
        chunk_count = 0
        for offset in range(0, len(content_bytes), chunk_size):
            chunk = content_bytes[offset : offset + chunk_size]
            msg = self.TrajectoryFile()
            msg.buffer = chunk + b"\0" * (2049 - len(chunk))
            msg.len = int(len(chunk))
            self.pub_file.publish(msg)
            chunk_count += 1
            self.rospy.sleep(0.02)

        deadline = time.time() + float(timeout_s)
        while time.time() < deadline:
            receive_ready = self.receive_state_time >= before_time and self.receive_state is not None
            verify_ready = self.verify_state_time >= before_time and self.verify_state is not None
            if receive_ready and verify_ready:
                break
            self.rospy.sleep(0.05)

        if self.receive_state is False:
            raise RuntimeError(
                "product controller rejected trajectory file receive "
                f"(receive_state={self.receive_state}, verify_state={self.verify_state}, "
                f"file_size={len(content_bytes)}, chunks={chunk_count}, project_name={project_name!r})"
            )
        if self.verify_state is False:
            raise RuntimeError(
                "product controller trajectory file verification failed "
                f"(receive_state={self.receive_state}, verify_state={self.verify_state}, "
                f"file_size={len(content_bytes)}, chunks={chunk_count}, project_name={project_name!r})"
            )

        if execute:
            self.rospy.wait_for_service("/control_plan_number", timeout=5.0)
            proxy = self.rospy.ServiceProxy("/control_plan_number", self.Set_Control_Plan_Number)
            response = proxy(True)
            if not bool(response.state):
                raise RuntimeError("control_plan_number rejected trajectory execution")

        return ProductTrajectoryUploadResult(
            project_name=str(project_name),
            file_size=len(content_bytes),
            plan_speed=int(plan_speed),
            chunk_count=chunk_count,
            receive_state=self.receive_state,
            verify_state=self.verify_state,
            executed=bool(execute),
            run_state=self.run_state,
        )


def upload_product_trajectory(
    *,
    trajectory_content: str,
    project_name: str,
    plan_speed: int,
    clear_existing: bool = True,
    execute: bool = False,
    timeout_s: float = 20.0,
) -> ProductTrajectoryUploadResult:
    executor = ProductTrajectoryExecutor()
    return executor.upload(
        trajectory_content=trajectory_content,
        project_name=project_name,
        plan_speed=plan_speed,
        clear_existing=clear_existing,
        execute=execute,
        timeout_s=timeout_s,
    )
