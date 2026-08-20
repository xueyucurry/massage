"""
ft.py - 膀胱经恒力按摩动作演示程序（ROS 2 控制版）

复用 lasttime.py 的视觉检测与轨迹生成流程，
将机械臂控制从 Python SDK 直连切换到 FAIRINO ROS 2 service/topic。
"""

import os
import sys
import time
import math
import threading
import json
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

# ===== 用户常改：各部位/动作按摩力度，单位 N =====
# 直接改下面的数字即可。
BACK_MASSAGE_FORCE_N = "10.0"          # 背部膀胱经
BACK_SHUN_JIN_FORCE_N = "2.0"          # 背部膀胱经顺筋
THIGH_OUTER_MASSAGE_FORCE_N = "50.0"   # 大腿外侧
THIGH_INNER_MASSAGE_FORCE_N = "30.0"   # 大腿内侧

# ===== 用户常改：动作次数和速度 =====
DIAN_JIN_REPEAT_DEFAULT = "3"           # 每个按摩点执行点筋次数
FEN_JIN_REPEAT_DEFAULT = "3"            # 每个按摩点执行分筋次数
DIAN_JIN_MODE_DEFAULT = "dian"           # 默认执行真正点筋；small_fen 仅作显式兼容模式
ROBOT_MOTION_SPEED_SCALE_DEFAULT = "2.0"  # 所有机械臂运动速度倍率
SHUN_JIN_MOTION_SPEED_SCALE_DEFAULT = "1.0" # 顺筋动作速度倍率，1.0 为原速

# ===== 用户常改：顺筋表皮保护参数 =====
SHUN_JIN_SEGMENT_LENGTH_DEFAULT_MM = "25.0" # 单次连续接触的最大轨迹长度
SHUN_JIN_RELEASE_LIFT_DEFAULT_MM = "4.0"    # 每段结束后的卸力抬升距离
SHUN_JIN_RELEASE_DWELL_DEFAULT_S = "0.3"    # 抬升后等待表皮回弹时间
SHUN_JIN_TANGENTIAL_WARN_DEFAULT_N = "2.0"  # 切向力预警值
SHUN_JIN_TANGENTIAL_RELEASE_DEFAULT_N = "3.0" # 切向力达到该值时提前卸力

# ===== 用户常改：贴近提速参数 =====
BACK_HOVER_HEIGHT_DEFAULT_MM = "20.0"      # 背部悬空距离，越小越快，建议不低于 15
THIGH_HOVER_HEIGHT_DEFAULT_MM = "20.0"     # 大腿悬空距离，越小越快，建议不低于 15
FORCE_APPROACH_SPEED_SCALE_DEFAULT = "3.0" # 贴近速度倍率
FORCE_APPROACH_STEP_DEFAULT_MM = "3.0"     # 粗贴近步长

# ===== 用户常改：点位裁剪 =====
THIGH_OUTER_TAIL_SKIP_POINTS_DEFAULT = "3" # 大腿外侧中线末尾不按摩的点数
THIGH_INNER_TAIL_SKIP_POINTS_DEFAULT = "1" # 大腿内侧末尾不按摩的点数

import cv2
import numpy as np
import torch
import rclpy
from rclpy.node import Node

from fairino_msgs.msg import RobotNonrtState
from fairino_msgs.srv import RemoteCmdInterface
try:
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState

    MOVEIT_MSGS_AVAILABLE = True
except Exception:
    Duration = None
    GetPositionIK = None
    JointState = None
    PoseStamped = None
    MOVEIT_MSGS_AVAILABLE = False
from force_control import (
    FORCE_SENSOR_COMPANY,
    FORCE_SENSOR_DEVICE,
    ForceControlConfig,
    SENSOR_ID,
    _tool_z_unit_from_rpy,
)
from lasttime import (
    BLEND_BLOCKING,
    DIAN_JIN_DEPTH_MM,
    ENABLE_LIVE_PREVIEW_WINDOW,
    FEN_JIN_LATERAL_MM,
    HOVER_HEIGHT_MM,
    INIT_POSE_P24,
    INIT_SAFE_Z_MM,
    LastTimeDemo as _SdkLastTimeDemo,
    MOVE_VEL_FAST,
    MOVE_VEL_SLOW,
    PLANE_FIT_MIN_POINTS,
    PLANE_FIT_RADIUS_PX,
    PLANE_FIT_STEP_PX,
    ROBOT_IP,
    SAMPLE_POINTS,
    LIVE_PREVIEW_WINDOW_NAME,
    build_pose_from_frame,
    _build_split_axis,
    _camera_origin_mm_from_matrix,
    _estimate_patch_plane_normal_camera,
    _fallback_tool_z_axis,
    _smooth_unit_vectors,
    _transform_vector_camera_to_robot,
    _normalize_vec,
    _rpy_from_tool_z_vector,
)
from dianjing import _load_camera_to_robot_matrix, _transform_points
from thigh_outerline_confirm import (
    DIRECTION_MODES as THIGH_DIRECTION_MODES,
    RealSenseReader as ThighRealSenseReader,
    RTMPoseHipKneeDetector,
    build_offset_line as build_thigh_offset_line,
    detect_pose as detect_thigh_pose,
    draw_polyline as draw_thigh_polyline,
    draw_text_box as draw_thigh_text_box,
    estimate_outward_direction as estimate_thigh_outward_direction,
    sample_depth_m as sample_thigh_depth_m,
    save_confirmation as save_thigh_confirmation,
)
from rtmpose_detector import DEFAULT_RTMPOSE_CONFIG, DEFAULT_RTMPOSE_WEIGHTS, ROTATIONS


ROS2_WORKSPACE = str(SCRIPT_DIR / "fairino_ros2" / "frcobot_ros2-master")
ROS2_SERVICE_NAME = os.environ.get("FAIRINO_REMOTE_SERVICE", "fairino_remote_command_service")
ROS2_STATE_TOPIC = os.environ.get("FAIRINO_STATE_TOPIC", "nonrt_state_data")
ROS2_SERVICE_WAIT_S = float(os.environ.get("ROS2_SERVICE_WAIT_S", "20.0"))
ROS2_CALL_TIMEOUT_S = float(os.environ.get("ROS2_CALL_TIMEOUT_S", "20.0"))
ROS2_STATE_WAIT_S = float(os.environ.get("ROS2_STATE_WAIT_S", "12.0"))
ROS2_MOTION_DONE_WAIT_S = float(os.environ.get("ROS2_MOTION_DONE_WAIT_S", "15.0"))
ROS2_MOTION_DONE_POSE_TOL_MM = float(os.environ.get("ROS2_MOTION_DONE_POSE_TOL_MM", "3.0"))
ROS2_MOTION_DONE_ORI_TOL_DEG = float(os.environ.get("ROS2_MOTION_DONE_ORI_TOL_DEG", "3.0"))
ROS2_MOTION_DONE_JOINT_TOL_DEG = float(
    os.environ.get("ROS2_MOTION_DONE_JOINT_TOL_DEG", "1.0")
)
ROS2_MOTION_DONE_STABLE_SAMPLES = max(
    1,
    int(os.environ.get("ROS2_MOTION_DONE_STABLE_SAMPLES", "2")),
)
ROS2_MOTION_DONE_TOL_FRACTION = max(
    0.01,
    min(1.0, float(os.environ.get("ROS2_MOTION_DONE_TOL_FRACTION", "0.25"))),
)
ROS2_MOTION_STATE_POLL_S = max(
    0.001,
    float(os.environ.get("ROS2_MOTION_STATE_POLL_S", "0.02")),
)
ROS2_MOVE_ACC = float(os.environ.get("ROS2_MOVE_ACC", "0"))
ROS2_MOVE_OVL = float(os.environ.get("ROS2_MOVE_OVL", "100"))
ROS2_TOOL = int(os.environ.get("ROBOT_TOOL_ID", "0"))
ROS2_USER = int(os.environ.get("ROBOT_USER_ID", "0"))
ROS2_SERVO_INTERPOLATION = os.environ.get(
    "FT_SERVO_INTERPOLATION", "1"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
ROS2_SERVO_INTERPOLATION_HZ = max(
    62.5,
    min(250.0, float(os.environ.get("FT_SERVO_INTERPOLATION_HZ", "125.0"))),
)
ROS2_SERVO_MIN_SAMPLES = max(
    4,
    int(os.environ.get("FT_SERVO_MIN_SAMPLES", "8")),
)
ROS2_SERVO_MAX_DISTANCE_MM = max(
    0.0,
    float(os.environ.get("FT_SERVO_MAX_DISTANCE_MM", "30.0")),
)
ROS2_SERVO_MAX_ORIENTATION_DEG = max(
    0.0,
    float(os.environ.get("FT_SERVO_MAX_ORIENTATION_DEG", "15.0")),
)
ROS2_SERVO_LINEAR_SPEED_MM_S_AT_100 = max(
    1.0,
    float(os.environ.get("FT_SERVO_LINEAR_SPEED_MM_S_AT_100", "1000.0")),
)
ROS2_SERVO_ANGULAR_SPEED_DEG_S_AT_100 = max(
    1.0,
    float(os.environ.get("FT_SERVO_ANGULAR_SPEED_DEG_S_AT_100", "180.0")),
)
ROS2_LIFT_SAFE_Z_MM = float(os.environ.get("ROS2_LIFT_SAFE_Z_MM", str(INIT_SAFE_Z_MM)))
ROS2_KEEP_CURRENT_ORIENTATION = os.environ.get(
    "FT_KEEP_CURRENT_ORIENTATION",
    os.environ.get("LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION", "0"),
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
ROS2_TRANSIT_MARGIN_MM = float(os.environ.get("ROS2_TRANSIT_MARGIN_MM", "80.0"))
ROS2_SEGMENT_MAX_STEP_MM = float(os.environ.get("ROS2_SEGMENT_MAX_STEP_MM", "50.0"))
ROS2_SEGMENT_MAX_STEPS = int(os.environ.get("ROS2_SEGMENT_MAX_STEPS", "0"))
ROS2_SEGMENT_TIMEOUT_S = float(os.environ.get("ROS2_SEGMENT_TIMEOUT_S", "180.0"))
ROS2_TRANSIT_LIFT_FIRST = os.environ.get("ROS2_TRANSIT_LIFT_FIRST", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
ROS2_TRANSIT_LIFT_TOL_MM = float(os.environ.get("ROS2_TRANSIT_LIFT_TOL_MM", "3.0"))
ROS2_USE_LEGACY_SAFE_POSE = os.environ.get(
    "ROS2_USE_LEGACY_SAFE_POSE", "0"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
ROS2_RESET_ERRORS = os.environ.get("ROS2_RESET_ERRORS", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
MOVEIT_IK_ENABLE = os.environ.get("LASTTIME_MOVEIT_IK", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
MOVEIT_IK_SERVICE = os.environ.get("MOVEIT_IK_SERVICE", "compute_ik")
MOVEIT_GROUP_NAME = os.environ.get("MOVEIT_GROUP_NAME", "fairino5_v6_group")
MOVEIT_IK_LINK_NAME = os.environ.get("MOVEIT_IK_LINK_NAME", "wrist3_link")
MOVEIT_IK_FRAME_ID = os.environ.get("MOVEIT_IK_FRAME_ID", "base_link")
MOVEIT_IK_WAIT_S = float(os.environ.get("MOVEIT_IK_WAIT_S", "0.2"))
MOVEIT_IK_TIMEOUT_S = float(os.environ.get("MOVEIT_IK_TIMEOUT_S", "0.15"))
MOVEIT_IK_CALL_TIMEOUT_S = float(os.environ.get("MOVEIT_IK_CALL_TIMEOUT_S", "1.0"))
MOVEIT_IK_AVOID_COLLISIONS = os.environ.get("MOVEIT_IK_AVOID_COLLISIONS", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
MOVEIT_JOINT_FALLBACK = os.environ.get("MOVEIT_JOINT_FALLBACK", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
MOVEIT_JOINT_NAMES = [
    item.strip()
    for item in os.environ.get("MOVEIT_JOINT_NAMES", "j1,j2,j3,j4,j5,j6").split(",")
    if item.strip()
]
LASTTIME_ROS2_FORCE = os.environ.get("LASTTIME_ROS2_FORCE", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}

FORCE_TARGET_N = float(os.environ.get("LASTTIME_FORCE_N", BACK_MASSAGE_FORCE_N))
BACK_SHUN_JIN_FORCE_TARGET_N = float(
    os.environ.get("BACK_SHUN_JIN_FORCE_N", BACK_SHUN_JIN_FORCE_N)
)
THIGH_OUTER_FORCE_TARGET_N = float(
    os.environ.get(
        "THIGH_OUTER_FORCE_N",
        os.environ.get(
            "LASTTIME_THIGH_OUTER_FORCE_N",
            os.environ.get("THIGH_FORCE_N", os.environ.get("LASTTIME_THIGH_FORCE_N", THIGH_OUTER_MASSAGE_FORCE_N)),
        ),
    )
)
THIGH_INNER_FORCE_TARGET_N = float(
    os.environ.get(
        "THIGH_INNER_FORCE_N",
        os.environ.get(
            "LASTTIME_THIGH_INNER_FORCE_N",
            os.environ.get("THIGH_FORCE_N", os.environ.get("LASTTIME_THIGH_FORCE_N", THIGH_INNER_MASSAGE_FORCE_N)),
        ),
    )
)
FORCE_CONTACT_OFFSET_MM = float(
    os.environ.get(
        "LASTTIME_FORCE_CONTACT_OFFSET_MM",
        "0.0",
    )
)
TOOL_TIP_LENGTH_MM = float(os.environ.get("LASTTIME_TOOL_TIP_LENGTH_MM", "95.0"))
BACK_HOVER_HEIGHT_MM = float(os.environ.get("BACK_HOVER_HEIGHT_MM", BACK_HOVER_HEIGHT_DEFAULT_MM))
BACK_POSTURE_SEED_ENABLE = os.environ.get(
    "BACK_POSTURE_SEED_ENABLE", "0"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
BACK_POSTURE_SEED_FILE = Path(
    os.environ.get(
        "BACK_POSTURE_SEED_FILE",
        str(SCRIPT_DIR / "back_posture_seed.json"),
    )
)
BACK_POSTURE_CONFIG_TOL_DEG = float(
    os.environ.get("BACK_POSTURE_CONFIG_TOL_DEG", "2.0")
)
BACK_TRAJECTORY_PROBE_ONLY = os.environ.get(
    "BACK_TRAJECTORY_PROBE_ONLY", "0"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
BACK_TRAJECTORY_PROBE_CLEARANCE_MM = max(
    float(BACK_HOVER_HEIGHT_MM),
    float(os.environ.get("BACK_TRAJECTORY_PROBE_CLEARANCE_MM", "60.0")),
)
BACK_TRAJECTORY_PROBE_VEL = max(
    1.0,
    float(os.environ.get("BACK_TRAJECTORY_PROBE_VEL", "10.0")),
)
BACK_MIN_DEPTH_RATIO = float(os.environ.get("BACK_MIN_DEPTH_RATIO", "0.50"))
BACK_MIN_LINE_LENGTH_PX = float(
    os.environ.get("BACK_MIN_LINE_LENGTH_PX", "220.0")
)
BACK_LINE_TRIM_NECK_RATIO = float(
    os.environ.get("BACK_LINE_TRIM_NECK_RATIO", os.environ.get("BACK_LINE_TRIM_RATIO", "0.08"))
)
BACK_LINE_TRIM_TAIL_RATIO = float(
    os.environ.get("BACK_LINE_TRIM_TAIL_RATIO", os.environ.get("BACK_LINE_TRIM_RATIO", "0.10"))
)
BACK_LOCK_HORIZONTAL = os.environ.get("BACK_LOCK_HORIZONTAL", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
BACK_LINE_OFFSET_FILE = Path(
    os.environ.get("BACK_LINE_OFFSET_FILE", str(SCRIPT_DIR / "back_line_offset.json"))
)
try:
    with BACK_LINE_OFFSET_FILE.open("r", encoding="utf-8") as offset_file:
        _BACK_LINE_OFFSET_CONFIG = json.load(offset_file)
except (OSError, ValueError, TypeError):
    _BACK_LINE_OFFSET_CONFIG = {}
BACK_LINE_OFFSET_X_PX = float(
    os.environ.get("BACK_LINE_OFFSET_X_PX", _BACK_LINE_OFFSET_CONFIG.get("offset_x_px", 0.0))
)
BACK_LINE_OFFSET_Y_PX = float(
    os.environ.get("BACK_LINE_OFFSET_Y_PX", _BACK_LINE_OFFSET_CONFIG.get("offset_y_px", 0.0))
)
FORCE_FEN_LATERAL_MM = float(
    os.environ.get("LASTTIME_FORCE_FEN_LATERAL_MM", str(min(abs(FEN_JIN_LATERAL_MM), 12.0)))
)
DIAN_JIN_MODE = os.environ.get("FT_DIAN_JIN_MODE", DIAN_JIN_MODE_DEFAULT).strip().lower()
DIAN_AS_SMALL_FEN_LATERAL_MM = float(
    os.environ.get(
        "FT_DIAN_AS_SMALL_FEN_LATERAL_MM",
        str(min(abs(FORCE_FEN_LATERAL_MM) * 0.5, 6.0)),
    )
)
FORCE_MAX_DIS_MM = float(os.environ.get("LASTTIME_FORCE_MAX_DIS_MM", "8.0"))
FORCE_MAX_ANG_DEG = float(os.environ.get("LASTTIME_FORCE_MAX_ANG_DEG", "3.0"))
FORCE_PID_P = float(os.environ.get("LASTTIME_FORCE_PID_P", "0.003"))
FORCE_SOFTWARE_LIMIT_N = float(
    os.environ.get("LASTTIME_FORCE_SOFTWARE_LIMIT_N", "45.0")
)
FORCE_SOFTWARE_NORMAL_LIMIT_N = float(
    os.environ.get(
        "LASTTIME_FORCE_NORMAL_LIMIT_N",
        str(max(90.0, FORCE_SOFTWARE_LIMIT_N, FORCE_TARGET_N + 30.0)),
    )
)
FORCE_SOFTWARE_TANGENTIAL_LIMIT_N = float(
    os.environ.get(
        "LASTTIME_FORCE_TANGENTIAL_LIMIT_N",
        str(max(80.0, FORCE_SOFTWARE_LIMIT_N)),
    )
)
FT_CONTINUE_ON_POINT_ERROR = os.environ.get("FT_CONTINUE_ON_POINT_ERROR", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
FT_SHUN_MIN_POINTS = int(os.environ.get("FT_SHUN_MIN_POINTS", "2"))
FORCE_PRESTART_LIMIT_N = float(
    os.environ.get("LASTTIME_FORCE_PRESTART_LIMIT_N", "8.0")
)
FORCE_START_SETTLE_S = float(os.environ.get("LASTTIME_FORCE_START_SETTLE_S", "0.03"))
FORCE_STOP_SETTLE_S = float(os.environ.get("LASTTIME_FORCE_STOP_SETTLE_S", "0.25"))
FORCE_SOFTWARE_TORQUE_LIMIT_NM = float(
    os.environ.get("LASTTIME_FORCE_SOFTWARE_TORQUE_LIMIT_NM", "4.5")
)
FORCE_GUARD_ENABLE = os.environ.get("LASTTIME_FORCE_GUARD", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
FORCE_GUARD_LIMIT_N = float(os.environ.get("LASTTIME_FORCE_GUARD_LIMIT_N", "30.0"))
FORCE_GUARD_TORQUE_LIMIT_NM = float(
    os.environ.get("LASTTIME_FORCE_GUARD_TORQUE_LIMIT_NM", "3.5")
)
FORCE_DIAN_DWELL_S = float(os.environ.get("LASTTIME_FORCE_DIAN_DWELL_S", "0.6"))
FORCE_FEN_DWELL_S = float(os.environ.get("LASTTIME_FORCE_FEN_DWELL_S", "0.25"))
DIAN_JIN_REPEAT_COUNT = max(1, int(os.environ.get("FT_DIAN_JIN_REPEAT_COUNT", DIAN_JIN_REPEAT_DEFAULT)))
FEN_JIN_REPEAT_COUNT = max(1, int(os.environ.get("FT_FEN_JIN_REPEAT_COUNT", FEN_JIN_REPEAT_DEFAULT)))
FORCE_SHUN_DWELL_S = float(os.environ.get("LASTTIME_FORCE_SHUN_DWELL_S", "0.0"))
FORCE_SHUN_RECONTACT = os.environ.get("LASTTIME_FORCE_SHUN_RECONTACT", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
FORCE_SHUN_SEGMENT_LENGTH_MM = max(
    0.0,
    float(
        os.environ.get(
            "LASTTIME_FORCE_SHUN_SEGMENT_LENGTH_MM",
            SHUN_JIN_SEGMENT_LENGTH_DEFAULT_MM,
        )
    ),
)
FORCE_SHUN_RELEASE_LIFT_MM = max(
    0.0,
    float(
        os.environ.get(
            "LASTTIME_FORCE_SHUN_RELEASE_LIFT_MM",
            SHUN_JIN_RELEASE_LIFT_DEFAULT_MM,
        )
    ),
)
FORCE_SHUN_RELEASE_DWELL_S = max(
    0.0,
    float(
        os.environ.get(
            "LASTTIME_FORCE_SHUN_RELEASE_DWELL_S",
            SHUN_JIN_RELEASE_DWELL_DEFAULT_S,
        )
    ),
)
FORCE_SHUN_TANGENTIAL_WARN_N = max(
    0.0,
    float(
        os.environ.get(
            "LASTTIME_FORCE_SHUN_TANGENTIAL_WARN_N",
            SHUN_JIN_TANGENTIAL_WARN_DEFAULT_N,
        )
    ),
)
FORCE_SHUN_TANGENTIAL_RELEASE_N = max(
    FORCE_SHUN_TANGENTIAL_WARN_N,
    float(
        os.environ.get(
            "LASTTIME_FORCE_SHUN_TANGENTIAL_RELEASE_N",
            SHUN_JIN_TANGENTIAL_RELEASE_DEFAULT_N,
        )
    ),
)
FORCE_MONITOR_HZ = float(os.environ.get("LASTTIME_FORCE_MONITOR_HZ", "20.0"))
FORCE_SENSOR_BUS = int(os.environ.get("LASTTIME_FORCE_SENSOR_BUS", "1"))
FORCE_ALLOW_SKIP_ZERO = os.environ.get("LASTTIME_FORCE_ALLOW_SKIP_ZERO", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
FORCE_ZERO_MAX_ABS_N = float(os.environ.get("LASTTIME_FORCE_ZERO_MAX_ABS_N", "3.0"))
FORCE_AXIS_SIGN = -1.0 if float(os.environ.get("LASTTIME_FORCE_AXIS_SIGN", "-1.0")) < 0 else 1.0
FORCE_TARGET_TOL_N = float(os.environ.get("LASTTIME_FORCE_TOL_N", "1.2"))
FORCE_APPROACH_SPEED_SCALE = float(os.environ.get("FT_APPROACH_SPEED_SCALE", FORCE_APPROACH_SPEED_SCALE_DEFAULT))
FORCE_APPROACH_SPEED_MAX = float(os.environ.get("FT_APPROACH_SPEED_MAX", "100.0"))


def _scaled_force_approach_velocity(base_vel):
    scaled = abs(float(base_vel)) * max(0.0, FORCE_APPROACH_SPEED_SCALE)
    if FORCE_APPROACH_SPEED_MAX > 0.0:
        scaled = min(scaled, float(FORCE_APPROACH_SPEED_MAX))
    return max(1.0, scaled)


FORCE_APPROACH_STEP_MM = float(os.environ.get("LASTTIME_FORCE_APPROACH_STEP_MM", FORCE_APPROACH_STEP_DEFAULT_MM))
FORCE_APPROACH_CONTACT_N = float(os.environ.get("LASTTIME_FORCE_APPROACH_CONTACT_N", "2.0"))
FORCE_APPROACH_CONTACT_STEP_MM = float(os.environ.get("LASTTIME_FORCE_APPROACH_CONTACT_STEP_MM", "0.3"))
FORCE_APPROACH_CONTACT_VEL_BASE = float(os.environ.get("LASTTIME_FORCE_APPROACH_CONTACT_VEL", "5.0"))
FORCE_APPROACH_CONTACT_VEL = _scaled_force_approach_velocity(FORCE_APPROACH_CONTACT_VEL_BASE)
FORCE_APPROACH_FINE_STEP_MM = float(os.environ.get("LASTTIME_FORCE_APPROACH_FINE_STEP_MM", "0.6"))
FORCE_APPROACH_FINE_RATIO = float(os.environ.get("LASTTIME_FORCE_APPROACH_FINE_RATIO", "0.55"))
FORCE_APPROACH_NEAR_STEP_MM = float(os.environ.get("LASTTIME_FORCE_APPROACH_NEAR_STEP_MM", "0.3"))
FORCE_APPROACH_NEAR_RATIO = float(os.environ.get("LASTTIME_FORCE_APPROACH_NEAR_RATIO", "0.85"))
FORCE_APPROACH_SETTLE_S = float(os.environ.get("LASTTIME_FORCE_APPROACH_SETTLE_S", "0.05"))
FORCE_APPROACH_VEL_BASE = float(os.environ.get("LASTTIME_FORCE_APPROACH_VEL", "20.0"))
FORCE_APPROACH_FINE_VEL_BASE = float(os.environ.get("LASTTIME_FORCE_APPROACH_FINE_VEL", "8.0"))
FORCE_APPROACH_NEAR_VEL_BASE = float(os.environ.get("LASTTIME_FORCE_APPROACH_NEAR_VEL", "5.0"))
FORCE_APPROACH_VEL = _scaled_force_approach_velocity(FORCE_APPROACH_VEL_BASE)
FORCE_APPROACH_FINE_VEL = _scaled_force_approach_velocity(FORCE_APPROACH_FINE_VEL_BASE)
FORCE_APPROACH_NEAR_VEL = _scaled_force_approach_velocity(FORCE_APPROACH_NEAR_VEL_BASE)
FORCE_APPROACH_PRECONTACT_CLEARANCE_MM = float(
    os.environ.get("LASTTIME_FORCE_APPROACH_PRECONTACT_CLEARANCE_MM", "0.0")
)
FORCE_APPROACH_PRECONTACT_VEL_BASE = float(
    os.environ.get("LASTTIME_FORCE_APPROACH_PRECONTACT_VEL", "12.0")
)
FORCE_APPROACH_PRECONTACT_VEL = _scaled_force_approach_velocity(FORCE_APPROACH_PRECONTACT_VEL_BASE)
FORCE_APPROACH_MAX_OFFSET_MM = float(
    os.environ.get(
        "LASTTIME_FORCE_APPROACH_MAX_OFFSET_MM",
        str(max(35.0, FORCE_CONTACT_OFFSET_MM)),
    )
)
FORCE_HOLD_KP_MM_PER_N = float(os.environ.get("LASTTIME_FORCE_HOLD_KP_MM_PER_N", "0.04"))
FORCE_HOLD_MAX_STEP_MM = float(os.environ.get("LASTTIME_FORCE_HOLD_MAX_STEP_MM", "0.15"))
FORCE_CONTINUOUS_APPROACH = os.environ.get(
    "FT_CONTINUOUS_FORCE_APPROACH", "1"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
FORCE_CONTINUOUS_APPROACH_HZ = max(
    62.5,
    min(
        250.0,
        float(
            os.environ.get(
                "FT_CONTINUOUS_FORCE_APPROACH_HZ",
                str(ROS2_SERVO_INTERPOLATION_HZ),
            )
        ),
    ),
)
FORCE_CONTINUOUS_COARSE_SPEED_MM_S = max(
    0.1,
    min(250.0, float(os.environ.get("FT_CONTINUOUS_FORCE_COARSE_SPEED_MM_S", "18.0"))),
)
FORCE_CONTINUOUS_CONTACT_SPEED_MM_S = max(
    0.1,
    min(250.0, float(os.environ.get("FT_CONTINUOUS_FORCE_CONTACT_SPEED_MM_S", "4.0"))),
)
FORCE_CONTINUOUS_FINE_SPEED_MM_S = max(
    0.1,
    min(250.0, float(os.environ.get("FT_CONTINUOUS_FORCE_FINE_SPEED_MM_S", "4.0"))),
)
FORCE_CONTINUOUS_NEAR_SPEED_MM_S = max(
    0.1,
    min(250.0, float(os.environ.get("FT_CONTINUOUS_FORCE_NEAR_SPEED_MM_S", "3.0"))),
)
FORCE_CONTINUOUS_ACCEL_MM_S2 = max(
    1.0,
    min(2000.0, float(os.environ.get("FT_CONTINUOUS_FORCE_ACCEL_MM_S2", "120.0"))),
)
FORCE_CONTINUOUS_DECEL_MM_S2 = max(
    1.0,
    min(4000.0, float(os.environ.get("FT_CONTINUOUS_FORCE_DECEL_MM_S2", "240.0"))),
)
FORCE_CONTINUOUS_FILTER_ALPHA = max(
    0.01,
    min(1.0, float(os.environ.get("FT_CONTINUOUS_FORCE_FILTER_ALPHA", "0.35"))),
)
FORCE_CONTINUOUS_TARGET_STABLE_SAMPLES = max(
    1,
    min(20, int(os.environ.get("FT_CONTINUOUS_FORCE_TARGET_STABLE_SAMPLES", "2"))),
)
FORCE_CONTINUOUS_TIMEOUT_S = max(
    1.0,
    min(30.0, float(os.environ.get("FT_CONTINUOUS_FORCE_TIMEOUT_S", "20.0"))),
)
FORCE_RELEASE_LIMIT_N = float(
    os.environ.get("LASTTIME_FORCE_RELEASE_LIMIT_N", "5.0")
)
FORCE_RELEASE_TIMEOUT_S = float(os.environ.get("LASTTIME_FORCE_RELEASE_TIMEOUT_S", "2.0"))
LIVE_FORCE_TARGET_MIN_N = float(os.environ.get("FT_LIVE_FORCE_TARGET_MIN_N", "1.0"))
LIVE_FORCE_TARGET_MAX_N = float(os.environ.get("FT_LIVE_FORCE_TARGET_MAX_N", "80.0"))

MASSAGE_TARGET_ENV = os.environ.get("MASSAGE_TARGET", "").strip().lower()
THIGH_SIDE = os.environ.get("THIGH_SIDE", "right").strip().lower()
if THIGH_SIDE not in {"nearest", "auto", "left", "right"}:
    THIGH_SIDE = "right"
THIGH_OFFSET_MM = float(os.environ.get("THIGH_OFFSET_MM", "25.0"))
THIGH_LINE_SHIFT_MM = float(os.environ.get("THIGH_LINE_SHIFT_MM", "0.0"))
THIGH_OUTER_OFFSET_MM = float(os.environ.get("THIGH_OUTER_OFFSET_MM", THIGH_OFFSET_MM))
THIGH_OUTER_LINE_SHIFT_MM = float(os.environ.get("THIGH_OUTER_LINE_SHIFT_MM", THIGH_LINE_SHIFT_MM))
THIGH_INNER_OFFSET_MM = float(os.environ.get("THIGH_INNER_OFFSET_MM", THIGH_OFFSET_MM))
THIGH_INNER_LINE_SHIFT_MM = float(os.environ.get("THIGH_INNER_LINE_SHIFT_MM", THIGH_LINE_SHIFT_MM))
THIGH_DIRECTION = os.environ.get("THIGH_DIRECTION", "image-down").strip().lower()
if THIGH_DIRECTION not in THIGH_DIRECTION_MODES:
    THIGH_DIRECTION = "image-down"
THIGH_FLIP_DIRECTION = os.environ.get("THIGH_FLIP_DIRECTION", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
THIGH_SAMPLE_POINTS = int(os.environ.get("THIGH_SAMPLE_POINTS", str(SAMPLE_POINTS)))
THIGH_STABLE_FRAMES = int(os.environ.get("THIGH_STABLE_FRAMES", "5"))
THIGH_DETECTION_TIMEOUT_S = float(os.environ.get("THIGH_DETECTION_TIMEOUT_S", "30.0"))
THIGH_MIN_DEPTH_RATIO = float(os.environ.get("THIGH_MIN_DEPTH_RATIO", "0.70"))
THIGH_KPT_THR = float(os.environ.get("THIGH_KPT_THR", "0.25"))
THIGH_WIDTH = int(os.environ.get("THIGH_CAMERA_WIDTH", "640"))
THIGH_HEIGHT = int(os.environ.get("THIGH_CAMERA_HEIGHT", "480"))
THIGH_FPS = int(os.environ.get("THIGH_CAMERA_FPS", "30"))
THIGH_DEVICE = os.environ.get("THIGH_DEVICE", "auto").strip()
THIGH_ROTATION = os.environ.get("THIGH_ROTATION", "none").strip().lower()
if THIGH_ROTATION not in ROTATIONS:
    THIGH_ROTATION = "none"
THIGH_TRY_ROTATIONS = os.environ.get("THIGH_TRY_ROTATIONS", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
THIGH_ALIGN_DEPTH = os.environ.get("THIGH_ALIGN_DEPTH", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
THIGH_DISPLAY = os.environ.get("THIGH_DISPLAY", "1").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
THIGH_HOVER_HEIGHT_MM = float(os.environ.get("THIGH_HOVER_HEIGHT_MM", THIGH_HOVER_HEIGHT_DEFAULT_MM))
THIGH_FORCE_APPROACH_MAX_OFFSET_MM = float(
    os.environ.get("THIGH_FORCE_APPROACH_MAX_OFFSET_MM", "35.0")
)
THIGH_LOCAL_NORMAL_MAX_TILT_DEG = float(
    os.environ.get("THIGH_LOCAL_NORMAL_MAX_TILT_DEG", "0.0")
)
THIGH_LOCAL_NORMAL_LIMIT_ENABLED = THIGH_LOCAL_NORMAL_MAX_TILT_DEG > 0.0
THIGH_AUTO_REACHABLE_ORIENTATION = os.environ.get(
    "THIGH_AUTO_REACHABLE_ORIENTATION", "1"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
THIGH_REACHABILITY_STEP_DEG = float(os.environ.get("THIGH_REACHABILITY_STEP_DEG", "5.0"))
THIGH_REACHABILITY_MIN_TILT_DEG = float(os.environ.get("THIGH_REACHABILITY_MIN_TILT_DEG", "0.0"))
THIGH_INNER_POSTURE_SEED_FILE = Path(
    os.environ.get(
        "THIGH_INNER_POSTURE_SEED_FILE",
        str(SCRIPT_DIR / "thigh_inner_posture_seed.json"),
    )
)
THIGH_INNER_POSTURE_SEED_ENABLE = os.environ.get(
    "THIGH_INNER_POSTURE_SEED_ENABLE", "1"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
THIGH_INNER_USE_SEED_ORIENTATION = os.environ.get(
    "THIGH_INNER_USE_SEED_ORIENTATION", "1"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
THIGH_INNER_POSTURE_SWITCH_VEL = float(
    os.environ.get("THIGH_INNER_POSTURE_SWITCH_VEL", "10.0")
)
THIGH_INNER_POSTURE_JOINT_TOL_DEG = float(
    os.environ.get("THIGH_INNER_POSTURE_JOINT_TOL_DEG", "2.0")
)
THIGH_INNER_POSTURE_NEAR_JOINT_FALLBACK_DEG = float(
    os.environ.get("THIGH_INNER_POSTURE_NEAR_JOINT_FALLBACK_DEG", "20.0")
)
THIGH_INNER_POSTURE_LIFT_Z_MM = float(
    os.environ.get("THIGH_INNER_POSTURE_LIFT_Z_MM", "390.0")
)
THIGH_INNER_POSTURE_PROBE_ONLY = os.environ.get(
    "THIGH_INNER_POSTURE_PROBE_ONLY", "0"
).strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
THIGH_INNER_POSTURE_PROBE_DWELL_S = max(
    0.0,
    float(os.environ.get("THIGH_INNER_POSTURE_PROBE_DWELL_S", "5.0")),
)
THIGH_OUTER_TAIL_SKIP_POINTS = int(
    os.environ.get("THIGH_OUTER_TAIL_SKIP_POINTS", THIGH_OUTER_TAIL_SKIP_POINTS_DEFAULT)
)
THIGH_INNER_SKIP_POINTS = int(os.environ.get("THIGH_INNER_SKIP_POINTS", "3"))
THIGH_INNER_TAIL_SKIP_POINTS = int(
    os.environ.get("THIGH_INNER_TAIL_SKIP_POINTS", THIGH_INNER_TAIL_SKIP_POINTS_DEFAULT)
)
ROBOT_MOTION_SPEED_SCALE = float(os.environ.get("FT_ROBOT_MOTION_SPEED_SCALE", ROBOT_MOTION_SPEED_SCALE_DEFAULT))
SHUN_JIN_MOTION_SPEED_SCALE = float(
    os.environ.get("FT_SHUN_JIN_MOTION_SPEED_SCALE", SHUN_JIN_MOTION_SPEED_SCALE_DEFAULT)
)
TRANSIT_SPEED_SCALE = float(os.environ.get("FT_TRANSIT_SPEED_SCALE", "2.0"))
TRANSIT_SPEED_MAX = float(os.environ.get("FT_TRANSIT_SPEED_MAX", "100.0"))


def _scaled_transit_velocity(base_vel):
    scaled = abs(float(base_vel)) * max(0.0, float(TRANSIT_SPEED_SCALE))
    if TRANSIT_SPEED_MAX > 0.0:
        scaled = min(scaled, float(TRANSIT_SPEED_MAX))
    return max(1.0, scaled)


def _massage_frame_distance_mm(previous_frame, current_frame):
    previous = previous_frame.get("point_mm") if previous_frame else None
    current = current_frame.get("point_mm") if current_frame else None
    if previous is None or current is None or len(previous) < 3 or len(current) < 3:
        return 0.0
    return math.sqrt(sum((float(current[i]) - float(previous[i])) ** 2 for i in range(3)))


def _tangential_force_n(force_torque):
    if force_torque is None or len(force_torque) < 2:
        return 0.0
    return math.hypot(float(force_torque[0]), float(force_torque[1]))


def _normalized_vector(values, fallback):
    vector = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm > 1e-9:
        return vector / norm
    fallback_vector = np.asarray(fallback, dtype=np.float64)
    fallback_norm = float(np.linalg.norm(fallback_vector))
    if fallback_norm > 1e-9:
        return fallback_vector / fallback_norm
    return np.asarray([0.0, 0.0, 1.0], dtype=np.float64)


def _interpolate_angle_deg(start, end, fraction):
    delta = (float(end) - float(start) + 180.0) % 360.0 - 180.0
    return float(start) + delta * float(fraction)


def _interpolate_shun_frame(previous_frame, current_frame, fraction):
    fraction = max(0.0, min(1.0, float(fraction)))
    if fraction >= 1.0:
        return dict(current_frame)

    frame = dict(previous_frame)
    previous_point = np.asarray(previous_frame["point_mm"], dtype=np.float64)
    current_point = np.asarray(current_frame["point_mm"], dtype=np.float64)
    frame["point_mm"] = (
        previous_point + (current_point - previous_point) * fraction
    ).tolist()

    previous_tool_z = np.asarray(previous_frame["tool_z_unit"], dtype=np.float64)
    current_tool_z = np.asarray(current_frame["tool_z_unit"], dtype=np.float64)
    tool_z = _normalized_vector(
        previous_tool_z + (current_tool_z - previous_tool_z) * fraction,
        current_tool_z,
    )
    previous_split = np.asarray(previous_frame["split_axis_unit"], dtype=np.float64)
    current_split = np.asarray(current_frame["split_axis_unit"], dtype=np.float64)
    split_axis = previous_split + (current_split - previous_split) * fraction
    split_axis = split_axis - tool_z * float(np.dot(split_axis, tool_z))
    split_axis = _normalized_vector(split_axis, current_split)
    frame["tool_z_unit"] = tool_z.tolist()
    frame["split_axis_unit"] = split_axis.tolist()

    previous_pose = previous_frame["base_pose"]
    current_pose = current_frame["base_pose"]
    frame["base_pose"] = [
        _interpolate_angle_deg(previous_pose[i], current_pose[i], fraction)
        for i in range(3)
    ]
    frame["index"] = current_frame.get("index", previous_frame.get("index", 0))
    frame["shun_interpolated"] = True
    frame["shun_interpolation_fraction"] = fraction
    return frame


def _subdivide_shun_edge(previous_frame, current_frame, max_step_mm):
    if previous_frame is None:
        return [current_frame]
    distance_mm = _massage_frame_distance_mm(previous_frame, current_frame)
    max_step_mm = float(max_step_mm)
    if max_step_mm <= 0.0 or distance_mm <= max_step_mm:
        return [current_frame]
    step_count = max(1, int(math.ceil(distance_mm / max_step_mm)))
    return [
        _interpolate_shun_frame(previous_frame, current_frame, step_index / step_count)
        for step_index in range(1, step_count + 1)
    ]


def _linear_transit_waypoints(start_pose, target_pose, max_step_mm):
    """Build collinear waypoints whose intermediate poses can be queued with blending."""
    start = [float(value) for value in start_pose]
    target = [float(value) for value in target_pose]
    distance_mm = math.sqrt(
        sum((target[i] - start[i]) ** 2 for i in range(3))
    )
    step_mm = max(1.0, float(max_step_mm))
    step_count = max(1, int(math.ceil(distance_mm / step_mm)))
    waypoints = []
    for step_index in range(1, step_count + 1):
        fraction = step_index / step_count
        if step_index == step_count:
            waypoints.append(target)
            continue
        waypoints.append(
            [
                start[i] + (target[i] - start[i]) * fraction
                if i < 3
                else _interpolate_angle_deg(start[i], target[i], fraction)
                for i in range(6)
            ]
        )
    return waypoints


TRANSIT_MOVE_VEL_FAST = _scaled_transit_velocity(MOVE_VEL_FAST)
TRANSIT_MOVE_VEL_SLOW = _scaled_transit_velocity(MOVE_VEL_SLOW)
_ROBOT_MOTION_SPEED_SCALE_OVERRIDE = None
FT_TRAJECTORY_OUTPUT_DIR = Path(
    os.environ.get(
        "FT_TRAJECTORY_OUTPUT_DIR",
        str(SCRIPT_DIR / "ft_locked_trajectory_output"),
    )
)


def _fmt_value(value):
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.6f}".rstrip("0").rstrip(".")


def _shortest_angle_delta_deg(start, end):
    return (float(end) - float(start) + 180.0) % 360.0 - 180.0


def _pose_distance(current_pose, target_pose):
    pos_dist = math.sqrt(
        sum((float(current_pose[i]) - float(target_pose[i])) ** 2 for i in range(3))
    )
    ori_dist = max(
        abs(_shortest_angle_delta_deg(current_pose[i], target_pose[i]))
        for i in range(3, 6)
    )
    return pos_dist, ori_dist


def _servo_interpolation_metrics(start_pose, target_pose):
    linear_distance_mm = math.sqrt(
        sum((float(target_pose[i]) - float(start_pose[i])) ** 2 for i in range(3))
    )
    orientation_distance_deg = max(
        abs(_shortest_angle_delta_deg(start_pose[i], target_pose[i]))
        for i in range(3, 6)
    )
    return linear_distance_mm, orientation_distance_deg


def _motion_completion_tolerances(start_pose, target_pose):
    linear_distance_mm, orientation_distance_deg = _servo_interpolation_metrics(
        start_pose,
        target_pose,
    )
    position_tolerance_mm = min(
        ROS2_MOTION_DONE_POSE_TOL_MM,
        max(0.02, linear_distance_mm * ROS2_MOTION_DONE_TOL_FRACTION),
    )
    orientation_tolerance_deg = min(
        ROS2_MOTION_DONE_ORI_TOL_DEG,
        max(0.02, orientation_distance_deg * ROS2_MOTION_DONE_TOL_FRACTION),
    )
    return position_tolerance_mm, orientation_tolerance_deg


def _servo_interpolation_duration_s(start_pose, target_pose, speed_percent):
    linear_distance_mm, orientation_distance_deg = _servo_interpolation_metrics(
        start_pose,
        target_pose,
    )
    speed_ratio = max(0.01, min(1.0, abs(float(speed_percent)) / 100.0))
    # Quintic smoothstep reaches a peak normalized velocity of 1.875. Account
    # for that peak instead of treating distance/duration as the peak speed.
    smoothstep_peak_rate = 1.875
    linear_duration_s = smoothstep_peak_rate * linear_distance_mm / (
        ROS2_SERVO_LINEAR_SPEED_MM_S_AT_100 * speed_ratio
    )
    angular_duration_s = smoothstep_peak_rate * orientation_distance_deg / (
        ROS2_SERVO_ANGULAR_SPEED_DEG_S_AT_100 * speed_ratio
    )
    min_duration_s = float(ROS2_SERVO_MIN_SAMPLES) / ROS2_SERVO_INTERPOLATION_HZ
    return max(min_duration_s, linear_duration_s, angular_duration_s)


def _scaled_robot_motion_speed(vel):
    speed_scale = (
        ROBOT_MOTION_SPEED_SCALE
        if _ROBOT_MOTION_SPEED_SCALE_OVERRIDE is None
        else _ROBOT_MOTION_SPEED_SCALE_OVERRIDE
    )
    scaled = abs(float(vel)) * max(0.0, float(speed_scale))
    return int(max(1.0, min(100.0, round(scaled))))


class _RobotMotionSpeedScaleOverride:
    def __init__(self, speed_scale):
        self.speed_scale = float(speed_scale)
        self.previous = None

    def __enter__(self):
        global _ROBOT_MOTION_SPEED_SCALE_OVERRIDE
        self.previous = _ROBOT_MOTION_SPEED_SCALE_OVERRIDE
        _ROBOT_MOTION_SPEED_SCALE_OVERRIDE = self.speed_scale
        return self

    def __exit__(self, exc_type, exc, tb):
        global _ROBOT_MOTION_SPEED_SCALE_OVERRIDE
        _ROBOT_MOTION_SPEED_SCALE_OVERRIDE = self.previous
        return False


def _parse_ret_code(cmd_res):
    if not cmd_res:
        return -9999
    head = str(cmd_res).split(",", 1)[0].strip()
    try:
        return int(float(head))
    except ValueError:
        return -9999


def _parse_inverse_kin_joints(cmd_res):
    parts = [part.strip() for part in str(cmd_res).split(",")]
    if len(parts) < 7 or _parse_ret_code(cmd_res) != 0:
        return None
    try:
        joints = [float(value) for value in parts[1:7]]
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in joints):
        return None
    return joints


def _parse_force_approach_response(cmd_res):
    """Parse ServoCartForceApproach's structured command response."""
    parts = [part.strip() for part in str(cmd_res).split(",")]
    if len(parts) < 10:
        return None
    try:
        return {
            "ret": int(float(parts[0])),
            "status": int(float(parts[1])),
            "travel_mm": float(parts[2]),
            "elapsed_s": float(parts[3]),
            "force": [float(value) for value in parts[4:10]],
        }
    except ValueError:
        return None


def _quat_from_rpy_deg(rx_deg, ry_deg, rz_deg):
    rx = math.radians(float(rx_deg))
    ry = math.radians(float(ry_deg))
    rz = math.radians(float(rz_deg))
    cr = math.cos(rx * 0.5)
    sr = math.sin(rx * 0.5)
    cp = math.cos(ry * 0.5)
    sp = math.sin(ry * 0.5)
    cy = math.cos(rz * 0.5)
    sy = math.sin(rz * 0.5)
    return (
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
        cy * cp * cr + sy * sp * sr,
    )


def _state_field(msg, *names, default=None):
    if msg is None:
        return default
    for name in names:
        if hasattr(msg, name):
            return getattr(msg, name)
    return default


def _state_pose(msg):
    pose = [
        _state_field(msg, "cart_x_cur_pos"),
        _state_field(msg, "cart_y_cur_pos"),
        _state_field(msg, "cart_z_cur_pos"),
        _state_field(msg, "cart_a_cur_pos"),
        _state_field(msg, "cart_b_cur_pos"),
        _state_field(msg, "cart_c_cur_pos"),
    ]
    if any(value is None for value in pose):
        raise RuntimeError("状态话题缺少当前 TCP 位姿字段")
    return [float(value) for value in pose]


def _state_joints_deg(msg):
    joints = [
        _state_field(msg, "j1_cur_pos"),
        _state_field(msg, "j2_cur_pos"),
        _state_field(msg, "j3_cur_pos"),
        _state_field(msg, "j4_cur_pos"),
        _state_field(msg, "j5_cur_pos"),
        _state_field(msg, "j6_cur_pos"),
    ]
    if any(value is None for value in joints):
        raise RuntimeError("状态话题缺少当前关节角字段")
    return [float(value) for value in joints]


def _normalize_massage_target(value):
    text = str(value or "").strip().lower()
    if text in {"1", "back", "spine", "bladder", "bladder_meridian", "beibu", "背部", "膀胱经"}:
        return "back"
    if text in {"2", "leg", "thigh", "outer_thigh", "tuibu", "腿部", "大腿"}:
        return "leg"
    if text in {
        "3",
        "leg_inner",
        "inner_leg",
        "inner_thigh",
        "thigh_inner",
        "datui_neice",
        "腿内侧",
        "大腿内侧",
        "内侧",
    }:
        return "leg_inner"
    return None


def _is_thigh_target(value):
    return _normalize_massage_target(value) in {"leg", "leg_inner"}


def _force_target_for_massage_target(value):
    target = _normalize_massage_target(value)
    if target == "leg":
        return THIGH_OUTER_FORCE_TARGET_N
    if target == "leg_inner":
        return THIGH_INNER_FORCE_TARGET_N
    return FORCE_TARGET_N


def _force_target_for_massage_action(value, action):
    target = _normalize_massage_target(value) or "back"
    if target == "back" and str(action or "").strip().lower() == "shun_jin":
        return BACK_SHUN_JIN_FORCE_TARGET_N
    return _force_target_for_massage_target(target)


def _thigh_offset_for_massage_target(value):
    return THIGH_INNER_OFFSET_MM if _normalize_massage_target(value) == "leg_inner" else THIGH_OUTER_OFFSET_MM


def _thigh_line_shift_for_massage_target(value):
    return THIGH_INNER_LINE_SHIFT_MM if _normalize_massage_target(value) == "leg_inner" else THIGH_OUTER_LINE_SHIFT_MM


def _massage_target_label(value):
    target = _normalize_massage_target(value)
    if target == "leg":
        return "腿部大腿外侧中线"
    if target == "leg_inner":
        return "腿部大腿内侧"
    return "背部膀胱经"


def _select_massage_target():
    env_target = _normalize_massage_target(MASSAGE_TARGET_ENV)
    if env_target is not None:
        return env_target

    if not sys.stdin.isatty():
        print("未检测到交互输入，默认选择背部膀胱经按摩")
        return "back"

    print("请选择按摩部位：")
    print("  1) 背部膀胱经按摩")
    print("  2) 腿部大腿外侧中线按摩")
    print("  3) 腿部大腿内侧按摩")
    choice = input("输入 1/2/3，直接回车默认 1: ").strip()
    return _normalize_massage_target(choice or "1") or "back"


def _estimate_patch_plane_normal_camera_np(
    reader,
    depth_image,
    pixel_u,
    pixel_v,
    radius_px=PLANE_FIT_RADIUS_PX,
    step_px=PLANE_FIT_STEP_PX,
    min_points=PLANE_FIT_MIN_POINTS,
):
    if depth_image is None:
        return None

    points = []
    for du in range(-int(radius_px), int(radius_px) + 1, max(1, int(step_px))):
        for dv in range(-int(radius_px), int(radius_px) + 1, max(1, int(step_px))):
            u = float(pixel_u + du)
            v = float(pixel_v + dv)
            depth_m = sample_thigh_depth_m(depth_image, (u, v), reader.depth_scale, radius=0)
            if depth_m is None:
                continue
            try:
                points.append(np.asarray(reader.deproject((u, v), depth_m), dtype=np.float64))
            except Exception:
                continue

    if len(points) < max(3, int(min_points)):
        return None

    arr = np.asarray(points, dtype=np.float64)
    center = np.mean(arr, axis=0)
    try:
        _, _, vh = np.linalg.svd(arr - center, full_matrices=False)
    except Exception:
        return None
    return _normalize_vec(vh[-1])


def _depth_patch_stats_from_image(depth_image, pixel_u, pixel_v, depth_scale, radius_px=4):
    if depth_image is None:
        return None
    height, width = depth_image.shape[:2]
    cu = int(round(float(pixel_u)))
    cv = int(round(float(pixel_v)))
    values = []
    for dy in range(-int(radius_px), int(radius_px) + 1):
        for dx in range(-int(radius_px), int(radius_px) + 1):
            x = cu + dx
            y = cv + dy
            if not (0 <= x < width and 0 <= y < height):
                continue
            raw = depth_image[y, x]
            value = float(raw) if np.issubdtype(depth_image.dtype, np.floating) else float(raw) * float(depth_scale)
            if value > 0.1:
                values.append(value)
    if not values:
        return None
    arr = np.asarray(values, dtype=np.float64)
    return {
        "median_m": float(np.median(arr)),
        "min_m": float(np.min(arr)),
        "max_m": float(np.max(arr)),
        "std_mm": float(np.std(arr) * 1000.0),
        "valid_count": int(arr.size),
    }


def _depth_patch_stats_from_realsense(depth_frame, pixel_u, pixel_v, radius_px=4):
    if depth_frame is None:
        return None
    width = depth_frame.get_width()
    height = depth_frame.get_height()
    cu = int(round(float(pixel_u)))
    cv = int(round(float(pixel_v)))
    values = []
    for dy in range(-int(radius_px), int(radius_px) + 1):
        for dx in range(-int(radius_px), int(radius_px) + 1):
            x = cu + dx
            y = cv + dy
            if not (0 <= x < width and 0 <= y < height):
                continue
            value = float(depth_frame.get_distance(x, y))
            if value > 0.1:
                values.append(value)
    if not values:
        return None
    arr = np.asarray(values, dtype=np.float64)
    return {
        "median_m": float(np.median(arr)),
        "min_m": float(np.min(arr)),
        "max_m": float(np.max(arr)),
        "std_mm": float(np.std(arr) * 1000.0),
        "valid_count": int(arr.size),
    }


def _pixel_pt(point):
    return (int(round(float(point[0]))), int(round(float(point[1]))))


def _sample_pixel_line(start, end, num_points):
    sx, sy = float(start[0]), float(start[1])
    ex, ey = float(end[0]), float(end[1])
    count = max(2, int(num_points))
    return [
        (
            sx + (ex - sx) * (idx / max(1, count - 1)),
            sy + (ey - sy) * (idx / max(1, count - 1)),
        )
        for idx in range(count)
    ]


def _line_length_px(line):
    p0 = np.asarray(line[0], dtype=np.float64)
    p1 = np.asarray(line[1], dtype=np.float64)
    return float(np.linalg.norm(p1 - p0))


def _canonical_back_lines_from_spine(spine_line, meridian_lines=None, outer_scale=2.0):
    if spine_line is None:
        return spine_line, meridian_lines, None, None

    center_start = np.asarray(spine_line[0], dtype=np.float64)
    center_end = np.asarray(spine_line[1], dtype=np.float64)
    raw_start = center_start.copy()
    raw_end = center_end.copy()
    raw_length_px = float(np.linalg.norm(raw_end - raw_start))
    tangent = _normalize_vec(raw_end - raw_start)
    if tangent is None:
        return spine_line, meridian_lines, None, None

    neck_trim_ratio = max(0.0, min(0.45, float(BACK_LINE_TRIM_NECK_RATIO)))
    tail_trim_ratio = max(0.0, min(0.45, float(BACK_LINE_TRIM_TAIL_RATIO)))
    total_trim_ratio = neck_trim_ratio + tail_trim_ratio
    if total_trim_ratio > 0.80:
        scale = 0.80 / max(total_trim_ratio, 1e-6)
        neck_trim_ratio *= scale
        tail_trim_ratio *= scale
    neck_trim_px = raw_length_px * neck_trim_ratio
    tail_trim_px = raw_length_px * tail_trim_ratio
    center_start = raw_start + tangent * neck_trim_px
    center_end = raw_end - tangent * tail_trim_px
    image_offset = np.asarray(
        [BACK_LINE_OFFSET_X_PX, BACK_LINE_OFFSET_Y_PX],
        dtype=np.float64,
    )
    center_start = center_start + image_offset
    center_end = center_end + image_offset

    detected_tangent = tangent.copy()
    if BACK_LOCK_HORIZONTAL:
        line_length_px = float(np.linalg.norm(center_end - center_start))
        line_center = (center_start + center_end) * 0.5
        x_direction = -1.0 if float(tangent[0]) < 0.0 else 1.0
        tangent = np.array([x_direction, 0.0], dtype=np.float64)
        center_start = line_center - tangent * (line_length_px * 0.5)
        center_end = line_center + tangent * (line_length_px * 0.5)

    lateral_raw = None
    if meridian_lines is not None and len(meridian_lines) == 2 and meridian_lines[0] is not None and meridian_lines[1] is not None:
        left = meridian_lines[0]
        right = meridian_lines[1]
        left_start = np.asarray(left[0], dtype=np.float64)
        left_end = np.asarray(left[1], dtype=np.float64)
        right_start = np.asarray(right[0], dtype=np.float64)
        right_end = np.asarray(right[1], dtype=np.float64)
        lateral_raw = ((right_start - left_start) + (right_end - left_end)) * 0.5
    else:
        inner_offset = 0.0

    if lateral_raw is None or float(np.linalg.norm(lateral_raw)) <= 1e-6:
        lateral_raw = np.array([-tangent[1], tangent[0]], dtype=np.float64)

    lateral = lateral_raw - float(np.dot(lateral_raw, tangent)) * tangent
    lateral = _normalize_vec(lateral)
    if lateral is None:
        lateral = np.array([-tangent[1], tangent[0]], dtype=np.float64)
        lateral = _normalize_vec(lateral)
    if lateral is None:
        return spine_line, meridian_lines, None, None

    if meridian_lines is not None and len(meridian_lines) == 2 and meridian_lines[0] is not None and meridian_lines[1] is not None:
        left = meridian_lines[0]
        right = meridian_lines[1]
        candidates = [
            np.asarray(right[0], dtype=np.float64) - center_start,
            np.asarray(right[1], dtype=np.float64) - center_end,
            center_start - np.asarray(left[0], dtype=np.float64),
            center_end - np.asarray(left[1], dtype=np.float64),
        ]
        projected_offsets = [abs(float(np.dot(vec, lateral))) for vec in candidates]
        inner_offset = float(np.median(projected_offsets))

    if inner_offset <= 1.0:
        inner_offset = 20.0

    outer_offset = inner_offset * float(outer_scale)

    def _line_at_offset(offset):
        p0 = center_start + lateral * float(offset)
        p1 = center_end + lateral * float(offset)
        return (
            (float(p0[0]), float(p0[1])),
            (float(p1[0]), float(p1[1])),
        )

    canonical_spine = ((float(center_start[0]), float(center_start[1])), (float(center_end[0]), float(center_end[1])))
    canonical_inner = (_line_at_offset(-inner_offset), _line_at_offset(inner_offset))
    canonical_outer = (_line_at_offset(-outer_offset), _line_at_offset(outer_offset))
    meta = {
        "inner_offset_px": inner_offset,
        "outer_offset_px": outer_offset,
        "line_length_px": _line_length_px(canonical_spine),
        "raw_line_length_px": raw_length_px,
        "neck_trim_px": neck_trim_px,
        "tail_trim_px": tail_trim_px,
        "neck_trim_ratio": neck_trim_ratio,
        "tail_trim_ratio": tail_trim_ratio,
        "offset_x_px": float(image_offset[0]),
        "offset_y_px": float(image_offset[1]),
        "detected_angle_deg": math.degrees(
            math.atan2(float(detected_tangent[1]), float(detected_tangent[0]))
        ),
        "output_angle_deg": math.degrees(
            math.atan2(float(tangent[1]), float(tangent[0]))
        ),
        "horizontal_locked": bool(BACK_LOCK_HORIZONTAL),
    }
    return canonical_spine, canonical_inner, canonical_outer, meta


def _limit_unit_vector_to_cone(vec, center_vec, max_angle_deg):
    unit = _normalize_vec(vec)
    center = _normalize_vec(center_vec)
    if unit is None or center is None:
        return unit, 0.0, False

    limit = max(0.0, min(179.0, float(max_angle_deg)))
    dot = float(np.clip(np.dot(unit, center), -1.0, 1.0))
    angle_deg = math.degrees(math.acos(dot))
    if angle_deg <= limit:
        return unit, angle_deg, False

    perp = unit - dot * center
    perp_norm = float(np.linalg.norm(perp))
    if perp_norm <= 1e-9:
        basis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if abs(float(np.dot(basis, center))) > 0.9:
            basis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        perp = basis - float(np.dot(basis, center)) * center
        perp_norm = float(np.linalg.norm(perp))
    if perp_norm <= 1e-9:
        return center, angle_deg, True

    perp = perp / perp_norm
    limit_rad = math.radians(limit)
    limited = _normalize_vec(center * math.cos(limit_rad) + perp * math.sin(limit_rad))
    if limited is None:
        limited = center
    return limited, angle_deg, True


def _angle_between_unit_vectors(vec_a, vec_b):
    unit_a = _normalize_vec(vec_a)
    unit_b = _normalize_vec(vec_b)
    if unit_a is None or unit_b is None:
        return 0.0
    dot = float(np.clip(np.dot(unit_a, unit_b), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def _project_axis_to_tool_plane(axis, tool_z_unit):
    axis_unit = _normalize_vec(axis)
    tool_z = _normalize_vec(tool_z_unit)
    if axis_unit is None or tool_z is None:
        return None
    projected = axis_unit - float(np.dot(axis_unit, tool_z)) * tool_z
    projected = _normalize_vec(projected)
    if projected is not None:
        return projected
    basis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(np.dot(basis, tool_z))) > 0.9:
        basis = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    return _normalize_vec(basis - float(np.dot(basis, tool_z)) * tool_z)


class Ros2RobotProxy:
    """用 ROS 2 service/topic 模拟旧 SDK 的最小控制接口。"""

    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self._owns_context = False
        self.node = None
        self.client = None
        self.moveit_ik_client = None
        self._moveit_unavailable_reported = False
        self._sub = None
        self.latest_state = None
        self.latest_state_time = 0.0
        self._state_sequence = 0
        self._servo_interpolation_available = bool(ROS2_SERVO_INTERPOLATION)
        self._servo_interpolation_warning_reported = False
        self.default_ik_config = -1

    def connect(self):
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_context = True

        self.node = Node("lasttime_ros2_client")
        self.client = self.node.create_client(RemoteCmdInterface, ROS2_SERVICE_NAME)
        if MOVEIT_IK_ENABLE and MOVEIT_MSGS_AVAILABLE:
            self.moveit_ik_client = self.node.create_client(GetPositionIK, MOVEIT_IK_SERVICE)
        self._sub = self.node.create_subscription(
            RobotNonrtState,
            ROS2_STATE_TOPIC,
            self._state_callback,
            10,
        )

        print(f"等待 ROS2 控制服务: {ROS2_SERVICE_NAME}")
        deadline = time.time() + ROS2_SERVICE_WAIT_S
        while time.time() < deadline:
            if self.client.wait_for_service(timeout_sec=0.5):
                break
            self.spin_once(0.05)

        if not self.client.service_is_ready():
            raise RuntimeError(f"未找到 ROS2 控制服务: {ROS2_SERVICE_NAME}")

        self.wait_for_state(ROS2_STATE_WAIT_S, required=True)
        self._ensure_robot_ready()
        pose = self.get_actual_tcp_pose()
        pose_text = ", ".join(_fmt_value(v) for v in pose)
        print(f"机械臂初始化完成（ROS 2 控制服务已连接）")
        print(f"当前 TCP 位姿: [{pose_text}]")

    def _state_callback(self, msg):
        self.latest_state = msg
        self.latest_state_time = time.time()
        self._state_sequence += 1

    def spin_once(self, timeout_sec=0.1):
        if self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def wait_for_state(self, timeout_sec, required):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.latest_state is not None:
                return True
            self.spin_once(0.1)
        if required:
            raise RuntimeError(f"在 {timeout_sec:.1f}s 内未收到状态话题: {ROS2_STATE_TOPIC}")
        return False

    def _call(self, cmd_str, timeout_sec=ROS2_CALL_TIMEOUT_S, raise_on_error=True):
        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd_str
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"ROS2 指令超时: {cmd_str}")

        exc = future.exception()
        if exc is not None:
            raise RuntimeError(f"ROS2 指令异常 {cmd_str}: {exc}")

        cmd_res = future.result().cmd_res
        ret = _parse_ret_code(cmd_res)
        if raise_on_error and ret != 0:
            raise RuntimeError(f"ROS2 指令失败 {cmd_str}: {cmd_res}")
        return ret, cmd_res

    def _ensure_robot_ready(self):
        self.wait_for_state(ROS2_STATE_WAIT_S, required=False)

        if self.latest_state is None or int(_state_field(self.latest_state, "robot_mode", default=0)) != 0:
            self._call("Mode(0)")
            time.sleep(0.2)
            self.wait_for_state(ROS2_STATE_WAIT_S, required=False)

        ret, cmd_res = self._call("RobotEnable(1)", raise_on_error=False)
        if ret != 0 and ROS2_RESET_ERRORS:
            self._call("ResetAllError()", raise_on_error=False)
            time.sleep(0.5)
            ret, cmd_res = self._call("RobotEnable(1)", raise_on_error=False)
        if ret != 0:
            raise RuntimeError(f"机械臂使能失败: {cmd_res}")
        time.sleep(0.5)
        self.wait_for_state(ROS2_STATE_WAIT_S, required=False)

        self.SetSpeed(MOVE_VEL_FAST)

    def get_actual_tcp_pose(self):
        self.wait_for_state(ROS2_STATE_WAIT_S, required=True)
        return _state_pose(self.latest_state)

    def get_fresh_actual_tcp_pose(self, timeout_sec=1.0):
        """Return TCP feedback sampled after this method was called.

        Force-approach commands run inside the control service.  Their response
        can reach this client before the matching state-topic callback, so the
        cached ``latest_state`` may still describe the pre-approach hover pose.
        Never use that stale sample to decide that a safety retract can be
        skipped.
        """
        minimum_sequence = int(self._state_sequence)
        deadline = time.time() + max(0.05, float(timeout_sec))
        while time.time() < deadline:
            self.spin_once(min(0.1, max(0.0, deadline - time.time())))
            if self.latest_state is not None and self._state_sequence > minimum_sequence:
                return _state_pose(self.latest_state)
        raise RuntimeError(
            f"在 {float(timeout_sec):.1f}s 内未收到新的状态反馈: {ROS2_STATE_TOPIC}"
        )

    def get_actual_joint_positions_deg(self):
        self.wait_for_state(ROS2_STATE_WAIT_S, required=True)
        return _state_joints_deg(self.latest_state)

    def _moveit_service_ready(self):
        if not MOVEIT_IK_ENABLE:
            return False
        if not MOVEIT_MSGS_AVAILABLE or self.moveit_ik_client is None:
            if not self._moveit_unavailable_reported:
                print("[MoveIt] Python 消息包不可用，跳过 MoveIt IK")
                self._moveit_unavailable_reported = True
            return False
        if self.moveit_ik_client.service_is_ready():
            return True
        ready = self.moveit_ik_client.wait_for_service(timeout_sec=max(0.0, MOVEIT_IK_WAIT_S))
        if not ready and not self._moveit_unavailable_reported:
            print(f"[MoveIt] 未发现 IK 服务 {MOVEIT_IK_SERVICE}，跳过 MoveIt 兜底")
            self._moveit_unavailable_reported = True
        return ready

    def moveit_inverse_kin_joints_deg(self, desc_pos):
        if not self._moveit_service_ready():
            return None

        pose = [float(v) for v in desc_pos]
        req = GetPositionIK.Request()
        req.ik_request.group_name = MOVEIT_GROUP_NAME
        req.ik_request.avoid_collisions = MOVEIT_IK_AVOID_COLLISIONS
        req.ik_request.ik_link_name = MOVEIT_IK_LINK_NAME
        req.ik_request.timeout = Duration(
            sec=int(MOVEIT_IK_TIMEOUT_S),
            nanosec=int((MOVEIT_IK_TIMEOUT_S % 1.0) * 1e9),
        )

        seed_deg = self.get_actual_joint_positions_deg()
        joint_state = JointState()
        joint_state.header.stamp = self.node.get_clock().now().to_msg()
        joint_state.header.frame_id = MOVEIT_IK_FRAME_ID
        joint_state.name = list(MOVEIT_JOINT_NAMES)
        joint_state.position = [math.radians(v) for v in seed_deg]
        req.ik_request.robot_state.joint_state = joint_state

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = MOVEIT_IK_FRAME_ID
        pose_stamped.pose.position.x = pose[0] / 1000.0
        pose_stamped.pose.position.y = pose[1] / 1000.0
        pose_stamped.pose.position.z = pose[2] / 1000.0
        qx, qy, qz, qw = _quat_from_rpy_deg(pose[3], pose[4], pose[5])
        pose_stamped.pose.orientation.x = qx
        pose_stamped.pose.orientation.y = qy
        pose_stamped.pose.orientation.z = qz
        pose_stamped.pose.orientation.w = qw
        req.ik_request.pose_stamped = pose_stamped

        future = self.moveit_ik_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=MOVEIT_IK_CALL_TIMEOUT_S)
        if not future.done() or future.exception() is not None or future.result() is None:
            return None

        result = future.result()
        if int(result.error_code.val) != 1:
            return None

        names = list(result.solution.joint_state.name)
        positions = list(result.solution.joint_state.position)
        by_name = {name: pos for name, pos in zip(names, positions)}
        try:
            return [math.degrees(float(by_name[name])) for name in MOVEIT_JOINT_NAMES]
        except KeyError:
            return None

    def wait_motion_done(
        self,
        timeout_sec=ROS2_MOTION_DONE_WAIT_S,
        *,
        target_pose=None,
        target_joints_deg=None,
        after_state_sequence=None,
        require_motion_done=True,
        pose_tolerance_mm=ROS2_MOTION_DONE_POSE_TOL_MM,
        orientation_tolerance_deg=ROS2_MOTION_DONE_ORI_TOL_DEG,
    ):
        """Wait for post-command feedback that proves the requested target arrived.

        The controller may still publish ``motion_done=1`` from the preceding
        motion for a short time after accepting a new MoveL/MoveJ.  Treating that
        stale bit as completion caused thousands of targets to be queued while the
        robot was still travelling.  A completion now needs a newer feedback
        sample, target agreement and consecutive stable samples.
        """
        minimum_sequence = (
            self._state_sequence
            if after_state_sequence is None
            else int(after_state_sequence)
        )
        deadline = time.time() + timeout_sec
        checked_sequence = minimum_sequence
        stable_samples = 0
        while time.time() < deadline:
            self.spin_once(min(ROS2_MOTION_STATE_POLL_S, max(0.0, deadline - time.time())))
            if self.latest_state is None or self._state_sequence <= checked_sequence:
                continue
            checked_sequence = self._state_sequence

            motion_done = _state_field(
                self.latest_state,
                "motion_done",
                "robot_motion_done",
                default=None,
            )
            target_reached = True
            if target_pose is not None:
                current = _state_pose(self.latest_state)
                pos_dist, ori_dist = _pose_distance(current, target_pose)
                target_reached = (
                    pos_dist <= float(pose_tolerance_mm)
                    and ori_dist <= float(orientation_tolerance_deg)
                )
            elif target_joints_deg is not None:
                current = _state_joints_deg(self.latest_state)
                target_reached = max(
                    abs(_shortest_angle_delta_deg(current[i], target_joints_deg[i]))
                    for i in range(6)
                ) <= ROS2_MOTION_DONE_JOINT_TOL_DEG

            done_reached = (
                not require_motion_done
                or motion_done is None
                or int(motion_done) == 1
            )
            if target_reached and done_reached:
                stable_samples += 1
                if stable_samples >= ROS2_MOTION_DONE_STABLE_SAMPLES:
                    return True
            else:
                stable_samples = 0
        return False

    def pose_close_to(
        self,
        target_pose,
        position_tolerance_mm=ROS2_MOTION_DONE_POSE_TOL_MM,
        orientation_tolerance_deg=ROS2_MOTION_DONE_ORI_TOL_DEG,
    ):
        try:
            current = self.get_actual_tcp_pose()
        except Exception:
            return False
        pos_dist, ori_dist = _pose_distance(current, target_pose)
        if (
            pos_dist <= float(position_tolerance_mm)
            and ori_dist <= float(orientation_tolerance_deg)
        ):
            print(
                f"MoveL 等待 motion_done 超时，但实际位姿已接近目标 "
                f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)，按成功处理"
            )
            return True
        print(
            f"MoveL 等待 motion_done 超时，实际位姿未到目标 "
            f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)"
        )
        return False

    def SetSpeed(self, vel):
        ret, _ = self._call(f"SetSpeed({_fmt_value(_scaled_robot_motion_speed(vel))})")
        return ret

    def inverse_kin(self, desc_pos, config=-1):
        cmd = "GetInverseKin(0," + ",".join(_fmt_value(v) for v in desc_pos) + f",{int(config)})"
        ret, cmd_res = self._call(cmd, raise_on_error=False)
        return ret, cmd_res

    def inverse_kin_joints_deg(self, desc_pos, config=-1):
        ret, cmd_res = self.inverse_kin(desc_pos, config=config)
        if ret != 0:
            return ret, None
        joints = _parse_inverse_kin_joints(cmd_res)
        if joints is None:
            return -1002, None
        return 0, joints

    def inverse_kin_ok(self, desc_pos, config=-1):
        ret, _ = self.inverse_kin(desc_pos, config=config)
        return ret == 0

    def MoveJ(
        self,
        joint_pos_deg,
        tool=ROS2_TOOL,
        user=ROS2_USER,
        vel=MOVE_VEL_FAST,
        blendT=BLEND_BLOCKING,
    ):
        jnt_cmd = "JNTPoint(1," + ",".join(_fmt_value(v) for v in joint_pos_deg) + ")"
        ret, _ = self._call(jnt_cmd, raise_on_error=False)
        if ret != 0:
            return ret

        cmd = "MoveJ(" + ",".join(
            [
                "JNT1",
                _fmt_value(_scaled_robot_motion_speed(vel)),
                _fmt_value(int(tool)),
                _fmt_value(int(user)),
            ]
        ) + ")"
        ret, _ = self._call(cmd, raise_on_error=False)
        if ret == 0 and float(blendT) < 0:
            state_sequence_after_call = self._state_sequence
            if not self.wait_motion_done(
                target_joints_deg=joint_pos_deg,
                after_state_sequence=state_sequence_after_call,
            ):
                return -1001
        return ret

    def MoveCart(
        self,
        desc_pos,
        tool=ROS2_TOOL,
        user=ROS2_USER,
        vel=MOVE_VEL_FAST,
        acc=ROS2_MOVE_ACC,
        ovl=ROS2_MOVE_OVL,
        blendT=BLEND_BLOCKING,
        config=-1,
        timeout_sec=ROS2_MOTION_DONE_WAIT_S,
    ):
        self.spin_once(ROS2_MOTION_STATE_POLL_S)
        start_pose = self.get_actual_tcp_pose()
        position_tolerance_mm, orientation_tolerance_deg = _motion_completion_tolerances(
            start_pose,
            desc_pos,
        )
        effective_config = int(config)
        if effective_config == -1:
            effective_config = int(self.default_ik_config)

        point_name = "CART1"
        if 0 <= effective_config <= 7:
            ret, joint_pos_deg = self.inverse_kin_joints_deg(
                desc_pos,
                config=effective_config,
            )
            if ret != 0 or joint_pos_deg is None:
                print(
                    f"MoveL 指定 config={effective_config} 逆解失败 "
                    f"(err={ret}, target={[float(v) for v in desc_pos]})"
                )
                return ret
            point_cmd = "JNTPoint(1," + ",".join(
                _fmt_value(v) for v in joint_pos_deg
            ) + ")"
            point_name = "JNT1"
        else:
            point_cmd = "CARTPoint(1," + ",".join(_fmt_value(v) for v in desc_pos) + ")"
        ret, _ = self._call(point_cmd, raise_on_error=False)
        if ret != 0:
            return ret

        cmd = "MoveL(" + ",".join(
            [
                point_name,
                _fmt_value(_scaled_robot_motion_speed(vel)),
                _fmt_value(int(tool)),
                _fmt_value(int(user)),
            ]
        ) + ")"
        ret, _ = self._call(cmd, raise_on_error=False)
        if ret == 0 and float(blendT) < 0:
            state_sequence_after_call = self._state_sequence
            if not self.wait_motion_done(
                timeout_sec=timeout_sec,
                target_pose=desc_pos,
                after_state_sequence=state_sequence_after_call,
                pose_tolerance_mm=position_tolerance_mm,
                orientation_tolerance_deg=orientation_tolerance_deg,
            ):
                if self.pose_close_to(
                    desc_pos,
                    position_tolerance_mm,
                    orientation_tolerance_deg,
                ):
                    return 0
                return -1001
        return ret

    def MoveCartInterpolated(
        self,
        desc_pos,
        tool=ROS2_TOOL,
        user=ROS2_USER,
        vel=MOVE_VEL_FAST,
    ):
        """Move to the same Cartesian target through an internal timed servo stream.

        The custom command runs inside the ROS2 server so service round trips cannot
        disturb the requested interpolation cadence. Non-zero tool/user coordinates
        retain the original MoveCart path because ServoCart mode 0 uses base poses.
        """
        if not self._servo_interpolation_available or int(tool) != 0 or int(user) != 0:
            return None

        # Refresh the local state before deciding whether this short force-control
        # move is eligible. The server obtains the authoritative start pose again.
        self.spin_once(0.02)
        start_pose = self.get_actual_tcp_pose()
        linear_distance_mm, orientation_distance_deg = _servo_interpolation_metrics(
            start_pose,
            desc_pos,
        )
        if (
            linear_distance_mm > ROS2_SERVO_MAX_DISTANCE_MM
            or orientation_distance_deg > ROS2_SERVO_MAX_ORIENTATION_DEG
        ):
            return None

        speed_percent = _scaled_robot_motion_speed(vel)
        duration_s = _servo_interpolation_duration_s(
            start_pose,
            desc_pos,
            speed_percent,
        )
        position_tolerance_mm, orientation_tolerance_deg = _motion_completion_tolerances(
            start_pose,
            desc_pos,
        )
        cmd = "ServoCartSmooth(" + ",".join(
            _fmt_value(value)
            for value in [
                *desc_pos,
                duration_s,
                ROS2_SERVO_INTERPOLATION_HZ,
            ]
        ) + ")"
        ret, _ = self._call(
            cmd,
            timeout_sec=max(ROS2_CALL_TIMEOUT_S, duration_s + 5.0),
            raise_on_error=False,
        )
        state_sequence_after_call = self._state_sequence
        if ret == -1:
            self._servo_interpolation_available = False
            if not self._servo_interpolation_warning_reported:
                print(
                    "[Force] 控制服务不支持 ServoCartSmooth；本次进程停止重复探测，"
                    "请重新构建并重启 ROS2 控制服务"
                )
                self._servo_interpolation_warning_reported = True
        elif ret == 0 and not self.wait_motion_done(
            timeout_sec=max(1.0, duration_s + 1.0),
            target_pose=desc_pos,
            after_state_sequence=state_sequence_after_call,
            require_motion_done=False,
            pose_tolerance_mm=position_tolerance_mm,
            orientation_tolerance_deg=orientation_tolerance_deg,
        ):
            return -1001
        return ret

    def CloseRPC(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()
            self._owns_context = False


class Ros2ForceController:
    """通过同一个 ROS2 command server 控制六维力传感器和恒力控制。"""

    def __init__(
        self,
        robot_proxy,
        target_force_n=FORCE_TARGET_N,
        approach_max_offset_mm=FORCE_APPROACH_MAX_OFFSET_MM,
    ):
        self.robot = robot_proxy
        self.target_force_n = abs(float(target_force_n))
        self.approach_max_offset_mm = float(approach_max_offset_mm)
        self.active = False
        self.guard_active = False
        self.config = ForceControlConfig()
        self.config.target_force_z = self.target_force_n
        self.config.ft_pid = [FORCE_PID_P, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.config.max_dis = FORCE_MAX_DIS_MM
        self.config.max_ang = FORCE_MAX_ANG_DEG
        self.config.software_force_limit = max(
            abs(FORCE_SOFTWARE_NORMAL_LIMIT_N),
            self.target_force_n + 30.0,
        )
        self.tangential_force_limit = max(
            abs(FORCE_SOFTWARE_TANGENTIAL_LIMIT_N),
            self.config.software_force_limit,
        )
        self.config.enable_collision_guard = FORCE_GUARD_ENABLE
        self.config.guard_force_limit = max(abs(FORCE_GUARD_LIMIT_N), self.target_force_n + 10.0)
        self.config.guard_torque_limit = abs(FORCE_GUARD_TORQUE_LIMIT_NM)
        self._monitor_period = 1.0 / max(1.0, FORCE_MONITOR_HZ)

    def _cmd(self, name, *values):
        return f"{name}(" + ",".join(_fmt_value(v) for v in values) + ")"

    def _call_force(self, cmd, context, required=True):
        try:
            ret, cmd_res = self.robot._call(cmd, raise_on_error=False)
        except Exception as exc:
            if required:
                raise
            print(f"[Force] {context} 异常: {exc}")
            return -9999, str(exc)
        if ret != 0:
            message = f"{context} 失败: {cmd_res}"
            if required:
                raise RuntimeError(message)
            print(f"[Force] {message}")
        return ret, cmd_res

    def _parse_force_response(self, cmd_res):
        parts = [p.strip() for p in str(cmd_res).split(",")]
        if len(parts) < 7:
            return None
        try:
            ret = int(float(parts[0]))
            values = [float(v) for v in parts[1:7]]
        except ValueError:
            return None
        if ret != 0:
            return None
        return values

    def continuous_approach(self, target_pose, context):
        """Run one force-aware continuous servo session toward ``target_pose``.

        Safety failures deliberately raise instead of falling back to MoveCart: once
        contact is possible, a position-only retry could continue pressing after the
        force-aware controller has already stopped.
        """
        target_force_n = abs(float(self.target_force_n))
        fine_ratio = max(0.0, min(1.0, float(FORCE_APPROACH_FINE_RATIO)))
        near_ratio = max(fine_ratio, min(1.0, float(FORCE_APPROACH_NEAR_RATIO)))
        cmd = self._cmd(
            "ServoCartForceApproach",
            *target_pose,
            FORCE_CONTINUOUS_APPROACH_HZ,
            FORCE_AXIS_SIGN,
            target_force_n,
            max(0.0, min(target_force_n, abs(float(FORCE_APPROACH_CONTACT_N)))),
            fine_ratio,
            near_ratio,
            FORCE_CONTINUOUS_COARSE_SPEED_MM_S,
            FORCE_CONTINUOUS_CONTACT_SPEED_MM_S,
            FORCE_CONTINUOUS_FINE_SPEED_MM_S,
            FORCE_CONTINUOUS_NEAR_SPEED_MM_S,
            FORCE_CONTINUOUS_ACCEL_MM_S2,
            FORCE_CONTINUOUS_DECEL_MM_S2,
            FORCE_CONTINUOUS_FILTER_ALPHA,
            FORCE_CONTINUOUS_TARGET_STABLE_SAMPLES,
            self.config.software_force_limit,
            self.tangential_force_limit,
            FORCE_SOFTWARE_TORQUE_LIMIT_NM,
            FORCE_CONTINUOUS_TIMEOUT_S,
        )
        ret, cmd_res = self.robot._call(
            cmd,
            timeout_sec=max(ROS2_CALL_TIMEOUT_S, FORCE_CONTINUOUS_TIMEOUT_S + 5.0),
            raise_on_error=False,
        )
        result = _parse_force_approach_response(cmd_res)
        if result is None:
            if ret == -1:
                raise RuntimeError(
                    f"{context}: 控制服务不支持连续伺服贴近，"
                    "请重新构建并重启 ROS2 控制服务"
                )
            raise RuntimeError(f"{context}: 连续伺服贴近返回不可解析: {cmd_res}")
        if ret != 0:
            reason = {
                -2201: "法向力超过软件限位",
                -2202: "切向力超过软件限位",
                -2203: "力矩超过软件限位",
                -2204: "连续伺服期间无法读取六维力",
                -2205: "连续伺服贴近超时",
            }.get(ret, f"控制器/SDK 错误 {ret}")
            data = result["force"]
            raise RuntimeError(
                f"{context}: {reason}; "
                f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N "
                f"travel={result['travel_mm']:.2f}mm"
            )
        return result

    def _reading_available(self):
        data = self.read()
        return data is not None

    def _allow_zero_failure(self):
        data = self.read()
        if data is None or not FORCE_ALLOW_SKIP_ZERO:
            return False
        max_force = max(abs(data[0]), abs(data[1]), abs(data[2]))
        if max_force <= FORCE_ZERO_MAX_ABS_N:
            print(
                "[Force] FT_SetZero 失败但当前读数接近零点，继续执行 "
                f"(Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N, "
                f"threshold={FORCE_ZERO_MAX_ABS_N:.1f}N)"
            )
            return True
        print(
            "[Force] FT_SetZero 失败且当前力偏置过大，停止 "
            f"(Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N)"
        )
        return False

    def connect_and_init(self):
        print("\n[Force] 使用 ROS2 控制服务初始化力传感器...")
        print("[Force] 校零要求：末端必须悬空、无外部接触。")
        self._call_force(
            self._cmd("FT_SetConfig", FORCE_SENSOR_COMPANY, FORCE_SENSOR_DEVICE, 0, FORCE_SENSOR_BUS),
            "FT_SetConfig",
        )
        print(
            f"[Force] FT_SetConfig OK "
            f"(company={FORCE_SENSOR_COMPANY}, device={FORCE_SENSOR_DEVICE}, bus={FORCE_SENSOR_BUS})"
        )
        self._call_force(self._cmd("FT_Activate", 0), "FT_Activate(0)", required=False)
        time.sleep(0.3)
        ret, _ = self._call_force(self._cmd("FT_Activate", 1), "FT_Activate(1)", required=False)
        if ret != 0:
            if self._reading_available():
                print(f"[Force] FT_Activate(1) 警告 err={ret}；但力传感器读数可用，继续")
            else:
                raise RuntimeError(f"FT_Activate(1) 失败 err={ret}")
        else:
            print("[Force] FT_Activate OK")
        time.sleep(0.5)

        self._call_force(self._cmd("SetForceSensorPayload", 0.0), "SetForceSensorPayload(0)")
        self._call_force(self._cmd("SetForceSensorPayloadCog", 0.0, 0.0, 0.0), "SetForceSensorPayloadCog(0,0,0)")
        print("[Force] 负载参数已置零，准备传感器校零")

        self._call_force(self._cmd("FT_SetZero", 0), "FT_SetZero(0)", required=False)
        time.sleep(0.3)
        zero_ok = False
        for attempt in range(1, 6):
            time.sleep(0.6)
            ret, _ = self._call_force(self._cmd("FT_SetZero", 1), "FT_SetZero(1)", required=False)
            if ret == 0:
                print(f"[Force] FT_SetZero(1) OK (attempt {attempt}/5)")
                zero_ok = True
                break
        if not zero_ok:
            if not self._allow_zero_failure():
                raise RuntimeError("FT_SetZero(1) 多次失败，请确认末端悬空且传感器负载已置零")

        ret, _ = self._call_force(self._cmd("FT_SetRCS", 0, 0, 0, 0, 0, 0, 0), "FT_SetRCS", required=False)
        if ret != 0:
            if self._reading_available():
                print(f"[Force] FT_SetRCS 警告 err={ret}；但力传感器读数可用，继续")
            else:
                raise RuntimeError(f"FT_SetRCS 失败 err={ret}")
        else:
            print("[Force] FT_SetRCS OK (工具坐标系)")

        if self.config.enable_collision_guard:
            self._enable_guard()
        data = self.read()
        if data is not None:
            print(
                "[Force] 校零后读数: "
                f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f} "
                f"Mx={data[3]:.3f} My={data[4]:.3f} Mz={data[5]:.3f}"
            )
        print(
            f"[Force] 软件恒力贴近参数: target={self.target_force_n:.1f}N, "
            f"approach_step={FORCE_APPROACH_STEP_MM:.2f}mm, "
            f"fine_step={FORCE_APPROACH_FINE_STEP_MM:.2f}mm, "
            f"near_step={FORCE_APPROACH_NEAR_STEP_MM:.2f}mm, "
            f"vel_scale={FORCE_APPROACH_SPEED_SCALE:.1f}, "
            f"max_offset={self.approach_max_offset_mm:.1f}mm, "
            f"normal_limit={self.config.software_force_limit:.1f}N, "
            f"tangent_limit={self.tangential_force_limit:.1f}N, "
            f"axis_sign={FORCE_AXIS_SIGN:+.0f}"
        )

    def _enable_guard(self):
        if not self.config.enable_collision_guard or self.guard_active:
            return
        fl = self.config.guard_force_limit
        tl = self.config.guard_torque_limit
        self._call_force(
            self._cmd(
                "FT_Guard",
                1,
                SENSOR_ID,
                1, 1, 1, 1, 1, 1,
                0, 0, 0, 0, 0, 0,
                fl, fl, fl, tl, tl, tl,
                fl, fl, fl, tl, tl, tl,
            ),
            "FT_Guard(1)",
        )
        print(f"[Force] 碰撞守护已开启 (力阈值={fl:.1f}N, 力矩阈值={tl:.1f}Nm)")
        self.guard_active = True

    def _disable_guard(self):
        if not self.guard_active:
            return
        fl = self.config.guard_force_limit
        tl = self.config.guard_torque_limit
        self._call_force(
            self._cmd(
                "FT_Guard",
                0,
                SENSOR_ID,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                fl, fl, fl, tl, tl, tl,
                fl, fl, fl, tl, tl, tl,
            ),
            "FT_Guard(0)",
            required=False,
        )
        print("[Force] 碰撞守护已关闭")
        self.guard_active = False

    def _ft_control_cmd(self, flag, select, ft, ft_pid, max_dis, max_ang, is_no_block):
        return self._cmd(
            "FT_Control",
            int(flag),
            SENSOR_ID,
            *select,
            *ft,
            *ft_pid,
            0,
            0,
            max_dis,
            max_ang,
            self.config.filter_sign,
            0,
            int(is_no_block),
        )

    def update_target_force(self, target_force_n, context="力度调整"):
        target_force_n = abs(float(target_force_n))
        self.target_force_n = target_force_n
        self.config.target_force_z = target_force_n
        self.config.software_force_limit = max(
            abs(FORCE_SOFTWARE_NORMAL_LIMIT_N),
            target_force_n + 30.0,
        )
        self.tangential_force_limit = max(
            abs(FORCE_SOFTWARE_TANGENTIAL_LIMIT_N),
            self.config.software_force_limit,
        )
        self.config.guard_force_limit = max(abs(FORCE_GUARD_LIMIT_N), target_force_n + 10.0)

        if not self.active:
            return

        target_fz = FORCE_AXIS_SIGN * target_force_n
        cmd = self._ft_control_cmd(
            flag=1,
            select=[0, 0, 1, 0, 0, 0],
            ft=[0, 0, target_fz, 0, 0, 0],
            ft_pid=self.config.ft_pid,
            max_dis=self.config.max_dis,
            max_ang=self.config.max_ang,
            is_no_block=1,
        )
        self._call_force(cmd, f"{context}: FT_Control目标力更新")
        print(f"[Force] {context}: 目标力已更新为 {target_force_n:.1f}N")

    def _stop_robot_motion(self, context):
        self._call_force(self._cmd("StopMotion"), f"{context}: StopMotion", required=False)

    def start(self, context):
        if self.robot is None:
            raise RuntimeError("力控通道未初始化")
        if self.active:
            return
        data = self.read()
        if data is not None:
            max_force = max(abs(data[0]), abs(data[1]), abs(data[2]))
            if max_force > FORCE_PRESTART_LIMIT_N:
                self._stop_robot_motion("力控启动前力偏大")
                raise RuntimeError(
                    f"{context}: 力控启动前已有过大接触力，"
                    f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N"
                )
        self._enable_guard()
        target_fz = FORCE_AXIS_SIGN * abs(self.target_force_n)
        cmd = self._ft_control_cmd(
            flag=1,
            select=[0, 0, 1, 0, 0, 0],
            ft=[0, 0, target_fz, 0, 0, 0],
            ft_pid=self.config.ft_pid,
            max_dis=self.config.max_dis,
            max_ang=self.config.max_ang,
            is_no_block=1,
        )
        self._call_force(cmd, f"{context}: FT_Control(1)")
        print(
            f"[Force] 恒力控制已启动 (目标力={target_fz:.1f}N, "
            f"PID_P={self.config.ft_pid[0]}, max_dis={self.config.max_dis:.1f}mm)"
        )
        self.active = True
        time.sleep(FORCE_START_SETTLE_S)
        self.check_limits(context)

    def stop(self, context=""):
        if self.robot is None:
            return
        if self.active:
            cmd = self._ft_control_cmd(
                flag=0,
                select=[0, 0, 0, 0, 0, 0],
                ft=[0, 0, 0, 0, 0, 0],
                ft_pid=[0, 0, 0, 0, 0, 0],
                max_dis=0,
                max_ang=0,
                is_no_block=1,
            )
            ret, _ = self._call_force(cmd, f"{context}: FT_Control(0)", required=False)
            if ret == 0:
                print(f"[Force] 恒力控制已停止 {context}".rstrip())
            self.active = False
            time.sleep(FORCE_STOP_SETTLE_S)
            try:
                self.robot.spin_once(0.05)
            except Exception:
                pass

        self._disable_guard()

    def read(self):
        if self.robot is None:
            return None
        ret, cmd_res = self._call_force(self._cmd("FT_GetForceTorqueRCS", 1), "FT_GetForceTorqueRCS", required=False)
        if ret == 0:
            data = self._parse_force_response(cmd_res)
            if data is not None:
                return data

        self.robot.spin_once(0.02)
        msg = self.robot.latest_state
        data = [
            _state_field(msg, "ft_fx_data"),
            _state_field(msg, "ft_fy_data"),
            _state_field(msg, "ft_fz_data"),
            _state_field(msg, "ft_tx_data"),
            _state_field(msg, "ft_ty_data"),
            _state_field(msg, "ft_tz_data"),
        ]
        if any(v is None for v in data):
            return None
        return [float(v) for v in data]

    def check_limits(self, context):
        data = self.read()
        if data is None:
            raise RuntimeError(f"{context}: 无法读取六维力数据")
        normal_force = abs(float(data[2]))
        tangent_force = math.sqrt(float(data[0]) * float(data[0]) + float(data[1]) * float(data[1]))
        max_torque = max(abs(data[3]), abs(data[4]), abs(data[5]))
        if normal_force > self.config.software_force_limit:
            self._stop_robot_motion("软件法向力限触发")
            self.stop("软件法向力限触发")
            self._stop_robot_motion("软件法向力限触发")
            raise RuntimeError(
                f"{context}: 软件法向力限触发，"
                f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N "
                f"normal={normal_force:.2f}N limit={self.config.software_force_limit:.1f}N"
            )
        if tangent_force > getattr(self, "tangential_force_limit", self.config.software_force_limit):
            self._stop_robot_motion("软件横向力限触发")
            self.stop("软件横向力限触发")
            self._stop_robot_motion("软件横向力限触发")
            raise RuntimeError(
                f"{context}: 软件横向力限触发，"
                f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N "
                f"tangent={tangent_force:.2f}N "
                f"limit={getattr(self, 'tangential_force_limit', self.config.software_force_limit):.1f}N"
            )
        if max_torque > FORCE_SOFTWARE_TORQUE_LIMIT_NM:
            self._stop_robot_motion("软件力矩限触发")
            self.stop("软件力矩限触发")
            self._stop_robot_motion("软件力矩限触发")
            raise RuntimeError(
                f"{context}: 软件力矩限触发，"
                f"Mx={data[3]:.3f} My={data[4]:.3f} Mz={data[5]:.3f}Nm"
            )
        return data

    def dwell(self, seconds, context):
        deadline = time.time() + max(0.0, float(seconds))
        last_print = 0.0
        while time.time() < deadline:
            data = self.check_limits(context)
            now = time.time()
            if now - last_print > 0.5:
                print(
                    f"[Force] {context}: "
                    f"Fx={data[0]:.1f} Fy={data[1]:.1f} Fz={data[2]:.1f}N"
                )
                last_print = now
            time.sleep(self._monitor_period)

    def close(self):
        if self.robot is None:
            return
        try:
            self.stop("关闭")
            print("[Force] 力控通道已关闭")
        except Exception as exc:
            print(f"[Force] 关闭力控通道失败: {exc}")
        finally:
            self.robot = None


class LastTimeRos2Demo(_SdkLastTimeDemo):
    """复用原 lasttime 的视觉逻辑，替换为 ROS 2 机械臂控制。"""

    def __init__(self, massage_target="back"):
        super().__init__()
        self.force_controller = None
        self.massage_target = _normalize_massage_target(massage_target) or "back"
        self.camera_to_robot = None
        self.back_line_meta = {}
        self.back_posture_seed = None
        self._back_posture_active = False
        self.inner_posture_seed = None
        self._inner_posture_active = False
        self._inner_posture_transitioning = False
        self._inner_posture_return_joints_deg = None
        self._inner_previous_motion_orientation = None
        self.force_target_n = float(_force_target_for_massage_target(self.massage_target))
        if _is_thigh_target(self.massage_target):
            self.hover_height_mm = float(THIGH_HOVER_HEIGHT_MM)
            self.force_approach_max_offset_mm = float(THIGH_FORCE_APPROACH_MAX_OFFSET_MM)
        else:
            self.hover_height_mm = float(BACK_HOVER_HEIGHT_MM)
            self.force_approach_max_offset_mm = float(FORCE_APPROACH_MAX_OFFSET_MM)
        if self.massage_target == "leg_inner" and THIGH_INNER_POSTURE_SEED_ENABLE:
            self.inner_posture_seed = self._load_inner_posture_seed()
        if self.massage_target == "back" and BACK_POSTURE_SEED_ENABLE:
            self.back_posture_seed = self._load_back_posture_seed()

    def _load_back_posture_seed(self):
        path = Path(BACK_POSTURE_SEED_FILE)
        if not path.is_file():
            raise RuntimeError(f"背部姿态种子不存在: {path}")
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise RuntimeError(f"无法读取背部姿态种子 {path}: {exc}") from exc

        orientation = [float(value) for value in payload.get("orientation_rpy", [])]
        config = int(payload.get("ik_config", -1))
        if len(orientation) != 3 or not all(math.isfinite(value) for value in orientation):
            raise RuntimeError(f"背部姿态种子必须包含 3 维有限 orientation_rpy: {path}")
        if not 0 <= config <= 7:
            raise RuntimeError(f"背部姿态种子的 ik_config 必须在 0~7: {config}")
        print(
            f"[BackPosture] 已加载背部姿态种子: config={config}, "
            f"orientation={self._fmt_pose(orientation) if hasattr(self, '_fmt_pose') else orientation}, "
            f"file={path}"
        )
        return {
            "orientation_rpy": orientation,
            "ik_config": config,
            "path": str(path),
        }

    def _load_inner_posture_seed(self):
        path = Path(THIGH_INNER_POSTURE_SEED_FILE)
        if not path.is_file():
            raise RuntimeError(f"大腿内侧构型种子不存在: {path}")
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise RuntimeError(f"无法读取大腿内侧构型种子 {path}: {exc}") from exc

        pose = [float(value) for value in payload.get("pose", [])]
        joints = [float(value) for value in payload.get("joint_deg", [])]
        config = int(payload.get("ik_config", -1))
        if len(pose) != 6 or len(joints) != 6:
            raise RuntimeError(f"大腿内侧构型种子必须包含 6 维 pose 和 joint_deg: {path}")
        if not all(math.isfinite(value) for value in pose + joints):
            raise RuntimeError(f"大腿内侧构型种子包含非有限数值: {path}")
        if not 0 <= config <= 7:
            raise RuntimeError(f"大腿内侧构型种子的 ik_config 必须在 0~7: {config}")
        if pose[2] < float(THIGH_INNER_POSTURE_LIFT_Z_MM):
            raise RuntimeError(
                f"大腿内侧构型种子 Z={pose[2]:.1f}mm 低于切换安全高度 "
                f"{THIGH_INNER_POSTURE_LIFT_Z_MM:.1f}mm"
            )
        joint_soft_limits = payload.get("joint_soft_limits_deg")
        if joint_soft_limits is not None:
            if (
                not isinstance(joint_soft_limits, list)
                or len(joint_soft_limits) != 6
                or any(
                    not isinstance(limit, list) or len(limit) != 2
                    for limit in joint_soft_limits
                )
            ):
                raise RuntimeError(
                    f"大腿内侧构型种子的 joint_soft_limits_deg 必须包含 6 组上下限: {path}"
                )
            joint_soft_limits = [
                [float(limit[0]), float(limit[1])] for limit in joint_soft_limits
            ]
            for index, (joint, limits) in enumerate(zip(joints, joint_soft_limits), start=1):
                lower, upper = limits
                if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                    raise RuntimeError(f"大腿内侧构型种子 J{index} 软限位无效: {limits}")
                if joint < lower or joint > upper:
                    raise RuntimeError(
                        f"大腿内侧构型种子 J{index}={joint:.3f}deg 超出软限位 "
                        f"[{lower:.3f}, {upper:.3f}]deg"
                    )

        print(
            f"[InnerPosture] 已加载图2构型种子: config={config}, "
            f"pose={self._fmt_pose(pose) if hasattr(self, '_fmt_pose') else pose}, file={path}"
        )
        return {
            "pose": pose,
            "joint_deg": joints,
            "ik_config": config,
            "joint_soft_limits_deg": joint_soft_limits,
            "path": str(path),
        }

    def _fixed_motion_orientation(self):
        if (
            getattr(self, "massage_target", None) == "back"
            and getattr(self, "back_posture_seed", None) is not None
        ):
            return list(self.back_posture_seed["orientation_rpy"])
        if (
            getattr(self, "massage_target", None) == "leg_inner"
            and THIGH_INNER_POSTURE_SEED_ENABLE
            and THIGH_INNER_USE_SEED_ORIENTATION
            and getattr(self, "inner_posture_seed", None) is not None
        ):
            return list(self.inner_posture_seed["pose"][3:6])
        if ROS2_KEEP_CURRENT_ORIENTATION and getattr(self, "motion_orientation", None) is not None:
            return list(self.motion_orientation)
        return None

    def _current_force_target_n(self):
        return abs(float(self.force_target_n))

    def set_force_target_n(self, target_force_n, context="力度调整"):
        target_force_n = abs(float(target_force_n))
        min_force = min(float(LIVE_FORCE_TARGET_MIN_N), float(LIVE_FORCE_TARGET_MAX_N))
        max_force = max(float(LIVE_FORCE_TARGET_MIN_N), float(LIVE_FORCE_TARGET_MAX_N))
        target_force_n = max(min_force, min(max_force, target_force_n))
        previous = self._current_force_target_n()
        self.force_target_n = target_force_n
        if self.force_controller is not None:
            self.force_controller.update_target_force(target_force_n, context=context)
        return previous, target_force_n

    def init_robot(self):
        print(f"连接机械臂 {ROBOT_IP}...")
        print(f"ROS2 控制服务: {ROS2_SERVICE_NAME}")
        print(f"ROS2 状态话题: {ROS2_STATE_TOPIC}")
        if ROS2_SERVO_INTERPOLATION:
            print(
                f"力控短距离运动: {ROS2_SERVO_INTERPOLATION_HZ:g}Hz 五次插值 "
                f"(最大 {ROS2_SERVO_MAX_DISTANCE_MM:g}mm，失败自动回退 MoveCart)"
            )
        if FORCE_CONTINUOUS_APPROACH:
            print(
                f"力控贴近: {FORCE_CONTINUOUS_APPROACH_HZ:g}Hz 连续伺服 "
                f"(速度 {FORCE_CONTINUOUS_COARSE_SPEED_MM_S:g}/"
                f"{FORCE_CONTINUOUS_CONTACT_SPEED_MM_S:g}/"
                f"{FORCE_CONTINUOUS_FINE_SPEED_MM_S:g}/"
                f"{FORCE_CONTINUOUS_NEAR_SPEED_MM_S:g}mm/s，"
                "安全故障不回退 MoveCart)"
            )
        print(
            "贴近姿态: "
            + (
                "保持当前 TCP 姿态并沿工具 Z 轴贴近"
                if ROS2_KEEP_CURRENT_ORIENTATION
                else "跟随局部深度平面法向"
            )
        )
        self.robot = Ros2RobotProxy(ROBOT_IP)
        self.robot.connect()

    def init_force_controller(self):
        if not LASTTIME_ROS2_FORCE:
            return True
        if self.force_controller is not None:
            return True
        self.force_controller = Ros2ForceController(
            self.robot,
            self.force_target_n,
            approach_max_offset_mm=self.force_approach_max_offset_mm,
        )
        self.force_controller.connect_and_init()
        return True

    def close_force_controller(self):
        if self.force_controller is not None:
            self.force_controller.close()
            self.force_controller = None

    def _print_trajectory_depth_report(self, label):
        if not self.massage_frames:
            return
        print(f"[轨迹深度] {label}: {len(self.massage_frames)} 个采样点")
        prev_depth = None
        for frame in self.massage_frames:
            idx = int(frame.get("index", 0)) + 1
            pixel = frame.get("pixel", (None, None))
            point = frame.get("point_mm", [0.0, 0.0, 0.0])
            depth_m = frame.get("depth_m")
            stats = frame.get("depth_patch_stats") or {}
            std_mm = stats.get("std_mm")
            count = stats.get("valid_count")
            warn = ""
            if std_mm is not None and float(std_mm) > 18.0:
                warn += " depth-noisy"
            if count is not None and int(count) < 12:
                warn += " few-depth"
            if prev_depth is not None and depth_m is not None and abs(float(depth_m) - prev_depth) > 0.08:
                warn += " depth-jump"
            if depth_m is not None:
                prev_depth = float(depth_m)
            depth_text = "-" if depth_m is None else f"{float(depth_m):.3f}m"
            std_text = "-" if std_mm is None else f"{float(std_mm):.1f}mm"
            count_text = "-" if count is None else str(int(count))
            print(
                f"  P{idx:02d}: pixel=({float(pixel[0]):.1f},{float(pixel[1]):.1f}) "
                f"depth={depth_text} std={std_text} n={count_text} "
                f"robot=({float(point[0]):.1f},{float(point[1]):.1f},{float(point[2]):.1f}){warn}"
            )

    def _thigh_target_label(self):
        return _massage_target_label(self.massage_target)

    def _thigh_trajectory_type(self):
        return "thigh_inner" if self.massage_target == "leg_inner" else "thigh_outerline"

    def _crop_thigh_target_samples(self, line_pixels, depths, surface_points):
        if self.massage_target == "leg":
            skip = max(0, int(THIGH_OUTER_TAIL_SKIP_POINTS))
            if skip <= 0:
                return line_pixels, depths, surface_points, 0

            total = len(line_pixels)
            if total <= skip:
                empty_pixels = np.asarray(line_pixels)[:0]
                return empty_pixels, [], [], min(skip, total)

            keep_until = total - skip
            return (
                np.asarray(line_pixels)[:keep_until],
                list(depths)[:keep_until],
                list(surface_points)[:keep_until],
                skip,
            )

        if self.massage_target != "leg_inner":
            return line_pixels, depths, surface_points, 0

        head_skip = max(0, int(THIGH_INNER_SKIP_POINTS))
        tail_skip = max(0, int(THIGH_INNER_TAIL_SKIP_POINTS))
        skip = head_skip + tail_skip
        if skip <= 0:
            return line_pixels, depths, surface_points, 0

        total = len(line_pixels)
        if total <= skip:
            empty_pixels = np.asarray(line_pixels)[:0]
            return empty_pixels, [], [], min(skip, total)

        keep_until = total - tail_skip if tail_skip > 0 else total
        return (
            np.asarray(line_pixels)[head_skip:keep_until],
            list(depths)[head_skip:keep_until],
            list(surface_points)[head_skip:keep_until],
            skip,
        )

    def _canonicalize_back_analysis(self, analysis):
        if analysis is None:
            return analysis
        if self.massage_target != "back":
            return analysis

        updated = dict(analysis)
        if updated.get("back_lines_canonical"):
            meta = updated.get("back_line_meta") or {}
            if meta:
                self.back_line_meta = dict(meta)
            return updated

        spine_line, meridian_lines, outer_meridian_lines, meta = _canonical_back_lines_from_spine(
            updated.get("spine_line"),
            updated.get("meridian_lines"),
            outer_scale=2.0,
        )
        if meridian_lines is None or outer_meridian_lines is None:
            return updated

        updated["spine_line"] = spine_line
        updated["meridian_lines"] = meridian_lines
        updated["outer_meridian_lines"] = outer_meridian_lines
        updated["back_line_meta"] = meta or {}
        updated["back_lines_canonical"] = True
        self.back_line_meta = dict(updated["back_line_meta"])
        return updated

    def _analyze_visual_frame(self, img):
        return self._canonicalize_back_analysis(super()._analyze_visual_frame(img))

    def _locked_analysis(self):
        analysis = super()._locked_analysis()
        if self.massage_target != "back":
            return analysis
        analysis = dict(analysis)
        analysis["back_lines_canonical"] = True
        analysis["back_line_meta"] = dict(getattr(self, "back_line_meta", {}) or {})
        return analysis

    def _attach_back_depth_samples(self, analysis, depth_frame, require_depth=True):
        if self.massage_target != "back" or analysis is None:
            return analysis

        updated = self._canonicalize_back_analysis(analysis)
        outer_lines = updated.get("outer_meridian_lines")
        line_meta = updated.get("back_line_meta") or {}
        line_length_px = float(line_meta.get("line_length_px") or 0.0)
        geometry_valid = line_length_px >= BACK_MIN_LINE_LENGTH_PX
        updated["back_geometry_valid"] = geometry_valid
        updated["back_geometry_reason"] = (
            None
            if geometry_valid
            else (
                f"line-too-short:{line_length_px:.1f}px"
                f"<{BACK_MIN_LINE_LENGTH_PX:.1f}px"
            )
        )
        if not outer_lines:
            updated["back_sample_pixels"] = []
            updated["back_sample_depths"] = []
            updated["back_depth_valid_ratio"] = 0.0
            if require_depth:
                updated["visual_motion_ready"] = False
            return updated

        sample_line = outer_lines[0]
        sample_pixels = _sample_pixel_line(sample_line[0], sample_line[1], SAMPLE_POINTS)
        depths = []
        if depth_frame is not None:
            width = int(depth_frame.get_width())
            height = int(depth_frame.get_height())
            for u, v in sample_pixels:
                ui = int(round(float(u)))
                vi = int(round(float(v)))
                if 0 <= ui < width and 0 <= vi < height:
                    depth_m = float(depth_frame.get_distance(ui, vi))
                    depths.append(depth_m if depth_m > 0.1 else None)
                else:
                    depths.append(None)
        else:
            depths = [None] * len(sample_pixels)

        valid_ratio = sum(1 for d in depths if d is not None) / max(len(depths), 1)
        updated["back_sample_pixels"] = sample_pixels
        updated["back_sample_depths"] = depths
        updated["back_depth_valid_ratio"] = valid_ratio
        if require_depth:
            if not geometry_valid:
                updated["visual_motion_ready"] = False
                updated["visual_status"] = "geometry"
            elif valid_ratio < BACK_MIN_DEPTH_RATIO:
                updated["visual_motion_ready"] = False
                updated["visual_status"] = "depth"
        return updated

    def _draw_detection_overlay(self, img, analysis):
        if self.massage_target != "back":
            return super()._draw_detection_overlay(img, analysis)

        img_display = img.copy()
        analysis = self._canonicalize_back_analysis(analysis)
        outer_meridian_lines = analysis.get("outer_meridian_lines")
        meridian_lines = analysis.get("meridian_lines")
        visual_status = str(analysis.get("visual_status", "search"))
        visual_motion_ready = bool(analysis.get("visual_motion_ready", False))

        if outer_meridian_lines:
            for line in outer_meridian_lines:
                cv2.line(img_display, _pixel_pt(line[0]), _pixel_pt(line[1]), (255, 0, 255), 3, cv2.LINE_AA)
        if meridian_lines:
            for line in meridian_lines:
                cv2.line(img_display, _pixel_pt(line[0]), _pixel_pt(line[1]), (0, 255, 0), 2, cv2.LINE_AA)

        sample_pixels = analysis.get("back_sample_pixels")
        sample_depths = analysis.get("back_sample_depths")
        if not sample_pixels and self.massage_pixels:
            sample_pixels = self.massage_pixels
            sample_depths = [frame.get("depth_m") for frame in self.massage_frames]
        sample_pixels = sample_pixels or []
        sample_depths = sample_depths or [None] * len(sample_pixels)
        for pt, depth_m in zip(sample_pixels, sample_depths):
            if not (0 <= float(pt[0]) < img_display.shape[1] and 0 <= float(pt[1]) < img_display.shape[0]):
                continue
            color = (0, 255, 0) if depth_m is not None else (0, 0, 255)
            cv2.circle(img_display, _pixel_pt(pt), 4, color, -1, cv2.LINE_AA)

        meta = analysis.get("back_line_meta") or {}
        valid_ratio = analysis.get("back_depth_valid_ratio", None)
        depth_text = "-" if valid_ratio is None else f"{float(valid_ratio) * 100:.0f}%"
        line_text = "-"
        if meta:
            line_text = (
                f"len={float(meta.get('line_length_px', 0.0)):.0f}px "
                f"inner={float(meta.get('inner_offset_px', 0.0)):.0f}px "
                f"outer={float(meta.get('outer_offset_px', 0.0)):.0f}px "
                f"trimN={float(meta.get('neck_trim_px', 0.0)):.0f}px"
            )
        status = "ready" if visual_motion_ready else visual_status
        draw_thigh_text_box(
            img_display,
            [
                f"BACK detect status={status} lines=4",
                f"depth valid={depth_text} samples={len(sample_pixels)} {line_text}",
            ],
        )
        return img_display

    def _annotate_back_depth_diagnostics(self):
        if self.detector is None or self.stable_depth_frame is None:
            return
        matrix = getattr(self.detector, "camera_to_robot", None)
        for frame in self.massage_frames:
            pixel = frame.get("pixel")
            if pixel is None:
                continue
            u, v = float(pixel[0]), float(pixel[1])
            stats = _depth_patch_stats_from_realsense(self.stable_depth_frame, u, v)
            p3 = self.detector.get_point3d_from_depth(u, v, self.stable_depth_frame)
            if p3 is not None:
                frame["point_cam_m"] = [float(x) for x in p3.tolist()]
                frame["depth_m"] = float(p3[2])
                if matrix is not None:
                    robot_m = self.detector.transform_points_to_robot([p3.tolist()])
                    if robot_m:
                        robot_mm = np.asarray(robot_m[0], dtype=np.float64) * 1000.0
                        point_mm = np.asarray(frame.get("point_mm", robot_mm.tolist()), dtype=np.float64)
                        frame["robot_reprojection_error_mm"] = float(np.linalg.norm(robot_mm - point_mm))
            if stats is not None:
                frame["depth_patch_stats"] = stats
        self._print_trajectory_depth_report("背部膀胱经")

    def _save_trajectory_debug_image(self, path):
        if self.locked_color_frame is None:
            return None
        img = self._draw_detection_overlay(self.locked_color_frame.copy(), self._locked_analysis())
        for frame in self.massage_frames:
            idx = int(frame.get("index", 0)) + 1
            pixel = frame.get("pixel")
            if pixel is None:
                continue
            x = int(round(float(pixel[0])))
            y = int(round(float(pixel[1])))
            color = (0, 255, 255)
            cv2.circle(img, (x, y), 7, color, -1, cv2.LINE_AA)
            label = str(idx)
            depth_m = frame.get("depth_m")
            if depth_m is not None:
                label = f"{idx}:{float(depth_m):.2f}m"
            cv2.putText(
                img,
                label,
                (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        image_path = path.with_suffix(".png")
        if cv2.imwrite(str(image_path), img):
            return str(image_path)
        return None

    def _contact_pose_from_frame(self, frame, split_offset_mm=0.0):
        return self._pose_from_frame_offset(
            frame,
            FORCE_CONTACT_OFFSET_MM,
            split_offset_mm=split_offset_mm,
        )

    def _frame_with_tool_z_unit(self, frame, tool_z_unit, normal_source_suffix=None):
        tool_z = _normalize_vec(tool_z_unit)
        if tool_z is None:
            return dict(frame)
        updated = dict(frame)
        updated["tool_z_unit"] = tool_z.tolist()
        split_axis = _project_axis_to_tool_plane(frame.get("split_axis_unit"), tool_z)
        if split_axis is None:
            split_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        updated["split_axis_unit"] = split_axis.tolist()
        updated["base_pose"] = tuple(
            float(v)
            for v in _rpy_from_tool_z_vector(tool_z, INIT_POSE_P24[5], INIT_POSE_P24[3])
        )
        if normal_source_suffix:
            source = str(frame.get("normal_source", "unknown"))
            if normal_source_suffix not in source:
                source = f"{source}{normal_source_suffix}"
            updated["normal_source"] = source
        return updated

    def _candidate_leg_frames_by_reachability(self, frame):
        desired_tool_z = np.asarray(frame["tool_z_unit"], dtype=np.float64)
        reference_tool_z = np.asarray(
            _tool_z_unit_from_rpy(INIT_POSE_P24[3], INIT_POSE_P24[4], INIT_POSE_P24[5]),
            dtype=np.float64,
        )
        raw_angle_deg = _angle_between_unit_vectors(desired_tool_z, reference_tool_z)
        step_deg = max(1.0, abs(float(THIGH_REACHABILITY_STEP_DEG)))
        min_tilt_deg = max(0.0, float(THIGH_REACHABILITY_MIN_TILT_DEG))

        yielded = set()
        angle = raw_angle_deg
        while angle >= min_tilt_deg - 1e-6:
            key = round(max(min_tilt_deg, angle), 3)
            if key not in yielded:
                yielded.add(key)
                if angle >= raw_angle_deg - 1e-6:
                    yield raw_angle_deg, dict(frame)
                else:
                    limited_tool_z, _, _ = _limit_unit_vector_to_cone(
                        desired_tool_z,
                        reference_tool_z,
                        max(min_tilt_deg, angle),
                    )
                    yield max(min_tilt_deg, angle), self._frame_with_tool_z_unit(
                        frame,
                        limited_tool_z,
                        normal_source_suffix="+reachable",
                    )
            angle -= step_deg

        if 0.0 not in yielded:
            yield 0.0, self._frame_with_tool_z_unit(
                frame,
                reference_tool_z,
                normal_source_suffix="+reachable",
            )

    def _pose_ik_ok(self, pose):
        try:
            config = -1
            if self.massage_target == "leg_inner" and self.inner_posture_seed is not None:
                config = int(self.inner_posture_seed["ik_config"])
            elif self.massage_target == "back" and self.back_posture_seed is not None:
                config = int(self.back_posture_seed["ik_config"])
            return self.robot.inverse_kin_ok(pose, config=config)
        except Exception as exc:
            print(f"逆解探针异常: {exc}")
            return False

    def _leg_frame_reachability_ok(self, frame):
        hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
        if not self._pose_ik_ok(hover_pose):
            return False
        contact_pose = self._pose_from_frame_offset(frame, FORCE_CONTACT_OFFSET_MM)
        return self._pose_ik_ok(contact_pose)

    def _adjust_leg_frames_for_reachability(self):
        if not _is_thigh_target(self.massage_target) or not THIGH_AUTO_REACHABLE_ORIENTATION:
            return True
        if not self.massage_frames:
            return True

        adjusted_frames = []
        adjusted_count = 0
        failed_indices = []
        selected_angles = []

        print("[Reach] 大腿姿态可达性检查：用控制器逆解筛选最接近局部法向的姿态")
        for frame in self.massage_frames:
            index = int(frame.get("index", len(adjusted_frames)))
            selected = None
            selected_angle = None
            for angle_deg, candidate in self._candidate_leg_frames_by_reachability(frame):
                if self._leg_frame_reachability_ok(candidate):
                    selected = candidate
                    selected_angle = angle_deg
                    break
            if selected is None:
                failed_indices.append(index)
                if not FT_CONTINUE_ON_POINT_ERROR:
                    adjusted_frames.append(frame)
                continue
            original_angle = _angle_between_unit_vectors(
                frame["tool_z_unit"],
                _tool_z_unit_from_rpy(INIT_POSE_P24[3], INIT_POSE_P24[4], INIT_POSE_P24[5]),
            )
            selected_angles.append(float(selected_angle))
            if selected_angle < original_angle - 1e-3:
                adjusted_count += 1
                print(
                    f"[Reach] 点{index}: 原始法向角 {original_angle:.1f}deg 不可达，"
                    f"采用 {selected_angle:.1f}deg"
                )
            adjusted_frames.append(selected)

        if failed_indices:
            print(f"[Reach] 以下点未找到可达姿态: {failed_indices}")
            if not FT_CONTINUE_ON_POINT_ERROR:
                return False
            print("[Reach] 单点失败容错已开启，跳过这些点继续后续流程")
            if not adjusted_frames:
                print("[Reach] 没有剩余可达点，无法执行动作")
                return False

        self.massage_frames = adjusted_frames
        self.massage_points_mm = [frame["point_mm"] for frame in adjusted_frames]
        if selected_angles:
            print(
                f"[Reach] 大腿姿态可达性检查完成：调整 {adjusted_count}/{len(adjusted_frames)} 点，"
                f"selected_angle_range={min(selected_angles):.1f}-{max(selected_angles):.1f}deg"
            )
        return True

    def _back_frame_reachability_ok(self, frame):
        """Validate the full force/contact envelope for one back point."""
        max_offset = max(
            float(self.force_approach_max_offset_mm),
            float(FORCE_CONTACT_OFFSET_MM),
        )
        lateral_mm = abs(float(FORCE_FEN_LATERAL_MM))
        if DIAN_JIN_MODE in {"small_fen", "small-fen", "small_split", "split", "fen"}:
            lateral_mm = max(lateral_mm, abs(float(DIAN_AS_SMALL_FEN_LATERAL_MM)))

        probes = [
            (-float(self.hover_height_mm), 0.0),
            (max_offset, 0.0),
        ]
        if lateral_mm > 0.0:
            probes.extend(
                [
                    (max_offset, lateral_mm),
                    (max_offset, -lateral_mm),
                ]
            )
        return all(
            self._pose_ik_ok(
                self._pose_from_frame_offset(
                    frame,
                    offset_mm,
                    split_offset_mm=lateral_offset_mm,
                )
            )
            for offset_mm, lateral_offset_mm in probes
        )

    def _adjust_back_frames_for_reachability(self):
        """Drop points whose complete fixed-posture force envelope is unreachable."""
        if (
            self.massage_target != "back"
            or not BACK_POSTURE_SEED_ENABLE
            or self.back_posture_seed is None
            or not self.massage_frames
        ):
            return True

        reachable_frames = []
        failed_points = []
        print(
            "[BackReach] 背部固定构型可达性检查："
            "验证悬空位、最大贴近位和分筋横向包络"
        )
        for fallback_index, frame in enumerate(self.massage_frames):
            point_no = int(frame.get("index", fallback_index)) + 1
            if self._back_frame_reachability_ok(frame):
                reachable_frames.append(frame)
            else:
                failed_points.append(point_no)

        if failed_points:
            print(f"[BackReach] 完整动作包络不可达点: {failed_points}")
            if not FT_CONTINUE_ON_POINT_ERROR:
                return False
            print("[BackReach] 单点失败容错已开启，执行前跳过这些点")
        if not reachable_frames:
            print("[BackReach] 没有剩余完整动作包络可达的背部点")
            return False

        self.massage_frames = reachable_frames
        self.massage_points_mm = [frame["point_mm"] for frame in reachable_frames]
        print(
            f"[BackReach] 检查完成：保留 {len(reachable_frames)}/"
            f"{len(reachable_frames) + len(failed_points)} 点"
        )
        return True

    def _apply_motion_orientation(self, pose):
        fixed_orientation = self._fixed_motion_orientation()
        if fixed_orientation is not None:
            pose[3], pose[4], pose[5] = fixed_orientation
        return pose

    def _pose_from_frame_offset(self, frame, offset_mm, split_offset_mm=0.0):
        point_mm = np.asarray(frame["point_mm"], dtype=np.float64)
        contact_axis_unit = np.asarray(frame["tool_z_unit"], dtype=np.float64)
        split_axis_unit = np.asarray(frame["split_axis_unit"], dtype=np.float64)
        fixed_orientation = self._fixed_motion_orientation()
        if fixed_orientation is not None:
            # FT_SetRCS(0) reports force in the active tool frame. When posture is
            # frozen, move along that same tool Z axis so commanded penetration and
            # measured Fz retain the same physical meaning.
            contact_axis_unit = np.asarray(
                _tool_z_unit_from_rpy(*fixed_orientation),
                dtype=np.float64,
            )
            projected_split = _project_axis_to_tool_plane(
                split_axis_unit,
                contact_axis_unit,
            )
            if projected_split is not None:
                split_axis_unit = projected_split
        tcp_offset_mm = float(offset_mm) - float(TOOL_TIP_LENGTH_MM)
        pos = point_mm + contact_axis_unit * tcp_offset_mm + split_axis_unit * float(split_offset_mm)
        rx, ry, rz = frame["base_pose"]
        pose = [float(pos[0]), float(pos[1]), float(pos[2]), float(rx), float(ry), float(rz)]
        return self._apply_motion_orientation(pose)

    def _move_cart_force_checked(self, pose, context, vel=MOVE_VEL_SLOW):
        ret = self.robot.MoveCart(
            desc_pos=pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=vel,
            blendT=BLEND_BLOCKING,
        )
        if ret != 0:
            print(f"    警告：{context}失败 (err={ret})")
            return False
        if self.force_controller is not None and self.force_controller.active:
                self.force_controller.check_limits(context)
        return True

    def _current_pose_distance(self, pose):
        try:
            fresh_pose = getattr(self.robot, "get_fresh_actual_tcp_pose", None)
            if callable(fresh_pose):
                current = fresh_pose()
            else:
                current = self.robot.get_actual_tcp_pose()
        except Exception:
            return None, None
        return _pose_distance(current, pose)

    def _current_pose_close_to(
        self,
        pose,
        pos_tol=ROS2_MOTION_DONE_POSE_TOL_MM,
        ori_tol=ROS2_MOTION_DONE_ORI_TOL_DEG,
    ):
        pos_dist, ori_dist = self._current_pose_distance(pose)
        if pos_dist is None or ori_dist is None:
            return False, pos_dist, ori_dist
        return (
            pos_dist <= float(pos_tol) and ori_dist <= float(ori_tol),
            pos_dist,
            ori_dist,
        )

    def _move_force_pose_checked(
        self,
        pose,
        context,
        vel=MOVE_VEL_SLOW,
        close_success=False,
        close_pos_tol=ROS2_MOTION_DONE_POSE_TOL_MM,
        close_ori_tol=ROS2_MOTION_DONE_ORI_TOL_DEG,
    ):
        if close_success:
            close, pos_dist, ori_dist = self._current_pose_close_to(
                pose,
                close_pos_tol,
                close_ori_tol,
            )
            if close:
                print(
                    f"[Force] {context}: 当前已接近目标，跳过重复移动 "
                    f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)"
                )
                if self.force_controller is not None:
                    self.force_controller.check_limits(context)
                return True

        try:
            ret = self.robot.MoveCartInterpolated(
                desc_pos=pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=vel,
            )
        except Exception as exc:
            print(
                f"[Force] {context}: {ROS2_SERVO_INTERPOLATION_HZ:g}Hz "
                f"插值不可用 ({exc})，回退到原 MoveCart"
            )
            ret = None
        if ret not in {None, 0}:
            print(
                f"[Force] {context}: {ROS2_SERVO_INTERPOLATION_HZ:g}Hz "
                f"插值运动返回 err={ret}，"
                "回退到原 MoveCart"
            )
        if ret is None or ret != 0:
            ret = self.robot.MoveCart(
                desc_pos=pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=vel,
                blendT=BLEND_BLOCKING,
            )
        if ret != 0:
            if close_success:
                close, pos_dist, ori_dist = self._current_pose_close_to(
                    pose,
                    close_pos_tol,
                    close_ori_tol,
                )
                if close:
                    print(
                        f"[Force] {context}: MoveCart 返回 err={ret}，"
                        f"但当前已接近目标，按成功处理 "
                        f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)"
                    )
                    if self.force_controller is not None:
                        self.force_controller.check_limits(context)
                    return True
            print(f"    警告：{context}失败 (err={ret})")
            return False
        if self.force_controller is not None:
            self.force_controller.check_limits(context)
        return True

    def _force_axis_value(self, data):
        return FORCE_AXIS_SIGN * float(data[2])

    def _check_shun_tangential_force(self, context):
        if self.force_controller is None:
            return 0.0, False
        data = self.force_controller.check_limits(context)
        tangent_n = math.hypot(float(data[0]), float(data[1]))
        release_requested = (
            FORCE_SHUN_TANGENTIAL_RELEASE_N > 0.0
            and tangent_n >= FORCE_SHUN_TANGENTIAL_RELEASE_N
        )
        if release_requested:
            print(
                f"[Force] {context}: 顺筋切向力达到卸力阈值 "
                f"Ft={tangent_n:.2f}N >= {FORCE_SHUN_TANGENTIAL_RELEASE_N:.2f}N"
            )
        elif FORCE_SHUN_TANGENTIAL_WARN_N > 0.0 and tangent_n >= FORCE_SHUN_TANGENTIAL_WARN_N:
            print(
                f"[Force] {context}: 顺筋切向力预警 "
                f"Ft={tangent_n:.2f}N >= {FORCE_SHUN_TANGENTIAL_WARN_N:.2f}N"
            )
        return tangent_n, release_requested

    def _release_shun_contact(self, frame, contact_offset_mm, context):
        lift_mm = max(0.0, float(FORCE_SHUN_RELEASE_LIFT_MM))
        hover_offset = -float(self.hover_height_mm)
        release_offset = max(
            hover_offset,
            float(contact_offset_mm) - lift_mm,
        )
        release_pose = self._pose_from_frame_offset(frame, release_offset)
        if not self._move_force_pose_checked(
            release_pose,
            f"{context} 卸力 {lift_mm:.1f}mm",
            MOVE_VEL_SLOW,
        ):
            return release_offset, False
        if FORCE_SHUN_RELEASE_DWELL_S > 0.0:
            time.sleep(FORCE_SHUN_RELEASE_DWELL_S)
        self.force_controller.check_limits(f"{context} 卸力后检查")
        print(
            f"[Force] {context}: 已抬升 "
            f"{float(contact_offset_mm) - release_offset:.1f}mm，"
            f"等待 {FORCE_SHUN_RELEASE_DWELL_S:.2f}s"
        )
        return release_offset, True

    def _recontact_shun_after_release(self, frame, release_offset_mm, context):
        return self._approach_to_target_force(
            frame,
            f"{context} 重新贴合",
            start_offset_mm=release_offset_mm,
        )

    def _read_force_axis(self, context):
        if self.force_controller is None:
            raise RuntimeError("力传感器未初始化")
        data = self.force_controller.check_limits(context)
        return self._force_axis_value(data), data

    def _wait_force_released(self, context):
        if self.force_controller is None:
            return True

        deadline = time.time() + max(0.0, FORCE_RELEASE_TIMEOUT_S)
        last_print = 0.0
        last_data = None
        while True:
            data = self.force_controller.check_limits(context)
            last_data = data
            max_force = max(abs(data[0]), abs(data[1]), abs(data[2]))
            press_n = self._force_axis_value(data)
            if max_force <= FORCE_RELEASE_LIMIT_N:
                print(
                    f"[Force] {context}: 已卸力 "
                    f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N, "
                    f"press={press_n:.2f}N"
                )
                return True

            now = time.time()
            if now - last_print > 0.4:
                print(
                    f"[Force] {context}: 等待卸力 "
                    f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N, "
                    f"press={press_n:.2f}N"
                )
                last_print = now

            if now >= deadline:
                break
            time.sleep(self.force_controller._monitor_period)

        data = last_data or [0.0] * 6
        print(
            f"[Force] {context}: 卸力超时 "
            f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N, "
            f"limit={FORCE_RELEASE_LIMIT_N:.1f}N"
        )
        return False

    def _move_to_hover_for_force(self, frame, context):
        hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
        if not self._move_force_pose_checked(
            hover_pose,
            context,
            MOVE_VEL_SLOW,
            close_success=True,
        ):
            return False
        if not self._wait_force_released(f"{context} 卸力检查"):
            force_n, data = self._read_force_axis(f"{context} 力检查")
            raise RuntimeError(
                f"{context}: 悬空位已有过大接触力，"
                f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N"
            )
        force_n, data = self._read_force_axis(f"{context} 力检查")
        print(
            f"[Force] {context}: 悬空位读数 "
            f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N, "
            f"press={force_n:.2f}N"
        )
        if max(abs(data[0]), abs(data[1]), abs(data[2])) > FORCE_PRESTART_LIMIT_N:
            raise RuntimeError(
                f"{context}: 悬空位已有过大接触力，"
                f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N"
            )
        return True

    def _approach_to_target_force(self, frame, context, split_offset_mm=0.0, start_offset_mm=None):
        if start_offset_mm is None:
            offset = -float(self.hover_height_mm)
        else:
            offset = float(start_offset_mm)
        max_offset = max(float(self.force_approach_max_offset_mm), float(FORCE_CONTACT_OFFSET_MM))
        step = max(0.05, abs(float(FORCE_APPROACH_STEP_MM)))
        contact_step = min(step, max(0.05, abs(float(FORCE_APPROACH_CONTACT_STEP_MM))))
        fine_step = min(step, max(0.05, abs(float(FORCE_APPROACH_FINE_STEP_MM))))
        near_step = min(fine_step, max(0.05, abs(float(FORCE_APPROACH_NEAR_STEP_MM))))
        contact_threshold_n = max(0.1, abs(float(FORCE_APPROACH_CONTACT_N)))
        approach_vel = abs(float(FORCE_APPROACH_VEL))
        contact_vel = max(
            1.0,
            min(approach_vel, abs(float(FORCE_APPROACH_CONTACT_VEL))),
        )
        fine_vel = max(1.0, min(approach_vel, abs(float(FORCE_APPROACH_FINE_VEL))))
        near_vel = max(1.0, min(fine_vel, abs(float(FORCE_APPROACH_NEAR_VEL))))
        fine_ratio = max(0.0, min(1.0, float(FORCE_APPROACH_FINE_RATIO)))
        near_ratio = max(fine_ratio, min(1.0, float(FORCE_APPROACH_NEAR_RATIO)))
        last_print = 0.0

        start_text = "从悬空位沿法向贴近" if start_offset_mm is None else "沿法向补偿贴近"
        print(
            f"[Force] {context}: {start_text} "
            f"offset {offset:+.1f}mm -> {max_offset:+.1f}mm, "
            f"target={self._current_force_target_n():.1f}N"
        )
        precontact_clearance = max(0.0, float(FORCE_APPROACH_PRECONTACT_CLEARANCE_MM))
        if precontact_clearance > 0.0:
            precontact_offset = min(max_offset, -precontact_clearance)
            if precontact_offset > offset:
                force_n, data = self._read_force_axis(f"{context} 预贴近检查")
                if force_n >= contact_threshold_n:
                    print(
                        f"[Force] {context}: 悬空段已检测到接触力 "
                        f"press={force_n:.2f}N，跳过连续预贴近"
                    )
                elif max(abs(data[0]), abs(data[1]), abs(data[2])) > FORCE_PRESTART_LIMIT_N:
                    raise RuntimeError(
                        f"{context}: 预贴近前力读数过大，"
                        f"Fx={data[0]:.2f} Fy={data[1]:.2f} Fz={data[2]:.2f}N"
                    )
                else:
                    pose = self._pose_from_frame_offset(frame, precontact_offset, split_offset_mm)
                    print(
                        f"[Force] {context}: 连续预贴近 "
                        f"{offset:+.1f}mm -> {precontact_offset:+.1f}mm "
                        f"(clearance={precontact_clearance:.1f}mm)"
                    )
                    if not self._move_force_pose_checked(
                        pose,
                        f"{context} 连续预贴近",
                        abs(float(FORCE_APPROACH_PRECONTACT_VEL)),
                    ):
                        return offset, False
                    offset = precontact_offset
                    time.sleep(max(0.03, float(FORCE_APPROACH_SETTLE_S)))

        if FORCE_CONTINUOUS_APPROACH:
            target_pose = self._pose_from_frame_offset(frame, max_offset, split_offset_mm)
            result = self.force_controller.continuous_approach(
                target_pose,
                context,
            )
            planned_travel_mm = max(0.0, max_offset - offset)
            travel_mm = max(0.0, min(planned_travel_mm, float(result["travel_mm"])))
            offset = min(max_offset, offset + travel_mm)
            data = result["force"]
            force_n = self._force_axis_value(data)
            if int(result["status"]) == 1:
                print(
                    f"[Force] {context}: 连续贴近达到目标力 "
                    f"press={force_n:.2f}N, offset={offset:+.1f}mm, "
                    f"travel={travel_mm:.1f}mm, elapsed={result['elapsed_s']:.2f}s"
                )
                return offset, True
            print(
                f"[Force] {context}: 连续贴近已到最大 offset={offset:+.1f}mm，"
                f"仍未达到目标力 press={force_n:.2f}N, "
                f"elapsed={result['elapsed_s']:.2f}s"
            )
            return offset, False

        while True:
            target_n = self._current_force_target_n()
            force_n, data = self._read_force_axis(context)
            now = time.time()
            if now - last_print > 0.4:
                print(
                    f"[Force] {context}: offset={offset:+.1f}mm "
                    f"Fz={data[2]:.2f}N press={force_n:.2f}N target={target_n:.1f}N"
                )
                last_print = now

            if force_n >= target_n:
                print(
                    f"[Force] {context}: 达到目标力 "
                    f"press={force_n:.2f}N, Fz={data[2]:.2f}N, offset={offset:+.1f}mm"
                )
                return offset, True

            if offset >= max_offset:
                print(
                    f"[Force] {context}: 已到最大贴近 offset={offset:+.1f}mm，"
                    f"仍未达到目标力 press={force_n:.2f}N"
                )
                return offset, False

            move_step = step
            move_vel = approach_vel
            settle_s = float(FORCE_APPROACH_SETTLE_S)
            if force_n >= contact_threshold_n:
                move_step = contact_step
                move_vel = contact_vel
                settle_s = max(settle_s, 0.10)
            if target_n > 1e-6 and force_n >= target_n * near_ratio:
                move_step = near_step
                move_vel = min(move_vel, near_vel)
                settle_s = max(settle_s, 0.08)
            elif target_n > 1e-6 and force_n >= target_n * fine_ratio:
                move_step = min(move_step, fine_step)
                move_vel = min(move_vel, fine_vel)
                settle_s = max(settle_s, 0.06)

            offset = min(max_offset, offset + move_step)
            pose = self._pose_from_frame_offset(frame, offset, split_offset_mm)
            if not self._move_force_pose_checked(pose, f"{context} 贴近", move_vel):
                return offset, False
            time.sleep(settle_s)

    def _hold_target_force(self, frame, split_offset_mm, start_offset_mm, seconds, context):
        offset = float(start_offset_mm)
        max_offset = max(float(self.force_approach_max_offset_mm), float(FORCE_CONTACT_OFFSET_MM))
        min_offset = -float(self.hover_height_mm)
        deadline = time.time() + max(0.0, float(seconds))
        last_print = 0.0

        while time.time() < deadline:
            target_n = self._current_force_target_n()
            force_n, data = self._read_force_axis(context)
            err_n = target_n - force_n
            if abs(err_n) > FORCE_TARGET_TOL_N:
                delta = FORCE_HOLD_KP_MM_PER_N * err_n
                delta = max(-FORCE_HOLD_MAX_STEP_MM, min(FORCE_HOLD_MAX_STEP_MM, delta))
                next_offset = max(min_offset, min(max_offset, offset + delta))
                if abs(next_offset - offset) > 1e-4:
                    offset = next_offset
                    pose = self._pose_from_frame_offset(frame, offset, split_offset_mm)
                    if not self._move_force_pose_checked(pose, f"{context} 恒力微调"):
                        return offset, False

            now = time.time()
            if now - last_print > 0.4:
                print(
                    f"[Force] {context}: offset={offset:+.1f}mm "
                    f"Fz={data[2]:.2f}N press={force_n:.2f}N target={target_n:.1f}N"
                )
                last_print = now
            time.sleep(self.force_controller._monitor_period)

        return offset, True

    def _retract_to_hover(self, frame, context):
        hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
        if not self._move_force_pose_checked(
            hover_pose,
            context,
            MOVE_VEL_SLOW,
            close_success=True,
        ):
            return False
        return self._wait_force_released(f"{context} 卸力检查")

    def _fmt_pose(self, pose):
        return "[" + ", ".join(_fmt_value(v) for v in pose) + "]"

    def _moveit_joint_fallback_allowed(self, context):
        if not MOVEIT_JOINT_FALLBACK:
            return False
        if self._inner_posture_active or self._inner_posture_transitioning:
            # A generic MoveIt solution is allowed to choose another shoulder/
            # elbow/wrist branch.  Never use it while the body-clearance posture
            # is being entered, held, or exited.
            return False
        text = str(context)
        safe_keywords = (
            "安全高度",
            "高位平移",
            "移动到起始位置",
            "回到起点",
            "返回安全位置",
            "移动到悬空位",
        )
        return any(keyword in text for keyword in safe_keywords)

    def _moveit_movej_to_pose(self, pose, context, vel=TRANSIT_MOVE_VEL_FAST):
        if not self._moveit_joint_fallback_allowed(context):
            return False
        joints_deg = self.robot.moveit_inverse_kin_joints_deg(pose)
        if joints_deg is None:
            return False
        joint_text = ", ".join(_fmt_value(v) for v in joints_deg)
        print(f"{context}: MoveL 失败，使用 MoveIt IK + MoveJ 兜底 joints=[{joint_text}]")
        ret = self.robot.MoveJ(
            joints_deg,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=vel,
            blendT=BLEND_BLOCKING,
        )
        if ret == 0:
            return True
        print(f"{context}: MoveIt MoveJ 兜底失败 (err={ret})")
        return False

    def _joint_target_error_deg(self, target_joints_deg):
        current = self.robot.get_actual_joint_positions_deg()
        return max(
            abs(_shortest_angle_delta_deg(current[index], target_joints_deg[index]))
            for index in range(6)
        )

    def _enter_inner_posture_seed(self):
        if self.massage_target != "leg_inner" or not THIGH_INNER_POSTURE_SEED_ENABLE:
            return True
        if self._inner_posture_active:
            return True
        if self.inner_posture_seed is None:
            print("错误：大腿内侧模式未加载图2构型种子")
            return False

        seed_pose = list(self.inner_posture_seed["pose"])
        seed_joints = list(self.inner_posture_seed["joint_deg"])
        config = int(self.inner_posture_seed["ik_config"])
        print(
            f"[InnerPosture] 先在当前分支移动到高位切换点，再切换到图2构型 "
            f"config={config}"
        )

        self._inner_previous_motion_orientation = (
            None if self.motion_orientation is None else list(self.motion_orientation)
        )
        self._inner_posture_transitioning = True
        self.robot.default_ik_config = -1
        try:
            with _RobotMotionSpeedScaleOverride(1.0):
                reached_staging_pose = self._move_to_work_pose(
                    seed_pose,
                    "大腿内侧图2构型切换位",
                    THIGH_INNER_POSTURE_SWITCH_VEL,
                )

                # Remember the automatically selected return branch at exactly
                # the same Cartesian staging pose.  Exiting can then switch back
                # without guessing the home configuration number.
                self._inner_posture_return_joints_deg = (
                    self.robot.get_actual_joint_positions_deg()
                )
                if not reached_staging_pose:
                    current_pose = self.robot.get_actual_tcp_pose()
                    near_joint_error = max(
                        abs(
                            _shortest_angle_delta_deg(
                                self._inner_posture_return_joints_deg[index],
                                seed_joints[index],
                            )
                        )
                        for index in range(6)
                    )
                    if (
                        float(current_pose[2]) < THIGH_INNER_POSTURE_LIFT_Z_MM
                        or near_joint_error
                        > THIGH_INNER_POSTURE_NEAR_JOINT_FALLBACK_DEG
                    ):
                        print(
                            "错误：图2切换位 MoveL 失败，且不满足高位近关节兜底条件 "
                            f"(z={current_pose[2]:.1f}mm, "
                            f"joint_delta={near_joint_error:.2f}deg)"
                        )
                        self._inner_posture_return_joints_deg = None
                        return False
                    print(
                        "[InnerPosture] 图2切换位 MoveL 未完成，但已处于高位且接近种子；"
                        "允许低速 MoveJ 受限兜底 "
                        f"(z={current_pose[2]:.1f}mm, "
                        f"joint_delta={near_joint_error:.2f}deg <= "
                        f"{THIGH_INNER_POSTURE_NEAR_JOINT_FALLBACK_DEG:.1f}deg)"
                    )
                ret = self.robot.MoveJ(
                    seed_joints,
                    tool=ROS2_TOOL,
                    user=ROS2_USER,
                    vel=THIGH_INNER_POSTURE_SWITCH_VEL,
                    blendT=BLEND_BLOCKING,
                )
            if ret != 0:
                print(f"错误：切换图2构型 MoveJ 失败 (err={ret})")
                return False

            # From this point onward every ordinary MoveL endpoint is solved on
            # the taught branch.  A configured IK failure stops instead of
            # silently returning to the colliding posture.
            self.robot.default_ik_config = config
            self._inner_posture_active = True
            self.motion_orientation = list(seed_pose[3:6])
            joint_error = self._joint_target_error_deg(seed_joints)
            close, pos_dist, ori_dist = self._current_pose_close_to(
                seed_pose,
                pos_tol=5.0,
                ori_tol=5.0,
            )
            if joint_error > THIGH_INNER_POSTURE_JOINT_TOL_DEG or not close:
                print(
                    "错误：图2构型切换后的反馈不符合种子 "
                    f"(joint={joint_error:.2f}deg, pos={pos_dist}, ori={ori_dist})"
                )
                return False
            print(
                f"[InnerPosture] 图2构型已锁定: config={config}, "
                f"joint_error={joint_error:.2f}deg"
            )
            return True
        finally:
            self._inner_posture_transitioning = False

    def _activate_back_posture_seed(self):
        if self.massage_target != "back" or not BACK_POSTURE_SEED_ENABLE:
            return True
        if self.back_posture_seed is None:
            print("错误：背部模式未加载姿态种子")
            return False

        config = int(self.back_posture_seed["ik_config"])
        current_pose = self.robot.get_actual_tcp_pose()
        current_joints = self.robot.get_actual_joint_positions_deg()
        ret, config_joints = self.robot.inverse_kin_joints_deg(
            current_pose,
            config=config,
        )
        if ret != 0 or config_joints is None:
            print(f"错误：当前位姿无法按背部种子 config={config} 逆解 (err={ret})")
            return False
        config_error = max(
            abs(_shortest_angle_delta_deg(current_joints[index], config_joints[index]))
            for index in range(6)
        )
        if config_error > BACK_POSTURE_CONFIG_TOL_DEG:
            print(
                f"错误：当前机械臂不是背部种子要求的 config={config} 分支 "
                f"(joint_error={config_error:.2f}deg > "
                f"{BACK_POSTURE_CONFIG_TOL_DEG:.2f}deg)"
            )
            return False

        self.robot.default_ik_config = config
        self._back_posture_active = True
        print(
            f"[BackPosture] 已锁定当前构型 config={config}, "
            f"joint_error={config_error:.2f}deg"
        )
        return True

    def _back_probe_pose_path(self, hover_poses):
        if not hover_poses:
            return []
        current = self.robot.get_actual_tcp_pose()
        first = list(hover_poses[0])
        transit_z = max(
            float(current[2]),
            float(first[2]) + ROS2_TRANSIT_MARGIN_MM,
            ROS2_LIFT_SAFE_Z_MM,
        )
        transit = [first[0], first[1], transit_z, first[3], first[4], first[5]]
        path = []
        if ROS2_TRANSIT_LIFT_FIRST and float(current[2]) < transit_z - ROS2_TRANSIT_LIFT_TOL_MM:
            lift = [current[0], current[1], transit_z, current[3], current[4], current[5]]
            path.extend(_linear_transit_waypoints(current, lift, ROS2_SEGMENT_MAX_STEP_MM))
            current = lift
        path.extend(_linear_transit_waypoints(current, transit, ROS2_SEGMENT_MAX_STEP_MM))
        path.extend(_linear_transit_waypoints(transit, first, ROS2_SEGMENT_MAX_STEP_MM))
        for previous, target in zip(hover_poses, hover_poses[1:]):
            path.extend(_linear_transit_waypoints(previous, target, ROS2_SEGMENT_MAX_STEP_MM))
        return path

    def _execute_back_hover_probe(self):
        if self.massage_target != "back" or not BACK_TRAJECTORY_PROBE_ONLY:
            return False
        if not self.massage_frames:
            print("错误：背部悬空探针没有已保存轨迹")
            return False

        hover_poses = [
            self._pose_from_frame_offset(frame, -BACK_TRAJECTORY_PROBE_CLEARANCE_MM)
            for frame in self.massage_frames
        ]
        probe_path = self._back_probe_pose_path(hover_poses)
        failed_steps = [index for index, pose in enumerate(probe_path, start=1) if not self._pose_ik_ok(pose)]
        if failed_steps:
            print(f"错误：背部悬空探针分段逆解失败: {failed_steps}")
            return False
        print(
            f"[BackProbe] 全路径逆解通过: frames={len(hover_poses)}, "
            f"segments={len(probe_path)}, clearance={BACK_TRAJECTORY_PROBE_CLEARANCE_MM:.1f}mm"
        )

        with _RobotMotionSpeedScaleOverride(1.0):
            if not self._move_to_work_pose(
                hover_poses[0],
                "背部悬空探针进入首点",
                BACK_TRAJECTORY_PROBE_VEL,
            ):
                return False
            for index, pose in enumerate(hover_poses[1:], start=2):
                if not self._move_pose_segmented(
                    pose,
                    f"背部悬空探针点 {index}/{len(hover_poses)}",
                    vel=BACK_TRAJECTORY_PROBE_VEL,
                ):
                    return False
        print(
            "[BackProbe] 背部全轨迹悬空探针完成；"
            "未接触、未启动力控，停在最后一个悬空点"
        )
        return True

    def _leave_inner_posture_seed(self):
        if not getattr(self, "_inner_posture_active", False):
            return True
        if self.inner_posture_seed is None or self._inner_posture_return_joints_deg is None:
            print("错误：缺少大腿内侧构型退出所需的高位返回关节")
            return False

        seed_pose = list(self.inner_posture_seed["pose"])
        seed_joints = list(self.inner_posture_seed["joint_deg"])
        self._inner_posture_transitioning = True
        try:
            current = self.robot.get_actual_tcp_pose()
            lift_z = max(
                float(current[2]),
                float(seed_pose[2]),
                float(THIGH_INNER_POSTURE_LIFT_Z_MM),
            )
            if float(current[2]) < lift_z - ROS2_TRANSIT_LIFT_TOL_MM:
                lift_pose = list(current)
                lift_pose[2] = lift_z
                with _RobotMotionSpeedScaleOverride(1.0):
                    if not self._move_pose_segmented(
                        lift_pose,
                        "大腿内侧退出前原地抬升",
                        THIGH_INNER_POSTURE_SWITCH_VEL,
                    ):
                        return False

            print("[InnerPosture] 返回高位图2种子，再切回进入前构型")
            with _RobotMotionSpeedScaleOverride(1.0):
                ret = self.robot.MoveJ(
                    seed_joints,
                    tool=ROS2_TOOL,
                    user=ROS2_USER,
                    vel=THIGH_INNER_POSTURE_SWITCH_VEL,
                    blendT=BLEND_BLOCKING,
                )
                if ret == 0:
                    ret = self.robot.MoveJ(
                        self._inner_posture_return_joints_deg,
                        tool=ROS2_TOOL,
                        user=ROS2_USER,
                        vel=THIGH_INNER_POSTURE_SWITCH_VEL,
                        blendT=BLEND_BLOCKING,
                    )
            if ret != 0:
                print(f"错误：退出图2构型 MoveJ 失败 (err={ret})")
                return False

            self.robot.default_ik_config = -1
            self._inner_posture_active = False
            self.motion_orientation = self._inner_previous_motion_orientation
            self._inner_posture_return_joints_deg = None
            print("[InnerPosture] 已在高位切回进入前构型")
            return True
        finally:
            self._inner_posture_transitioning = False

    def _build_session_safe_pose(self):
        if ROS2_USE_LEGACY_SAFE_POSE:
            safe_pose = INIT_POSE_P24.copy()
            safe_pose[2] = ROS2_LIFT_SAFE_Z_MM
            self.motion_orientation = [safe_pose[3], safe_pose[4], safe_pose[5]]
            print(f"使用旧 P24 安全位: {self._fmt_pose(safe_pose)}")
            return safe_pose, True

        current_pose = self.robot.get_actual_tcp_pose()
        safe_pose = [float(v) for v in current_pose]
        self.motion_orientation = [safe_pose[3], safe_pose[4], safe_pose[5]]
        if safe_pose[2] < ROS2_LIFT_SAFE_Z_MM:
            safe_pose[2] = ROS2_LIFT_SAFE_Z_MM
            print(
                f"当前 TCP Z={current_pose[2]:.1f}mm，"
                f"先在当前位置竖直抬升到 Z={ROS2_LIFT_SAFE_Z_MM:.1f}mm"
            )
            return safe_pose, True

        print(
            f"当前 TCP 已在安全高度 Z={safe_pose[2]:.1f}mm，"
            "跳过旧 P24 固定安全位"
            )
        return safe_pose, False

    def _recover_robot_ready(self, context):
        print(f"{context}: 尝试恢复机械臂状态")
        for cmd in (
            "StopMotion()",
            "ResetAllError()",
            "Mode(0)",
            "RobotEnable(1)",
            f"SetSpeed({_fmt_value(_scaled_robot_motion_speed(MOVE_VEL_FAST))})",
        ):
            try:
                ret, _ = self.robot._call(cmd, raise_on_error=False)
                if ret != 0:
                    print(f"{context}: {cmd} 返回 {ret}")
            except Exception as exc:
                print(f"{context}: {cmd} 异常: {exc}")
            time.sleep(0.1)
        self.robot.wait_for_state(ROS2_STATE_WAIT_S, required=False)

    def _move_pose_segmented(self, pose, context, vel=TRANSIT_MOVE_VEL_FAST, max_step_mm=ROS2_SEGMENT_MAX_STEP_MM):
        target = [float(v) for v in pose]
        start_pose = self.robot.get_actual_tcp_pose()
        waypoints = _linear_transit_waypoints(start_pose, target, max_step_mm)
        start_time = time.time()

        # Prefer one native MoveL for the whole straight leg.  FAIRINO performs
        # its own continuous trajectory planning for that command, so there are
        # no artificial 50 mm boundaries at which Python can make it stop.  The
        # old segmented path remains a reachability/error fallback.
        if len(waypoints) > 1:
            direct_timeout_s = (
                ROS2_SEGMENT_TIMEOUT_S
                if ROS2_SEGMENT_TIMEOUT_S > 0.0
                else ROS2_MOTION_DONE_WAIT_S
            )
            print(
                f"{context}: 长距离直达 MoveL "
                f"({len(waypoints)} 个旧分段合并为一次连续运动)"
            )
            ret = self.robot.MoveCart(
                desc_pos=target,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=vel,
                blendT=BLEND_BLOCKING,
                timeout_sec=direct_timeout_s,
            )
            if ret == 0:
                return True
            close, pos_dist, ori_dist = self._current_pose_close_to(target)
            if close:
                print(
                    f"{context}: 长距离直达返回 err={ret}，但已到目标，按成功处理 "
                    f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)"
                )
                return True
            print(f"{context}: 长距离直达失败 (err={ret})，停止后改用连续分段兜底")
            self._recover_robot_ready(f"{context} 长距离直达")
            start_pose = self.robot.get_actual_tcp_pose()
            waypoints = _linear_transit_waypoints(start_pose, target, max_step_mm)

        if ROS2_SEGMENT_MAX_STEPS > 0 and len(waypoints) > ROS2_SEGMENT_MAX_STEPS:
            print(
                f"{context}: 分段移动需要 {len(waypoints)} 步，"
                f"超过最大步数 {ROS2_SEGMENT_MAX_STEPS}"
            )
            return False

        if len(waypoints) > 1:
            print(
                f"{context}: 连续分段移动 {len(waypoints)} 段，"
                "中间点平滑衔接、仅最终点等待到位"
            )

        for step, waypoint in enumerate(waypoints, start=1):
            if ROS2_SEGMENT_TIMEOUT_S > 0 and time.time() - start_time > ROS2_SEGMENT_TIMEOUT_S:
                print(f"{context}: 分段移动超过超时时间 {ROS2_SEGMENT_TIMEOUT_S:.1f}s")
                return False

            is_final = step == len(waypoints)
            # Intermediate collinear waypoints are intentionally non-blocking.
            # The controller's MoveL blend queue then keeps its velocity instead
            # of decelerating to zero every ROS service round trip.  The final
            # point remains strictly blocking and uses the normal fresh-state +
            # actual-pose completion check.
            blend_t = BLEND_BLOCKING if is_final else 0.0
            if is_final and ROS2_SEGMENT_TIMEOUT_S > 0.0:
                motion_timeout_s = max(
                    1.0,
                    ROS2_SEGMENT_TIMEOUT_S - (time.time() - start_time),
                )
            else:
                motion_timeout_s = ROS2_MOTION_DONE_WAIT_S

            ret = self.robot.MoveCart(
                desc_pos=waypoint,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=vel,
                blendT=blend_t,
                timeout_sec=motion_timeout_s,
            )
            if ret == 0:
                continue

            if ret == 14:
                self._recover_robot_ready(f"{context} 分段{step}")
                ret = self.robot.MoveCart(
                    desc_pos=waypoint,
                    tool=ROS2_TOOL,
                    user=ROS2_USER,
                    vel=vel,
                    blendT=blend_t,
                    timeout_sec=motion_timeout_s,
                )
                if ret == 0:
                    continue

            if self._moveit_movej_to_pose(waypoint, f"{context} 分段{step}", vel=vel):
                continue

            print(f"{context}: 分段移动失败 step={step}, err={ret}, target={self._fmt_pose(waypoint)}")
            return False
        return True

    def _move_cart_checked(self, pose, context, vel=TRANSIT_MOVE_VEL_FAST, required=True):
        print(f"{context}: target={self._fmt_pose(pose)}")
        ret = self.robot.MoveCart(
            desc_pos=pose,
            tool=ROS2_TOOL,
            user=ROS2_USER,
            vel=vel,
            blendT=BLEND_BLOCKING,
        )
        if ret == 0:
            return True

        close, pos_dist, ori_dist = self._current_pose_close_to(pose)
        if close:
            print(
                f"{context}: MoveCart 返回 err={ret}，但当前已接近目标，按成功处理 "
                f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)"
            )
            return True

        if ret == 14:
            self._recover_robot_ready(context)
            ret = self.robot.MoveCart(
                desc_pos=pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=vel,
                blendT=BLEND_BLOCKING,
            )
            if ret == 0:
                return True
            close, pos_dist, ori_dist = self._current_pose_close_to(pose)
            if close:
                print(
                    f"{context}: 恢复后 MoveCart 返回 err={ret}，"
                    f"但当前已接近目标，按成功处理 "
                    f"(pos={pos_dist:.2f}mm, ori={ori_dist:.2f}deg)"
                )
                return True

        if self._moveit_movej_to_pose(pose, context, vel=vel):
            return True

        try:
            current = self.robot.get_actual_tcp_pose()
            print(f"{context}: current={self._fmt_pose(current)}")
        except Exception:
            pass

        message = f"{context}失败 (err={ret})"
        if required:
            print(f"错误：{message}")
        else:
            print(f"警告：{message}")
        return False

    def _move_to_initial_safe_pose(self, safe_pose, should_move_to_safe):
        if not should_move_to_safe:
            return True

        return self._move_cart_checked(safe_pose, "移动到安全高度", TRANSIT_MOVE_VEL_FAST)

    def _move_to_work_pose(self, target_pose, context, vel=TRANSIT_MOVE_VEL_FAST):
        target = [float(v) for v in target_pose]
        current = self.robot.get_actual_tcp_pose()
        transit_z = max(
            float(current[2]),
            float(target[2]) + ROS2_TRANSIT_MARGIN_MM,
            ROS2_LIFT_SAFE_Z_MM,
        )
        transit_pose = [
            target[0],
            target[1],
            transit_z,
            target[3],
            target[4],
            target[5],
        ]
        print(f"{context}: 分段靠近，过渡Z={transit_z:.1f}mm")
        if ROS2_TRANSIT_LIFT_FIRST and float(current[2]) < transit_z - ROS2_TRANSIT_LIFT_TOL_MM:
            lift_pose = [
                current[0],
                current[1],
                transit_z,
                current[3],
                current[4],
                current[5],
            ]
            if not self._move_pose_segmented(lift_pose, f"{context} 原地抬升", vel=vel):
                return False

        if not self._move_pose_segmented(transit_pose, f"{context} 高位平移", vel=vel):
            return False
        return self._move_pose_segmented(target, f"{context} 下降", vel=vel)

    def _move_to_hover_with_fallback(self, hover_pose, context):
        if self._move_cart_checked(
            hover_pose,
            context,
            TRANSIT_MOVE_VEL_SLOW,
            required=False,
        ):
            return True
        print(f"{context}: 低位直达失败，尝试安全转场兜底")
        return self._move_to_work_pose(
            hover_pose,
            f"{context} 兜底",
            TRANSIT_MOVE_VEL_SLOW,
        )

    def init_leg_vision(self):
        print("初始化腿部视觉系统...")
        matrix = _load_camera_to_robot_matrix()
        if matrix is None:
            raise RuntimeError("无法加载 camera_to_robot.json，不能将大腿轨迹转换到机械臂坐标")
        self.camera_to_robot = np.asarray(matrix, dtype=np.float64)
        if self.camera_to_robot.shape != (4, 4):
            raise RuntimeError("camera_to_robot.json 中的矩阵不是 4x4")
        self.camera_origin_mm = _camera_origin_mm_from_matrix(self.camera_to_robot)

        device = "cuda:0" if THIGH_DEVICE == "auto" and torch.cuda.is_available() else (
            "cpu" if THIGH_DEVICE == "auto" else THIGH_DEVICE
        )
        rotations = ROTATIONS if THIGH_TRY_ROTATIONS else (THIGH_ROTATION,)
        print(f"RTMPose 配置: {DEFAULT_RTMPOSE_CONFIG}")
        print(f"RTMPose 权重: {DEFAULT_RTMPOSE_WEIGHTS}")
        print(f"RTMPose device={device}, rotations={','.join(rotations)}")
        self.thigh_rotations = rotations
        self.thigh_pose_detector = RTMPoseHipKneeDetector(
            pose2d=DEFAULT_RTMPOSE_CONFIG,
            pose2d_weights=None,
            device=device,
            side="nearest",
            kpt_thr=THIGH_KPT_THR,
            rotations=rotations,
        )
        print("腿部视觉系统初始化完成")

    def _build_leg_frames_from_capture(self, reader, depth_image, line_pixels, surface_points_camera_m):
        valid_items = []
        for pixel, point_cam in zip(line_pixels, surface_points_camera_m):
            if point_cam is None:
                continue
            u, v = float(pixel[0]), float(pixel[1])
            if not (0 <= u < THIGH_WIDTH and 0 <= v < THIGH_HEIGHT):
                continue
            valid_items.append(((u, v), [float(point_cam[0]), float(point_cam[1]), float(point_cam[2])]))

        if len(valid_items) < max(3, THIGH_SAMPLE_POINTS // 2):
            raise RuntimeError(f"腿部轨迹有效深度点不足: {len(valid_items)}/{len(line_pixels)}")

        valid_pixels = [item[0] for item in valid_items]
        points_cam = [item[1] for item in valid_items]
        points_robot_m = _transform_points(points_cam, self.camera_to_robot.tolist())
        points_robot_mm = [
            np.asarray([float(p[0] * 1000.0), float(p[1] * 1000.0), float(p[2] * 1000.0)], dtype=np.float64)
            for p in points_robot_m
        ]

        tool_z_units = []
        normal_sources = []
        depth_stats = []
        normal_ok_count = 0
        for pixel, point_robot_mm in zip(valid_pixels, points_robot_mm):
            depth_stats.append(
                _depth_patch_stats_from_image(
                    depth_image,
                    pixel[0],
                    pixel[1],
                    reader.depth_scale,
                )
            )
            normal_cam = _estimate_patch_plane_normal_camera_np(
                reader,
                depth_image,
                pixel[0],
                pixel[1],
            )
            if normal_cam is None:
                tool_z_units.append(_fallback_tool_z_axis(point_robot_mm, self.camera_origin_mm))
                normal_sources.append("fallback")
                continue

            outward_robot = _transform_vector_camera_to_robot(self.camera_to_robot, normal_cam)
            if outward_robot is None:
                tool_z_units.append(_fallback_tool_z_axis(point_robot_mm, self.camera_origin_mm))
                normal_sources.append("fallback")
                continue

            if self.camera_origin_mm is not None:
                to_camera = np.asarray(self.camera_origin_mm, dtype=np.float64) - point_robot_mm
                if float(np.dot(outward_robot, to_camera)) < 0.0:
                    outward_robot = -outward_robot

            tool_z_units.append(-outward_robot)
            normal_sources.append("plane")
            normal_ok_count += 1

        tool_z_units = _smooth_unit_vectors(tool_z_units)
        thigh_reference_tool_z = np.asarray(
            _tool_z_unit_from_rpy(INIT_POSE_P24[3], INIT_POSE_P24[4], INIT_POSE_P24[5]),
            dtype=np.float64,
        )
        normal_limited_count = 0
        max_normal_angle_deg = 0.0

        frames = []
        for idx, point_robot_mm in enumerate(points_robot_mm):
            if len(points_robot_mm) == 1:
                tangent = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            else:
                prev_pt = points_robot_mm[max(0, idx - 1)]
                next_pt = points_robot_mm[min(len(points_robot_mm) - 1, idx + 1)]
                tangent = _normalize_vec(next_pt - prev_pt)
                if tangent is None:
                    tangent = np.array([0.0, 1.0, 0.0], dtype=np.float64)

            tool_z_unit = np.asarray(tool_z_units[idx], dtype=np.float64)
            tool_z_unit_norm = _normalize_vec(tool_z_unit)
            reference_tool_z_norm = _normalize_vec(thigh_reference_tool_z)
            normal_angle_deg = 0.0
            if tool_z_unit_norm is not None and reference_tool_z_norm is not None:
                normal_angle_deg = math.degrees(
                    math.acos(
                        float(np.clip(np.dot(tool_z_unit_norm, reference_tool_z_norm), -1.0, 1.0))
                    )
                )
            max_normal_angle_deg = max(max_normal_angle_deg, float(normal_angle_deg))
            if THIGH_LOCAL_NORMAL_LIMIT_ENABLED:
                tool_z_unit, _normal_angle_deg, normal_limited = _limit_unit_vector_to_cone(
                    tool_z_unit,
                    thigh_reference_tool_z,
                    THIGH_LOCAL_NORMAL_MAX_TILT_DEG,
                )
                if normal_limited:
                    normal_limited_count += 1
                    normal_sources[idx] = f"{normal_sources[idx]}+limited"
            split_axis_unit = _build_split_axis(tool_z_unit, tangent)
            if frames and float(np.dot(split_axis_unit, np.asarray(frames[-1]["split_axis_unit"], dtype=np.float64))) < 0.0:
                split_axis_unit = -split_axis_unit

            base_pose = tuple(
                float(v)
                for v in _rpy_from_tool_z_vector(tool_z_unit, INIT_POSE_P24[5], INIT_POSE_P24[3])
            )
            frames.append(
                {
                    "index": idx,
                    "pixel": valid_pixels[idx],
                    "point_cam_m": points_cam[idx],
                    "depth_m": float(points_cam[idx][2]),
                    "depth_patch_stats": depth_stats[idx],
                    "normal_source": normal_sources[idx],
                    "point_mm": point_robot_mm.tolist(),
                    "tool_z_unit": tool_z_unit.tolist(),
                    "split_axis_unit": split_axis_unit.tolist(),
                    "base_pose": base_pose,
                }
            )

        self.massage_pixels = valid_pixels
        self.massage_frames = frames
        self.massage_points_mm = [frame["point_mm"] for frame in frames]

        raw_count = len(self.massage_points_mm)

        label = self._thigh_target_label()
        post_normal_ok_count = sum(
            1 for frame in self.massage_frames if "plane" in str(frame.get("normal_source", ""))
        )
        post_normal_limited_count = sum(
            1 for frame in self.massage_frames if "limited" in str(frame.get("normal_source", ""))
        )
        print(f"{label}轨迹生成了 {len(self.massage_points_mm)} 个按摩点 (raw={raw_count})")
        print(f"局部平面法向成功估计：{post_normal_ok_count}/{len(self.massage_points_mm)}")
        if THIGH_LOCAL_NORMAL_LIMIT_ENABLED:
            print(
                f"腿部法向安全限幅：{post_normal_limited_count}/{len(self.massage_points_mm)} "
                f"limit={THIGH_LOCAL_NORMAL_MAX_TILT_DEG:.1f}deg max_raw={max_normal_angle_deg:.1f}deg"
            )
        else:
            print(f"腿部法向安全限幅：关闭 max_raw={max_normal_angle_deg:.1f}deg")
        self._print_trajectory_depth_report(label)
        return True

    def capture_thigh_trajectory(self):
        label = self._thigh_target_label()
        thigh_offset_mm = _thigh_offset_for_massage_target(self.massage_target)
        thigh_line_shift_mm = _thigh_line_shift_for_massage_target(self.massage_target)
        print(f"等待{label}检测稳定...")
        print(
            f"腿部参数: side={THIGH_SIDE}, offset={thigh_offset_mm:.1f}mm, "
            f"line_shift={thigh_line_shift_mm:.1f}mm, "
            f"direction={THIGH_DIRECTION}, stable_frames={THIGH_STABLE_FRAMES}, "
            f"min_depth_ratio={THIGH_MIN_DEPTH_RATIO:.2f}"
        )
        reader = ThighRealSenseReader(
            THIGH_WIDTH,
            THIGH_HEIGHT,
            THIGH_FPS,
            align_depth=THIGH_ALIGN_DEPTH,
        )
        stable_count = 0
        locked = None
        start = time.time()

        try:
            reader.start()
            while time.time() - start < THIGH_DETECTION_TIMEOUT_S:
                color, depth, _ = reader.get_frame()
                line_pixels = np.empty((0, 2), dtype=np.float64)
                depths = []
                surface_points = []
                valid_ratio = 0.0
                selection = detect_thigh_pose(
                    self.thigh_pose_detector,
                    color,
                    depth,
                    reader.depth_scale,
                    THIGH_SIDE,
                    THIGH_KPT_THR,
                    self.thigh_rotations,
                )
                vis = color.copy()
                key = 255

                if selection.valid and selection.keypoints is not None and selection.scores is not None:
                    outward_3d, outward_2d, direction_source = estimate_thigh_outward_direction(
                        selection,
                        depth,
                        reader,
                        THIGH_FLIP_DIRECTION,
                        direction_mode=THIGH_DIRECTION,
                    )
                    line_pixels, depths, surface_points = build_thigh_offset_line(
                        selection,
                        depth,
                        reader,
                        outward_3d,
                        outward_2d,
                        thigh_offset_mm,
                        max(2, THIGH_SAMPLE_POINTS),
                        line_shift_mm=thigh_line_shift_mm,
                    )
                    line_pixels, depths, surface_points, skipped_points = self._crop_thigh_target_samples(
                        line_pixels,
                        depths,
                        surface_points,
                    )
                    valid_ratio = sum(1 for d in depths if d is not None) / max(len(depths), 1)
                    draw_thigh_polyline(vis, line_pixels, (255, 0, 255), 4)
                    for pt, depth_m in zip(line_pixels, depths):
                        if not (0 <= pt[0] < vis.shape[1] and 0 <= pt[1] < vis.shape[0]):
                            continue
                        color_dot = (0, 255, 0) if depth_m is not None else (0, 0, 255)
                        cv2.circle(vis, tuple(np.round(pt).astype(int)), 3, color_dot, -1, cv2.LINE_AA)

                    stable_count = stable_count + 1 if valid_ratio >= THIGH_MIN_DEPTH_RATIO else 0
                    draw_thigh_text_box(
                        vis,
                        [
                            f"{label}: side={selection.side} offset={thigh_offset_mm:.1f}mm line_shift={thigh_line_shift_mm:.1f}mm direction={THIGH_DIRECTION}",
                            f"depth valid={valid_ratio * 100:.0f}% stable={stable_count}/{THIGH_STABLE_FRAMES} dir={direction_source} skip={skipped_points}",
                            "s lock now | q quit",
                        ],
                    )
                    if stable_count >= THIGH_STABLE_FRAMES:
                        locked = (color.copy(), depth.copy() if depth is not None else None, selection, line_pixels, depths, surface_points, direction_source)
                        print(f"{label}检测稳定！")
                        break
                else:
                    stable_count = 0
                    draw_thigh_text_box(
                        vis,
                        [
                            f"{label} detection: {selection.reason}",
                            "调整人体/相机位置后等待稳定，q退出",
                        ],
                    )

                if THIGH_DISPLAY:
                    cv2.imshow("Thigh Detection", vis)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        return False
                    if key == ord("s") and selection.valid and len(line_pixels) > 0:
                        locked = (color.copy(), depth.copy() if depth is not None else None, selection, line_pixels, depths, surface_points, direction_source)
                        print(f"手动锁定当前{label}")
                        break

            if locked is None:
                print(f"{label}检测超时")
                return False

            color, depth, selection, line_pixels, depths, surface_points, direction_source = locked
            self.locked_color_frame = color.copy()
            self.outer_meridian_lines = (
                (
                    (float(line_pixels[0][0]), float(line_pixels[0][1])),
                    (float(line_pixels[-1][0]), float(line_pixels[-1][1])),
                ),
            )
            self.meridian_lines = None
            self.spine_line = None
            self._set_preview_tracking_state(
                spine_line=None,
                meridian_lines=None,
                outer_meridian_lines=self.outer_meridian_lines,
                tracking_label="LEG_LOCKED",
            )

            saved_path = save_thigh_confirmation(
                selection,
                reader,
                thigh_offset_mm,
                THIGH_FLIP_DIRECTION,
                direction_source,
                line_pixels,
                depths,
                surface_points,
                line_shift_mm=thigh_line_shift_mm,
            )
            print(f"已保存腿部检测结果: {saved_path}")
            return self._build_leg_frames_from_capture(reader, depth, line_pixels, surface_points)

        finally:
            reader.stop()
            if THIGH_DISPLAY:
                try:
                    cv2.destroyWindow("Thigh Detection")
                except Exception:
                    pass

    def run_leg_live_preview_until_motion_done(self):
        reader = ThighRealSenseReader(
            THIGH_WIDTH,
            THIGH_HEIGHT,
            THIGH_FPS,
            align_depth=THIGH_ALIGN_DEPTH,
        )
        fallback_frame = self.locked_color_frame.copy() if self.locked_color_frame is not None else None
        done_status_set = False
        try:
            reader.start()
            while True:
                try:
                    color, _, _ = reader.get_frame()
                    img = color
                except Exception:
                    if fallback_frame is None:
                        continue
                    img = fallback_frame.copy()

                motion_alive = self.motion_thread is not None and self.motion_thread.is_alive()
                if not motion_alive and not done_status_set:
                    if self.motion_error is not None:
                        self.update_preview_status("动作异常，按q退出")
                    elif self.motion_success:
                        self.update_preview_status("动作完成，按q退出")
                    else:
                        self.update_preview_status("动作结束，按q退出")
                    done_status_set = True

                detection = self._draw_detection_overlay(img, self._locked_analysis())
                preview = self._draw_preview_overlay(img)
                cv2.imshow("Detection", detection)
                cv2.imshow(LIVE_PREVIEW_WINDOW_NAME, preview)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                time.sleep(0.03)
        finally:
            reader.stop()
            try:
                cv2.destroyWindow("Detection")
            except Exception:
                pass
            try:
                cv2.destroyWindow(LIVE_PREVIEW_WINDOW_NAME)
            except Exception:
                pass

    def _save_locked_trajectory(self, label, extra=None):
        FT_TRAJECTORY_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        path = FT_TRAJECTORY_OUTPUT_DIR / f"{label}_trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        data = {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "target": label,
            "hover_height_mm": float(self.hover_height_mm),
            "force_target_n": float(self.force_target_n),
            "tool_tip_length_mm": float(TOOL_TIP_LENGTH_MM),
            "pixels": self.massage_pixels,
            "points_mm": self.massage_points_mm,
            "frames": self.massage_frames,
        }
        if extra:
            data.update(extra)
        debug_image = self._save_trajectory_debug_image(path)
        if debug_image:
            data["debug_image"] = debug_image
        with path.open("w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        print(f"[轨迹] 已保存: {path}")
        if debug_image:
            print(f"[轨迹] 调试图已保存: {debug_image}")
        return path

    def _draw_command_banner(self, img, lines):
        if not lines:
            return img
        out = img
        height, width = out.shape[:2]
        line_h = 24
        box_h = line_h * len(lines) + 12
        y0 = max(0, height - box_h - 8)
        overlay = out.copy()
        cv2.rectangle(overlay, (8, y0), (width - 8, height - 8), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.48, out, 0.52, 0, out)
        for idx, line in enumerate(lines):
            cv2.putText(
                out,
                str(line),
                (16, y0 + 24 + idx * line_h),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        return out

    def _start_motion_thread_once(self, name):
        if not self.massage_frames:
            print("请先按 s 保存轨迹，再按 g 执行动作")
            return False
        if self.motion_thread is not None and self.motion_thread.is_alive():
            print("机械臂动作已经在执行中")
            return False
        self.motion_success = False
        self.motion_error = None
        self.update_preview_status("连接机械臂")
        self.motion_thread = threading.Thread(
            target=self._motion_worker,
            name=name,
            daemon=True,
        )
        self.motion_thread.start()
        print("[动作] 已启动机械臂按摩动作")
        return True

    def _update_finished_motion_status(self, done_status_set):
        if self.motion_thread is None or self.motion_thread.is_alive() or done_status_set:
            return done_status_set
        if self.motion_error is not None:
            self.update_preview_status("动作异常，按q退出")
            print(f"[动作] 执行异常: {self.motion_error}")
        elif self.motion_success:
            self.update_preview_status("动作完成，按q退出")
            print("[动作] 执行完成")
        else:
            self.update_preview_status("动作结束，按q退出")
            print("[动作] 执行结束")
        return True

    def run_back_interactive(self):
        print("背部模式：实时检测膀胱经；按 s 保存轨迹，按 g 执行动作，按 q 退出。")
        self.init_vision()
        locked = False
        motion_started = False
        done_status_set = False
        saved_path = None

        try:
            while True:
                frames = self.detector.pipeline.wait_for_frames()
                frames = self.detector.align.process(frames)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue

                img = np.asanyarray(color_frame.get_data())
                self.frame_idx += 1

                if locked:
                    analysis = self._locked_analysis()
                    analysis = self._attach_back_depth_samples(analysis, depth_frame, require_depth=False)
                    tracking_label = "LOCKED"
                else:
                    analysis = self._analyze_visual_frame(img)
                    analysis = self._attach_back_depth_samples(analysis, depth_frame, require_depth=True)
                    tracking_label = (
                        "READY" if analysis["visual_motion_ready"]
                        else str(analysis["visual_status"]).upper()
                    )
                    self._set_preview_tracking_state(
                        spine_line=analysis["spine_line"],
                        meridian_lines=analysis["meridian_lines"],
                        outer_meridian_lines=analysis["outer_meridian_lines"],
                        tracking_label=tracking_label,
                    )

                detection = self._draw_detection_overlay(img, analysis)
                status = "saved" if locked else ("ready" if analysis["visual_motion_ready"] else "searching")
                self._draw_command_banner(
                    detection,
                    [
                        f"BACK | status={status} | track={tracking_label}",
                        "s save trajectory | g start robot | q quit",
                        f"saved={saved_path.name if saved_path else '-'}",
                    ],
                )
                cv2.imshow("Detection", detection)
                if locked:
                    cv2.imshow(LIVE_PREVIEW_WINDOW_NAME, self._draw_preview_overlay(img))

                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    if not analysis["visual_motion_ready"]:
                        print("当前背部检测还未稳定，暂不保存；请等状态变 READY 后再按 s")
                        continue
                    try:
                        depth_frame.keep()
                    except Exception:
                        pass
                    self.stable_depth_frame = depth_frame
                    self.locked_color_frame = img.copy()
                    self.spine_line = analysis["spine_line"]
                    self.meridian_lines = analysis["meridian_lines"]
                    self.outer_meridian_lines = analysis["outer_meridian_lines"]
                    self._set_preview_tracking_state(
                        spine_line=self.spine_line,
                        meridian_lines=self.meridian_lines,
                        outer_meridian_lines=self.outer_meridian_lines,
                        tracking_label="LOCKED",
                    )
                    if not self.capture_trajectory():
                        print("背部轨迹生成失败，请重新检测后再按 s")
                        continue
                    self._annotate_back_depth_diagnostics()
                    saved_path = self._save_locked_trajectory(
                        "back",
                        extra={"trajectory_type": "bladder_meridian"},
                    )
                    locked = True
                    self.update_preview_status("轨迹已保存，按g执行", 0)
                elif key == ord("g"):
                    if not locked:
                        print("请先按 s 保存轨迹，再按 g 执行动作")
                        continue
                    motion_started = self._start_motion_thread_once("ft_back_motion_worker") or motion_started
                elif key == ord("q"):
                    if self.motion_thread is not None and self.motion_thread.is_alive():
                        print("机械臂动作执行中，等待完成；如需急停请使用实体急停或另行停止运动")
                    else:
                        break

                done_status_set = self._update_finished_motion_status(done_status_set)
                time.sleep(0.02)

        finally:
            try:
                cv2.destroyWindow("Detection")
            except Exception:
                pass
            try:
                cv2.destroyWindow(LIVE_PREVIEW_WINDOW_NAME)
            except Exception:
                pass

        if motion_started:
            if self.motion_thread is not None:
                self.motion_thread.join()
                self.motion_thread = None
            if self.motion_error is not None:
                raise self.motion_error
            return bool(self.motion_success)
        print("未启动机械臂动作，已退出")
        return True

    def run_leg_interactive(self):
        label = self._thigh_target_label()
        thigh_offset_mm = _thigh_offset_for_massage_target(self.massage_target)
        thigh_line_shift_mm = _thigh_line_shift_for_massage_target(self.massage_target)
        print(f"腿部模式：实时检测{label}；按 s 保存轨迹，按 g 执行动作，按 q 退出。")
        if self.massage_target == "leg_inner":
            print(f"大腿内侧模式复用外侧中线检测实现，画面和保存轨迹都会去掉前 {THIGH_INNER_SKIP_POINTS} 个检测点。")
        self.init_leg_vision()
        reader = ThighRealSenseReader(
            THIGH_WIDTH,
            THIGH_HEIGHT,
            THIGH_FPS,
            align_depth=THIGH_ALIGN_DEPTH,
        )
        locked = False
        motion_started = False
        done_status_set = False
        saved_path = None
        stable_count = 0

        try:
            reader.start()
            while True:
                color, depth, _ = reader.get_frame()
                vis = color.copy()
                line_pixels = np.empty((0, 2), dtype=np.float32)
                depths = []
                surface_points = []
                selection = None
                valid_ratio = 0.0
                direction_source = "none"

                if locked:
                    detection = self._draw_detection_overlay(color, self._locked_analysis())
                    preview = self._draw_preview_overlay(color)
                    self._draw_command_banner(
                        detection,
                        [
                            f"{label} | status=saved",
                            "g start robot | q quit",
                            f"saved={saved_path.name if saved_path else '-'}",
                        ],
                    )
                    cv2.imshow("Detection", detection)
                    cv2.imshow(LIVE_PREVIEW_WINDOW_NAME, preview)
                else:
                    selection = detect_thigh_pose(
                        self.thigh_pose_detector,
                        color,
                        depth,
                        reader.depth_scale,
                        THIGH_SIDE,
                        THIGH_KPT_THR,
                        self.thigh_rotations,
                    )
                    if selection.valid and selection.keypoints is not None and selection.scores is not None:
                        outward_3d, outward_2d, direction_source = estimate_thigh_outward_direction(
                            selection,
                            depth,
                            reader,
                            THIGH_FLIP_DIRECTION,
                            direction_mode=THIGH_DIRECTION,
                        )
                        line_pixels, depths, surface_points = build_thigh_offset_line(
                            selection,
                            depth,
                            reader,
                            outward_3d,
                            outward_2d,
                            thigh_offset_mm,
                            max(2, THIGH_SAMPLE_POINTS),
                            line_shift_mm=thigh_line_shift_mm,
                        )
                        line_pixels, depths, surface_points, skipped_points = self._crop_thigh_target_samples(
                            line_pixels,
                            depths,
                            surface_points,
                        )
                        valid_ratio = sum(1 for d in depths if d is not None) / max(len(depths), 1)
                        stable_count = stable_count + 1 if valid_ratio >= THIGH_MIN_DEPTH_RATIO else 0
                        draw_thigh_polyline(vis, line_pixels, (255, 0, 255), 4)
                        for pt, depth_m in zip(line_pixels, depths):
                            if not (0 <= pt[0] < vis.shape[1] and 0 <= pt[1] < vis.shape[0]):
                                continue
                            dot_color = (0, 255, 0) if depth_m is not None else (0, 0, 255)
                            cv2.circle(vis, tuple(np.round(pt).astype(int)), 3, dot_color, -1, cv2.LINE_AA)
                        draw_thigh_text_box(
                            vis,
                            [
                                f"{label} detect side={selection.side} offset={thigh_offset_mm:.1f}mm line_shift={thigh_line_shift_mm:.1f}mm direction={THIGH_DIRECTION}",
                                f"depth valid={valid_ratio * 100:.0f}% stable={stable_count}/{THIGH_STABLE_FRAMES} dir={direction_source} skip={skipped_points}",
                                "s save trajectory | g start robot | q quit",
                            ],
                        )
                    else:
                        stable_count = 0
                        reason = selection.reason if selection is not None else "未检测"
                        draw_thigh_text_box(
                            vis,
                            [
                                f"{label} detect: {reason}",
                                "s save trajectory after valid | q quit",
                            ],
                        )
                    cv2.imshow("Thigh Detection", vis)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    if locked:
                        print("腿部轨迹已经保存，如需重新保存请先退出后重新检测")
                        continue
                    if selection is None or not selection.valid or len(line_pixels) == 0:
                        print("当前腿部检测无效，不能保存")
                        continue
                    if valid_ratio < THIGH_MIN_DEPTH_RATIO:
                        print(
                            f"当前腿部轨迹深度有效率不足: {valid_ratio * 100:.0f}% "
                            f"< {THIGH_MIN_DEPTH_RATIO * 100:.0f}%"
                        )
                        continue

                    self.locked_color_frame = color.copy()
                    self.outer_meridian_lines = (
                        (
                            (float(line_pixels[0][0]), float(line_pixels[0][1])),
                            (float(line_pixels[-1][0]), float(line_pixels[-1][1])),
                        ),
                    )
                    self.meridian_lines = None
                    self.spine_line = None
                    self._set_preview_tracking_state(
                        spine_line=None,
                        meridian_lines=None,
                        outer_meridian_lines=self.outer_meridian_lines,
                        tracking_label="LEG_LOCKED",
                    )
                    saved_raw_path = save_thigh_confirmation(
                        selection,
                        reader,
                        thigh_offset_mm,
                        THIGH_FLIP_DIRECTION,
                        direction_source,
                        line_pixels,
                        depths,
                        surface_points,
                        line_shift_mm=thigh_line_shift_mm,
                    )
                    if not self._build_leg_frames_from_capture(reader, depth, line_pixels, surface_points):
                        print("腿部轨迹生成失败，请重新检测后再按 s")
                        continue
                    saved_path = self._save_locked_trajectory(
                        self._thigh_trajectory_type(),
                        extra={
                            "trajectory_type": self._thigh_trajectory_type(),
                            "raw_confirmation_json": str(saved_raw_path),
                            "thigh_side": THIGH_SIDE,
                            "thigh_offset_mm": float(thigh_offset_mm),
                            "thigh_line_shift_mm": float(thigh_line_shift_mm),
                            "thigh_direction": THIGH_DIRECTION,
                            "thigh_inner_skip_points": int(THIGH_INNER_SKIP_POINTS)
                            if self.massage_target == "leg_inner"
                            else 0,
                            "thigh_inner_tail_skip_points": int(THIGH_INNER_TAIL_SKIP_POINTS)
                            if self.massage_target == "leg_inner"
                            else 0,
                        },
                    )
                    locked = True
                    self.update_preview_status(f"{label}轨迹已保存，按g执行", 0)
                    try:
                        cv2.destroyWindow("Thigh Detection")
                    except Exception:
                        pass
                elif key == ord("g"):
                    if not locked:
                        print("请先按 s 保存轨迹，再按 g 执行动作")
                        continue
                    motion_started = self._start_motion_thread_once("ft_leg_motion_worker") or motion_started
                elif key == ord("q"):
                    if self.motion_thread is not None and self.motion_thread.is_alive():
                        print("机械臂动作执行中，等待完成；如需急停请使用实体急停或另行停止运动")
                    else:
                        break

                done_status_set = self._update_finished_motion_status(done_status_set)
                time.sleep(0.02)

        finally:
            reader.stop()
            for win in ("Thigh Detection", "Detection", LIVE_PREVIEW_WINDOW_NAME):
                try:
                    cv2.destroyWindow(win)
                except Exception:
                    pass

        if motion_started:
            if self.motion_thread is not None:
                self.motion_thread.join()
                self.motion_thread = None
            if self.motion_error is not None:
                raise self.motion_error
            return bool(self.motion_success)
        print("未启动机械臂动作，已退出")
        return True

    def execute_dian_jin(self, frame):
        hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
        dian_jin_pose = self._pose_from_frame_offset(
            frame,
            -(self.hover_height_mm - DIAN_JIN_DEPTH_MM),
        )
        use_small_fen = DIAN_JIN_MODE in {"small_fen", "small-fen", "small_split", "split", "fen"}
        small_fen_lateral_mm = abs(float(DIAN_AS_SMALL_FEN_LATERAL_MM))

        for repeat_idx in range(DIAN_JIN_REPEAT_COUNT):
            round_text = f"{repeat_idx + 1}/{DIAN_JIN_REPEAT_COUNT}"
            action_label = "点筋小幅分筋" if use_small_fen else "点筋"
            self.update_preview_status(f"{action_label} {round_text}", frame.get("index"))

            if LASTTIME_ROS2_FORCE:
                ok = False
                try:
                    if not self._move_to_hover_for_force(frame, f"点筋悬空位 {round_text}"):
                        return False
                    offset, reached = self._approach_to_target_force(frame, f"{action_label}中心 {round_text}")
                    if not reached:
                        return False
                    if use_small_fen:
                        offset, ok = self._hold_target_force(
                            frame,
                            0.0,
                            offset,
                            FORCE_FEN_DWELL_S,
                            f"{action_label}中心保压 {round_text}",
                        )
                        if not ok:
                            return False
                        for label, split_offset in (
                            (f"{action_label}偏移+ {round_text}", small_fen_lateral_mm),
                            (f"{action_label}偏移- {round_text}", -small_fen_lateral_mm),
                            (f"{action_label}回中心 {round_text}", 0.0),
                        ):
                            pose = self._pose_from_frame_offset(frame, offset, split_offset)
                            if not self._move_force_pose_checked(pose, label):
                                return False
                            offset, ok = self._hold_target_force(
                                frame,
                                split_offset,
                                offset,
                                FORCE_FEN_DWELL_S,
                                f"{label}保压",
                            )
                            if not ok:
                                return False
                    else:
                        offset, ok = self._hold_target_force(
                            frame,
                            0.0,
                            offset,
                            FORCE_DIAN_DWELL_S,
                            f"点筋保压 {round_text}",
                        )
                except Exception as exc:
                    print(f"    警告：{action_label}{round_text}力控失败 ({exc})")
                    return False
                finally:
                    if not self._retract_to_hover(frame, f"点筋结束回悬空位 {round_text}"):
                        ok = False
                if not ok:
                    return False
                continue

            if use_small_fen:
                for label, split_offset in (
                    (f"点筋小幅分筋偏移+ {round_text}", small_fen_lateral_mm),
                    (f"点筋小幅分筋偏移- {round_text}", -small_fen_lateral_mm),
                    (f"点筋小幅分筋回中心 {round_text}", 0.0),
                ):
                    pose = self._pose_from_frame_offset(
                        frame,
                        -(self.hover_height_mm - DIAN_JIN_DEPTH_MM),
                        split_offset_mm=split_offset,
                    )
                    ret = self.robot.MoveCart(
                        desc_pos=pose,
                        tool=ROS2_TOOL,
                        user=ROS2_USER,
                        vel=MOVE_VEL_SLOW,
                        blendT=BLEND_BLOCKING,
                    )
                    if ret != 0:
                        print(f"    警告：{label}失败 (err={ret})")
                        return False
                    time.sleep(0.2)
                ret = self.robot.MoveCart(
                    desc_pos=hover_pose,
                    tool=ROS2_TOOL,
                    user=ROS2_USER,
                    vel=MOVE_VEL_SLOW,
                    blendT=BLEND_BLOCKING,
                )
                if ret != 0:
                    print(f"    警告：点筋小幅分筋{round_text}回到悬空位失败 (err={ret})")
                    return False
                continue

            ret = self.robot.MoveCart(
                desc_pos=dian_jin_pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=MOVE_VEL_SLOW,
                blendT=BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：点筋{round_text}失败 (err={ret})")
                return False
            time.sleep(0.3)

            ret = self.robot.MoveCart(
                desc_pos=hover_pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=MOVE_VEL_SLOW,
                blendT=BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：点筋{round_text}回到悬空位失败 (err={ret})")
                return False
        return True

    def execute_fen_jin(self, frame):
        self.update_preview_status("分筋", frame.get("index"))
        hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
        center_pose = self._contact_pose_from_frame(frame)
        positive_pose = self._contact_pose_from_frame(frame, FEN_JIN_LATERAL_MM)
        negative_pose = self._contact_pose_from_frame(frame, -FEN_JIN_LATERAL_MM)

        if LASTTIME_ROS2_FORCE:
            ok = False
            try:
                if not self._move_to_hover_for_force(frame, "分筋悬空位"):
                    return False
                offset, reached = self._approach_to_target_force(frame, "分筋中心")
                if not reached:
                    return False
                offset, ok = self._hold_target_force(
                    frame,
                    0.0,
                    offset,
                    FORCE_FEN_DWELL_S,
                    "分筋中心保压",
                )
                if not ok:
                    return False

                for repeat_idx in range(FEN_JIN_REPEAT_COUNT):
                    round_text = f"{repeat_idx + 1}/{FEN_JIN_REPEAT_COUNT}"
                    for label, split_offset in (
                        (f"分筋偏移+ {round_text}", FORCE_FEN_LATERAL_MM),
                        (f"分筋偏移- {round_text}", -FORCE_FEN_LATERAL_MM),
                        (f"分筋回中心 {round_text}", 0.0),
                    ):
                        pose = self._pose_from_frame_offset(frame, offset, split_offset)
                        if not self._move_force_pose_checked(pose, label):
                            return False
                        offset, ok = self._hold_target_force(
                            frame,
                            split_offset,
                            offset,
                            FORCE_FEN_DWELL_S,
                            f"{label}保压",
                        )
                        if not ok:
                            return False
            except Exception as exc:
                print(f"    警告：分筋力控失败 ({exc})")
                return False
            finally:
                if not self._retract_to_hover(frame, "分筋结束回悬空位"):
                    ok = False
            if not ok:
                return False
            return True

        positive_pose = self._pose_from_frame_offset(
            frame,
            -self.hover_height_mm,
            split_offset_mm=FEN_JIN_LATERAL_MM,
        )
        negative_pose = self._pose_from_frame_offset(
            frame,
            -self.hover_height_mm,
            split_offset_mm=-FEN_JIN_LATERAL_MM,
        )

        for repeat_idx in range(FEN_JIN_REPEAT_COUNT):
            round_text = f"{repeat_idx + 1}/{FEN_JIN_REPEAT_COUNT}"
            for label, pose in (
                (f"分筋偏移+ {round_text}", positive_pose),
                (f"分筋偏移- {round_text}", negative_pose),
                (f"分筋回悬空位 {round_text}", hover_pose),
            ):
                ret = self.robot.MoveCart(
                    desc_pos=pose,
                    tool=ROS2_TOOL,
                    user=ROS2_USER,
                    vel=MOVE_VEL_SLOW,
                    blendT=BLEND_BLOCKING,
                )
                if ret != 0:
                    print(f"    警告：{label}失败 (err={ret})")
                    return False
                time.sleep(0.2)
        return True

    def execute_shun_jin(self, frames=None):
        with _RobotMotionSpeedScaleOverride(SHUN_JIN_MOTION_SPEED_SCALE):
            return self._execute_shun_jin(frames)

    def _execute_shun_jin(self, frames=None):
        print("顺筋动作...")
        frames = list(frames) if frames is not None else list(self.massage_frames)
        if not frames:
            print("    警告：顺筋没有可用点")
            return FT_CONTINUE_ON_POINT_ERROR
        if len(frames) < max(1, FT_SHUN_MIN_POINTS):
            print(f"    警告：顺筋候选点不足 {FT_SHUN_MIN_POINTS} 个，将按可用点继续")

        if LASTTIME_ROS2_FORCE:
            last_hover_pose = None
            start_index = None
            offset = -float(self.hover_height_mm)

            for candidate_index, start_frame in enumerate(frames):
                point_no = int(start_frame.get("index", candidate_index)) + 1
                last_hover_pose = self._pose_from_frame_offset(start_frame, -self.hover_height_mm)
                try:
                    if not self._move_to_hover_for_force(start_frame, f"顺筋悬空起点 点{point_no}"):
                        print(f"    警告：顺筋起点 点{point_no} 悬空位失败")
                        if not FT_CONTINUE_ON_POINT_ERROR:
                            return False
                        continue
                    offset, reached = self._approach_to_target_force(
                        start_frame,
                        f"顺筋起点 点{point_no}",
                    )
                    if reached:
                        start_index = candidate_index
                        break
                    print(f"    警告：顺筋起点 点{point_no} 未达到目标力，尝试下一个候选点")
                    if not self._retract_to_hover(start_frame, f"顺筋起点失败回悬空位 点{point_no}"):
                        print(f"    警告：顺筋起点 点{point_no} 回悬空位失败")
                    if not FT_CONTINUE_ON_POINT_ERROR:
                        return False
                except Exception as exc:
                    print(f"    警告：顺筋起点 点{point_no} 力控失败 ({exc})")
                    return False

            if start_index is None:
                print("    警告：顺筋没有找到可贴近的起点")
                return FT_CONTINUE_ON_POINT_ERROR

            ok = True
            moved_count = 0
            skipped_points = []
            safety_abort = False
            previous_contact_frame = None
            try:
                for local_i, frame in enumerate(frames[start_index:]):
                    frame_index = start_index + local_i
                    point_no = int(frame.get("index", frame_index)) + 1
                    has_next_frame = frame_index + 1 < len(frames)
                    self.update_preview_status("顺筋", frame.get("index", point_no - 1))
                    print(f"  移动到点{point_no}...")
                    motion_frames = _subdivide_shun_edge(
                        previous_contact_frame,
                        frame,
                        FORCE_SHUN_SEGMENT_LENGTH_MM,
                    )
                    motion_failed = False
                    for substep_index, motion_frame in enumerate(motion_frames):
                        substep_no = substep_index + 1
                        substep_count = len(motion_frames)
                        last_hover_pose = self._pose_from_frame_offset(
                            motion_frame,
                            -self.hover_height_mm,
                        )
                        pose = self._pose_from_frame_offset(motion_frame, offset)
                        move_context = (
                            f"顺筋移动 点{point_no}"
                            if substep_count == 1
                            else f"顺筋移动 点{point_no} 子段{substep_no}/{substep_count}"
                        )
                        if not self._move_force_pose_checked(pose, move_context):
                            motion_failed = True
                            break

                        tangent_n, tangent_release = self._check_shun_tangential_force(
                            f"顺筋切向力检查 点{point_no} 子段{substep_no}/{substep_count}"
                        )
                        has_more_contact_motion = substep_no < substep_count or has_next_frame
                        segment_release = (
                            previous_contact_frame is not None
                            and has_more_contact_motion
                            and FORCE_SHUN_SEGMENT_LENGTH_MM > 0.0
                            and FORCE_SHUN_RELEASE_LIFT_MM > 0.0
                        )
                        if tangent_release and not has_more_contact_motion:
                            print(
                                f"    警告：顺筋末段切向力 {tangent_n:.2f}N 超限，"
                                "立即结束并返回悬空位"
                            )
                            safety_abort = True
                            motion_failed = True
                            break
                        if has_more_contact_motion and (segment_release or tangent_release):
                            if tangent_release:
                                reason = f"切向力 {tangent_n:.2f}N 超限"
                            else:
                                reason = (
                                    f"接触子段完成（上限 {FORCE_SHUN_SEGMENT_LENGTH_MM:.1f}mm）"
                                )
                            print(
                                f"  顺筋点{point_no} 子段{substep_no}/{substep_count}"
                                f"触发卸力：{reason}"
                            )
                            release_offset, release_ok = self._release_shun_contact(
                                motion_frame,
                                offset,
                                f"顺筋分段 点{point_no} 子段{substep_no}/{substep_count}",
                            )
                            if not release_ok:
                                safety_abort = True
                                motion_failed = True
                                break
                            offset, reached = self._recontact_shun_after_release(
                                motion_frame,
                                release_offset,
                                f"顺筋下一段起点 点{point_no} 子段{substep_no}/{substep_count}",
                            )
                            if not reached:
                                print(
                                    f"    警告：顺筋点{point_no} 子段{substep_no}/{substep_count}"
                                    "卸力后未重新达到目标力"
                                )
                                safety_abort = True
                                motion_failed = True
                                break

                    if motion_failed:
                        skipped_points.append(point_no)
                        ok = False
                        if safety_abort or not FT_CONTINUE_ON_POINT_ERROR:
                            break
                        print(f"    警告：顺筋点{point_no}移动失败，跳过该点继续")
                        continue
                    if FORCE_SHUN_RECONTACT:
                        offset, reached = self._approach_to_target_force(
                            frame,
                            f"顺筋贴近补偿 点{point_no}",
                            start_offset_mm=offset,
                        )
                        if not reached:
                            skipped_points.append(point_no)
                            ok = False
                            if not FT_CONTINUE_ON_POINT_ERROR:
                                break
                            print(f"    警告：顺筋点{point_no}未重新达到目标力，跳过该点继续")
                            continue
                    offset, hold_ok = self._hold_target_force(
                        frame,
                        0.0,
                        offset,
                        FORCE_SHUN_DWELL_S,
                        f"顺筋保压 点{point_no}",
                    )
                    if not hold_ok:
                        skipped_points.append(point_no)
                        ok = False
                        if not FT_CONTINUE_ON_POINT_ERROR:
                            break
                        print(f"    警告：顺筋点{point_no}保压失败，跳过该点继续")
                        continue
                    moved_count += 1
                    previous_contact_frame = frame
                if skipped_points:
                    print(f"    警告：顺筋跳过点: {skipped_points}")
                if moved_count <= 0:
                    print("    警告：顺筋没有完成任何候选点")
                    ok = False
                if FT_CONTINUE_ON_POINT_ERROR and skipped_points and not safety_abort:
                    ok = True
            except Exception as exc:
                print(f"    警告：顺筋力控失败 ({exc})")
                ok = False
            finally:
                if last_hover_pose is not None:
                    if not self._move_force_pose_checked(last_hover_pose, "顺筋结束回悬空位"):
                        if FT_CONTINUE_ON_POINT_ERROR:
                            print("    警告：顺筋结束回悬空位失败，将继续返回安全位置")
                        else:
                            ok = False
            return ok

        moved_count = 0
        skipped_points = []
        for i, frame in enumerate(frames):
            point_no = int(frame.get("index", i)) + 1
            self.update_preview_status("顺筋", frame.get("index", i))
            print(f"  移动到点{point_no}...")
            pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
            ret = self.robot.MoveCart(
                desc_pos=pose,
                tool=ROS2_TOOL,
                user=ROS2_USER,
                vel=MOVE_VEL_SLOW,
                blendT=BLEND_BLOCKING,
            )
            if ret != 0:
                print(f"    警告：顺筋点{point_no}移动失败 (err={ret})")
                skipped_points.append(point_no)
                if not FT_CONTINUE_ON_POINT_ERROR:
                    return False
                continue
            moved_count += 1
        if skipped_points:
            print(f"    警告：顺筋跳过点: {skipped_points}")
        return moved_count > 0 or FT_CONTINUE_ON_POINT_ERROR

    def execute_massage_sequence(self):
        print("\n开始执行按摩序列...")

        try:
            print("移动到安全高度...")
            self.update_preview_status("移动到安全高度")
            safe_pose, should_move_to_safe = self._build_session_safe_pose()
            if not self._move_to_initial_safe_pose(safe_pose, should_move_to_safe):
                return False

            if not self._activate_back_posture_seed():
                return False

            # A high-posture probe validates only the recorded Cartesian/joint
            # branch transition.  It must not depend on a valid camera frame or
            # a saved massage trajectory.
            if not (
                self.massage_target == "leg_inner"
                and THIGH_INNER_POSTURE_PROBE_ONLY
            ) and not self._adjust_leg_frames_for_reachability():
                return False

            if not self._adjust_back_frames_for_reachability():
                return False

            if not self._enter_inner_posture_seed():
                return False

            if self.massage_target == "leg_inner" and THIGH_INNER_POSTURE_PROBE_ONLY:
                print(
                    "[InnerPosture] 探针模式：仅验证高位图2构型，不下降、不接触、不启动力控 "
                    f"(停留 {THIGH_INNER_POSTURE_PROBE_DWELL_S:.1f}s)"
                )
                time.sleep(THIGH_INNER_POSTURE_PROBE_DWELL_S)
                if not self._leave_inner_posture_seed():
                    return False
                print(
                    "[InnerPosture] 高位图2构型探针完成；"
                    "停在高位种子切换点，不执行额外自动返回"
                )
                return True

            if self.massage_target == "back" and BACK_TRAJECTORY_PROBE_ONLY:
                return self._execute_back_hover_probe()

            if LASTTIME_ROS2_FORCE:
                self.set_force_target_n(
                    _force_target_for_massage_action(self.massage_target, "point_actions"),
                    context="点筋/分筋目标力",
                )
                print(f"初始化 {self.force_target_n:.1f}N 恒力控制（请确认末端悬空无接触）...")
                self.init_force_controller()

            print("移动到起始位置...")
            first_frame = self.massage_frames[0]
            self.update_preview_status("移动到起始位置", first_frame.get("index", 0))
            first_pose = self._pose_from_frame_offset(first_frame, -self.hover_height_mm)
            if not self._move_to_work_pose(first_pose, "移动到起始位置", TRANSIT_MOVE_VEL_FAST):
                return False

            point_action_text = "点筋小幅分筋" if DIAN_JIN_MODE in {"small_fen", "small-fen", "small_split", "split", "fen"} else "点筋"
            print(
                f"\n执行点位动作（严格顺序：{point_action_text} → 独立分筋；"
                "完成后才允许进入顺筋）..."
            )
            point_failures = []
            shun_candidate_frames = []
            for i, frame in enumerate(self.massage_frames):
                self.update_preview_status("到达悬空位", i)
                print(f"\n处理点 {i + 1}/{len(self.massage_points_mm)}...")
                point_no = int(frame.get("index", i)) + 1
                hover_pose = self._pose_from_frame_offset(frame, -self.hover_height_mm)
                if not self._move_to_hover_with_fallback(hover_pose, "移动到悬空位"):
                    point_failures.append((point_no, "悬空位"))
                    print(f"    警告：点{point_no}悬空位不可达，跳过该点")
                    if not FT_CONTINUE_ON_POINT_ERROR:
                        return False
                    continue

                print(f"  [点{point_no}] 开始{point_action_text}...")
                if not self.execute_dian_jin(frame):
                    point_failures.append((point_no, point_action_text))
                    print(
                        f"    警告：点{point_no}{point_action_text}失败；"
                        "为保证动作顺序，本点不进入分筋和顺筋"
                    )
                    if not FT_CONTINUE_ON_POINT_ERROR:
                        return False
                    continue

                print(f"  [点{point_no}] {point_action_text}完成，开始独立分筋...")
                if not self.execute_fen_jin(frame):
                    point_failures.append((point_no, "分筋"))
                    print(
                        f"    警告：点{point_no}分筋失败；"
                        "为保证动作顺序，本点不进入顺筋"
                    )
                    if not FT_CONTINUE_ON_POINT_ERROR:
                        return False
                    continue
                print(f"  [点{point_no}] 独立分筋完成，可进入顺筋")
                shun_candidate_frames.append(frame)

            if point_failures:
                summary = ", ".join(f"点{point}:{stage}" for point, stage in point_failures)
                print(f"\n[容错] {point_action_text}/分筋阶段跳过: {summary}")

            if not shun_candidate_frames:
                print(
                    f"错误：没有任何点完整执行{point_action_text}和独立分筋；"
                    "禁止跳过分筋直接执行顺筋"
                )
                return False
            shun_frames = shun_candidate_frames

            self.set_force_target_n(
                _force_target_for_massage_action(self.massage_target, "shun_jin"),
                context="顺筋目标力",
            )
            print("\n回到起点...")
            shun_first_frame = shun_frames[0]
            shun_first_pose = self._pose_from_frame_offset(shun_first_frame, -self.hover_height_mm)
            self.update_preview_status("回到起点", shun_first_frame.get("index", 0))
            with _RobotMotionSpeedScaleOverride(SHUN_JIN_MOTION_SPEED_SCALE):
                if not self._move_to_work_pose(shun_first_pose, "回到起点", MOVE_VEL_FAST):
                    print("    警告：回到顺筋起点失败，仍将尝试顺筋")
                    if not FT_CONTINUE_ON_POINT_ERROR:
                        return False

            print("\n执行顺筋动作...")
            shun_ok = self.execute_shun_jin(shun_frames)
            if not shun_ok:
                print("    警告：顺筋阶段未完全成功，继续返回安全位置")
                if not FT_CONTINUE_ON_POINT_ERROR:
                    return False

            print("\n返回安全位置...")
            self.update_preview_status("返回安全位置")
            if not self._leave_inner_posture_seed():
                print(
                    "错误：未能在高位退出图2构型；保持当前位置，"
                    "不继续自动返回，等待人工确认"
                )
                return False
            if not self._move_to_work_pose(safe_pose, "返回安全位置", TRANSIT_MOVE_VEL_FAST):
                return False

            if point_failures or not shun_ok:
                print("\n按摩序列执行完成（存在跳过点/软失败，详见上方警告）")
            else:
                print("\n按摩序列执行完成！")
            return True

        except Exception as e:
            print(f"\n错误：{e}")
            return False
        finally:
            self.close_force_controller()
            if getattr(self, "_back_posture_active", False):
                self.robot.default_ik_config = -1
                self._back_posture_active = False
            if getattr(self, "_inner_posture_active", False):
                print(
                    "[安全] 图2构型仍处于锁定状态。异常流程不会自动执行跨构型 MoveJ；"
                    "请保持急停可用并人工确认周围空间后再恢复。"
                )

    def run(self):
        try:
            if self.massage_target == "leg_inner" and THIGH_INNER_POSTURE_PROBE_ONLY:
                print(
                    "[InnerPosture] 直接高位探针：跳过视觉检测和按摩轨迹，"
                    "仅连接机械臂并验证图2构型"
                )
                self.init_robot()
                return self.execute_massage_sequence()
            if _is_thigh_target(self.massage_target):
                return self.run_leg_interactive()
            return self.run_back_interactive()

        except KeyboardInterrupt:
            print("\n用户中断")
            return False
        except Exception as e:
            print(f"\n错误：{e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    print("=" * 60)
    print("ft.py - 恒力按摩动作演示（ROS 2 控制版）")
    print("=" * 60)
    print()
    massage_target = _select_massage_target()
    selected_hover_mm = THIGH_HOVER_HEIGHT_MM if _is_thigh_target(massage_target) else BACK_HOVER_HEIGHT_MM
    selected_approach_max_mm = (
        THIGH_FORCE_APPROACH_MAX_OFFSET_MM
        if _is_thigh_target(massage_target)
        else FORCE_APPROACH_MAX_OFFSET_MM
    )
    selected_force_target_n = float(_force_target_for_massage_target(massage_target))
    selected_thigh_offset_mm = _thigh_offset_for_massage_target(massage_target)
    selected_thigh_line_shift_mm = _thigh_line_shift_for_massage_target(massage_target)
    selected_normal_limit_n = max(abs(FORCE_SOFTWARE_NORMAL_LIMIT_N), abs(selected_force_target_n) + 30.0)
    selected_tangent_limit_n = max(abs(FORCE_SOFTWARE_TANGENTIAL_LIMIT_N), selected_normal_limit_n)
    print("配置参数：")
    print(f"  按摩部位: {_massage_target_label(massage_target)}")
    print(f"  机械臂IP: {ROBOT_IP}")
    print(f"  悬空高度: {selected_hover_mm}mm")
    print(f"  工具端补偿: 法兰/传感器中心到按摩头={TOOL_TIP_LENGTH_MM:.1f}mm")
    if DIAN_JIN_MODE in {"small_fen", "small-fen", "small_split", "split", "fen"}:
        print(f"  点筋动作: 小幅分筋替代，偏移={DIAN_AS_SMALL_FEN_LATERAL_MM:.1f}mm")
    else:
        print(f"  点筋深度: {DIAN_JIN_DEPTH_MM}mm")
    print(f"  点筋次数: {DIAN_JIN_REPEAT_COUNT}次/点")
    print(f"  分筋偏移: {FEN_JIN_LATERAL_MM}mm")
    print(f"  分筋次数: {FEN_JIN_REPEAT_COUNT}轮/点")
    print("  动作顺序: 点筋 → 独立分筋 → 顺筋（分筋未完成的点禁止进入顺筋）")
    print(f"  采样点数: {SAMPLE_POINTS}")
    print(f"  ROS2工作空间: {ROS2_WORKSPACE}")
    print(f"  ROS2控制服务: {ROS2_SERVICE_NAME}")
    print(f"  ROS2状态话题: {ROS2_STATE_TOPIC}")
    print(f"  末端姿态: {'保持当前TCP姿态' if ROS2_KEEP_CURRENT_ORIENTATION else '局部深度平面法向'}")
    print(
        f"  平面拟合: radius={PLANE_FIT_RADIUS_PX}px "
        f"step={PLANE_FIT_STEP_PX}px min_pts={PLANE_FIT_MIN_POINTS}"
    )
    if _is_thigh_target(massage_target):
        thigh_normal_limit_text = (
            f"{THIGH_LOCAL_NORMAL_MAX_TILT_DEG:.1f}deg"
            if THIGH_LOCAL_NORMAL_LIMIT_ENABLED
            else "off"
        )
        print(
            f"  腿部检测: side={THIGH_SIDE} offset={selected_thigh_offset_mm:.1f}mm "
            f"line_shift={selected_thigh_line_shift_mm:.1f}mm "
            f"direction={THIGH_DIRECTION} samples={THIGH_SAMPLE_POINTS} "
            f"stable={THIGH_STABLE_FRAMES} min_depth={THIGH_MIN_DEPTH_RATIO:.2f} "
            f"hover={THIGH_HOVER_HEIGHT_MM:.1f}mm approach_max={THIGH_FORCE_APPROACH_MAX_OFFSET_MM:.1f}mm "
            f"normal_tilt_limit={thigh_normal_limit_text} "
            f"outer_tail_skip={THIGH_OUTER_TAIL_SKIP_POINTS if massage_target == 'leg' else 0} "
            f"inner_skip={THIGH_INNER_SKIP_POINTS if massage_target == 'leg_inner' else 0} "
            f"inner_tail_skip={THIGH_INNER_TAIL_SKIP_POINTS if massage_target == 'leg_inner' else 0}"
        )
        if massage_target == "leg_inner":
            print(
                f"  大腿内侧图2构型: {'开启' if THIGH_INNER_POSTURE_SEED_ENABLE else '关闭'} "
                f"seed={THIGH_INNER_POSTURE_SEED_FILE} "
                f"seed_orientation={'on' if THIGH_INNER_USE_SEED_ORIENTATION else 'off'} "
                f"switch_vel={THIGH_INNER_POSTURE_SWITCH_VEL:.1f} "
                f"switch_min_z={THIGH_INNER_POSTURE_LIFT_Z_MM:.1f}mm "
                f"probe_only={'on' if THIGH_INNER_POSTURE_PROBE_ONLY else 'off'}"
            )
    else:
        print(
            f"  背部膀胱经线段缩短: neck={BACK_LINE_TRIM_NECK_RATIO * 100:.1f}% "
            f"tail={BACK_LINE_TRIM_TAIL_RATIO * 100:.1f}% "
            f"offset=({BACK_LINE_OFFSET_X_PX:+.1f}, {BACK_LINE_OFFSET_Y_PX:+.1f})px "
            f"horizontal_lock={'on' if BACK_LOCK_HORIZONTAL else 'off'}"
        )
        print(
            f"  背部姿态种子: {'开启' if BACK_POSTURE_SEED_ENABLE else '关闭'} "
            f"seed={BACK_POSTURE_SEED_FILE} "
            f"hover_probe={'on' if BACK_TRAJECTORY_PROBE_ONLY else 'off'} "
            f"probe_clearance={BACK_TRAJECTORY_PROBE_CLEARANCE_MM:.1f}mm "
            f"probe_vel={BACK_TRAJECTORY_PROBE_VEL:.1f}"
        )
    print(f"  演示预览窗口: {'开启（实时跟踪，仅展示）' if ENABLE_LIVE_PREVIEW_WINDOW else '关闭'}")
    print(f"  工具/工件坐标系: tool={ROS2_TOOL}, user={ROS2_USER}")
    print(
        f"  转场速度: scale={TRANSIT_SPEED_SCALE:.1f} "
        f"fast={TRANSIT_MOVE_VEL_FAST:.1f}(base={MOVE_VEL_FAST}) "
        f"slow={TRANSIT_MOVE_VEL_SLOW:.1f}(base={MOVE_VEL_SLOW}); "
        f"机械臂命令速度倍率={ROBOT_MOTION_SPEED_SCALE:.1f} "
        f"顺筋速度倍率={SHUN_JIN_MOTION_SPEED_SCALE:.1f}"
    )
    print(
        "  长距离转场: 单次原生 MoveL 连续直达；"
        f"失败时按≤{ROS2_SEGMENT_MAX_STEP_MM:.1f}mm 共线段平滑兜底"
    )
    print(
        f"  单点失败容错: {'开启' if FT_CONTINUE_ON_POINT_ERROR else '关闭'} "
        f"shun_min_points={FT_SHUN_MIN_POINTS}"
    )
    print(
        f"  顺筋表皮保护: segment={FORCE_SHUN_SEGMENT_LENGTH_MM:.1f}mm "
        f"lift={FORCE_SHUN_RELEASE_LIFT_MM:.1f}mm "
        f"release_wait={FORCE_SHUN_RELEASE_DWELL_S:.2f}s "
        f"tangent_warn={FORCE_SHUN_TANGENTIAL_WARN_N:.1f}N "
        f"tangent_release={FORCE_SHUN_TANGENTIAL_RELEASE_N:.1f}N"
    )
    print(
        f"  MoveIt IK兜底: {'开启' if MOVEIT_IK_ENABLE and MOVEIT_JOINT_FALLBACK else '关闭'} "
        f"group={MOVEIT_GROUP_NAME} link={MOVEIT_IK_LINK_NAME}"
    )
    print(
        f"  安全位策略: {'旧P24固定安全位' if ROS2_USE_LEGACY_SAFE_POSE else '当前位置竖直抬升'} "
        f"safe_z={ROS2_LIFT_SAFE_Z_MM:.1f}mm"
    )
    if LASTTIME_ROS2_FORCE:
        print(
            f"  恒力控制: 软件贴近闭环 target={selected_force_target_n:.1f}N "
            f"contact_offset={FORCE_CONTACT_OFFSET_MM:.1f}mm "
            f"normal_limit={selected_normal_limit_n:.1f}N "
            f"tangent_limit={selected_tangent_limit_n:.1f}N "
            f"prestart_limit={FORCE_PRESTART_LIMIT_N:.1f}N "
            f"approach_step={FORCE_APPROACH_STEP_MM:.2f}mm "
            f"contact={FORCE_APPROACH_CONTACT_N:.1f}N/{FORCE_APPROACH_CONTACT_STEP_MM:.2f}mm "
            f"fine={FORCE_APPROACH_FINE_STEP_MM:.2f}mm "
            f"near={FORCE_APPROACH_NEAR_STEP_MM:.2f}mm "
            f"vel_scale={FORCE_APPROACH_SPEED_SCALE:.1f} "
            f"approach_vel={FORCE_APPROACH_VEL:.1f} "
            f"fine_vel={FORCE_APPROACH_FINE_VEL:.1f} "
            f"near_vel={FORCE_APPROACH_NEAR_VEL:.1f} "
            f"max_offset={selected_approach_max_mm:.1f}mm "
            f"hold_kp={FORCE_HOLD_KP_MM_PER_N:.3f} "
            f"hold_step={FORCE_HOLD_MAX_STEP_MM:.2f}mm "
            f"release_limit={FORCE_RELEASE_LIMIT_N:.1f}N "
            f"release_timeout={FORCE_RELEASE_TIMEOUT_S:.1f}s "
            f"force_fen={FORCE_FEN_LATERAL_MM:.1f}mm "
            f"shun_recontact={'on' if FORCE_SHUN_RECONTACT else 'off'} "
            f"guard={'on' if FORCE_GUARD_ENABLE else 'off'}"
        )
        print(
            f"  连续贴近: {'on' if FORCE_CONTINUOUS_APPROACH else 'off'} "
            f"hz={FORCE_CONTINUOUS_APPROACH_HZ:g} "
            f"speed={FORCE_CONTINUOUS_COARSE_SPEED_MM_S:g}/"
            f"{FORCE_CONTINUOUS_CONTACT_SPEED_MM_S:g}/"
            f"{FORCE_CONTINUOUS_FINE_SPEED_MM_S:g}/"
            f"{FORCE_CONTINUOUS_NEAR_SPEED_MM_S:g}mm/s "
            f"accel/decel={FORCE_CONTINUOUS_ACCEL_MM_S2:g}/"
            f"{FORCE_CONTINUOUS_DECEL_MM_S2:g}mm/s^2 "
            f"stable={FORCE_CONTINUOUS_TARGET_STABLE_SAMPLES} "
            f"timeout={FORCE_CONTINUOUS_TIMEOUT_S:g}s"
        )
    else:
        print("  恒力控制: 关闭")
    print()

    demo = LastTimeRos2Demo(massage_target=massage_target)
    success = demo.run()

    if success:
        print("\n演示完成！")
        return 0

    print("\n演示失败")
    return 1


if __name__ == "__main__":
    sys.exit(main())
