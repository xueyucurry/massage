from __future__ import annotations

import os


PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(PACKAGE_DIR)
ROS_VENDOR_PYTHON_DIR = os.path.join(PROJECT_DIR, "ros_vendor", "python")
LOCAL_TRAJECTORY_CONFIG = os.path.join(PROJECT_DIR, "ros_vendor", "trajectory_generate.yaml")

DEFAULT_OUTPUT_DIR = os.path.join(PROJECT_DIR, "rm_demo_output")
DEFAULT_MODEL_PATH = os.environ.get(
    "RM_DEMO_MODEL_PATH",
    os.path.join(PROJECT_DIR, "yolo11l-pose.pt"),
)
DEFAULT_MATRIX_PATH = os.environ.get(
    "RM_DEMO_MATRIX_PATH",
    os.path.join(PROJECT_DIR, "camera_to_robot.json"),
)

DEFAULT_HOST = os.environ.get("RM_ARM_HOST", "192.168.1.18")
DEFAULT_REMOTE_SSH = os.environ.get("RM_ARM_REMOTE_SSH", "rm@192.168.1.11")
DEFAULT_REMOTE_DIR = os.environ.get("RM_DEMO_REMOTE_DIR", "/home/rm/massage/rm_demo")
DEFAULT_TRAJECTORY_CONFIG = os.environ.get(
    "RM_DEMO_TRAJECTORY_CONFIG",
    LOCAL_TRAJECTORY_CONFIG
    if os.path.isfile(LOCAL_TRAJECTORY_CONFIG)
    else "/home/rm/rm_healthcare_robot/rm_healthcare_robot_server/install/share/rm_healthcare_robot_server_launcher/config/trajectory_generate.yaml",
)

DEFAULT_SIDE = "left"
DEFAULT_FINGER_WIDTH_MM = 45.0
DEFAULT_CAPTURE_WIDTH = 640
DEFAULT_CAPTURE_HEIGHT = 480
DEFAULT_CAPTURE_FPS = 30
DEFAULT_CAPTURE_WARMUP_FRAMES = 20
DEFAULT_SAMPLE_POINTS = 30
DEFAULT_PLAN_POINTS = 6
DEFAULT_HOVER_MM = 20.0
DEFAULT_DWELL_S = 0.8
DEFAULT_SPEED = 3
DEFAULT_SAFE_LIFT_MM = 40.0
DEFAULT_TARGET_FORCE_N = 8
DEFAULT_MAX_FORCE_N = 12
DEFAULT_TOUCH_STEP_MM = 3.0
DEFAULT_MAX_PRESS_MM = 10.0
DEFAULT_POSITION_SPEED = 5
DEFAULT_CAPTURE_SETTLE_S = 1.0
DEFAULT_CONTROL_BACKEND = os.environ.get("RM_DEMO_CONTROL_BACKEND", "json")

DEFAULT_DETECTOR_BACKEND = os.environ.get("RM_DEMO_DETECTOR_BACKEND", "auto")
DEFAULT_TRANSFORM_BACKEND = os.environ.get("RM_DEMO_TRANSFORM_BACKEND", "auto")
DEFAULT_CAPTURE_POSITIONING = os.environ.get("RM_DEMO_CAPTURE_POSITIONING", "prepare")
DEFAULT_CAPTURE_PREPARE_SECTION = os.environ.get("RM_DEMO_CAPTURE_PREPARE_SECTION", "arm_massage_prepare")
DEFAULT_CAMERA_TOOL_NAME = os.environ.get("RM_DEMO_CAMERA_TOOL_NAME", "camera")
DEFAULT_RESTORE_TOOL_NAME = os.environ.get("RM_DEMO_RESTORE_TOOL_NAME", "camera")
DEFAULT_SHIFTING_NUMBER = int(os.environ.get("RM_DEMO_SHIFTING_NUMBER", "0"))
DEFAULT_INSTALL_ANG = [
    float(os.environ.get("RM_DEMO_INSTALL_ANG_X", "0")),
    float(os.environ.get("RM_DEMO_INSTALL_ANG_Y", "0")),
    float(os.environ.get("RM_DEMO_INSTALL_ANG_Z", "0")),
]

DEFAULT_CONF = 0.5
DEPTH_MEDIAN_RADIUS = 2
ESTIMATED_SHOULDER_MM = 360.0
MIN_VISUAL_OFFSET_PX = 12.0
AREA_TOP_TRIM_RATIO = 0.08
AREA_BOTTOM_TRIM_RATIO = 0.06

ROS_FORCE_POSE_CMD_TOPIC = "/rm_driver/ForcePositionMovePose_Cmd"
ROS_FORCE_STATE_TOPIC = "/rm_driver/Force_Position_State"
ROS_GET_SIX_FORCE_TOPIC = "/rm_driver/GetSixForce"
ROS_GET_SIX_FORCE_CMD_TOPIC = "/rm_driver/GetSixForce_Cmd"
ROS_MOVEJ_CMD_TOPIC = os.environ.get("RM_DEMO_ROS_MOVEJ_CMD_TOPIC", "/rm_driver/MoveJ_Cmd")
ROS_MOVEJ_P_CMD_TOPIC = os.environ.get("RM_DEMO_ROS_MOVEJ_P_CMD_TOPIC", "/rm_driver/MoveJ_P_Cmd")
ROS_MOVEL_CMD_TOPIC = os.environ.get("RM_DEMO_ROS_MOVEL_CMD_TOPIC", "/rm_driver/MoveL_Cmd")
ROS_STOP_CMD_TOPIC = os.environ.get("RM_DEMO_ROS_STOP_CMD_TOPIC", "/rm_driver/Stop_Cmd")
ROS_GET_ARM_STATE_CMD_TOPIC = os.environ.get("RM_DEMO_ROS_GET_ARM_STATE_CMD_TOPIC", "/rm_driver/GetArmState_Cmd")
ROS_ARM_CURRENT_STATE_TOPIC = os.environ.get("RM_DEMO_ROS_ARM_CURRENT_STATE_TOPIC", "/rm_driver/ArmCurrentState")
ROS_ARM_STATE_TOPIC = os.environ.get("RM_DEMO_ROS_ARM_STATE_TOPIC", "/rm_driver/ArmState")
ROS_SET_FORCE_POSITION_NEW_TOPIC = "/rm_driver/SetForcePositionNew_Cmd"
ROS_SET_FORCE_POSITION_TOPIC = "/rm_driver/SetForcePosition_Cmd"
ROS_SET_FORCE_SENSOR_TOPIC = "/rm_driver/SetForceSensor_Cmd"
ROS_START_FORCE_POSITION_MOVE_TOPIC = "/rm_driver/StartForcePositionMove_Cmd"
ROS_STOP_FORCE_POSITION_MOVE_TOPIC = "/rm_driver/StopForcePositionMove_Cmd"
