# ft.py 技术文档

## 目标和边界

`ft.py` 的目标是在保留 `lasttime.py` 已有背部视觉和轨迹生成能力的基础上，将机械臂控制切换为 FAIRINO ROS 2 service/topic，并扩展腿部大腿外侧中线和大腿内侧轨迹。它提供交互式轨迹锁定、轨迹文件落盘、力传感器初始化、软件贴近/保压闭环、安全转场、单点容错和动作状态预览。

当前项目在 `ft.py` 之上增加了 `ft_agent_api.py`、`ft_agent_process.py` 和 `py-xiaozhi` MCP 工具，用于把检测、轨迹保存和按摩动作封装成语音可调用的智能体接口。顶层 `./massage` 脚本负责启动 GUI、清理旧进程、回起始位、标定和 ROS 2 服务管理。

当前代码边界：

- 没有 argparse 命令行参数，配置全部来自环境变量和交互输入。
- 机械臂控制通过 ROS 2 `RemoteCmdInterface` 发送 FAIRINO 字符串命令。
- 视觉检测、轨迹锁定和实机动作耦合在同一个 Python 进程中。
- `Ros2ForceController.start()` 封装了 `FT_Control(1)`，但当前分筋、顺筋执行路径主要使用 `_approach_to_target_force()` 和 `_hold_target_force()` 进行软件贴近/保压微调，并未在执行序列中调用 `start()`。
- 语音智能体不直接重写 `ft.py` 逻辑，而是通过接口层调用现有检测、轨迹加载和动作执行函数。

## 运行时依赖

### Python 依赖

主要导入：

- `cv2`、`numpy`、`torch`
- `rclpy`
- `fairino_msgs.msg.RobotNonrtState`
- `fairino_msgs.srv.RemoteCmdInterface`
- 可选 MoveIt 消息：`moveit_msgs.srv.GetPositionIK`、`sensor_msgs.msg.JointState`、`geometry_msgs.msg.PoseStamped`

项目内依赖：

- `lasttime.py`：背部检测、轨迹采样、点位到姿态构建、预览状态。
- `force_control.py`：力传感器厂商/设备常量、`ForceControlConfig`、工具坐标轴函数。
- `dianjing.py`：相机到机械臂矩阵加载和点坐标批量变换。
- `thigh_outerline_confirm.py`：RealSense 读取、RTMPose 髋膝检测、大腿偏移线生成、确认文件保存。
- `rtmpose_detector.py`：RTMPose 默认配置、权重、旋转选项和推理实现。
- `ft_agent_api.py`：面向智能体的检测、加载轨迹、执行动作、暂停/恢复 checkpoint 封装。
- `ft_agent_process.py`：小智后台子进程入口，负责把 CLI 参数转成 `ft_agent_api.py` 调用，并把结果写入 JSON。
- `py-xiaozhi/src/mcp/tools/fairino_massage`：小智 MCP 工具注册和运行时状态管理。
- `py-xiaozhi/src/display/gui_display.*`：按摩机器人 GUI 展示和快捷指令入口。

### ROS 2 依赖

`ft.py` 创建一个 `lasttime_ros2_client` 节点，使用：

| 类型 | 名称 | 说明 |
| --- | --- | --- |
| Service client | `FAIRINO_REMOTE_SERVICE`，默认 `fairino_remote_command_service` | 发送 FAIRINO 命令字符串 |
| Topic subscription | `FAIRINO_STATE_TOPIC`，默认 `nonrt_state_data` | 读取 TCP 位姿、关节角、运动完成状态、力传感器数据 |
| Optional service client | `MOVEIT_IK_SERVICE`，默认 `compute_ik` | MoveIt 逆解兜底 |

推荐用 `run_lasttime_ros2.sh` 启动。该脚本负责 source ROS 2 环境、准备 FAIRINO 动态库、启动或复用 `ros2_cmd_server`，并在进入 `ft.py` 前执行探针。

## 总体架构

```text
手动流程
    ./massage start
        └── run_lasttime_ros2.sh
            ├── 启动/复用 ros2_cmd_server + MoveIt
            ├── 探针 SetSpeed + nonrt_state_data
            └── python ft.py
                ├── OpenCV 检测窗口
                ├── 按 s 锁定并保存轨迹
                └── 按 g 执行点筋/分筋/顺筋

语音流程
    ./massage gui
        ├── 清理旧小智/agent 进程，状态置为 idle
        ├── 可选回到 massage_home_pose.json 记录位
        └── py-xiaozhi GUI
            └── MCP: self.fairino_massage.*
                ├── FairinoMassageRuntime
                ├── ft_agent_state/current_session.json
                ├── ft_agent_state/current_control.json
                └── ft_agent_process.py
                    ├── detect -> ft_agent_api.detect_meridian()
                    └── execute -> ft_agent_api.execute_actions()

标定流程
    ./massage calibrate
        └── shared/calibration/calibrate_camera_to_robot_aruco.py
            └── shared/calibration/camera_to_robot.json
```

核心类：

| 类 | 职责 |
| --- | --- |
| `Ros2RobotProxy` | 用 ROS 2 service/topic 模拟旧 SDK 最小控制接口 |
| `Ros2ForceController` | 通过同一 ROS 2 command service 初始化和读取六维力传感器，执行软件力限检查 |
| `LastTimeRos2Demo` | 继承 `lasttime.LastTimeDemo`，替换机械臂控制并扩展背部/腿部交互、轨迹保存和动作执行 |
| `FTMassageAgentInterface` | 继承 `LastTimeRos2Demo`，把检测、轨迹加载、动作执行和暂停/恢复控制暴露给 agent |
| `FairinoMassageRuntime` | 小智侧会话管理器，负责状态文件、控制文件、后台子进程和自动状态播报 |

## 顶层脚本职责

`./massage` 是当前推荐入口，集中处理环境和进程生命周期：

| 命令 | 技术动作 |
| --- | --- |
| `gui` | 清理旧小智 GUI 和 FAIRINO agent 子进程；写 `current_control.json` 请求停止；把旧运行态重置为 `idle`；按配置启动 ROS 2 / MoveIt 并回记录位；启动 `py-xiaozhi/main.py --mode gui` |
| `calibrate` | 使用 `CALIBRATION_PYTHON` 运行 ArUco 标定程序，工作目录为 `shared/calibration`，并设置 `PYTHONPATH=robots/fairino:shared/vision` |
| `start` | 启动 FAIRINO ROS 2 服务和 MoveIt，探针通过后进入手动 `ft.py` |
| `stop` | 写停止状态，终止 agent 和 `ft.py` 残留，按配置回记录位，然后停止 ROS 2 / MoveIt |
| `clean` | 清理残留 ROS 2、FAIRINO、MoveIt、RealSense 和 `ft.py` 相关进程 |
| `record-home` / `home` | 通过 ROS 2 service/topic 记录或移动到 `massage_home_pose.json` |

`gui` 启动时外部状态为 `idle`，界面展示“就绪”；`stop` 和自然执行完成时外部状态为 `stopped`。这个区分用于避免重启 GUI 后误显示上一次已停止或暂停的会话。

## 小智语音控制架构

小智注册的核心工具：

| 工具 | 运行路径 |
| --- | --- |
| `self.fairino_massage.detect` | `FairinoMassageRuntime.detect()` 启动检测线程，线程用 `run_ft_agent_process_env.sh detect` 拉起检测子进程 |
| `self.fairino_massage.start` | `FairinoMassageRuntime.start()` 启动执行线程，线程用 `run_ft_agent_process_ros2.sh execute` 拉起执行子进程 |
| `self.fairino_massage.pause` | 写 `current_control.json` 的 `request=pause`，执行进程在 checkpoint 消费 |
| `self.fairino_massage.resume` / `continue` / `resume_massage` | 从状态中的 `resume_*` 字段构造执行子进程启动参数 |
| `self.fairino_massage.stop` | 写 `request=stop`，等待执行进程退出，然后调用 `./massage home` |
| `self.fairino_massage.adjust_force` | 运行中写一次性 `force_adjust.seq` 和 `delta_n`，执行进程在 checkpoint 消费；暂停期间直接更新 `force_target_n`，恢复时由 `--force-target-n` 生效；方向由 `direction` 归一化，`softer + 10` 会落成 `-10N` |
| `self.fairino_massage.status` | 从状态文件读取会话、轨迹、进度和机械臂位姿 |

检测工具默认 `display=True`，因此会保留 `ft.py` 检测画面。MCP `detect` 返回成功只表示检测任务已经启动；检测真正完成后，后台线程写 `status=detected` 和 `trajectory_path`。默认 `FAIRINO_MASSAGE_ANNOUNCE_DETECT_DONE=1` 时，运行时会向小智发起内部状态查询请求，要求小智调用 `self.fairino_massage.status` 并播报检测完成。

## 目标部位选择

入口 `main()` 调用 `_select_massage_target()`：

1. 优先读取 `MASSAGE_TARGET`。
2. 如果没有环境变量且 stdin 是交互终端，提示输入 `1/2/3`。
3. 如果没有交互输入，默认 `back`。

归一化函数 `_normalize_massage_target()` 支持中文、数字和英文别名。`leg` 和 `leg_inner` 都属于腿部目标，使用腿部视觉与腿部力控参数。

## 视觉和轨迹

### 背部轨迹

背部流程复用 `lasttime.py`：

1. `init_vision()` 初始化背部视觉系统。
2. `_analyze_visual_frame()` 获取背部中线和膀胱经线。
3. `ft.py` 通过 `_canonicalize_back_analysis()` 重新规范化线段：
   - 按 `BACK_LINE_TRIM_NECK_RATIO` 和 `BACK_LINE_TRIM_TAIL_RATIO` 缩短中线。
   - 根据内侧线估计左右方向和内侧偏移。
   - 生成内侧线和外侧线，外侧偏移默认为内侧的 2 倍。
4. `_attach_back_depth_samples()` 沿外侧线采样深度，计算 `back_depth_valid_ratio`。
5. 按 `s` 后调用继承自 `lasttime.py` 的 `capture_trajectory()`：
   - 沿左外侧线采样 `SAMPLE_POINTS` 个像素点。
   - 用稳定深度帧反投影到相机坐标。
   - 用 `camera_to_robot` 变换到机械臂基坐标，单位转为 mm。
   - 局部平面拟合法向，失败时用相机方向兜底。
   - 计算 `tool_z_unit`、`split_axis_unit` 和 `base_pose`。
6. `_annotate_back_depth_diagnostics()` 给保存帧补充深度 patch 统计、相机点和重投影误差。

### 腿部轨迹

腿部流程由 `ft.py` 自身实现：

1. `init_leg_vision()` 加载 `camera_to_robot.json`，初始化 `RTMPoseHipKneeDetector`。
2. `run_leg_interactive()` 或 `capture_thigh_trajectory()` 从 RealSense 获取 RGB/depth。
3. `detect_thigh_pose()` 检测髋膝关键点。
4. `estimate_thigh_outward_direction()` 根据 `THIGH_DIRECTION` 和 `THIGH_FLIP_DIRECTION` 确定偏移方向。
5. `build_thigh_offset_line()` 用 `THIGH_OFFSET_MM` 生成按摩线并采样。
6. 大腿内侧模式调用 `_crop_thigh_target_samples()` 跳过前 `THIGH_INNER_SKIP_POINTS` 个点。
7. `_build_leg_frames_from_capture()` 将有效深度点转换为机械臂坐标，并对每个点估计局部平面法向、分筋轴和基础 RPY。
8. 如设置 `THIGH_LOCAL_NORMAL_MAX_TILT_DEG > 0`，用 `_limit_unit_vector_to_cone()` 限制腿部局部法向相对参考姿态的最大倾角。

### 轨迹帧结构

`frames` 中每个元素的典型字段：

| 字段 | 说明 |
| --- | --- |
| `index` | 采样点序号 |
| `pixel` | 图像坐标 |
| `point_cam_m` | 相机坐标，单位 m；腿部和背部诊断可用 |
| `depth_m` | 深度值 |
| `depth_patch_stats` | 深度 patch 的中位数、范围、标准差和有效点数 |
| `normal_source` | 法向来源，例如 `plane`、`fallback`、`limited`、`reachable` |
| `point_mm` | 机械臂基坐标点，单位 mm |
| `tool_z_unit` | 工具 Z 轴方向单位向量 |
| `split_axis_unit` | 分筋侧向单位向量 |
| `base_pose` | 根据 `tool_z_unit` 推导的 RPY |

## 坐标和姿态

基础坐标链：

```text
像素 (u, v)
    -> 深度采样
    -> 相机 3D 点，单位 m
    -> camera_to_robot 4x4 变换
    -> 机械臂基坐标，单位 mm
    -> 根据局部法向计算工具姿态
```

末端位姿由 `_pose_from_frame_offset()` 生成：

```text
tcp_offset_mm = offset_mm - TOOL_TIP_LENGTH_MM
pos = point_mm + tool_z_unit * tcp_offset_mm + split_axis_unit * split_offset_mm
pose = [x, y, z, rx, ry, rz]
```

含义：

- `point_mm` 是按摩目标表面点。
- `tool_z_unit` 是工具 Z 轴方向。
- `TOOL_TIP_LENGTH_MM` 是法兰/传感器中心到按摩头的长度补偿。
- `offset_mm` 为正时更接近目标点，为负时悬空。
- `split_offset_mm` 用于分筋横向偏移。

如果 `FT_KEEP_CURRENT_ORIENTATION=1` 且 `motion_orientation` 已记录，则 `_apply_motion_orientation()` 会覆盖轨迹计算得到的 RPY，保持当前 TCP 姿态。

### 标定文件链路

标定程序通过 `./massage calibrate` 启动，默认输出：

```text
/home/massage/massage/shared/calibration/camera_to_robot.json
```

`robots/fairino/camera_to_robot.json` 是指向该共享文件的软链接。运行时加载路径由 `dianjing._load_camera_to_robot_matrix()` 处理，查找顺序包含当前工作目录、`robots/fairino/camera_to_robot.json` 和 `shared/calibration/camera_to_robot.json`。

检测阶段使用当前矩阵完成坐标转换：

- 背部流程在保存轨迹时把膀胱经采样点从相机坐标转换到机器人基坐标。
- 腿部流程在 `init_leg_vision()` 中加载矩阵，`_build_leg_frames_from_capture()` 将 RTMPose 采样线转换成机器人坐标。
- 保存后的轨迹 JSON 包含 `frames[*].point_mm`、`points_mm` 和诊断用的 `point_cam_m`。

执行阶段只读取轨迹 JSON 中已经保存的机器人坐标，不重新加载标定矩阵计算旧轨迹。因此最新标定只对重新检测生成的新轨迹生效；暂停恢复和旧轨迹执行仍使用旧轨迹内的坐标。

## 机械臂 ROS 2 控制

`Ros2RobotProxy.connect()` 流程：

1. 初始化 `rclpy`。
2. 创建 `RemoteCmdInterface` service client。
3. 创建 `RobotNonrtState` subscriber。
4. 可选创建 MoveIt IK service client。
5. 等待 service ready。
6. 等待状态话题。
7. `_ensure_robot_ready()`：
   - 如果当前不是自动模式，调用 `Mode(0)`。
   - 调用 `RobotEnable(1)`。
   - 如失败且 `ROS2_RESET_ERRORS=1`，调用 `ResetAllError()` 后重试。
   - 设置速度 `SetSpeed(MOVE_VEL_FAST)`。

命令发送统一经 `_call(cmd_str)`：

```text
RemoteCmdInterface.Request.cmd_str = "MoveL(...)"
RemoteCmdInterface.Response.cmd_res = "0,..."
```

返回码由 `_parse_ret_code()` 解析。`MoveJ()` 和 `MoveCart()` 先写入 `JNTPoint(1,...)` 或 `CARTPoint(1,...)`，再发送 `MoveJ(JNT1,...)` 或 `MoveL(CART1,...)`。

阻塞运动由 `BLEND_BLOCKING` 控制。阻塞时会等待状态话题中的 `motion_done` 或 `robot_motion_done`，若超时但 TCP 位姿已接近目标，则按成功处理。

## 安全转场和可达性

### 安全位策略

`_build_session_safe_pose()` 支持两种策略：

- `ROS2_USE_LEGACY_SAFE_POSE=1`：使用旧 `INIT_POSE_P24`，Z 设置为 `ROS2_LIFT_SAFE_Z_MM`。
- 默认策略：读取当前 TCP 位姿，如果 Z 低于安全高度，则在当前位置竖直抬升；否则跳过固定 P24 安全位。

### 分段移动

`_move_to_work_pose()` 先构造高位过渡点：

```text
transit_z = max(current_z, target_z + ROS2_TRANSIT_MARGIN_MM, ROS2_LIFT_SAFE_Z_MM)
```

随后按配置执行：

1. 原地抬升到 `transit_z`。
2. 高位平移到目标 XY。
3. 下降到目标位姿。

`_move_pose_segmented()` 会把长距离运动拆成最大 `ROS2_SEGMENT_MAX_STEP_MM` 的小段，并受 `ROS2_SEGMENT_MAX_STEPS` 和 `ROS2_SEGMENT_TIMEOUT_S` 限制。

### MoveIt IK 兜底

如果 `LASTTIME_MOVEIT_IK=1` 且 `MOVEIT_JOINT_FALLBACK=1`，部分安全转场语义下的 MoveL 失败会尝试：

1. 调用 `compute_ik`。
2. 将返回关节角转为度。
3. 用 `MoveJ()` 执行兜底。

兜底仅在上下文包含安全高度、高位平移、移动到起始位置、回到起点、返回安全位置、移动到悬空位等关键词时启用。

### 腿部可达姿态调整

`_adjust_leg_frames_for_reachability()` 用控制器逆解探针检查每个腿部点的悬空位和接触位：

1. 从原始局部法向姿态开始。
2. 按 `THIGH_REACHABILITY_STEP_DEG` 逐步收敛到参考工具姿态。
3. 找到第一组悬空位和接触位都可逆解的姿态。
4. 如果某点失败：
   - `FT_CONTINUE_ON_POINT_ERROR=1` 时跳过该点。
   - `0` 时终止执行。

## 力传感器和软件力控

### 初始化

`Ros2ForceController.connect_and_init()` 通过 ROS 2 command service 执行：

1. `FT_SetConfig(company, device, 0, bus)`。
2. `FT_Activate(0)`。
3. `FT_Activate(1)`。
4. `SetForceSensorPayload(0.0)`。
5. `SetForceSensorPayloadCog(0,0,0)`。
6. `FT_SetZero(0)`。
7. 最多 5 次 `FT_SetZero(1)`。
8. `FT_SetRCS(0,0,0,0,0,0,0)`。
9. 可选 `FT_Guard(1,...)`。

如果校零失败，但 `LASTTIME_FORCE_ALLOW_SKIP_ZERO=1` 且当前三轴力最大值小于 `LASTTIME_FORCE_ZERO_MAX_ABS_N`，程序允许继续。

### 读数

`read()` 优先调用：

```text
FT_GetForceTorqueRCS(1)
```

如果 service 返回不可解析，则退回读取状态话题字段：

```text
ft_fx_data, ft_fy_data, ft_fz_data, ft_tx_data, ft_ty_data, ft_tz_data
```

`FORCE_AXIS_SIGN` 用于把 Fz 转换为当前按压力：

```text
press = FORCE_AXIS_SIGN * Fz
```

### 软件限幅

`check_limits()` 每次读取后检查：

- 法向力：`abs(Fz) <= software_force_limit`
- 横向力：`sqrt(Fx^2 + Fy^2) <= tangential_force_limit`
- 力矩：`max(abs(Mx), abs(My), abs(Mz)) <= LASTTIME_FORCE_SOFTWARE_TORQUE_LIMIT_NM`

超限时会调用 `StopMotion()`，停止力控并抛出异常。

### 贴近和保压

`_approach_to_target_force()` 从悬空位开始，沿 `tool_z_unit` 增加 `offset_mm`：

1. 可选连续预贴近到 `-LASTTIME_FORCE_APPROACH_PRECONTACT_CLEARANCE_MM`。
2. 循环读取当前按压力。
3. 未到目标力时按阶段选择步长和速度：
   - 粗贴近：`LASTTIME_FORCE_APPROACH_STEP_MM`
   - 接触后：`LASTTIME_FORCE_APPROACH_CONTACT_STEP_MM`
   - 接近目标：`LASTTIME_FORCE_APPROACH_FINE_STEP_MM`
   - 近目标：`LASTTIME_FORCE_APPROACH_NEAR_STEP_MM`
4. 达到目标力返回当前 offset。
5. 达到最大 offset 仍未达目标力则返回失败。

`_hold_target_force()` 在保压时间内执行比例微调：

```text
err_n = target_n - press
delta_mm = clamp(LASTTIME_FORCE_HOLD_KP_MM_PER_N * err_n, +/- LASTTIME_FORCE_HOLD_MAX_STEP_MM)
offset += delta_mm
```

每次微调后都会执行 MoveCart，并调用 `check_limits()`。

### 卸力

`_retract_to_hover()` 回到悬空位后调用 `_wait_force_released()`。最大三轴力小于 `LASTTIME_FORCE_RELEASE_LIMIT_N` 时认为卸力完成，否则等待到 `LASTTIME_FORCE_RELEASE_TIMEOUT_S`。

## 动作执行

### 点筋

`execute_dian_jin(frame)`：

1. 到当前点悬空位并确认卸力。
2. 贴近到目标力。
3. 保压 `LASTTIME_FORCE_DIAN_DWELL_S`。
4. 回悬空位并确认卸力。
5. 按 `FT_DIAN_JIN_REPEAT_COUNT` 重复执行，默认每个点 3 次。

非力控分支使用位置动作：从悬空位移动到 `hover - DIAN_JIN_DEPTH_MM` 再返回。

### 分筋

`execute_fen_jin(frame)`：

1. 到悬空位并确认卸力。
2. 中心贴近到目标力并保压。
3. 按 `FT_FEN_JIN_REPEAT_COUNT` 重复执行分筋轮次；每轮依次移动到 `+LASTTIME_FORCE_FEN_LATERAL_MM`、`-LASTTIME_FORCE_FEN_LATERAL_MM`、`0`，每个位置保压 `LASTTIME_FORCE_FEN_DWELL_S`。
4. 回悬空位并确认卸力。

非力控分支用 `FEN_JIN_LATERAL_MM` 在悬空位左右移动。

### 顺筋

`execute_shun_jin(frames)`：

1. 从候选点中寻找第一个能贴近到目标力的起点。
2. 沿后续采样点逐点移动，保持上一次贴近 offset 作为初值。
3. 默认每个点通过 `LASTTIME_FORCE_SHUN_RECONTACT=1` 重新沿法向贴近到目标力，避免身体曲面变化导致后段悬空。
4. 每个点保压 `LASTTIME_FORCE_SHUN_DWELL_S`。
5. 失败点按 `FT_CONTINUE_ON_POINT_ERROR` 决定跳过或终止。
6. 结束后回到最后悬空位。

非力控分支只沿各点悬空位移动。

### 完整序列

`execute_massage_sequence()`：

1. 构造并进入安全位。
2. 腿部轨迹执行可达姿态调整。
3. 初始化力传感器和力控通道。
4. 高位转场到第一个点悬空位。
5. 遍历采样点执行点筋和分筋。
6. 用成功到达过悬空位的点作为顺筋候选点。
7. 回到顺筋起点并执行顺筋。
8. 返回安全位。
9. 关闭力控通道。

## 线程模型

交互主线程负责相机帧读取、窗口刷新和键盘事件。按 `g` 后 `_start_motion_thread_once()` 启动 daemon 线程执行 `_motion_worker()`：

```text
motion worker
    -> init_robot()
    -> execute_massage_sequence()
    -> motion_success / motion_error
```

主线程继续刷新检测/预览窗口，并通过 `_update_finished_motion_status()` 将动作完成、异常或结束状态显示到窗口和控制台。

由于动作线程和预览线程共享状态，预览状态字段由 `preview_state_lock` 保护。机械臂控制对象只在动作线程中使用。

## 智能体状态机

语音链路使用两个 JSON 文件同步：

```text
robots/fairino/ft_agent_state/current_session.json
robots/fairino/ft_agent_state/current_control.json
```

`current_session.json` 是状态源，GUI、`status` 工具和运行时都读取它。主要字段：

| 字段 | 说明 |
| --- | --- |
| `status` | `idle`、`detecting`、`detected`、`running`、`pausing`、`paused`、`stopping`、`stopped`、`error` |
| `target` / `target_label` | 当前部位 |
| `trajectory_path` / `point_count` | 当前轨迹路径和点数 |
| `actions` | 本次动作集合，按点筋、分筋、顺筋归一化 |
| `stage` | 当前阶段，例如 `point_actions`、`shun_jin`、`completed` |
| `current_action` | 当前动作或内部步骤 |
| `current_point_index` | 当前点位索引 |
| `current_repeat_index` / `current_step_index` | 点筋/分筋内部重复次数和步骤 |
| `resume_*` | 暂停后继续按摩的恢复点 |
| `robot_tcp_pose` / `robot_joints_deg` | 最近一次写入的机械臂状态 |
| `force_target_n` | 当前目标力 |
| `worker_pid` | 执行子进程 PID |

`current_control.json` 是命令通道，包含 `request` 和可选 `force_adjust`。执行子进程通过 `control.checkpoint()` 在动作边界读取：

- `request=continue`：继续执行。
- `request=pause`：如果当前点可安全暂停，退回局部悬空位并写 `paused`；如果暂不可暂停，标记为 pending，直到下一个安全点。
- `request=stop`：退回局部悬空位，返回 `stopped`。
- `force_adjust`：按归一化后的 `delta_n` 更新目标力，受 `FT_LIVE_FORCE_TARGET_MIN_N` 和 `FT_LIVE_FORCE_TARGET_MAX_N` 限制。语音层传入的数值先按 `direction` 解释，避免“减轻 10N”被误当成 `+10N`。

暂停恢复粒度已经细化到点位、动作、重复次数和步骤。点筋完成一次后暂停，恢复时会从下一次点筋开始；分筋同理。顺筋当前只有一条顺序轨迹，恢复时从保存的点位继续。

停止动作分两级：执行进程先回当前按摩点贴近前的局部悬空位，`FairinoMassageRuntime.stop()` 再调用 `./massage home` 回到记录的起始位置。这样避免从接触状态直接抬高造成横向刮擦。

## GUI 状态展示

小智 GUI 由 QML 展示，Python display model 轮询 `current_session.json`。界面上的部位、轨迹、进度、阶段、动作、点位、力度和消息都来自状态文件，不直接查询机械臂。

进度估算在运行时完成：

- `point_actions` 阶段映射到 0%-69%。
- `shun_jin` 阶段映射到 70%-99%。
- `status=stopped` 且 `stage=completed` 时显示 100%。

底部快捷按钮只把文本意图送入小智对话流程，由模型按工具描述调用 MCP 工具。GUI 本身不直接写控制文件，避免绕过小智的工具调用和状态确认。

## 轨迹保存格式

`_save_locked_trajectory(label, extra)` 写入 JSON：

```json
{
  "generated_at": "2026-05-31T16:53:59",
  "target": "thigh_outerline",
  "hover_height_mm": 30.0,
  "force_target_n": 30.0,
  "tool_tip_length_mm": 95.0,
  "pixels": [[...]],
  "points_mm": [[...]],
  "frames": [{ "...": "..." }],
  "trajectory_type": "thigh_outerline",
  "debug_image": "/path/to/png"
}
```

背部额外字段：

- `trajectory_type: bladder_meridian`

腿部额外字段：

- `trajectory_type: thigh_outerline` 或 `thigh_inner`
- `raw_confirmation_json`
- `thigh_side`
- `thigh_offset_mm`
- `thigh_direction`
- `thigh_inner_skip_points`

调试 PNG 由 `_save_trajectory_debug_image()` 生成，包含检测线、采样点编号和深度文本。

## 关键配置分类

### 部位选择

- `MASSAGE_TARGET`
- `THIGH_OUTER_FORCE_N`
- `LASTTIME_THIGH_OUTER_FORCE_N`
- `THIGH_INNER_FORCE_N`
- `LASTTIME_THIGH_INNER_FORCE_N`
- `THIGH_FORCE_N`
- `LASTTIME_THIGH_FORCE_N`
- `LASTTIME_FORCE_N`

### 视觉检测

- `BACK_MIN_DEPTH_RATIO`
- `BACK_LINE_TRIM_NECK_RATIO`
- `BACK_LINE_TRIM_TAIL_RATIO`
- `THIGH_SIDE`
- `THIGH_OFFSET_MM`
- `THIGH_DIRECTION`
- `THIGH_FLIP_DIRECTION`
- `THIGH_SAMPLE_POINTS`
- `THIGH_STABLE_FRAMES`
- `THIGH_MIN_DEPTH_RATIO`
- `THIGH_KPT_THR`
- `THIGH_TRY_ROTATIONS`
- `THIGH_ROTATION`
- `THIGH_DEVICE`

### 坐标和姿态

- `LASTTIME_TOOL_TIP_LENGTH_MM`
- `FT_KEEP_CURRENT_ORIENTATION`
- `THIGH_LOCAL_NORMAL_MAX_TILT_DEG`
- `THIGH_AUTO_REACHABLE_ORIENTATION`
- `THIGH_REACHABILITY_STEP_DEG`
- `THIGH_REACHABILITY_MIN_TILT_DEG`

### 运动和转场

- `ROBOT_TOOL_ID`
- `ROBOT_USER_ID`
- `ROS2_LIFT_SAFE_Z_MM`
- `ROS2_USE_LEGACY_SAFE_POSE`
- `ROS2_TRANSIT_MARGIN_MM`
- `ROS2_TRANSIT_LIFT_FIRST`
- `ROS2_SEGMENT_MAX_STEP_MM`
- `FT_TRANSIT_SPEED_SCALE`
- `FT_TRANSIT_SPEED_MAX`
- `FT_ROBOT_MOTION_SPEED_SCALE`

### 力控和保护

- `LASTTIME_ROS2_FORCE`
- `LASTTIME_FORCE_SENSOR_BUS`
- `LASTTIME_FORCE_AXIS_SIGN`
- `LASTTIME_FORCE_PRESTART_LIMIT_N`
- `LASTTIME_FORCE_NORMAL_LIMIT_N`
- `LASTTIME_FORCE_TANGENTIAL_LIMIT_N`
- `LASTTIME_FORCE_SOFTWARE_TORQUE_LIMIT_NM`
- `LASTTIME_FORCE_APPROACH_*`
- `LASTTIME_FORCE_HOLD_*`
- `LASTTIME_FORCE_RELEASE_*`
- `FT_DIAN_JIN_REPEAT_COUNT`
- `FT_FEN_JIN_REPEAT_COUNT`
- `LASTTIME_FORCE_GUARD`

## 维护注意事项

1. `run_lasttime_ros2.sh` 会设置 `LASTTIME_FORCE_KEEP_CURRENT_ORIENTATION`，但 `ft.py` 当前读取的是 `FT_KEEP_CURRENT_ORIENTATION`。需要保持当前 TCP 姿态时，应设置 `FT_KEEP_CURRENT_ORIENTATION=1`。
2. `run_lasttime_ros2.sh` 会为 `ft.py` 设置 `HOVER_HEIGHT_MM=50.0`，但 `ft.py` 实际使用 `BACK_HOVER_HEIGHT_MM` 和 `THIGH_HOVER_HEIGHT_MM`。调整悬空高度时应改这两个变量。
3. 腿部内侧模式不是独立的内侧检测模型，而是在外侧中线检测结果上跳过前若干点。
4. `Ros2ForceController.start()` 和 `_ft_control_cmd()` 是可用封装，但当前动作序列没有调用 `start()`。如果未来改为硬件 `FT_Control` 闭环，需要重新审查软件贴近和保压逻辑的叠加关系。
5. 所有运动命令都依赖 `nonrt_state_data` 中字段命名。如果 FAIRINO ROS 2 消息字段变更，需同步 `_state_pose()`、`_state_joints_deg()` 和 `read()`。
6. `FT_CONTINUE_ON_POINT_ERROR=1` 会把部分点失败降级为跳过。调试精度或验收流程时建议设为 `0`，让问题尽早暴露。
7. 小智检测工具是异步启动：`success=true` 不是检测完成，应以后续 `status=detected` 和 `trajectory_path` 为准。
8. `./massage gui` 会把旧运行态清理为 `idle`；如果需要保留暂停会话用于恢复，不要重启 GUI。
9. 最新标定矩阵只在重新检测生成轨迹时生效。执行旧轨迹或恢复暂停轨迹不会重新计算坐标。
10. 语音停止会先回局部悬空位再回起始位；如果执行进程被强制杀死，运行时无法确认已经回到局部悬空位，会跳过回起始位并写入错误状态。

## 扩展建议

### 增加新的按摩部位

1. 在 `_normalize_massage_target()` 增加目标别名。
2. 在 `_massage_target_label()` 增加显示名称。
3. 在 `LastTimeRos2Demo.__init__()` 配置目标力、悬空高度和最大贴近 offset。
4. 实现对应的交互检测和 `frames` 构建函数。
5. 确保每个 frame 至少包含 `point_mm`、`tool_z_unit`、`split_axis_unit`、`base_pose`。
6. 在 `run()` 中分派到新模式。

### 调整力控策略

优先只改环境变量。若代码层面调整：

1. 先审查 `_approach_to_target_force()` 的阶段步长和速度。
2. 再审查 `_hold_target_force()` 的比例系数和最大步长。
3. 保留 `check_limits()` 在每次 MoveCart 后执行。
4. 用小目标力和较大悬空高度先做实机验证。

### 调整轨迹保存

统一从 `_save_locked_trajectory()` 扩展字段，避免分别改背部和腿部流程。新增字段应保持 JSON 可序列化，并避免保存过大的图像或深度矩阵。
