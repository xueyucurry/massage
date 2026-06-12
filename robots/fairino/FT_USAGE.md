# ft.py 使用文档

## 功能概述

`ft.py` 是 FAIRINO 机械臂的 ROS 2 控制版恒力按摩演示入口。程序基于 RealSense 深度相机、背部膀胱经视觉检测、大腿 RTMPose 姿态检测、相机到机械臂标定矩阵和六维力传感器，实现轨迹锁定、轨迹保存、机械臂安全转场、贴近目标力、点筋、分筋、顺筋等动作。

当前已实现的按摩部位：

| 部位 | 选择值 | 说明 |
| --- | --- | --- |
| 背部膀胱经 | `1` / `back` / `膀胱经` | 检测背部中线和膀胱经线，采样外侧膀胱经执行动作 |
| 腿部大腿外侧中线 | `2` / `leg` / `thigh` | 用 RTMPose 检测髋膝关键点，沿指定方向生成大腿外侧中线 |
| 腿部大腿内侧 | `3` / `leg_inner` / `inner_thigh` | 复用大腿外侧中线检测，保存和执行时跳过前若干采样点 |

程序不是命令行参数驱动的工具，主要通过环境变量配置，通过 OpenCV 窗口键盘交互控制。

## 运行前检查

运行实机前至少确认：

1. FAIRINO 控制器网络可达，默认 IP 为 `192.168.58.2`。
2. ROS 2 Humble 环境和 FAIRINO ROS 2 工作区已构建，默认工作区为 `robots/fairino/fairino_ros2/frcobot_ros2-master`。
3. RealSense 相机可用，RGB 和深度帧稳定。
4. 标定矩阵可加载，通常为 `/home/franka/massage/shared/calibration/camera_to_robot.json`。
5. 六维力传感器已接入控制柜 RS485，总线参数与 `LASTTIME_FORCE_SENSOR_BUS` 一致。
6. 末端校零前必须悬空且无外部接触。
7. 实机运行时操作员必须在急停按钮旁，首次运行建议降低目标力和速度。

## 基本运行

推荐通过启动脚本运行，脚本会拉起或复用 FAIRINO ROS 2 控制服务，并做服务探针：

```bash
cd /home/franka/massage/robots/fairino
LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
```

启动后如果是交互终端，程序会提示选择按摩部位。也可以用环境变量跳过菜单：

```bash
cd /home/franka/massage/robots/fairino
MASSAGE_TARGET=back LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
MASSAGE_TARGET=leg LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
MASSAGE_TARGET=leg_inner LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
```

非交互输入环境下，如果没有设置 `MASSAGE_TARGET`，程序默认进入背部膀胱经模式。

## 窗口和按键

### 背部模式

窗口：

- `Detection`：背部检测、深度有效率和命令提示。
- `Live Preview`：轨迹锁定后的实时预览窗口，窗口名可由 `LASTTIME_LIVE_PREVIEW_WINDOW` 修改。

按键：

| 按键 | 作用 |
| --- | --- |
| `s` | 检测状态为 ready 后保存当前轨迹 |
| `g` | 已保存轨迹后启动机械臂动作 |
| `q` | 未执行动作时退出；动作执行中不会直接退出，会等待动作线程结束 |

背部保存前要求检测稳定并且采样线深度有效率达到 `BACK_MIN_DEPTH_RATIO`。

### 腿部模式

窗口：

- `Thigh Detection`：未锁定前的大腿检测窗口。
- `Detection`：锁定后的检测结果窗口。
- `Live Preview`：轨迹和当前动作状态预览。

按键：

| 按键 | 作用 |
| --- | --- |
| `s` | 当前腿部检测有效且深度有效率足够时保存轨迹 |
| `g` | 已保存轨迹后启动机械臂动作 |
| `q` | 未执行动作时退出；动作执行中不会直接退出，会等待动作线程结束 |

腿部保存前要求检测有效、采样线非空，并且深度有效率达到 `THIGH_MIN_DEPTH_RATIO`。

## 操作流程

1. 启动脚本。
2. 选择按摩部位，或用 `MASSAGE_TARGET` 预先指定。
3. 等待检测窗口出现。
4. 调整人体和相机位置，直到检测线稳定且深度有效。
5. 按 `s` 保存轨迹。保存后程序会生成 JSON 轨迹文件和调试 PNG。
6. 检查画面中锁定轨迹是否合理。
7. 按 `g` 启动机械臂动作。
8. 程序自动连接 ROS 2 控制服务、上使能、移动到安全高度、初始化力传感器、执行动作并返回安全位置。
9. 动作完成后按 `q` 退出窗口。

## 按摩动作

完整动作序列如下：

1. 移动到安全高度。
2. 根据腿部轨迹可达性做姿态筛选和必要的法向角调整。
3. 初始化六维力传感器和力控通道。
4. 移动到第一个采样点的悬空位。
5. 对每个采样点执行点筋和分筋：
   - 点筋：从悬空位沿局部法向贴近，达到目标力后保压，默认每个点执行 3 次。
   - 分筋：贴近到目标力后，沿分筋轴正向、反向、回中心移动并保压，默认每个点执行 3 轮。
6. 回到顺筋起点。
7. 执行顺筋：沿采样点序列移动，并在每个点做目标力微调保压。
8. 返回安全位置。
9. 关闭力控通道。

如果 `LASTTIME_ROS2_FORCE=0`，程序会关闭恒力贴近流程，改用非力控的悬空/位置动作分支。

## 输出文件

轨迹默认保存到：

```text
/home/franka/massage/robots/fairino/ft_locked_trajectory_output
```

文件命名：

```text
back_trajectory_YYYYMMDD_HHMMSS.json
back_trajectory_YYYYMMDD_HHMMSS.png
thigh_outerline_trajectory_YYYYMMDD_HHMMSS.json
thigh_outerline_trajectory_YYYYMMDD_HHMMSS.png
thigh_inner_trajectory_YYYYMMDD_HHMMSS.json
thigh_inner_trajectory_YYYYMMDD_HHMMSS.png
```

JSON 主要字段：

| 字段 | 说明 |
| --- | --- |
| `generated_at` | 生成时间 |
| `target` | 轨迹目标类型 |
| `hover_height_mm` | 当前部位使用的悬空高度 |
| `force_target_n` | 当前部位使用的目标力 |
| `tool_tip_length_mm` | 法兰/传感器中心到按摩头的补偿长度 |
| `pixels` | 图像采样点 |
| `points_mm` | 机械臂基坐标下的采样点，单位 mm |
| `frames` | 每个采样点的完整运动帧，包含点位、局部法向、分筋轴和姿态 |
| `debug_image` | 轨迹调试图路径 |

腿部模式还会通过 `thigh_outerline_confirm.py` 保存原始确认结果，默认目录：

```text
/home/franka/massage/robots/fairino/rtmpose_thigh_confirm_output
```

## 常用配置

环境变量在 Python 进程启动时读取，修改后需要重新启动程序。

### 运行入口

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `ROBOT_IP` | `192.168.58.2` | FAIRINO 控制器 IP |
| `MASSAGE_TARGET` | 空 | 指定 `back`、`leg`、`leg_inner`，为空时交互选择 |
| `LASTTIME_ROS2_SCRIPT` | `lasttime_ros2.py` | 用启动脚本运行 `ft.py` 时必须设为 `ft.py` |
| `LASTTIME_ROS2_PROBE_ONLY` | `0` | 设为 `1` 时启动脚本只做 ROS 2 探针，不进入视觉流程 |
| `FT_TRAJECTORY_OUTPUT_DIR` | `robots/fairino/ft_locked_trajectory_output` | 轨迹保存目录 |

### ROS 2 和安全转场

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `FAIRINO_REMOTE_SERVICE` | `fairino_remote_command_service` | 远程控制 service 名 |
| `FAIRINO_STATE_TOPIC` | `nonrt_state_data` | 机械臂状态 topic 名 |
| `ROS2_SERVICE_WAIT_S` | `20.0` | 等待控制服务时间 |
| `ROS2_CALL_TIMEOUT_S` | `20.0` | 单次 service 调用超时 |
| `ROS2_STATE_WAIT_S` | `12.0` | 等待状态话题时间 |
| `ROBOT_TOOL_ID` | `0` | FAIRINO 工具坐标系编号 |
| `ROBOT_USER_ID` | `0` | FAIRINO 工件坐标系编号 |
| `ROS2_LIFT_SAFE_Z_MM` | `INIT_SAFE_Z_MM` | 安全高度 Z |
| `ROS2_USE_LEGACY_SAFE_POSE` | `0` | `1` 使用旧 P24 安全位；`0` 在当前位置竖直抬升 |
| `FT_KEEP_CURRENT_ORIENTATION` | `0` | `1` 保持当前 TCP 姿态；`0` 使用局部深度平面法向 |
| `FT_TRANSIT_SPEED_SCALE` | `2.0` | 安全转场速度缩放 |
| `FT_TRANSIT_SPEED_MAX` | `100.0` | 安全转场速度上限 |
| `FT_ROBOT_MOTION_SPEED_SCALE` | `2.0` | 最终发送到 `SetSpeed`、`MoveJ`、`MoveL` 的机械臂速度倍率，限幅到 100 |
| `ROS2_SEGMENT_MAX_STEP_MM` | `50.0` | 分段转场最大步长 |
| `ROS2_SEGMENT_TIMEOUT_S` | `180.0` | 分段转场总超时 |

### 部位和轨迹

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `BACK_HOVER_HEIGHT_MM` | `20.0` | 背部模式悬空高度 |
| `BACK_MIN_DEPTH_RATIO` | `0.50` | 背部保存轨迹所需最小深度有效率 |
| `BACK_LINE_TRIM_NECK_RATIO` | `0.08` | 背部采样线靠颈部端缩短比例 |
| `BACK_LINE_TRIM_TAIL_RATIO` | `0.04` | 背部采样线靠尾端缩短比例 |
| `THIGH_HOVER_HEIGHT_MM` | `20.0` | 腿部模式悬空高度 |
| `THIGH_SIDE` | `right` | 腿部检测侧，可为 `nearest`、`auto`、`left`、`right` |
| `THIGH_OFFSET_MM` | `30.0` | 从髋膝线偏移生成按摩线的距离 |
| `THIGH_DIRECTION` | `image-down` | 偏移方向，可为 `outer`、`image-down`、`image-up`、`image-left`、`image-right` |
| `THIGH_SAMPLE_POINTS` | `SAMPLE_POINTS` | 腿部采样点数 |
| `THIGH_STABLE_FRAMES` | `5` | 腿部检测稳定帧数 |
| `THIGH_MIN_DEPTH_RATIO` | `0.70` | 腿部保存轨迹所需最小深度有效率 |
| `THIGH_INNER_SKIP_POINTS` | `3` | 大腿内侧模式跳过前几个采样点 |

### 恒力控制

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `LASTTIME_ROS2_FORCE` | `1` | 是否启用 ROS 2 版恒力贴近和保压流程 |
| `LASTTIME_FORCE_N` | `30.0` | 背部目标力，单位 N |
| `THIGH_OUTER_FORCE_N` | `40.0` | 大腿外侧目标力，单位 N |
| `THIGH_INNER_FORCE_N` | `30.0` | 大腿内侧目标力，单位 N |
| `THIGH_FORCE_N` | 空 | 腿部外侧/内侧全局目标力覆盖，优先级低于上述分部位变量 |
| `LASTTIME_THIGH_FORCE_N` | 空 | 腿部目标力备用覆盖变量 |
| `LASTTIME_FORCE_SENSOR_BUS` | `1` | 六维力传感器总线编号 |
| `LASTTIME_FORCE_AXIS_SIGN` | `-1.0` | 力轴方向符号 |
| `LASTTIME_FORCE_PRESTART_LIMIT_N` | `8.0` | 力控启动前最大允许接触力 |
| `LASTTIME_FORCE_NORMAL_LIMIT_N` | `max(软件力限, 目标力+30)` | 软件法向力限 |
| `LASTTIME_FORCE_TANGENTIAL_LIMIT_N` | `max(80, 软件力限)` | 软件横向力限 |
| `LASTTIME_FORCE_SOFTWARE_TORQUE_LIMIT_NM` | `4.5` | 软件力矩限 |
| `FT_APPROACH_SPEED_SCALE` | `3.0` | 贴近阶段速度倍率 |
| `LASTTIME_FORCE_APPROACH_STEP_MM` | `3.0` | 贴近粗步长 |
| `LASTTIME_FORCE_APPROACH_CONTACT_N` | `2.0` | 进入接触细步阶段的压力阈值 |
| `LASTTIME_FORCE_APPROACH_CONTACT_STEP_MM` | `0.3` | 接触后步长；启动脚本默认覆盖为 `0.6` |
| `LASTTIME_FORCE_APPROACH_FINE_STEP_MM` | `0.6` | 接近目标力时的细步长；启动脚本默认覆盖为 `1.0` |
| `LASTTIME_FORCE_APPROACH_NEAR_STEP_MM` | `0.3` | 更接近目标力时的近端步长；启动脚本默认覆盖为 `0.5` |
| `LASTTIME_FORCE_HOLD_KP_MM_PER_N` | `0.04` | 保压微调比例；启动脚本默认覆盖为 `0.02` |
| `LASTTIME_FORCE_HOLD_MAX_STEP_MM` | `0.15` | 单次保压微调最大位移；启动脚本默认覆盖为 `0.08` |
| `LASTTIME_FORCE_RELEASE_LIMIT_N` | `5.0` | 回悬空位后的卸力判定阈值 |
| `FT_DIAN_JIN_REPEAT_COUNT` | `3` | 每个按摩点的点筋重复次数 |
| `FT_FEN_JIN_REPEAT_COUNT` | `3` | 每个按摩点的分筋重复轮数 |
| `LASTTIME_FORCE_GUARD` | `0` | 是否启用 FAIRINO `FT_Guard` 碰撞守护 |
| `LASTTIME_FORCE_ALLOW_SKIP_ZERO` | `1` | 校零失败但读数接近零点时是否允许继续 |

## 常见运行方式

只检查 ROS 2 控制服务：

```bash
cd /home/franka/massage/robots/fairino
LASTTIME_ROS2_PROBE_ONLY=1 LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
```

背部模式，降低目标力到 10 N：

```bash
cd /home/franka/massage/robots/fairino
MASSAGE_TARGET=back LASTTIME_FORCE_N=10 LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
```

腿部外侧模式，指定偏移方向和目标力：

```bash
cd /home/franka/massage/robots/fairino
MASSAGE_TARGET=leg THIGH_DIRECTION=image-down THIGH_OFFSET_MM=25 THIGH_OUTER_FORCE_N=40 LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
```

临时关闭力控，仅做位置动作分支：

```bash
cd /home/franka/massage/robots/fairino
MASSAGE_TARGET=back LASTTIME_ROS2_FORCE=0 LASTTIME_ROS2_SCRIPT=ft.py ./run_lasttime_ros2.sh
```

## 故障排查

| 现象 | 可能原因 | 处理建议 |
| --- | --- | --- |
| 找不到 ROS 2 控制服务 | 工作区未 source、服务未启动、控制器 IP 不通 | 先运行 `LASTTIME_ROS2_PROBE_ONLY=1`，检查脚本日志 `.ros2_cmd_server.log` |
| 状态话题超时 | ROS 2 discovery 或硬件节点异常 | 重启启动脚本，必要时执行 `ros2 daemon stop && ros2 daemon start` |
| 背部不能保存轨迹 | 检测未稳定或深度有效率不足 | 调整相机角度、人体位置、光照，观察红绿采样点 |
| 腿部不能保存轨迹 | RTMPose 未检测到髋膝点或深度有效率不足 | 调整人体朝向、相机视野、`THIGH_DIRECTION` 和 `THIGH_OFFSET_MM` |
| `FT_SetZero(1)` 失败 | 末端未悬空、传感器负载参数异常或总线异常 | 让末端完全无接触后重启；检查 `LASTTIME_FORCE_SENSOR_BUS` |
| 贴近到最大 offset 仍未达到目标力 | 标定、法向、工具长度或人体位置不准确 | 检查轨迹调试图、`LASTTIME_TOOL_TIP_LENGTH_MM`、`THIGH_FORCE_APPROACH_MAX_OFFSET_MM` |
| 移动失败或不可达 | 目标点超工作空间或姿态逆解失败 | 调整人体/相机位置，降低法向倾角限制，或开启 MoveIt 兜底 |
| 动作中出现力限触发 | 接触力或横向力过大 | 检查目标力、人体接触状态、分筋偏移、急停准备 |

## 相关文件

| 文件 | 说明 |
| --- | --- |
| `ft.py` | ROS 2 恒力按摩主程序 |
| `run_lasttime_ros2.sh` | ROS 2 控制服务启动和探针脚本 |
| `lasttime.py` | 背部视觉检测、轨迹生成和预览基础逻辑 |
| `force_control.py` | 力传感器常量和工具函数 |
| `thigh_outerline_confirm.py` | 大腿检测、偏移线生成和确认文件保存 |
| `dianjing.py` | 标定矩阵加载和点云坐标变换 |
| `RTMPOSE.py` | RTMPose 配置、权重和旋转枚举 |
| `run_shunjin_only.py` | 复用 `ft.py` 的顺筋单项测试入口 |
