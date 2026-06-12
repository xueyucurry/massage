# 产品已有功能理解笔记

本文档基于当前 `rm_demo` 审查结果、板卡 `192.168.1.11` 上已运行 ROS 接口、以及 `/home/rm/rm_healthcare_robot` 安装包暴露的消息/服务接口整理。目标是帮助理解原产品如何把视觉、运动、力控、温控串起来。

## 1. 总体链路

产品原有按摩流程可以理解为：

```text
相机采集 RGBD
  -> 视觉算法检测人体区域/穴位/轨迹像素点
  -> 深度 + 表面法向计算三维轨迹点
  -> 手眼标定 + 当前机械臂关节角转换成机器人位姿
  -> 轨迹生成服务生成产品原生轨迹文件
  -> rm_driver 上传并执行轨迹
  -> 轨迹内打开力控，按摩头下压接触并沿轨迹运动
  -> 温度服务独立设置/读取按摩头温度
```

这里真正让按摩头“贴住人体”的不是单纯视觉定位，而是：

```text
视觉给出大致表面点和法向
轨迹先到悬空点
然后打开力控
再给一个略微超过皮肤表面的下压目标
接触后由六维力控闭环维持目标力
```

## 2. 视觉算法

### 2.1 相机输入

当前产品相机是 RealSense RGBD，主要话题：

| 话题 | 类型/用途 |
|---|---|
| `/camera/color/image_raw` | RGB 图像 |
| `/camera/color/camera_info` | 彩色相机内参 |
| `/camera/aligned_depth_to_color/image_raw` | 对齐到 RGB 的深度图 |
| `/camera/aligned_depth_to_color/camera_info` | 对齐深度内参 |

在 RViz 点云可视化中，我们已用 RGB + aligned depth 生成：

```text
/rm_demo/back_rgbd_points
```

并用手眼标定接到 `world` 坐标系。

### 2.2 人体区域/穴位/轨迹检测

板卡上运行的视觉节点包括：

| 节点/服务 | 作用 |
|---|---|
| `/ai_service/area_detection` | 人体区域检测 |
| `/ai_service/calc_position_normal` | 根据 RGBD 和像素轨迹计算三维点 + 表面法向 |
| `/ai_predict_area_acupoints` | 区域/穴位相关视觉节点 |
| `/ai_scan_person_area` | 扫描人体区域相关节点 |
| `/ai_service/move_camera_above_person` | 原产品拍照位/相机对人体定位服务 |

本地封装里，`rm_demo/rm_product_ros.py` 的核心逻辑是：

```text
输入:
  color_image
  depth_image
  diagonal_point_coor   # 人体区域 bbox
  waypoints_pixel_coor  # 轨迹像素点

调用:
  /ai_service/calc_position_normal

输出:
  WaypointPositionVector[]
    point  = 相机坐标系下三维点
    vector = 表面法向量
```

对应代码位置：

```text
rm_demo/rm_product_ros.py
  attach_robot_points_via_product_services()
```

### 2.3 坐标转换

视觉算法先得到的是相机坐标下的：

```text
point + normal vector
```

随后调用：

```text
/calc_poses
```

输入：

```text
当前机械臂关节角
机械臂安装角 install_ang
相机坐标下的 waypoint point/vector
```

输出：

```text
机器人 Base/world 坐标下的 waypoint pose
```

这里会用到产品配置里的手眼标定矩阵：

```text
trajectory_generate.yaml
  eye_on_hand_calibrate
```

当前我们本地也在用这套矩阵，把相机点云、ArUco、机械臂 TF 放到同一个 `world` 坐标树中。

## 3. 工具坐标系

产品按摩使用工具坐标系：

```text
mas_rub
```

当前读取到的 `mas_rub` 相对法兰大致为：

```text
xyz = [0, -0.073916, 0.110916] m
rpy = [0.785, 0, 0] rad
```

产品轨迹文件中的运动命令会显式写：

```json
{"Cmd_Type":"MOVEL","Frame":"Base","Tool":"mas_rub", ...}
```

所以正常产品链路下，控制目标不是法兰中心，而是当前工具 TCP，即按摩头坐标系。

## 4. 轨迹生成

产品原生轨迹由服务生成：

```text
/generate_trajectory_rubbing
```

服务请求结构：

```text
float64[6] arm_curr_joint_ang
float32[3] install_ang
string tool_name
WaypointPositionVector[] waypoints
int8 force
int8 speed
int8 trajectory_type
```

服务响应：

```text
string trajectory_content
int16[] open_force_num_list
int16[] stop_force_num_list
```

本地封装：

```text
rm_demo/rm_product_trajectory.py
  generate_rubbing_trajectory()
```

也就是说，轨迹生成服务并不是只生成一串点，而是直接生成产品控制器可执行的“项目文件”。

## 5. 产品原生轨迹文件结构

我们之前生成的典型轨迹内容如下：

```json
{"file":6}
{"enabled":true,"name":"Stop_Force","num":1,"parent_number":0}
{"Cmd_Type":"MOVEJ","Frame":"Base","Tool":"mas_rub","name":"MOVEJ","num":2}
{"name":"prepare","joint":[0,22000,-105000,0,-67000,0],"num":3}
{"Cmd_Type":"MOVEL","Frame":"Base","Tool":"mas_rub","name":"MOVEL","num":4}
{"name":"p1_above_2cm","pose":[...],"num":5}
{"direction":2,"enabled":true,"mode":1,"n":20,"name":"Force","num":6,"sensor":1}
{"Cmd_Type":"MOVEL","Frame":"Base","Tool":"mas_rub","name":"MOVEL","num":7}
{"name":"p1_down_3cm","pose":[...],"num":8}
{"name":"p1_down_1cm","pose":[...],"num":9}
{"Loop_Type":2,"name":"loop","num":10}
{"Cmd_Type":"MOVES","Frame":"Base","Tool":"mas_rub","name":"MOVES","num":11}
{"name":"p2","pose":[...],"num":12}
{"name":"p3","pose":[...],"num":13}
```

关键含义：

| 段落 | 作用 |
|---|---|
| `Stop_Force` | 开始前先关闭残留力控 |
| `MOVEJ prepare` | 回到按摩准备姿态 |
| `p1_above_2cm` | 到第一轨迹点上方悬空位 |
| `Force` | 打开力控 |
| `p1_down_3cm/p1_down_1cm` | 朝人体方向下压，建立接触 |
| `MOVES p2...p30` | 在力控保持接触的状态下沿轨迹运动 |

轨迹文件单位通常是：

```text
pose xyz: 微米，1e-6 m
pose rpy: 毫弧度，1e-3 rad
joint: 毫度，1e-3 deg
```

例如：

```text
pose=[-845197, 32329, 288840, -1678, -504, 1790]
=> [-0.845197, 0.032329, 0.288840, -1.678, -0.504, 1.790]
```

## 6. 运动控制

产品底层驱动是：

```text
/rm_driver
```

常用直接运动话题：

| 话题 | 作用 |
|---|---|
| `/rm_driver/MoveJ_Cmd` | 关节空间运动 |
| `/rm_driver/MoveJ_P_Cmd` | 位姿点到点运动 |
| `/rm_driver/MoveL_Cmd` | 直线运动 |
| `/rm_driver/MoveC_Cmd` | 圆弧运动 |
| `/rm_driver/Stop_Cmd` 或 `/rm_driver/SetArmStop` | 停止 |
| `/rm_driver/ArmCurrentState` | 当前关节、TCP 位姿、错误码 |

产品项目文件执行通道：

| 接口 | 作用 |
|---|---|
| `/deletecurrenttrajectory` | 删除当前轨迹 |
| `/deletetrajectory` | 删除已有轨迹 |
| `/rm_driver/PrepareRunProject` | 准备接收项目文件 |
| `/rm_driver/SendTrajectoryFile` | 分块发送项目文件 |
| `/rm_driver/ReceiveTrajectoryFileState` | 接收状态 |
| `/rm_driver/TrajectoryFileVerifyState` | 校验状态 |
| `/control_plan_number` | 触发执行 |
| `/rm_driver/RunProjectState` | 执行状态 |

本地封装：

```text
rm_demo/rm_product_executor.py
  upload_product_trajectory()
```

## 7. 力控

### 7.1 产品原生轨迹中的力控

产品轨迹里的 `Force` 节点类似：

```json
{
  "direction": 2,
  "enabled": true,
  "mode": 1,
  "n": 20,
  "name": "Force",
  "sensor": 1
}
```

根据消息定义和现有例子可以确定：

| 字段 | 含义 |
|---|---|
| `sensor: 1` | 使用六维力传感器 |
| `direction: 2` | 力控方向枚举，当前产品背部流程里表现为 Z 方向 |
| `mode: 1` | 力控模式 |
| `n: 20` | 目标力度/内部力控数值 |

注意：`/generate_trajectory_rubbing` 请求里的 `force` 是 `int8` 力度等级。例子中 `force=2` 生成了 `n=20`，所以它不是简单原样透传，更像产品内部力度等级到目标力值的映射。

### 7.2 ROS 力控接口

产品驱动也暴露了直接力控话题：

| 话题 | 消息 | 作用 |
|---|---|---|
| `/rm_driver/SetForceSensor_Cmd` | `std_msgs/Empty` | 启用力传感器 |
| `/rm_driver/SetForcePositionNew_Cmd` | `rm_msgs/Force_Position_New` | 设置新力位混合参数 |
| `/rm_driver/SetForcePosition_Cmd` | `rm_msgs/Force_Position` | 设置力位混合方向/目标力 |
| `/rm_driver/StartForcePositionMove_Cmd` | `std_msgs/Empty` | 开始力位混合 |
| `/rm_driver/StopForcePositionMove_Cmd` | `std_msgs/Empty` | 停止力位混合 |
| `/rm_driver/ForcePositionMovePose_Cmd` | `rm_msgs/Force_Position_Move_Pose` | 力控位姿运动 |
| `/rm_driver/GetSixForce_Cmd` | `std_msgs/Empty` | 请求六维力 |
| `/rm_driver/GetSixForce` | `rm_msgs/Six_Force` | 六维力反馈 |
| `/rm_driver/Force_Position_State` | `rm_msgs/Force_Position_State` | 力控状态 |

`Force_Position_New` 定义里有一个重要注释：

```text
sensor: 0 一维力，1 六维力
coordinate: 0 工作坐标系力控，1 工具坐标系力控，按摩默认为 1
z_control_mode:
  0 力跟踪模式
  1 力跟踪模式 + 姿态自适应模式，标准推揉型项目采用该模式
force: 力度
```

因此产品按摩更推荐“工具坐标系力控”，即沿按摩头自身坐标系的某一轴进行力跟踪。

### 7.3 接触是如何实现的

接触建立过程是：

```text
1. 到 p1_above_2cm 悬空点
2. 开 Force
3. 继续向下压到 p1_down_3cm / p1_down_1cm
4. 传感器检测到接触力
5. 控制器用力位混合修正实际位姿
6. 后续 MOVES 沿轨迹走，力控维持接触
```

这就是为什么视觉深度不需要毫米级完全准确：只要下压目标略过表面，力控会在接触后接管法向方向。

## 8. 温度调节

### 8.1 温度 ROS 话题

当前板卡已运行温度节点：

```text
/rm_temperature_driver
/rm_temperature_server
```

主要话题：

| 话题 | 类型 | 作用 |
|---|---|---|
| `/rm_temperature_set` | `std_msgs/Float32` | 设置目标温度 |
| `/rm_temperature_set_result` | `std_msgs/Bool` | 设置结果 |
| `/rm_temperature_get` | `std_msgs/Empty` | 请求当前温度 |
| `/rm_temperature_get_result` | `std_msgs/Float32` | 当前温度结果 |
| `/rm_temperature_connect_state` | `std_msgs/Bool` | 温控设备连接状态 |
| `/rm_temperature_communication_state` | `std_msgs/Bool` | 温控通信状态 |
| `/rm_temperature_alarm_state` | `rm_healthcare_robot_msgs/Alarm` | 温控报警状态 |

### 8.2 温度 ROS 服务

同时也有服务：

```text
/temperature_set_srv
/temperature_get_srv
```

服务定义：

```text
# 设置控制温度
float32 tempurate
---
bool state
```

```text
# 获取当前温度
std_msgs/Empty get_temp
---
float32 temp_value
```

字段名 `tempurate` 是产品消息里的拼写，不是我们写错。

### 8.3 课程参数里的温度

产品课程节点中也有温度字段，例如：

```text
Keynote_Course_Node:
  technique_id
  time
  strength
  temp
  speed

Vip_Course_Node:
  pose
  technique_id
  time
  strength
  temp
  speed
```

这说明产品上层课程把：

```text
手法 technique
时长 time
力度 strength
温度 temp
速度 speed
```

作为一个理疗动作单元管理。底层实际温控仍然通过温度 topic/service 下发。

## 9. 课程与手法管理

板卡上已有：

```text
/rm_healthcare_robot_course_manager_server
/rm_healthcare_robot_technique_manage_server
```

相关服务：

```text
/rm_healthcare_robot_course_manager_server
/rm_healthcare_robot_technique_manage_server
```

从消息定义看，产品支持：

| 模块 | 作用 |
|---|---|
| Course | 管理标准课程、重点课程、VIP/定制课程 |
| Technique | 管理按摩头、按摩手法、手法简介/图片等 |
| Record | 保存理疗记录，包括时长、温度、力、图片、客户、用户 |

也就是说，产品不是只执行一条轨迹，而是有更上层的“课程/手法/记录”业务层。

## 10. 对侧卧膀胱经二次开发的影响

当前产品默认逻辑更偏向“俯卧背部、相机朝下、Base Z 下压”的使用场景。侧卧时需要特别处理：

### 10.1 拍照姿态

需要用新的侧卧拍照位：

```text
arm_side_lying_prepare
```

当前我们已经保存了这个姿态，并能从该姿态检测侧卧背部膀胱经。

### 10.2 法向方向

侧卧时，“到膀胱经上方”在物理意义上更像“到人体背部前方”，不是简单的世界 Z 上方。

因此接触方向应该来自：

```text
背部表面法向
ArUco 背部坐标系
或 RGBD 点云拟合出的局部法向
```

不能继续假设固定 Base Z。

### 10.3 工具姿态

需要让按摩头的实际接触轴对准背部法向。产品轨迹默认看起来更接近“工具 -Z 为接触轴”的假设；我们之前也在代码里加过 `tool_contact_axis` 修正逻辑。

侧卧后应该明确：

```text
物理按摩头真正接触轴 = mas_rub 的哪根轴
该轴应与背部法向反向或同向对齐
```

### 10.4 力控方向

如果仍使用产品原生 `Force direction=2`，需要验证它在当前工具/工作坐标系下是否就是你想要的“垂直背部方向”。

更合理的方向是：

```text
工具坐标系力控 coordinate=1
让工具 Z 或 -Z 先对齐背部法向
再使用 Z 方向力控
```

### 10.5 最小安全验证顺序

建议侧卧按摩逐步验证：

```text
1. 只显示 RGBD 点云 + aruco_back + rm_mas_rub
2. 只做悬空目标点定位，不接触
3. 工具接触轴对准 ArUco/背部法向
4. 低速移动到 p1_above 安全距离
5. 小步下探，同时读取 GetSixForce
6. 达到 2-3N 立即回撤
7. 再接产品力控轨迹
8. 最后接温度
```

## 11. 当前二开可复用的产品能力

可以直接复用：

| 能力 | 接口/文件 |
|---|---|
| RGBD 输入 | `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw` |
| 表面三维点 + 法向 | `/ai_service/calc_position_normal` |
| 相机点到机械臂位姿 | `/calc_poses` |
| 原生按摩轨迹生成 | `/generate_trajectory_rubbing` |
| 原生项目文件执行 | `/rm_driver/PrepareRunProject`, `/rm_driver/SendTrajectoryFile`, `/control_plan_number` |
| 工具坐标系切换 | `/change_arm_tool_frame` |
| 六维力读取 | `/rm_driver/GetSixForce` |
| 力位混合 | `/rm_driver/SetForcePositionNew_Cmd`, `/rm_driver/StartForcePositionMove_Cmd` |
| 温度设置/读取 | `/temperature_set_srv`, `/temperature_get_srv`, `/rm_temperature_set`, `/rm_temperature_get` |

需要二开改造：

| 问题 | 原因 |
|---|---|
| 侧卧拍照位 | 原产品默认更偏俯卧 |
| 侧卧“上方/前方”定义 | 人体背部法向不再等于 Base Z |
| 工具接触轴验证 | 需要确认按摩头实际接触轴，而不是法兰轴 |
| 力控方向 | 需要和背部法向一致 |
| 轨迹生成后姿态修正 | 产品生成姿态可能默认俯卧场景 |

## 12. 一句话总结

产品已有功能的核心能力是：

```text
视觉算法负责找点和法向；
轨迹服务负责把点和法向变成 mas_rub 工具轨迹；
rm_driver 负责执行轨迹文件；
力控负责让按摩头真正接触并保持压力；
温控节点负责独立设置和读取按摩头温度。
```

侧卧二次开发的关键不是重写全部产品能力，而是把“侧卧背部法向”和“按摩头接触轴”正确接入这条已有链路。
