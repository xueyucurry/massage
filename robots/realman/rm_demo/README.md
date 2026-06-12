# rm_demo

睿尔曼静态版 demo，目标是跑通这条链路：

1. 抓取一帧 RGB/深度
2. 在单帧上标出膀胱经
3. 用现有标定把经络点转到机械臂坐标
4. 生成单侧静态按摩点
5. 执行悬空/监测力值/实验性 ROS 力控

主要入口：

- `rm_static_demo.py`
- `rm_capture.py`
- `rm_detect.py`
- `rm_positioning.py`
- `rm_transform.py`
- `rm_plan.py`
- `rm_execute.py`
- `rm_json.py`

示例：

```bash
cd ~/massage

# 只抓图、检测、出计划，不运动
python3 rm_demo/rm_static_demo.py

# 先走官方预备拍摄位，再抓图检测
python3 rm_demo/rm_static_demo.py --capture-positioning prepare

# 使用侧卧拍照姿态 section，再抓图检测
python3 rm_demo/rm_bladder_demo.py \
  --capture-positioning prepare \
  --capture-prepare-section arm_side_lying_prepare

# 保存当前机械臂姿态为侧卧初始拍照姿态
.venv/bin/python run_bladder_split.py \
  --save-current-capture-pose-only \
  --capture-prepare-section arm_side_lying_prepare \
  --trajectory-config /home/franka/massage/robots/realman/ros_vendor/trajectory_generate.yaml

# 后续侧卧膀胱经检测/按摩时使用刚保存的拍照姿态
.venv/bin/python run_bladder_split.py \
  --product-flow \
  --product-flow-positioning prepare \
  --capture-prepare-section arm_side_lying_prepare \
  --trajectory-config /home/franka/massage/robots/realman/ros_vendor/trajectory_generate.yaml \
  --transform-backend product_ros

# 侧卧专用入口：只检测、转换、保存 plan，不运动
.venv/bin/python run_side_lying_bladder.py --mode preview

# 侧卧悬空沿 left_outer 轨迹运动，不接触人体
.venv/bin/python run_side_lying_bladder.py --mode hover --run

# 侧卧低力探触：沿背部法向小步靠近，达到目标力或最大力立即处理
.venv/bin/python run_side_lying_bladder.py \
  --mode touch_probe \
  --target-force-n 2 \
  --max-force-n 6 \
  --touch-step-mm 2 \
  --run

# 生成产品原生揉按轨迹；只把入口 p1_above/p1_down 从 Base-Z 改成侧卧背部法向
.venv/bin/python run_side_lying_bladder.py --mode product_generate

# 真正执行产品原生力控轨迹前，先建议用 pose_check 只到第一点悬空位验证
.venv/bin/python run_side_lying_bladder.py --mode pose_check --run

# 验证无误后再执行产品原生力控轨迹，可按需设置温度
.venv/bin/python run_side_lying_bladder.py \
  --mode product_execute \
  --product-force 2 \
  --temperature-c 38 \
  --run

# 还没写入 yaml 时，可直接传示教得到的 6 个关节角
python3 rm_demo/rm_bladder_demo.py \
  --capture-positioning prepare \
  --capture-joints 0 35 -95 20 -70 10

# 你已经手动把相机摆好了，就不要再让脚本移动相机
python3 rm_demo/rm_static_demo.py --capture-positioning none

# 如果知道产品里的真实按摩头工具名，可以再尝试现成上方位服务
python3 rm_demo/rm_static_demo.py \
  --capture-positioning prepare_then_service \
  --camera-tool-name camera \
  --restore-tool-name massage

# 空中预演
python3 rm_demo/rm_static_demo.py --mode hover --run

# 单点保守轻触 + 力值监测
python3 rm_demo/rm_static_demo.py \
  --capture-positioning none \
  --plan-points 1 \
  --mode touch_monitor \
  --target-force-n 3 \
  --max-force-n 8 \
  --touch-step-mm 3 \
  --run

# 到每个点并输出六维力
python3 rm_demo/rm_static_demo.py --mode monitor --target-force-n 8 --run

# 实验性 ROS 力控模式
python3 rm_demo/rm_static_demo.py --mode ros_force_pose --target-force-n 8 --run
```

说明：

- `monitor` 模式默认最稳，普通运动走已验证的 RM JSON 接口，力值通过 ROS `/rm_driver/GetSixForce` 读取。
- `ros_force_pose` 模式会额外使用 `/rm_driver/ForcePositionMovePose_Cmd`，当前按现网 topic 名默认接好，但仍建议先在小力度、小范围下验证。
- `--capture-positioning` 支持 `none|prepare|service|prepare_then_service`。`prepare` 默认读取 `trajectory_generate.yaml` 中的 `arm_massage_prepare` 关节位；侧卧拍背可用 `--capture-prepare-section` 指向新的 section，或用 `--capture-joints` 临时传入示教关节角。
- `run_bladder_split.py --save-current-capture-pose-only` 会读取当前机械臂 6 个关节角，写入指定 `--capture-prepare-section`，并在 `rm_demo_output/current_capture_pose_*.json` 记录 TCP pose 元数据。
- `--detector-backend auto` 会先试 `pose`，失败后自动回退到睿尔曼产品自带的 `area_detection` 服务。
- `--transform-backend auto` 会优先走睿尔曼产品现成的 `calc_position_normal + calc_poses`，只有失败时才回退到静态矩阵。
- `touch_monitor` 会从悬空位沿着接触方向按小步下探，实时读取六维力，达到 `--target-force-n` 或 `--max-force-n` 就回撤。
- 现在如果检测失败，错误里会直接带上最新抓图路径，便于先检查相机当前到底看到的是人体背部还是环境。 
- `run_side_lying_bladder.py` 固定使用 `arm_side_lying_prepare`、`product_ros`、`mas_rub`、`left_outer` 作为侧卧默认链路，并默认按 `mas_rub +Z` 作为按摩头接触轴；如需改上方那条线可加 `--auto-top-line`，如需改经线可传 `--side/--line-type`。
- 侧卧时产品原生轨迹默认的 `p1_above_2cm/p1_down_*` 是 Base-Z 偏移；侧卧入口在产品轨迹模式下默认开启 `--side-lying-product-correction`，只把这些入口点改为沿 `/calc_poses` 生成的背部法向偏移，`p2..` 按摩路径保留产品生成器原始姿态。
