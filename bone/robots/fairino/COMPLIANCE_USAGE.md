# FAIRINO 柔顺控制使用说明

相关文件：

- 启动器：[run_fairino_compliance.sh](/home/franka/massage/robots/fairino/run_fairino_compliance.sh)
- Python 脚本：[fairino_compliance_control.py](/home/franka/massage/fairino_compliance_control.py)
- 默认配置：[fairino_compliance.env](/home/franka/massage/robots/fairino/fairino_compliance.env)
- 多工位模板：[fairino_compliance.site_template.env](/home/franka/massage/robots/fairino/fairino_compliance.site_template.env)

## 最短使用流程

1. 复制一份配置文件

```bash
cp /home/franka/massage/robots/fairino/fairino_compliance.site_template.env /home/franka/massage/site_a.env
```

2. 编辑 `site_a.env`

至少先填这几个：

- `ROBOT_IP`
- `TOOL_ID`
- `USER_ID`
- `POSE_A`
- `POSE_B`
- `COMPLIANCE_P`
- `COMPLIANCE_FORCE`

3. 先检查配置

```bash
/home/franka/massage/robots/fairino/run_fairino_compliance.sh --config /home/franka/massage/site_a.env --print-config
```

4. 再做 dry-run

```bash
/home/franka/massage/robots/fairino/run_fairino_compliance.sh --config /home/franka/massage/site_a.env --dry-run
```

5. 最后正式执行

```bash
/home/franka/massage/robots/fairino/run_fairino_compliance.sh --config /home/franka/massage/site_a.env
```

## 配置项说明

### 基础连接

- `ROBOT_IP`：控制器 IP
- `TOOL_ID`：工具坐标系编号
- `USER_ID`：工件坐标系编号
- `MODE_AUTO`：`1` 表示启动时切自动模式
- `ROBOT_ENABLE`：`1` 表示启动时上使能

### 运动参数

- `POSE_A`：起点位姿，格式 `x,y,z,rx,ry,rz`
- `POSE_B`：终点位姿，格式 `x,y,z,rx,ry,rz`
- `CYCLES`：往返次数
- `MOVE_VEL`：`MoveL` 速度百分比
- `MOVE_OVL`：速度缩放
- `MOVE_BLEND_R`：`-1` 表示阻塞到位
- `MOVE_DWELL`：每段运动后的停留时间

### 传感器参数

- `SENSOR_COMPANY` / `SENSOR_DEVICE` / `SENSOR_BUS`
- `SENSOR_ID`
- `SKIP_ZERO`：`1` 表示跳过 `FT_SetZero(1)`
- `SLEEP_AFTER_SENSOR_CMD`
- `PAYLOAD_WEIGHT`
- `PAYLOAD_COG`

### 柔顺参数

- `COMPLIANCE_P`
- `COMPLIANCE_FORCE`

### FT_Control 参数

- `SELECT_DOF`
- `TARGET_FT`
- `FT_PID`
- `MAX_DIS`
- `MAX_ANG`
- `MB_M`
- `MB_B`
- `THRESHOLD`
- `ADJUST_COEFF`
- `FILTER_SIGN`
- `POS_ADAPT_SIGN`
- `IS_NO_BLOCK`

## 参数覆盖优先级

从高到低：

1. 运行命令最后追加的命令行参数
2. 运行时显式设置的环境变量
3. `--config` 指定的配置文件
4. `fairino_compliance.env`
5. 启动器内部默认值

例如：

```bash
COMPLIANCE_FORCE=15 /home/franka/massage/robots/fairino/run_fairino_compliance.sh --config /home/franka/massage/site_a.env --cycles 2
```

## 多工位建议

建议每个工位一份独立配置：

- `/home/franka/massage/site_neck.env`
- `/home/franka/massage/site_back.env`
- `/home/franka/massage/site_arm.env`

运行时切换：

```bash
/home/franka/massage/robots/fairino/run_fairino_compliance.sh --config /home/franka/massage/site_back.env
```

## 安全建议

- 第一次运行只用 `--print-config` 和 `--dry-run`
- 首次实机运行时把 `CYCLES` 设成 `1`
- 先用较小 `MOVE_VEL`
- 先用较小 `COMPLIANCE_FORCE`
- 校零前确认末端未接触外物
- 修改 `TOOL_ID` / `USER_ID` / `POSE_A` / `POSE_B` 后重新核对轨迹

## 当前脚本实际流程

启动器最终调用的 Python 脚本流程是：

1. `RPC`
2. 可选 `Mode(0)`
3. 可选 `RobotEnable(1)`
4. `FT_SetConfig`
5. `FT_Activate(0)` -> `FT_Activate(1)`
6. 可选 `FT_SetZero(1)`
7. `FT_SetRCS(0)`
8. 可选负载参数设置
9. `FT_Control(1, ...)`
10. `FT_ComplianceStart(...)`
11. `MoveL(A)` / `MoveL(B)` 往返
12. `FT_ComplianceStop()`
13. `FT_Control(0, ...)`
14. `CloseRPC()`
