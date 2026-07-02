# ft_agent_api.py 接口说明

`ft_agent_api.py` 是对 `ft.py` 的独立封装层，不修改 `ft.py` 本体。它面向 agent 或自动化脚本，提供经络检测、轨迹保存、轨迹加载，以及点筋、分筋、顺筋动作执行接口。

当前语音控制链路由 `py-xiaozhi` 调用 FAIRINO MCP 工具，工具再通过 `ft_agent_process.py` 在独立子进程中运行检测或按摩动作。GUI、MCP 工具和执行进程通过状态文件和控制文件同步，避免语音进程直接阻塞在长时间机械臂动作里。

## Python 调用

```python
import ft_agent_api as api_mod

api = api_mod.FTMassageAgentInterface("back")

detect = api.detect_meridian(save=True, display=False)
if detect["ok"]:
    result = api.execute_actions(["dian_jin", "fen_jin", "shun_jin"])
```

一键检测并执行：

```python
import ft_agent_api as api_mod

result = api_mod.run_agent_workflow(
    massage_target="back",
    actions=["dian_jin", "fen_jin", "shun_jin"],
    save_trajectory=True,
    display=False,
)
```

从已保存轨迹执行：

```python
import ft_agent_api as api_mod

api = api_mod.FTMassageAgentInterface("back")
api.load_trajectory("/path/to/back_trajectory_YYYYMMDD_HHMMSS.json")
api.execute_actions(["shun_jin"])
```

可暂停/恢复执行由上层会话管理器传入 `control` 对象实现。普通脚本不传 `control` 时，行为与原来一致。

```python
api.execute_actions(
    ["dian_jin", "fen_jin", "shun_jin"],
    control=my_control,
    start_stage="point_actions",
    start_point_index=0,
)
```

## 命令行调用

检测并保存轨迹：

```bash
cd /home/franka/massage/robots/fairino
python3 ft_agent_api.py detect --target back
```

检测、保存轨迹并执行完整动作：

```bash
cd /home/franka/massage/robots/fairino
python3 ft_agent_api.py run --target back --actions dian_jin,fen_jin,shun_jin
```

加载已有轨迹，只执行顺筋：

```bash
cd /home/franka/massage/robots/fairino
python3 ft_agent_api.py execute \
  --trajectory /home/franka/massage/robots/fairino/ft_locked_trajectory_output/back_trajectory_YYYYMMDD_HHMMSS.json \
  --actions shun_jin
```

命令行会输出 JSON，agent 可以解析其中的 `ok`、`trajectory`、`report`、`error` 字段。

## 支持的目标

| 参数 | 说明 |
| --- | --- |
| `back` | 背部膀胱经 |
| `leg` | 大腿外侧中线 |
| `leg_inner` | 大腿内侧 |

## 支持的动作

动作会按安全顺序归一化执行：点筋、分筋、顺筋。传入 `all` 或不传时执行完整序列。

| 动作 | 别名 |
| --- | --- |
| `dian_jin` | `点筋`、`point`、`press` |
| `fen_jin` | `分筋`、`split` |
| `shun_jin` | `顺筋`、`stroke`、`follow` |

## 主要接口

| 方法 | 说明 |
| --- | --- |
| `detect_meridian(save=True, display=False)` | 自动等待检测稳定，生成可执行轨迹，可保存 JSON |
| `load_trajectory(path)` | 从已保存 JSON 恢复轨迹 |
| `init_robot()` | 初始化 ROS 2 机械臂连接 |
| `execute_actions(actions, control=None, start_stage=None, start_point_index=0)` | 执行指定动作集合；会自动初始化机械臂；可由会话管理器传入暂停/恢复控制 |
| `run_workflow(...)` | 检测、保存轨迹、执行动作的一体化流程 |
| `close_vision()` | 关闭 RealSense pipeline |
| `close_robot()` | 关闭力控并释放机器人对象 |

## 语音智能体接入

`py-xiaozhi` 中新增了 FAIRINO 轨迹按摩 MCP 工具：

| 工具 | 语音意图示例 | 说明 |
| --- | --- | --- |
| `self.fairino_massage.detect` | “进行膀胱经检测”、“检测大腿内侧” | 后台启动检测，默认打开检测画面，检测稳定后保存轨迹 |
| `self.fairino_massage.start` | “开始按摩”、“开始全套按摩” | 使用当前轨迹后台执行动作 |
| `self.fairino_massage.shunjin` | “只做顺筋”、“检查顺筋效果” | 使用当前轨迹只执行顺筋，不执行点筋和分筋 |
| `self.fairino_massage.shun_jin` | “单独顺筋”、“测试顺筋贴合” | `shunjin` 的别名 |
| `self.fairino_massage.adjust_force` | “大力一些”、“小力一些” | 按摩运行中调整目标力度；默认每次增减 5N，不中断当前按摩 |
| `self.fairino_massage.pause` | “暂停按摩” | 在最近安全检查点暂停并保存状态 |
| `self.fairino_massage.resume` | “继续按摩” | 从上次暂停点继续 |
| `self.fairino_massage.continue` | “接着按摩”、“恢复按摩” | `resume` 的别名 |
| `self.fairino_massage.resume_massage` | “从暂停处继续” | `resume` 的别名 |
| `self.fairino_massage.stop` | “停止按摩” | 请求停止当前按摩任务 |
| `self.fairino_massage.status` | “现在按摩到哪里了” | 查询当前会话和进度 |

力度调整参数以方向为准：`direction=stronger` 表示增大，`direction=softer` 表示减小。用户说“减轻 10N”时应传 `direction=softer, delta_n=10`，底层会归一化为 `-10N`；用户说“增大 10N”时传 `direction=stronger, delta_n=10`，底层归一化为 `+10N`。

MCP `detect` 工具返回 `success=true` 只表示检测任务已启动，不表示轨迹已经保存。检测子进程完成后会写入 `status=detected`、`trajectory_path` 和 `point_count`。默认开启 `FAIRINO_MASSAGE_ANNOUNCE_DETECT_DONE=1` 时，运行时会请求小智再次调用 `self.fairino_massage.status`，由小智根据工具返回结果播报“检测已完成，轨迹已保存”。

`./massage gui` 启动前会清理旧小智 GUI 和旧 FAIRINO agent 子进程，并把状态重置为 `idle`。`idle` 在界面中显示为“就绪”，`stop` 或自然执行完成后才显示为 `stopped`。

会话状态默认保存到：

```text
/home/franka/massage/robots/fairino/ft_agent_state/current_session.json
```

控制文件默认保存到：

```text
/home/franka/massage/robots/fairino/ft_agent_state/current_control.json
```

状态中记录 `target`、`trajectory_path`、`actions`、`stage`、`current_action`、`current_point_index`、`current_repeat_index`、`current_step_index`、`resume_stage`、`resume_action`、`resume_point_index`、`resume_repeat_index`、`resume_step_index`、`robot_tcp_pose`、`robot_joints_deg`、`force_target_n`、`worker_pid` 等字段。

暂停采用软暂停策略：点筋/分筋会在动作检查点响应暂停，先退回当前按摩点贴近前的局部悬空位，再保存阶段、动作、点位、重复次数和步骤；力控顺筋会回到当前点悬空位后保存暂停点。继续按摩会从这些字段恢复，例如当前点筋已经完成 1 次，则恢复后只执行剩余点筋次数。紧急情况仍应使用现场物理急停。

停止按摩会先请求执行进程停止并回到当前点局部悬空位，随后通过顶层 `./massage home` 回到 `massage_home_pose.json` 记录的起始位置。这个回位流程可以用 `FAIRINO_MASSAGE_RETURN_HOME_ON_STOP=0` 关闭。自然执行完成后也会默认回到记录的起始位置，可用 `FAIRINO_MASSAGE_RETURN_HOME_ON_COMPLETE=0` 关闭。

运行中力度调整通过同一个控制文件传递，`self.fairino_massage.adjust_force` 会写入一次性 `force_adjust.seq`，执行进程在最近 checkpoint 消费并同步 `force_target_n` 到状态文件。默认步长由 `FAIRINO_MASSAGE_FORCE_ADJUST_STEP_N=5.0` 控制，底层目标力会被限制在 `FT_LIVE_FORCE_TARGET_MIN_N` 到 `FT_LIVE_FORCE_TARGET_MAX_N` 范围内。带数值调整时方向优先，`softer + 10` 会按 `-10N` 处理。

## 状态语义

| `status` | 含义 | 典型下一步 |
| --- | --- | --- |
| `idle` | 小智刚启动或已清理旧状态，等待指令 | 语音检测 |
| `detecting` | 检测子进程运行中 | 查看检测画面或查询状态 |
| `detected` | 轨迹已保存 | 开始按摩 |
| `running` | 按摩执行子进程运行中 | 暂停、停止、调整力度或查询状态 |
| `pausing` | 已收到暂停请求，等待最近检查点 | 等待进入 `paused` |
| `paused` | 已回到局部悬空位并保存恢复点 | 继续按摩或停止 |
| `stopping` | 正在停止或回起始位置 | 等待进入 `stopped` |
| `stopped` | 已停止；自然完成也归一化为停止 | 可重新检测或重新开始 |
| `error` | 检测或执行异常 | 查看 `last_result` 和日志 |

自然执行完成后外部状态写为 `stopped`，同时 `stage=completed`、`current_action=completed`，`status` 查询会返回 100% 进度；若自动回位成功，状态中会记录 `stop_home_result.ok=true`。这样 GUI 不会在重启或完成后误显示“仍在运行”。

## GUI 数据流

小智 GUI 的“FAIRINO 按摩机器人”界面读取同一份 `current_session.json`，展示连接状态、部位、轨迹、阶段、动作、点位、目标力度、进度、更新时间和消息。底部快捷按钮会把文字指令送入小智对话流程，由模型按工具描述调用 MCP 工具：

| 快捷按钮 | 对应意图 |
| --- | --- |
| 检测膀胱经 | 调用 `detect target=back` |
| 开始按摩 | 调用 `start actions=all` |
| 暂停 | 调用 `pause` |
| 继续 | 调用 `resume` |
| 停止 | 调用 `stop` |
| 查询状态 | 调用 `status` |

GUI 只负责展示和发起语音/文字意图，不直接控制机械臂。机械臂动作全部由 FAIRINO MCP 工具和 `ft_agent_process.py` 通过 ROS 2 执行。

## 标定和轨迹关系

`./massage calibrate` 会更新：

```text
/home/franka/massage/shared/calibration/camera_to_robot.json
```

`robots/fairino/camera_to_robot.json` 指向同一个共享文件。检测阶段会读取当前最新标定矩阵，把相机点转换成机器人坐标后保存到轨迹 JSON；执行阶段只加载轨迹中的 `frames[*].point_mm` 和 `points_mm`，不会重新读取标定矩阵重新计算旧轨迹。

因此，最新标定结果只对“标定之后重新检测生成的新轨迹”生效。重新标定后如果继续旧的暂停会话，或直接执行旧轨迹，仍会使用旧轨迹里保存的机器人坐标。

## 返回格式

成功示例：

```json
{
  "ok": true,
  "operation": "detect_meridian",
  "target": "back",
  "target_label": "背部膀胱经",
  "trajectory": {
    "trajectory_path": "/path/to/back_trajectory_YYYYMMDD_HHMMSS.json",
    "point_count": 10,
    "hover_height_mm": 20.0,
    "force_target_n": 30.0
  }
}
```

失败示例：

```json
{
  "ok": false,
  "operation": "detect_meridian",
  "target": "back",
  "error": "检测未成功或超时"
}
```

## 注意事项

- `ft_agent_api.py` 仍然会连接真实相机、真实 ROS 2 控制服务和真实机械臂；执行前必须做好急停和现场安全确认。
- `display=False` 适合 agent 后台调用；`display=True` 会弹出 OpenCV 检测窗口，并允许按 `s` 手动锁定。
- 腿部检测复用 `ft.py` 中已有 `capture_thigh_trajectory()`，接口层通过临时覆盖显示和超时参数实现非交互调用。
- 背部自动锁定逻辑在接口层实现，不需要修改 `ft.py`。
