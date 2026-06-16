# ft_agent_api.py 接口说明

`ft_agent_api.py` 是对 `ft.py` 的独立封装层，不修改 `ft.py` 本体。它面向 agent 或自动化脚本，提供经络检测、轨迹保存、轨迹加载，以及点筋、分筋、顺筋动作执行接口。

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
| `self.fairino_massage.detect` | “进行膀胱经检测”、“检测大腿内侧” | 调用检测并保存轨迹 |
| `self.fairino_massage.start` | “开始按摩”、“只做顺筋” | 使用当前轨迹后台执行动作 |
| `self.fairino_massage.pause` | “暂停按摩” | 在最近安全检查点暂停并保存状态 |
| `self.fairino_massage.resume` | “继续按摩” | 从上次暂停点继续 |
| `self.fairino_massage.stop` | “停止按摩” | 请求停止当前按摩任务 |
| `self.fairino_massage.status` | “现在按摩到哪里了” | 查询当前会话和进度 |

会话状态默认保存到：

```text
/home/franka/massage/robots/fairino/ft_agent_state/current_session.json
```

状态中记录 `target`、`trajectory_path`、`actions`、`stage`、`current_point_index`、`resume_stage`、`resume_point_index`、`robot_tcp_pose`、`robot_joints_deg` 等字段。暂停采用软暂停策略：点筋/分筋会在点位动作之间暂停；力控顺筋会先回到悬空位，再保存暂停点。紧急情况仍应使用现场物理急停。

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
