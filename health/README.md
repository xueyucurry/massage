# 4G 手环本地接收与查询

本目录独立接收 T10-4G 手环的 MQTT 数据，并保存、查询心率、血氧、体温和电量。它不依赖小智 GUI 或 `py-xiaozhi` 的任何代码。

## 日常命令

首次准备虚拟环境和唯一依赖：

```bash
cd /home/franka/massage/health
./health setup
```

后台启动接收器：

```bash
./health start
```

查询最新值、状态和历史：

```bash
./health
./health status
./health history
```

查看接收日志或停止接收器：

```bash
./health logs
./health stop
./health receiver-status
```

按保留周期清理旧数据（显式执行，示例保留最近 90 天）：

```bash
./health prune --days 90
```

接收器在后台运行；终端和 GUI 都可以关闭。重启电脑后重新执行 `./health start`。

## 当前配置

真实配置在 `health.json`，它不会纳入 Git。当前已配置：

- 云端 MQTT Broker：`141.147.29.128:1883`
- Topic：`b/vitals`
- 本机订阅账号：`xiaozhi`
- 已登记手环 MAC：`4754120002D6`
- 未登记手环放行：关闭

不要将 `health.json`、密码、私钥或数据库提交到仓库或发送到聊天中。

## iPhone 手环参数

在 `RLBeacon Tool` 中连接 `T10_4G`，默认连接密码为 `202012`。读取旧值后填写：

| 参数 | 值 |
|---|---|
| 网络协议 | `3`（MQTT） |
| Host | `141.147.29.128` |
| 端口 | `1883` |
| Topic | `b/vitals` |
| 用户名 | `band01` |
| 密码 | `band01` 的私有 MQTT 密码 |
| 上传间隔 | App 中填 `30`；当前实测约每 30 秒上报 |

保存后再次读取并逐项核对。过滤名称、过滤 MAC、RSSI 均不用于本项目。

## 数据与安全

- 数据库：`data/health_band.sqlite3`。
- 数据库内部以 UTC 保存时间；`./health`、`./health status` 和 `./health history` 会自动显示北京时间。
- 数据是原始测量值，不用于医疗诊断。
- **上传间隔单位说明**：App 界面标注为“分钟”，但当前手环固件实测按“秒”执行相同数值；例如填 `5` 实际约每 5 秒上报，填 `30` 实际约每 30 秒上报。当前配置值为 `30`。
- MQTT TLS 支持尚未由规格书确认；公网链路应尽快确认是否可启用 TLS，并使用强密码。

## 与其他目录的关系

本目录包含手环接收器、私有配置、数据库、文档和 MQTT 部署模板。`py-xiaozhi` 不包含接收器或 MQTT 凭据；仅保留一个只读 SQLite 展示/查询适配层，不复制手环数据。

唯一外部依赖是运行 `./health setup` 后在本目录生成的 `.venv/`，其中安装 `paho-mqtt`。云端 Mosquitto 服务继续运行在 `141.147.29.128`，其服务器配置不在本工作区。

小智 GUI 联动现通过明确的只读适配层读取 `health/data/health_band.sqlite3`；接收器与 MQTT 部署仍只由本目录维护。

迁移前的数据库已在逐条核对记录数和最后接收时间一致后删除；`health/data/health_band.sqlite3` 是当前唯一的手环数据存储位置。

## 小智只读展示、质量提示与隐私

`py-xiaozhi` 现在通过只读 SQLite 适配层读取本目录的 `data/health_band.sqlite3`；它不会订阅 MQTT、写入数据库或读取 `health.json` 中的账号密码。默认路径由小智代码自动定位，也可在启动小智前设置 `HEALTH_BAND_DB=/绝对路径/health_band.sqlite3` 覆盖。

- 推荐组合：手环每约 30 秒上传，GUI 最右侧独立的“健康监测”栏每 10 秒只读刷新一次，显示心率、血氧、体温、电量、最后接收时间和数据新鲜度；原“任务详情”栏保持独立。
- GUI 启动或点击“启动接收”后会开启新的实时显示会话：历史记录不会先显示，界面先提示“正在接收数据”，收到该会话的首条上报后才显示体征、开始 90 秒超时判断并累计趋势。SQLite 历史数据不会被删除，仍可用于历史查询。
- GUI 的健康卡片可直接“启动”或“停止”本机接收器，并显示进程状态；两个按钮仅调用固定的本地 `health start` / `health stop`，不会执行任意终端命令。关闭 GUI 不会停止已启动的接收器。
- 小智 MCP 已提供 `self.health_band.latest`、`self.health_band.history`、`self.health_band.status`、`self.health_band.assess`，用于语音查询最新值、历史数据、上报状态和数据完整性。
- 当前异常提示只检查“未收到数据、接收时间无法识别、超过 90 秒无新数据、测量字段缺失”；不对心率、血氧或体温作医学诊断。
- 质量状态还会提示手环低电量、最近上传间隔明显偏长，以及心率/血氧/体温为 `0` 等明显无效的传感器值。无效值在小智中显示为 `--`、不计入趋势；单条无效提示等待下一次上传，连续 3 条无效才提示检查佩戴或手环状态。
- 健康卡片显示最近 10 分钟的心率、血氧、体温趋势摘要；没有足够有效样本时显示“等待更多数据”。
- 个人阈值规则为可选项：复制 `health_alert_policy.example.json` 为未纳入 Git 的 `health_alert_policy.json` 后，再填写经确认的阈值。默认不启用心率、血氧、体温阈值；一旦启用，需连续 3 条记录越界才提示，仍不作诊断或控制按摩。
- 此阶段**不会**因手环数据自动暂停、停止或启动按摩。后续若要健康联锁，须单独确认个人阈值、连续异常次数、人工解除流程，并在 FAIRINO 执行器的安全检查点实现，不能由 GUI 直接控制机械臂。

如手环当前约每 30 秒上报，超过 90 秒无新数据会显示“数据超时”（约连续漏掉 3 次上报，能容忍短暂网络抖动）。该时间可在 MCP 查询 `status` 或 `assess` 时用 `stale_after_seconds` 调整（范围 30–3600 秒）。

## 数据保留与隐私

- 数据库只保存在本机；小智 GUI 不显示设备 MAC，MCP 查询也会自动脱敏 MAC。SIM 卡号从不返回给 GUI 或 MCP。
- 不会自动删除记录。需要释放空间时，由操作者显式运行 `./health prune --days 90`；该操作会永久删除超过保留天数的本地记录，请先自行备份。
- 建议先确认合适的保留期限（例如 90 天），并将数据库备份放在受访问控制的位置。

## 下次续作快速检查

1. 启动 GUI 后，健康栏应先显示“等待数据”，而不是旧数据库的体征；等本次会话首条上报（通常约 30–40 秒）后才显示实时数据。
2. 若手环上传心率/血氧/体温的 `0`，对应值应显示 `--` 和橙色数据质量提示；下一条有效数据会自动恢复。原始 `0` 仍可在 `./health history` 中看到。
3. 修改相关代码后运行：

   ```bash
   cd /home/franka/massage/py-xiaozhi
   PYTHONPATH=. .venv-linux/bin/python -m unittest test_health_band_reader.py

   cd /home/franka/massage/health
   .venv/bin/python -m unittest test_storage.py
   bash -n health
   ```

4. 小智 GUI 健康相关代码在 `py-xiaozhi/src/health_band/` 与 `py-xiaozhi/src/display/gui_display*.py/qml`；接收器与数据维护代码仍只在本目录。
