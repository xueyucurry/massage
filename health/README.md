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

本目录现在包含全部手环项目代码、私有配置、数据库、文档和 MQTT 部署模板。`py-xiaozhi` 中不再保留接收器、SQLite、查询或 MCP 手环代码。

唯一外部依赖是运行 `./health setup` 后在本目录生成的 `.venv/`，其中安装 `paho-mqtt`。云端 Mosquitto 服务继续运行在 `141.147.29.128`，其服务器配置不在本工作区。

如果未来重新需要小智 GUI 联动，应新建一个明确的适配层读取 `health/data/health_band.sqlite3`；本次迁移不保留该联动代码。

迁移前的数据库已在逐条核对记录数和最后接收时间一致后删除；`health/data/health_band.sqlite3` 是当前唯一的手环数据存储位置。
