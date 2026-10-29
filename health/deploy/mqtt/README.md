# Mosquitto 部署模板

这是新建服务器时使用的 Docker 部署模板。当前生产服务器已直接通过 Ubuntu 软件包部署 Mosquitto，不需要重跑本模板。

创建账号文件后启动：

```bash
docker run --rm -it -v "$PWD:/work" eclipse-mosquitto:2 \
  mosquitto_passwd -c /work/passwords band01
docker run --rm -it -v "$PWD:/work" eclipse-mosquitto:2 \
  mosquitto_passwd /work/passwords xiaozhi
docker compose up -d
```

手环使用 `band01` 发布到 `b/vitals`；本机接收器使用 `xiaozhi` 订阅同一 Topic。账号密码应不同，且长度小于 15 个字符。不要提交 `passwords` 文件。
