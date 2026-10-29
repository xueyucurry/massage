#!/usr/bin/env python3
"""MQTT receiver for T10-4G health-band reports."""

import argparse
import json
import logging
import signal
import sys
from pathlib import Path
from typing import Any, Dict

from storage import HealthStore, normalise_mac


PROJECT_ROOT = Path(__file__).resolve().parent
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)
logger = logging.getLogger("health.receiver")


def load_config(path: Path) -> Dict[str, Any]:
    try:
        config = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise SystemExit(f"找不到配置文件: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"配置文件不是有效 JSON: {exc}") from exc
    if not isinstance(config.get("mqtt"), dict):
        raise SystemExit("配置文件缺少 mqtt 对象")
    return config


class Receiver:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        database_path = config.get("database_path") or None
        if database_path and not Path(database_path).is_absolute():
            database_path = PROJECT_ROOT / database_path
        self.store = HealthStore(database_path)
        self.allowed_devices = {
            normalise_mac(item) for item in config.get("allowed_devices", []) if normalise_mac(item)
        }
        self.allow_unregistered = bool(config.get("allow_unregistered", False))

    def handle_message(self, topic: str, payload_bytes: bytes) -> None:
        try:
            payload = json.loads(payload_bytes.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            logger.warning("忽略非 JSON 上报: %s", exc)
            return
        if not isinstance(payload, dict):
            logger.warning("忽略非对象 JSON 上报")
            return
        device_mac = normalise_mac(payload.get("mac"))
        if not self.allow_unregistered and device_mac not in self.allowed_devices:
            logger.warning("拒绝未登记手环数据，mac=%s", device_mac or "未知")
            return
        try:
            if self.store.record_sample(payload, topic):
                logger.info("已保存手环数据，mac=%s", device_mac)
            else:
                logger.info("忽略重复手环数据，mac=%s", device_mac)
        except (TypeError, ValueError) as exc:
            logger.warning("忽略格式无效的手环数据：%s", exc)

    def run(self) -> None:
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise SystemExit("缺少 paho-mqtt；请执行 ./health setup") from exc

        mqtt_config = self.config["mqtt"]
        host = str(mqtt_config.get("host") or "").strip()
        topic = str(mqtt_config.get("topic") or "").strip()
        port = int(mqtt_config.get("port", 1883))
        if not host or not topic:
            raise SystemExit("mqtt.host 和 mqtt.topic 不能为空")

        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        if mqtt_config.get("username"):
            client.username_pw_set(str(mqtt_config["username"]), str(mqtt_config.get("password") or ""))
        if mqtt_config.get("tls", False):
            client.tls_set(ca_certs=mqtt_config.get("ca_cert") or None)

        def on_connect(client, userdata, flags, reason_code, properties):
            if reason_code.is_failure:
                logger.error("MQTT 连接失败：%s", reason_code)
                return
            result, _ = client.subscribe(topic, qos=1)
            if result == mqtt.MQTT_ERR_SUCCESS:
                logger.info("已订阅 %s", topic)
            else:
                logger.error("订阅失败：%s", result)

        client.on_connect = on_connect
        client.on_message = lambda _client, _userdata, message: self.handle_message(message.topic, message.payload)
        client.reconnect_delay_set(min_delay=1, max_delay=60)
        logger.info("正在连接 MQTT Broker %s:%s", host, port)
        client.connect_async(host, port, keepalive=60)
        client.loop_forever(retry_first_connection=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="接收并保存 4G 手环数据")
    parser.add_argument("--config", default="health.json", help="接收器配置文件")
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    Receiver(load_config(Path(args.config).expanduser())).run()


if __name__ == "__main__":
    main()
