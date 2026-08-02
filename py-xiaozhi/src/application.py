import asyncio
import json
import os
import re
import sys
import threading
import time
from pathlib import Path
from typing import Any, Awaitable

# 允许作为脚本直接运行：把项目根目录加入 sys.path（src 的上一级）
try:
    project_root = Path(__file__).resolve().parents[1]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
except Exception:
    pass

from src.constants.constants import DeviceState, ListeningMode
from src.plugins.calendar import CalendarPlugin
from src.plugins.iot import IoTPlugin
from src.plugins.manager import PluginManager
from src.plugins.mcp import McpPlugin
from src.plugins.shortcuts import ShortcutsPlugin
from src.plugins.ui import UIPlugin
from src.plugins.wake_word import WakeWordPlugin
from src.protocols.mqtt_protocol import MqttProtocol
from src.protocols.websocket_protocol import WebsocketProtocol
from src.utils.config_manager import ConfigManager
from src.utils.logging_config import get_logger
from src.utils.opus_loader import setup_opus

logger = get_logger(__name__)
setup_opus()

_CRITICAL_COMMAND_PREFIXES = (
    "请",
    "小智",
    "麻烦",
    "帮我",
    "给我",
    "现在",
    "马上",
    "立刻",
)
_CRITICAL_COMMAND_SUFFIXES = ("一下", "吧", "好吗", "好不好", "谢谢")
_STOP_MASSAGE_COMMANDS = {
    "停止",
    "停止按摩",
    "停止机械臂",
    "停止机械臂按摩",
    "结束按摩",
    "立即停止",
    "停下来",
    "别按了",
    "不要按了",
}
_PAUSE_MASSAGE_COMMANDS = {
    "暂停",
    "暂停按摩",
    "暂停机械臂",
    "暂停机械臂按摩",
    "先暂停",
    "先停一下",
    "停一下",
    "等一下",
    "别动",
}


def classify_critical_massage_command(text: str) -> str | None:
    """Return a local safety command only for a complete, explicit utterance."""
    normalized = re.sub(r"[\s，。！？、,.!?~～]+", "", str(text or "")).strip()
    if not normalized:
        return None

    changed = True
    while changed and normalized:
        changed = False
        for prefix in _CRITICAL_COMMAND_PREFIXES:
            if normalized.startswith(prefix) and len(normalized) > len(prefix):
                normalized = normalized[len(prefix) :]
                changed = True
                break

    if normalized in _STOP_MASSAGE_COMMANDS:
        return "stop"
    if normalized in _PAUSE_MASSAGE_COMMANDS:
        return "pause"

    changed = True
    while changed and normalized:
        changed = False
        for suffix in _CRITICAL_COMMAND_SUFFIXES:
            if normalized.endswith(suffix) and len(normalized) > len(suffix):
                normalized = normalized[: -len(suffix)]
                changed = True
                break

    if normalized in _STOP_MASSAGE_COMMANDS:
        return "stop"
    if normalized in _PAUSE_MASSAGE_COMMANDS:
        return "pause"
    return None


class Application:
    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = Application()
        return cls._instance

    def __init__(self):
        if Application._instance is not None:
            logger.error("尝试创建Application的多个实例")
            raise Exception("Application是单例类，请使用get_instance()获取实例")
        Application._instance = self

        logger.debug("初始化Application实例")

        # 配置
        self.config = ConfigManager.get_instance()

        # 状态
        self.running = False
        self.protocol = None

        # 设备状态（仅主程序改写，插件只读）
        self.device_state = DeviceState.IDLE
        try:
            aec_enabled_cfg = bool(self.config.get_config("AEC_OPTIONS.ENABLED", True))
        except Exception:
            aec_enabled_cfg = True
        self.aec_enabled = aec_enabled_cfg
        self.listening_mode = (
            ListeningMode.REALTIME if self.aec_enabled else ListeningMode.AUTO_STOP
        )
        self.keep_listening = False
        self._user_interaction_active = False
        self._assistant_response_active = False
        self._unsolicited_tts_blocked = False
        self._interaction_response_deadline = 0.0
        self._interaction_response_window_s = max(
            5.0,
            float(os.getenv("XIAOZHI_INTERACTION_RESPONSE_WINDOW_S", "30.0")),
        )
        self._critical_massage_lock: asyncio.Lock | None = None
        self._critical_massage_inflight: set[str] = set()
        self._critical_massage_last_at: dict[str, float] = {}

        # 统一任务池（替代 _main_tasks/_bg_tasks）
        self._tasks: set[asyncio.Task] = set()

        # 关停事件
        self._shutdown_event: asyncio.Event | None = None

        # 事件循环
        self._main_loop: asyncio.AbstractEventLoop | None = None

        # 并发控制
        self._state_lock: asyncio.Lock | None = None
        self._connect_lock: asyncio.Lock | None = None

        # 插件
        self.plugins = PluginManager()

    # -------------------------
    # 生命周期
    # -------------------------
    async def run(self, *, protocol: str = "websocket", mode: str = "gui") -> int:
        logger.info("启动Application，protocol=%s", protocol)
        try:
            self.running = True
            self._main_loop = asyncio.get_running_loop()
            self._initialize_async_objects()
            self._set_protocol(protocol)
            self._setup_protocol_callbacks()
            # 插件：setup（延迟导入AudioPlugin，确保上面setup_opus已执行）
            from src.plugins.audio import AudioPlugin

            # 按住说话专用模式不启动后台唤醒词，避免环境音误触发播报。
            massage_only = os.getenv(
                "XIAOZHI_MASSAGE_ONLY", "0"
            ).strip().lower() in {"1", "true", "yes", "on"}
            plugins = [McpPlugin(), IoTPlugin(), AudioPlugin()]
            if massage_only:
                logger.info("按摩专用模式已启用，日程提醒插件已禁用")
            else:
                plugins.append(CalendarPlugin())
            plugins.extend([UIPlugin(mode=mode), ShortcutsPlugin()])
            push_to_talk_only = os.getenv(
                "XIAOZHI_PUSH_TO_TALK_ONLY", "0"
            ).strip().lower() in {"1", "true", "yes", "on"}
            if push_to_talk_only:
                logger.info("按住说话专用模式已启用，后台唤醒词检测已禁用")
            else:
                plugins.insert(3, WakeWordPlugin())
            self.plugins.register(*plugins)
            await self.plugins.setup_all(self)
            # 启动后广播初始状态，确保 UI 就绪时能看到“待命”
            try:
                await self.plugins.notify_device_state_changed(self.device_state)
            except Exception:
                pass
            # await self.connect_protocol()
            # 插件：start
            await self.plugins.start_all()
            # 等待关停
            await self._wait_shutdown()
            return 0

        except Exception as e:
            logger.error(f"应用运行失败: {e}", exc_info=True)
            return 1
        finally:
            try:
                await self.shutdown()
            except Exception as e:
                logger.error(f"关闭应用时出错: {e}")

    async def connect_protocol(self):
        """
        确保协议通道打开并广播一次协议就绪。返回是否已打开。
        """
        # 已打开直接返回
        try:
            if self.is_audio_channel_opened():
                return True
            if not self._connect_lock:
                # 未初始化锁时，直接尝试一次
                opened = await asyncio.wait_for(
                    self.protocol.open_audio_channel(), timeout=12.0
                )
                if not opened:
                    logger.error("协议连接失败")
                    return False
                logger.info("协议连接已建立，按Ctrl+C退出")
                await self.plugins.notify_protocol_connected(self.protocol)
                return True

            async with self._connect_lock:
                if self.is_audio_channel_opened():
                    return True
                opened = await asyncio.wait_for(
                    self.protocol.open_audio_channel(), timeout=12.0
                )
                if not opened:
                    logger.error("协议连接失败")
                    return False
                logger.info("协议连接已建立，按Ctrl+C退出")
                await self.plugins.notify_protocol_connected(self.protocol)
                return True
        except asyncio.TimeoutError:
            logger.error("协议连接超时")
            return False

    def _initialize_async_objects(self) -> None:
        logger.debug("初始化异步对象")
        self._shutdown_event = asyncio.Event()
        self._state_lock = asyncio.Lock()
        self._connect_lock = asyncio.Lock()
        self._critical_massage_lock = asyncio.Lock()

    def _set_protocol(self, protocol_type: str) -> None:
        logger.debug("设置协议类型: %s", protocol_type)
        if protocol_type == "mqtt":
            self.protocol = MqttProtocol(asyncio.get_running_loop())
        else:
            self.protocol = WebsocketProtocol()

    # -------------------------
    # 手动聆听（按住说话）
    # -------------------------
    def authorize_user_interaction(self, source: str) -> None:
        self._user_interaction_active = True
        self._assistant_response_active = False
        self._unsolicited_tts_blocked = False
        self._interaction_response_deadline = (
            time.monotonic() + self._interaction_response_window_s
        )
        logger.info(f"已授权用户交互响应: source={source}")

    def finish_user_interaction(self, reason: str) -> None:
        if self._user_interaction_active or self._assistant_response_active:
            logger.info(f"用户交互响应结束: reason={reason}")
        self._user_interaction_active = False
        self._assistant_response_active = False
        self._interaction_response_deadline = 0.0

    def _response_is_authorized(self) -> bool:
        if self._assistant_response_active:
            return True
        return (
            self._user_interaction_active
            and time.monotonic() <= self._interaction_response_deadline
        )

    async def dispatch_local_critical_massage_command(
        self, text: str, source: str
    ) -> bool:
        """Run pause/stop locally so safety commands do not depend on the LLM."""
        command = classify_critical_massage_command(text)
        if command is None:
            return False

        if self._critical_massage_lock is None:
            self._critical_massage_lock = asyncio.Lock()

        now = time.monotonic()
        async with self._critical_massage_lock:
            last_at = self._critical_massage_last_at.get(command, 0.0)
            if command in self._critical_massage_inflight or now - last_at < 2.0:
                logger.info(
                    "[本地安全路由] 忽略重复%s指令: source=%s text=%r",
                    "停止" if command == "stop" else "暂停",
                    source,
                    text,
                )
                return True
            self._critical_massage_inflight.add(command)
            self._critical_massage_last_at[command] = now

        command_label = "停止" if command == "stop" else "暂停"
        logger.warning(
            "[本地安全路由] 命中%s按摩指令，直接调用本地工具: source=%s text=%r",
            command_label,
            source,
            text,
        )
        self.set_chat_message("assistant", f"已在本机执行{command_label}按摩指令")

        try:
            from src.mcp.tools.fairino_massage.tools import (
                pause_massage,
                stop_massage,
            )

            raw_result = (
                await stop_massage({})
                if command == "stop"
                else await pause_massage({})
            )
            try:
                result = json.loads(raw_result)
            except (TypeError, ValueError, json.JSONDecodeError):
                result = {"success": False, "message": str(raw_result)}

            logger.warning(
                "[本地安全路由] %s工具执行完成: success=%s message=%s",
                command_label,
                result.get("success"),
                result.get("message"),
            )
            if result.get("message"):
                self.set_chat_message("assistant", str(result["message"]))
            return True
        except Exception as exc:
            logger.error(
                "[本地安全路由] %s工具执行失败: %s",
                command_label,
                exc,
                exc_info=True,
            )
            self.set_chat_message(
                "assistant", f"{command_label}按摩工具执行失败：{exc}"
            )
            return True
        finally:
            async with self._critical_massage_lock:
                self._critical_massage_inflight.discard(command)

    async def start_listening_manual(self) -> None:
        try:
            ok = await self.connect_protocol()
            if not ok:
                return
            self.keep_listening = False
            self.finish_user_interaction("manual-listening-started")
            self.authorize_user_interaction("manual-input-start")

            # 如果说话中发送打断
            if self.device_state == DeviceState.SPEAKING:
                logger.info("说话中发送打断")
                await self.protocol.send_abort_speaking(None)
                await self.set_device_state(DeviceState.IDLE)
            await self.protocol.send_start_listening(ListeningMode.MANUAL)
            await self.set_device_state(DeviceState.LISTENING)
        except Exception:
            pass

    async def stop_listening_manual(self) -> None:
        try:
            self.authorize_user_interaction("manual-input-stop")
            await self.protocol.send_stop_listening()
            await self.set_device_state(DeviceState.IDLE)
        except Exception:
            pass

    # -------------------------
    # 自动/实时对话：根据 AEC 与当前配置选择模式，开启保持会话
    # -------------------------
    async def start_auto_conversation(self) -> None:
        try:
            ok = await self.connect_protocol()
            if not ok:
                return

            self.authorize_user_interaction("auto-conversation")
            mode = (
                ListeningMode.REALTIME if self.aec_enabled else ListeningMode.AUTO_STOP
            )
            self.listening_mode = mode
            self.keep_listening = True
            await self.protocol.send_start_listening(mode)
            await self.set_device_state(DeviceState.LISTENING)
        except Exception:
            pass

    async def start_wake_word_conversation(self) -> None:
        """唤醒词触发的一次性自动对话。

        与界面“自动对话”不同，这里不保持持续监听；完成一轮指令和回复后回到待命，
        由本地唤醒词继续过滤环境声音。
        """
        try:
            ok = await self.connect_protocol()
            if not ok:
                return

            self.authorize_user_interaction("wake-word")
            self.listening_mode = ListeningMode.AUTO_STOP
            self.keep_listening = False
            await self.protocol.send_start_listening(ListeningMode.AUTO_STOP)
            await self.set_device_state(DeviceState.LISTENING)
        except Exception:
            pass
    def _setup_protocol_callbacks(self) -> None:
        self.protocol.on_network_error(self._on_network_error)
        self.protocol.on_incoming_json(self._on_incoming_json)
        self.protocol.on_incoming_audio(self._on_incoming_audio)
        self.protocol.on_audio_channel_opened(self._on_audio_channel_opened)
        self.protocol.on_audio_channel_closed(self._on_audio_channel_closed)

    async def _wait_shutdown(self) -> None:
        await self._shutdown_event.wait()

    # -------------------------
    # 统一任务管理（精简）
    # -------------------------
    def spawn(self, coro: Awaitable[Any], name: str) -> asyncio.Task:
        """
        创建任务并登记，关停时统一取消。
        """
        if not self.running or (self._shutdown_event and self._shutdown_event.is_set()):
            logger.debug(f"跳过任务创建（应用正在关闭）: {name}")
            return None
        task = asyncio.create_task(coro, name=name)
        self._tasks.add(task)

        def _done(t: asyncio.Task):
            self._tasks.discard(t)
            if not t.cancelled() and t.exception():
                logger.error(f"任务 {name} 异常结束: {t.exception()}", exc_info=True)

        task.add_done_callback(_done)
        return task

    def schedule_command_nowait(self, fn, *args, **kwargs) -> None:
        """简化的“立即调度”：把任意可调用丢回主loop执行。

        - 若返回协程，会被自动创建子任务执行（fire-and-forget）。
        - 若是同步函数，直接在事件循环线程里运行（尽量保持轻量）。
        """
        if not self._main_loop or self._main_loop.is_closed():
            logger.warning("主事件循环未就绪，拒绝调度")
            return

        def _runner():
            try:
                res = fn(*args, **kwargs)
                if asyncio.iscoroutine(res):
                    self.spawn(res, name=f"call:{getattr(fn, '__name__', 'anon')}")
            except Exception as e:
                logger.error(f"调度的可调用执行失败: {e}", exc_info=True)

        # 确保在事件循环线程里执行
        self._main_loop.call_soon_threadsafe(_runner)

    # -------------------------
    # 协议回调
    # -------------------------
    def _on_network_error(self, error_message=None):
        if error_message:
            logger.error(error_message)

        self.keep_listening = False
        # 出错即请求关闭
        # if self._shutdown_event and not self._shutdown_event.is_set():
        #     self._shutdown_event.set()

    def _on_incoming_audio(self, data: bytes):
        if self._unsolicited_tts_blocked or not self._assistant_response_active:
            logger.debug("忽略未由用户交互授权的音频响应")
            return
        logger.debug(f"收到二进制消息，长度: {len(data)}")
        # 转发给插件
        self.spawn(self.plugins.notify_incoming_audio(data), "plugin:on_audio")

    def _on_incoming_json(self, json_data):
        try:
            msg_type = json_data.get("type") if isinstance(json_data, dict) else None
            logger.info(f"收到JSON消息: type={msg_type}")
            if msg_type == "stt":
                stt_text = str(json_data.get("text") or "").strip()
                logger.info("STT文本: %r", stt_text)
                if self.keep_listening and stt_text:
                    self.authorize_user_interaction("continuous-stt")
                elif not self._response_is_authorized():
                    logger.warning("忽略未由用户交互授权的 STT")
                    self.spawn(
                        self.protocol.send_abort_speaking(None),
                        "abort:unsolicited_stt",
                    )
                    return
                if classify_critical_massage_command(stt_text):
                    self.spawn(
                        self.dispatch_local_critical_massage_command(
                            stt_text, "stt"
                        ),
                        "local-safety:stt",
                    )

            if msg_type == "mcp":
                payload = json_data.get("payload") or {}
                if (
                    isinstance(payload, dict)
                    and payload.get("method") == "tools/call"
                    and not self._response_is_authorized()
                ):
                    logger.warning("阻止未由用户交互授权的 MCP 工具调用")
                    self.spawn(
                        self.protocol.send_abort_speaking(None),
                        "abort:unsolicited_mcp",
                    )
                    return

            # 将 TTS start/stop 映射为设备状态（支持自动/实时，且不污染手动模式）
            if msg_type == "tts":
                state = json_data.get("state")
                tts_text = str(json_data.get("text") or "").strip()
                if tts_text:
                    logger.info("TTS文本: %r", tts_text)
                if self._unsolicited_tts_blocked:
                    if state == "stop":
                        self._unsolicited_tts_blocked = False
                        logger.info("未授权 TTS 已结束，解除音频丢弃状态")
                    return
                if (
                    not self._response_is_authorized()
                    and not self._assistant_response_active
                ):
                    self._unsolicited_tts_blocked = True
                    logger.warning(
                        f"阻止未由用户交互授权的 TTS: state={state}"
                    )
                    self.spawn(
                        self.protocol.send_abort_speaking(None),
                        "abort:unsolicited_tts",
                    )
                    return
                if state == "start":
                    self._assistant_response_active = True
                    # 仅当保持会话且实时模式时，TTS开始期间保持LISTENING；否则显示SPEAKING
                    if (
                        self.keep_listening
                        and self.listening_mode == ListeningMode.REALTIME
                    ):
                        self.spawn(
                            self.set_device_state(DeviceState.LISTENING),
                            "state:tts_start_rt",
                        )
                    else:
                        self.spawn(
                            self.set_device_state(DeviceState.SPEAKING),
                            "state:tts_start_speaking",
                        )
                elif state == "stop":
                    if self.keep_listening:
                        # 继续对话：根据当前模式重启监听
                        async def _restart_listening():
                            try:
                                # REALTIME 且已在 LISTENING 时无需重复发送
                                if not (
                                    self.listening_mode == ListeningMode.REALTIME
                                    and self.device_state == DeviceState.LISTENING
                                ):
                                    await self.protocol.send_start_listening(
                                        self.listening_mode
                                    )
                            except Exception:
                                pass
                            self.keep_listening and await self.set_device_state(
                                DeviceState.LISTENING
                            )

                        self.spawn(_restart_listening(), "state:tts_stop_restart")
                    else:
                        self.spawn(
                            self.set_device_state(DeviceState.IDLE),
                            "state:tts_stop_idle",
                        )
            # 转发给插件
            self.spawn(self.plugins.notify_incoming_json(json_data), "plugin:on_json")
            if msg_type == "tts" and json_data.get("state") == "stop":
                self.finish_user_interaction("tts-stop")
        except Exception:
            logger.info("收到JSON消息")

    async def _on_audio_channel_opened(self):
        logger.info("协议通道已打开")
        # 通道打开后进入 LISTENING（：简化为直读直写）
        await self.set_device_state(DeviceState.LISTENING)

    async def _on_audio_channel_closed(self):
        logger.info("协议通道已关闭")
        # 通道关闭回到 IDLE
        await self.set_device_state(DeviceState.IDLE)

    async def set_device_state(self, state: DeviceState):
        """
        仅供主程序内部调用：设置设备状态。插件请只读获取。
        """
        # print(f"set_device_state: {state}")
        if not self._state_lock:
            self.device_state = state
            try:
                await self.plugins.notify_device_state_changed(state)
            except Exception:
                pass
            return
        async with self._state_lock:
            if self.device_state == state:
                return
            logger.info(f"设置设备状态: {state}")
            self.device_state = state
        # 锁外广播，避免插件回调引起潜在的长耗时阻塞
        try:
            await self.plugins.notify_device_state_changed(state)
            if state == DeviceState.LISTENING:
                await asyncio.sleep(0.5)
                self.aborted = False
        except Exception:
            pass

    # -------------------------
    # 只读访问器（提供给插件使用）
    # -------------------------
    def get_device_state(self):
        return self.device_state

    def is_idle(self) -> bool:
        return self.device_state == DeviceState.IDLE

    def is_listening(self) -> bool:
        return self.device_state == DeviceState.LISTENING

    def is_speaking(self) -> bool:
        return self.device_state == DeviceState.SPEAKING

    def get_listening_mode(self):
        return self.listening_mode

    def is_keep_listening(self) -> bool:
        return bool(self.keep_listening)

    def is_audio_channel_opened(self) -> bool:
        try:
            return bool(self.protocol and self.protocol.is_audio_channel_opened())
        except Exception:
            return False

    def get_state_snapshot(self) -> dict:
        return {
            "device_state": self.device_state,
            "listening_mode": self.listening_mode,
            "keep_listening": bool(self.keep_listening),
            "audio_opened": self.is_audio_channel_opened(),
        }

    async def abort_speaking(self, reason):
        """
        中止语音输出.
        """

        if self.aborted:
            logger.debug(f"已经中止，忽略重复的中止请求: {reason}")
            return

        logger.info(f"中止语音输出，原因: {reason}")
        self.aborted = True
        await self.protocol.send_abort_speaking(reason)
        await self.set_device_state(DeviceState.IDLE)

    # -------------------------
    # UI 辅助：供插件或工具直接调用
    # -------------------------
    def set_chat_message(self, role, message: str) -> None:
        """将文本更新转发为 UI 可识别的 JSON 消息（复用 UIPlugin 的 on_incoming_json）。
        role: "assistant" | "user" 影响消息类型映射。
        """
        try:
            msg_type = "tts" if str(role).lower() == "assistant" else "stt"
        except Exception:
            msg_type = "tts"
        payload = {"type": msg_type, "text": message}
        # 通过插件事件总线异步派发
        self.spawn(self.plugins.notify_incoming_json(payload), "ui:text_update")

    def set_emotion(self, emotion: str) -> None:
        """
        设置情绪表情：通过 UIPlugin 的 on_incoming_json 路由。
        """
        payload = {"type": "llm", "emotion": emotion}
        self.spawn(self.plugins.notify_incoming_json(payload), "ui:emotion_update")

    # -------------------------
    # 关停
    # -------------------------
    async def shutdown(self):
        if not self.running:
            return
        logger.info("正在关闭Application...")
        self.running = False

        if self._shutdown_event is not None:
            self._shutdown_event.set()

        try:
            # 取消所有登记任务
            if self._tasks:
                for t in list(self._tasks):
                    if not t.done():
                        t.cancel()
                await asyncio.gather(*self._tasks, return_exceptions=True)
                self._tasks.clear()

            # 关闭协议（限时，避免阻塞退出）
            if self.protocol:
                try:
                    try:
                        self._main_loop.create_task(self.protocol.close_audio_channel())
                    except asyncio.TimeoutError:
                        logger.warning("关闭协议超时，跳过等待")
                except Exception as e:
                    logger.error(f"关闭协议失败: {e}")

            # 插件：stop/shutdown
            try:
                await self.plugins.stop_all()
            except Exception:
                pass
            try:
                await self.plugins.shutdown_all()
            except Exception:
                pass

            logger.info("Application 关闭完成")
        except Exception as e:
            logger.error(f"关闭应用时出错: {e}", exc_info=True)
