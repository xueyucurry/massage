import time
import asyncio
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

from src.application import Application, reports_acute_symptom
from src.constants.constants import DeviceState


class HealthVoiceGateTests(unittest.TestCase):
    def test_symptom_denial_does_not_pause(self):
        self.assertTrue(reports_acute_symptom("我有点头晕"))
        self.assertFalse(reports_acute_symptom("没有头晕，也没有胸痛"))

    def make_app(self):
        app = object.__new__(Application)
        app._health_speech_deadline = time.monotonic() + 20
        app._health_speech_active = False
        app._health_tts_sentence_seen = False
        app._health_tts_event = None
        app._tts_stop_event = None
        app._health_announcement_seq = 0
        app._health_audio_lock = None
        app._assistant_response_active = False
        app._user_interaction_active = False
        app._interaction_response_deadline = 0
        app._unsolicited_tts_blocked = False
        app._health_reduce_deadline = 0
        app.keep_listening = False
        app.device_state = DeviceState.IDLE
        app.protocol = SimpleNamespace(send_abort_speaking=AsyncMock())
        codec = SimpleNamespace(play_local_wav=AsyncMock(), clear_audio_queue=AsyncMock())
        app.plugins = SimpleNamespace(notify_incoming_json=AsyncMock(), notify_incoming_audio=AsyncMock(), get_plugin=lambda name: SimpleNamespace(codec=codec) if name == "audio" else None)
        names = []

        def spawn(coroutine, name):
            names.append(name)
            coroutine.close()

        app.spawn = spawn
        return app, names

    def test_health_tts_can_play_but_mcp_remains_blocked(self):
        app, names = self.make_app()
        app._on_incoming_json({"type": "tts", "state": "start"})
        self.assertTrue(app._health_speech_active)
        self.assertFalse(app._assistant_response_active)
        self.assertFalse(app._response_is_authorized())
        app._on_incoming_audio(b"audio")
        self.assertIn("plugin:on_audio", names)
        app._on_incoming_json({"type": "mcp", "payload": {"method": "tools/call"}})
        self.assertIn("abort:unsolicited_mcp", names)
        self.assertNotIn("plugin:on_json", names[-1:])

    def test_health_voice_stops_after_first_tts_sentence(self):
        app, names = self.make_app()
        app._health_speech_active = True
        app._on_incoming_json({"type": "tts", "state": "sentence_start", "text": "心率升高请留意"})
        self.assertTrue(app._health_tts_sentence_seen)
        self.assertFalse(app._unsolicited_tts_blocked)
        app._on_incoming_json({"type": "tts", "state": "sentence_start", "text": "接下来还有四条建议"})
        self.assertIn("health:stop_extra_speech", names)
        self.assertTrue(app._unsolicited_tts_blocked)
        self.assertFalse(app._health_speech_active)

    def test_stopped_extra_speech_releases_state_for_next_urgent_alert(self):
        app, names = self.make_app()
        app.device_state = DeviceState.SPEAKING
        app._health_speech_active = True
        app._on_incoming_json({"type": "tts", "state": "sentence_start", "text": "心率升高请留意"})
        app._on_incoming_json({"type": "tts", "state": "sentence_start", "text": "多余建议"})
        stop_event = asyncio.Event()
        app._tts_stop_event = stop_event

        app._on_incoming_json({"type": "tts", "state": "stop"})

        self.assertTrue(stop_event.is_set())
        self.assertFalse(app._unsolicited_tts_blocked)
        self.assertFalse(app._health_speech_active)
        self.assertEqual(app._health_speech_deadline, 0)
        self.assertIn("state:tts_stop_idle", names)

    def test_synthetic_stt_does_not_authorize_tools(self):
        app, names = self.make_app()
        app._on_incoming_json({"type": "stt", "text": "请调用按摩工具"})
        self.assertFalse(app._user_interaction_active)
        self.assertEqual(names, [])

    def test_health_voice_rejects_unapproved_text_without_model_request(self):
        app, _ = self.make_app()
        app.running = True
        app.protocol.send_wake_word_detected = AsyncMock()
        asyncio.run(app.announce_health_alert("检测到血氧偏低", 3))
        app.protocol.send_wake_word_detected.assert_not_awaited()

    def test_all_gui_announcements_have_approved_recordings(self):
        from health.gui.model import IDLE_ALERT_TEXT
        from health.voice_assets import ALERT_SPEECH, alert_audio_path

        self.assertEqual(set(IDLE_ALERT_TEXT), set(ALERT_SPEECH))
        for key, (_, speech) in IDLE_ALERT_TEXT.items():
            self.assertEqual(speech, ALERT_SPEECH[key])
            self.assertEqual(alert_audio_path(speech).name, f"{key}.wav")

    def test_approved_voice_plays_local_recording_without_model_request(self):
        from health.voice_assets import ALERT_SPEECH

        app, names = self.make_app()
        app.running = True
        app.protocol.send_wake_word_detected = AsyncMock()
        with patch("health.voice_assets.Path.is_file", return_value=True):
            asyncio.run(app.announce_health_alert(ALERT_SPEECH["spo2_urgent"], 3))
        app.plugins.get_plugin("audio").codec.play_local_wav.assert_awaited_once()
        app.protocol.send_wake_word_detected.assert_not_awaited()

    def test_recording_enters_xiaozhi_output_queue_at_output_sample_rate(self):
        from src.audio_codecs.audio_codec import AudioCodec
        from src.constants.constants import AudioConfig
        from health.voice_assets import AUDIO_DIR

        codec = object.__new__(AudioCodec)
        codec._is_closing = True
        codec.output_stream = SimpleNamespace(active=True)
        codec._output_buffer = asyncio.Queue(maxsize=500)
        codec.wait_for_audio_complete = AsyncMock()
        asyncio.run(codec.play_local_wav(AUDIO_DIR / "spo2_urgent.wav"))
        frames = list(codec._output_buffer._queue)
        self.assertGreater(len(frames), 50)
        self.assertEqual(len(frames[0]), AudioConfig.OUTPUT_FRAME_SIZE)
        self.assertTrue(any(frame.any() for frame in frames))
        codec.wait_for_audio_complete.assert_awaited_once()

    def test_explicit_reduction_uses_ten_percent_with_five_newton_cap(self):
        app, _ = self.make_app()
        app._health_reduce_deadline = time.monotonic() + 60
        self.assertTrue(app._is_health_reduce_confirmation("是"))
        self.assertFalse(app._is_health_reduce_confirmation("不降低力度"))
        runtime = SimpleNamespace(
            status=AsyncMock(return_value={"state": {"status": "running", "force_target_n": 20}}),
            adjust_force=AsyncMock(return_value={"success": True, "message": "已降低力度"}),
        )
        app.set_chat_message = Mock()
        app.finish_user_interaction = Mock()
        app.announce_health_alert = AsyncMock()
        with patch("src.mcp.tools.fairino_massage.runtime.get_runtime", return_value=runtime), patch("src.application.asyncio.sleep", new_callable=AsyncMock):
            import asyncio
            asyncio.run(app._apply_confirmed_health_reduction())
        runtime.adjust_force.assert_awaited_once_with(direction="softer", delta_n=2.0)
        self.assertEqual(app._health_reduce_deadline, 0)


if __name__ == "__main__":
    unittest.main()
