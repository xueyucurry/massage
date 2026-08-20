import asyncio
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from src.mcp.mcp_server import Property, PropertyList, PropertyType
from src.mcp.tools.fairino_massage.manager import FairinoMassageToolsManager
from src.mcp.tools.fairino_massage.runtime import FairinoMassageRuntime


class _NoopThread:
    def __init__(self, target=None, args=(), **_kwargs):
        self.target = target
        self.args = args
        self.started = False

    def start(self):
        self.started = True

    def is_alive(self):
        return False


class PendingForceRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.state_path = self.root / "current_session.json"
        self.trajectory_path = self.root / "trajectory.json"
        self.trajectory_path.write_text(
            json.dumps({"target": "back", "frames": [{"index": 0}]}),
            encoding="utf-8",
        )
        self.runtime = FairinoMassageRuntime(self.state_path)
        self.runtime.control_path = self.root / "current_control.json"
        self.runtime._state.update(
            status="detected",
            target="back",
            target_label="背部膀胱经",
            trajectory_path=str(self.trajectory_path),
            default_force_target_n=10.0,
            force_target_n=10.0,
        )
        self.runtime._save_state_unlocked()

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_exact_fractional_force_updates_gui_state_immediately(self):
        result = asyncio.run(
            self.runtime.set_pending_force(
                target_force_n=1.5,
                target="back",
                actions="shun_jin",
            )
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["direction"], "softer")
        self.assertEqual(result["force_target_n"], 1.5)
        state = json.loads(self.state_path.read_text(encoding="utf-8"))
        self.assertEqual(state["force_target_n"], 1.5)
        self.assertEqual(state["pending_force_preset"]["force_target_n"], 1.5)

    def test_matching_start_consumes_and_locks_confirmed_preset(self):
        asyncio.run(
            self.runtime.set_pending_force(
                target_force_n=1.5,
                target="back",
                actions="shun_jin",
            )
        )

        with mock.patch(
            "src.mcp.tools.fairino_massage.runtime.threading.Thread",
            _NoopThread,
        ):
            result = asyncio.run(
                self.runtime.start(actions="shun_jin", target="back")
            )

        self.assertTrue(result["success"])
        self.assertTrue(result["pending_force_applied"])
        self.assertEqual(result["force_target_n"], 1.5)
        self.assertIsNone(result["state"]["pending_force_preset"])
        self.assertTrue(result["state"]["session_force_override_active"])
        self.assertEqual(
            result["state"]["last_pending_force_preset"]["status"], "applied"
        )
        self.assertTrue(self.runtime._worker_thread.args[5])

    def test_start_without_confirmation_uses_default_not_stale_force(self):
        self.runtime._state.update(force_target_n=1.5, pending_force_preset=None)
        self.runtime._save_state_unlocked()

        with mock.patch(
            "src.mcp.tools.fairino_massage.runtime.threading.Thread",
            _NoopThread,
        ):
            result = asyncio.run(self.runtime.start(actions="all", target="back"))

        self.assertTrue(result["success"])
        self.assertFalse(result["pending_force_applied"])
        self.assertEqual(result["force_target_n"], 10.0)
        self.assertFalse(result["state"]["session_force_override_active"])
        self.assertFalse(self.runtime._worker_thread.args[5])

    def test_mismatched_preset_is_discarded_instead_of_applied_later(self):
        asyncio.run(
            self.runtime.set_pending_force(
                target_force_n=1.5,
                target="back",
                actions="shun_jin",
            )
        )

        with mock.patch(
            "src.mcp.tools.fairino_massage.runtime.threading.Thread",
            _NoopThread,
        ):
            result = asyncio.run(self.runtime.start(actions="all", target="back"))

        self.assertTrue(result["pending_force_discarded"])
        self.assertEqual(result["force_target_n"], 10.0)
        self.assertIsNone(result["state"]["pending_force_preset"])
        self.assertEqual(
            result["state"]["last_pending_force_preset"]["status"], "discarded"
        )

    def test_qualitative_softer_preference_uses_default_step(self):
        result = asyncio.run(
            self.runtime.set_pending_force(
                direction="softer",
                target="back",
                actions="all",
            )
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["direction"], "softer")
        self.assertEqual(result["force_target_n"], 5.0)

    def test_missing_force_and_direction_does_not_create_a_preset(self):
        result = asyncio.run(self.runtime.set_pending_force())

        self.assertFalse(result["success"])
        self.assertEqual(result["state"]["force_target_n"], 10.0)
        self.assertIsNone(result["state"]["pending_force_preset"])


class NumberPropertyTests(unittest.TestCase):
    def test_number_property_preserves_fractional_force(self):
        properties = PropertyList([Property("target_force_n", PropertyType.NUMBER)])

        parsed = properties.parse_arguments({"target_force_n": 1.5})

        self.assertEqual(parsed["target_force_n"], 1.5)
        self.assertEqual(properties.to_json()["target_force_n"]["type"], "number")

    def test_pending_force_tool_is_registered_with_fractional_target(self):
        registered = []
        manager = FairinoMassageToolsManager()

        manager.init_tools(
            registered.append,
            PropertyList,
            Property,
            PropertyType,
        )

        tool = next(
            item
            for item in registered
            if item[0] == "self.fairino_massage.set_pending_force"
        )
        self.assertEqual(
            tool[2].to_json()["target_force_n"]["type"],
            "number",
        )
        self.assertIn("明确同意后才调用", tool[1])


if __name__ == "__main__":
    unittest.main()
