import json
import tempfile
import unittest
from pathlib import Path

import ft_agent_process


class AgentProcessStateIsolationTests(unittest.TestCase):
    def test_stale_session_cannot_overwrite_current_state(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            state_path = Path(temp_dir) / "current_session.json"
            state_path.write_text(
                json.dumps(
                    {
                        "session_id": "new-session",
                        "execution_id": "new-execution",
                        "status": "running",
                    }
                ),
                encoding="utf-8",
            )

            ft_agent_process._update_state(
                state_path,
                "old-session",
                execution_id="old-execution",
                status="paused",
            )

            state = json.loads(state_path.read_text(encoding="utf-8"))
            self.assertEqual(state["session_id"], "new-session")
            self.assertEqual(state["execution_id"], "new-execution")
            self.assertEqual(state["status"], "running")

    def test_stale_execution_cannot_overwrite_same_session(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            state_path = Path(temp_dir) / "current_session.json"
            state_path.write_text(
                json.dumps(
                    {
                        "session_id": "same-session",
                        "execution_id": "new-execution",
                        "status": "running",
                    }
                ),
                encoding="utf-8",
            )

            ft_agent_process._update_state(
                state_path,
                "same-session",
                execution_id="old-execution",
                status="paused",
            )

            state = json.loads(state_path.read_text(encoding="utf-8"))
            self.assertEqual(state["execution_id"], "new-execution")
            self.assertEqual(state["status"], "running")


if __name__ == "__main__":
    unittest.main()
