import unittest
from unittest import mock

import ft_agent_api
import ft_agent_process


class SessionForceOverrideTests(unittest.TestCase):
    def test_confirmed_session_force_overrides_each_action_default(self):
        demo = object.__new__(ft_agent_api.AgentFTMassageDemo)
        demo.massage_target = "back"
        demo.session_force_target_n = 1.5

        self.assertEqual(demo._force_target_for_session_action("point_actions"), 1.5)
        self.assertEqual(demo._force_target_for_session_action("shun_jin"), 1.5)

    def test_unconfirmed_session_keeps_existing_per_action_defaults(self):
        demo = object.__new__(ft_agent_api.AgentFTMassageDemo)
        demo.massage_target = "back"
        demo.session_force_target_n = None

        with mock.patch.object(
            ft_agent_api.ft,
            "_force_target_for_massage_action",
            side_effect=lambda _target, action: 2.0 if action == "shun_jin" else 10.0,
        ):
            self.assertEqual(
                demo._force_target_for_session_action("point_actions"), 10.0
            )
            self.assertEqual(demo._force_target_for_session_action("shun_jin"), 2.0)

    def test_execute_cli_distinguishes_value_from_confirmed_override(self):
        parser = ft_agent_process.build_parser()
        common = [
            "execute",
            "--session-id",
            "test-session",
            "--trajectory-path",
            "trajectory.json",
            "--state-path",
            "state.json",
            "--control-path",
            "control.json",
            "--result-path",
            "result.json",
            "--force-target-n",
            "1.5",
        ]

        default_args = parser.parse_args(common)
        override_args = parser.parse_args(common + ["--force-target-override"])

        self.assertFalse(default_args.force_target_override)
        self.assertTrue(override_args.force_target_override)


if __name__ == "__main__":
    unittest.main()
