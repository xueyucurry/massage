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


class AgentContinuousPointMotionTests(unittest.TestCase):
    def test_gui_voice_point_action_uses_continuous_small_sweep(self):
        demo = object.__new__(ft_agent_api.AgentFTMassageDemo)
        demo.hover_height_mm = 20.0
        demo._agent_control_point_index = 0
        demo._pose_from_frame_offset = mock.Mock(return_value=[0.0] * 6)
        demo._set_agent_control_context = mock.Mock()
        demo.update_preview_status = mock.Mock()
        demo._move_to_hover_for_force = mock.Mock(return_value=True)
        demo._approach_to_target_force = mock.Mock(return_value=(-5.0, True))
        demo._agent_control_checkpoint = mock.Mock(return_value=None)
        demo._continuous_fen_round = mock.Mock(return_value=(-5.0, True))
        demo._move_force_pose_checked = mock.Mock(return_value=True)
        demo._hold_target_force = mock.Mock(return_value=(-5.0, True))
        demo._next_resume_after_repeat = mock.Mock(
            return_value=("point_actions", "fen_jin", 0, 0, 0)
        )
        demo._retract_to_hover = mock.Mock(return_value=True)

        with mock.patch.object(ft_agent_api.ft, "LASTTIME_ROS2_FORCE", True), mock.patch.object(
            ft_agent_api.ft,
            "DIAN_JIN_MODE",
            "small_fen",
        ), mock.patch.object(ft_agent_api.ft, "DIAN_JIN_REPEAT_COUNT", 2), mock.patch.object(
            ft_agent_api.ft,
            "DIAN_AS_SMALL_FEN_LATERAL_MM",
            3.0,
        ):
            result = demo.execute_dian_jin({"index": 0})

        self.assertTrue(result)
        self.assertEqual(demo._continuous_fen_round.call_count, 2)
        for call in demo._continuous_fen_round.call_args_list:
            self.assertEqual(call.args[3], 1)
            self.assertEqual(call.kwargs["amplitude_mm"], 3.0)
        demo._move_force_pose_checked.assert_not_called()
        demo._hold_target_force.assert_not_called()
        self.assertEqual(demo._retract_to_hover.call_count, 2)


if __name__ == "__main__":
    unittest.main()
