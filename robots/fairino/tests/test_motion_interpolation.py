import types
import unittest
from unittest import mock

import ft


def _state(x, motion_done):
    return types.SimpleNamespace(
        cart_x_cur_pos=float(x),
        cart_y_cur_pos=0.0,
        cart_z_cur_pos=0.0,
        cart_a_cur_pos=0.0,
        cart_b_cur_pos=0.0,
        cart_c_cur_pos=0.0,
        j1_cur_pos=float(x),
        j2_cur_pos=0.0,
        j3_cur_pos=0.0,
        j4_cur_pos=0.0,
        j5_cur_pos=0.0,
        j6_cur_pos=0.0,
        robot_motion_done=int(motion_done),
    )


class MotionCompletionTests(unittest.TestCase):
    def _proxy_with_states(self, states):
        proxy = ft.Ros2RobotProxy("test")
        pending = iter(states)
        consumed = {"count": 0}

        def spin_once(_timeout_sec=0.1):
            state = next(pending)
            consumed["count"] += 1
            proxy._state_callback(state)

        proxy.spin_once = spin_once
        return proxy, consumed

    def test_stale_motion_done_does_not_complete_a_different_pose(self):
        proxy, consumed = self._proxy_with_states(
            [
                _state(0.0, 1),
                _state(5.0, 0),
                _state(10.0, 1),
                _state(10.0, 1),
            ]
        )

        reached = proxy.wait_motion_done(
            timeout_sec=1.0,
            target_pose=[10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            after_state_sequence=0,
        )

        self.assertTrue(reached)
        self.assertEqual(consumed["count"], 4)

    def test_pose_distance_wraps_euler_angles(self):
        pos_dist, ori_dist = ft._pose_distance(
            [0.0, 0.0, 0.0, 179.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, -179.0, 0.0, 0.0],
        )

        self.assertEqual(pos_dist, 0.0)
        self.assertAlmostEqual(ori_dist, 2.0)

    def test_short_move_tolerance_is_smaller_than_the_commanded_step(self):
        pos_tol, _ = ft._motion_completion_tolerances(
            [0.0] * 6,
            [0.08, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

        self.assertLess(pos_tol, 0.08)

    def test_fresh_tcp_pose_waits_past_cached_pre_approach_state(self):
        proxy, consumed = self._proxy_with_states([_state(55.0, 1)])
        proxy._state_callback(_state(0.0, 1))

        pose = proxy.get_fresh_actual_tcp_pose(timeout_sec=1.0)

        self.assertEqual(pose[0], 55.0)
        self.assertEqual(consumed["count"], 1)

    def test_force_close_check_uses_fresh_feedback(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        robot = types.SimpleNamespace(
            get_actual_tcp_pose=mock.Mock(return_value=[0.0] * 6),
            get_fresh_actual_tcp_pose=mock.Mock(
                return_value=[55.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ),
        )
        demo.robot = robot

        close, pos_dist, _ = demo._current_pose_close_to([0.0] * 6)

        self.assertFalse(close)
        self.assertEqual(pos_dist, 55.0)
        robot.get_fresh_actual_tcp_pose.assert_called_once_with()
        robot.get_actual_tcp_pose.assert_not_called()


class ConfiguredInverseKinematicsTests(unittest.TestCase):
    def _proxy(self, responses):
        proxy = ft.Ros2RobotProxy("test")
        proxy.latest_state = _state(0.0, 1)
        proxy.spin_once = mock.Mock()
        proxy.wait_motion_done = mock.Mock(return_value=True)
        calls = []

        def call(cmd, **_kwargs):
            calls.append(cmd)
            if cmd.startswith("GetInverseKin"):
                return responses.get("ik", (0, "0,1,2,3,4,5,6"))
            return 0, "0"

        proxy._call = call
        return proxy, calls

    def test_configured_move_uses_joint_target_for_linear_motion(self):
        proxy, calls = self._proxy({})

        ret = proxy.MoveCart(
            [10.0, 20.0, 30.0, 1.0, 2.0, 3.0],
            config=4,
        )

        self.assertEqual(ret, 0)
        self.assertTrue(calls[0].endswith(",4)"))
        self.assertEqual(calls[1], "JNTPoint(1,1,2,3,4,5,6)")
        self.assertTrue(calls[2].startswith("MoveL(JNT1,"))

    def test_default_config_is_applied_when_call_uses_reference_current(self):
        proxy, calls = self._proxy({})
        proxy.default_ik_config = 4

        ret = proxy.MoveCart([10.0, 20.0, 30.0, 1.0, 2.0, 3.0])

        self.assertEqual(ret, 0)
        self.assertTrue(calls[0].endswith(",4)"))
        self.assertTrue(calls[2].startswith("MoveL(JNT1,"))

    def test_configured_ik_failure_stops_before_motion_command(self):
        proxy, calls = self._proxy({"ik": (112, "112,0,0,0,nan,0,0")})

        ret = proxy.MoveCart([10.0, 20.0, 30.0, 1.0, 2.0, 3.0], config=4)

        self.assertEqual(ret, 112)
        self.assertEqual(len(calls), 1)

    def test_inverse_kin_parser_rejects_nonfinite_joint(self):
        self.assertIsNone(ft._parse_inverse_kin_joints("0,1,2,3,nan,5,6"))


class InnerThighPostureTests(unittest.TestCase):
    def test_seed_is_inside_recorded_joint_soft_limits(self):
        demo = ft.LastTimeRos2Demo("leg_inner")

        for joint, (lower, upper) in zip(
            demo.inner_posture_seed["joint_deg"],
            demo.inner_posture_seed["joint_soft_limits_deg"],
        ):
            self.assertGreaterEqual(joint, lower)
            self.assertLessEqual(joint, upper)

    def test_seed_orientation_replaces_position_but_not_visual_start_point(self):
        demo = ft.LastTimeRos2Demo("leg_inner")
        frame = {
            "point_mm": [100.0, 200.0, 300.0],
            "tool_z_unit": [0.0, 0.0, 1.0],
            "split_axis_unit": [1.0, 0.0, 0.0],
            "base_pose": [0.0, 0.0, 0.0],
        }

        pose = demo._pose_from_frame_offset(frame, ft.TOOL_TIP_LENGTH_MM)

        self.assertEqual(pose[:3], frame["point_mm"])
        self.assertEqual(pose[3:], demo.inner_posture_seed["pose"][3:6])

    def test_enter_and_leave_switch_at_same_taught_staging_pose(self):
        demo = ft.LastTimeRos2Demo("leg_inner")
        seed_pose = list(demo.inner_posture_seed["pose"])
        seed_joints = list(demo.inner_posture_seed["joint_deg"])
        return_joints = [10.0, -20.0, -30.0, 40.0, 50.0, 60.0]
        robot = types.SimpleNamespace(
            default_ik_config=-1,
            get_actual_joint_positions_deg=mock.Mock(
                side_effect=[return_joints, seed_joints]
            ),
            get_actual_tcp_pose=mock.Mock(return_value=seed_pose),
            MoveJ=mock.Mock(return_value=0),
        )
        demo.robot = robot
        demo._move_to_work_pose = mock.Mock(return_value=True)
        demo._current_pose_close_to = mock.Mock(return_value=(True, 0.0, 0.0))
        demo._move_pose_segmented = mock.Mock(return_value=True)

        self.assertTrue(demo._enter_inner_posture_seed())
        self.assertTrue(demo._inner_posture_active)
        self.assertEqual(robot.default_ik_config, 4)
        self.assertEqual(demo._inner_posture_return_joints_deg, return_joints)

        self.assertTrue(demo._leave_inner_posture_seed())
        self.assertFalse(demo._inner_posture_active)
        self.assertEqual(robot.default_ik_config, -1)
        self.assertEqual(robot.MoveJ.call_count, 3)

    def test_enter_allows_near_seed_high_joint_fallback(self):
        demo = ft.LastTimeRos2Demo("leg_inner")
        seed_pose = list(demo.inner_posture_seed["pose"])
        seed_joints = list(demo.inner_posture_seed["joint_deg"])
        near_joints = list(seed_joints)
        near_joints[3] -= 10.0
        high_pose = list(seed_pose)
        high_pose[2] += 80.0
        robot = types.SimpleNamespace(
            default_ik_config=-1,
            get_actual_joint_positions_deg=mock.Mock(
                side_effect=[near_joints, seed_joints]
            ),
            get_actual_tcp_pose=mock.Mock(return_value=high_pose),
            MoveJ=mock.Mock(return_value=0),
        )
        demo.robot = robot
        demo._move_to_work_pose = mock.Mock(return_value=False)
        demo._current_pose_close_to = mock.Mock(return_value=(True, 0.0, 0.0))

        self.assertTrue(demo._enter_inner_posture_seed())
        self.assertTrue(demo._inner_posture_active)
        self.assertEqual(demo._inner_posture_return_joints_deg, near_joints)
        robot.MoveJ.assert_called_once_with(
            seed_joints,
            tool=ft.ROS2_TOOL,
            user=ft.ROS2_USER,
            vel=ft.THIGH_INNER_POSTURE_SWITCH_VEL,
            blendT=ft.BLEND_BLOCKING,
        )

    def test_enter_rejects_low_or_far_joint_fallback(self):
        for current_pose, current_joints in (
            (
                [
                    *self._seed_pose_prefix(),
                    ft.THIGH_INNER_POSTURE_LIFT_Z_MM - 1.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                None,
            ),
            (None, None),
        ):
            demo = ft.LastTimeRos2Demo("leg_inner")
            seed_pose = list(demo.inner_posture_seed["pose"])
            seed_joints = list(demo.inner_posture_seed["joint_deg"])
            if current_pose is None:
                current_pose = list(seed_pose)
            if current_joints is None:
                current_joints = list(seed_joints)
            if current_pose[2] >= ft.THIGH_INNER_POSTURE_LIFT_Z_MM:
                current_joints[0] += ft.THIGH_INNER_POSTURE_NEAR_JOINT_FALLBACK_DEG + 1.0
            robot = types.SimpleNamespace(
                default_ik_config=-1,
                get_actual_joint_positions_deg=mock.Mock(return_value=current_joints),
                get_actual_tcp_pose=mock.Mock(return_value=current_pose),
                MoveJ=mock.Mock(return_value=0),
            )
            demo.robot = robot
            demo._move_to_work_pose = mock.Mock(return_value=False)

            self.assertFalse(demo._enter_inner_posture_seed())
            robot.MoveJ.assert_not_called()

    @staticmethod
    def _seed_pose_prefix():
        return [0.0, 0.0]


class BackPostureProbeTests(unittest.TestCase):
    def test_fixed_back_seed_orientation_preserves_visual_position(self):
        with mock.patch.object(ft, "BACK_POSTURE_SEED_ENABLE", True):
            demo = ft.LastTimeRos2Demo("back")
        frame = {
            "point_mm": [100.0, 200.0, 300.0],
            "tool_z_unit": [0.0, 0.0, 1.0],
            "split_axis_unit": [1.0, 0.0, 0.0],
            "base_pose": [0.0, 0.0, 0.0],
        }

        pose = demo._pose_from_frame_offset(frame, -60.0)

        self.assertEqual(pose[3:], demo.back_posture_seed["orientation_rpy"])

    def test_back_seed_rejects_wrong_current_config(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.massage_target = "back"
        demo.back_posture_seed = {"ik_config": 4}
        demo._back_posture_active = False
        demo.robot = types.SimpleNamespace(
            default_ik_config=-1,
            get_actual_tcp_pose=mock.Mock(return_value=[1.0] * 6),
            get_actual_joint_positions_deg=mock.Mock(return_value=[0.0] * 6),
            inverse_kin_joints_deg=mock.Mock(return_value=(0, [10.0] * 6)),
        )

        with mock.patch.object(ft, "BACK_POSTURE_SEED_ENABLE", True):
            self.assertFalse(demo._activate_back_posture_seed())

        self.assertEqual(demo.robot.default_ik_config, -1)

    def test_back_reachability_filter_drops_incomplete_force_envelope(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.massage_target = "back"
        demo.back_posture_seed = {"ik_config": 6}
        demo.hover_height_mm = 20.0
        demo.force_approach_max_offset_mm = 40.0
        demo.massage_frames = [
            {"index": 0, "point_mm": [1.0, 0.0, 0.0]},
            {"index": 1, "point_mm": [2.0, 0.0, 0.0]},
        ]
        demo.massage_points_mm = [frame["point_mm"] for frame in demo.massage_frames]
        demo._pose_from_frame_offset = mock.Mock(
            side_effect=lambda frame, offset, split_offset_mm=0.0: [
                frame["index"],
                offset,
                split_offset_mm,
                0.0,
                0.0,
                0.0,
            ]
        )
        demo._pose_ik_ok = mock.Mock(
            side_effect=lambda pose: not (pose[0] == 0 and pose[1] == 40.0)
        )

        with mock.patch.object(ft, "BACK_POSTURE_SEED_ENABLE", True), mock.patch.object(
            ft,
            "FT_CONTINUE_ON_POINT_ERROR",
            True,
        ):
            self.assertTrue(demo._adjust_back_frames_for_reachability())

        self.assertEqual([frame["index"] for frame in demo.massage_frames], [1])
        self.assertEqual(demo.massage_points_mm, [[2.0, 0.0, 0.0]])

    def test_back_hover_probe_never_calls_contact_actions(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.massage_target = "back"
        demo.massage_frames = [{"index": 0}, {"index": 1}]
        demo._pose_from_frame_offset = mock.Mock(
            side_effect=[[1.0, 2.0, 300.0, 4.0, 5.0, 6.0], [2.0, 3.0, 300.0, 4.0, 5.0, 6.0]]
        )
        demo._back_probe_pose_path = mock.Mock(return_value=[[1.0] * 6])
        demo._pose_ik_ok = mock.Mock(return_value=True)
        demo._move_to_work_pose = mock.Mock(return_value=True)
        demo._move_pose_segmented = mock.Mock(return_value=True)
        demo.execute_dian_jin = mock.Mock()
        demo.execute_fen_jin = mock.Mock()
        demo.execute_shun_jin = mock.Mock()

        with mock.patch.object(ft, "BACK_TRAJECTORY_PROBE_ONLY", True):
            self.assertTrue(demo._execute_back_hover_probe())

        demo.execute_dian_jin.assert_not_called()
        demo.execute_fen_jin.assert_not_called()
        demo.execute_shun_jin.assert_not_called()


class ServoInterpolationTests(unittest.TestCase):
    def test_default_short_move_has_at_least_eight_control_periods(self):
        duration_s = ft._servo_interpolation_duration_s(
            [0.0] * 6,
            [25.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            100.0,
        )

        self.assertGreaterEqual(
            duration_s,
            ft.ROS2_SERVO_MIN_SAMPLES / ft.ROS2_SERVO_INTERPOLATION_HZ,
        )

    def test_quintic_peak_speed_is_included_in_duration(self):
        duration_s = ft._servo_interpolation_duration_s(
            [0.0] * 6,
            [100.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            100.0,
        )

        self.assertGreaterEqual(duration_s, 1.875 * 100.0 / 1000.0)


class LongTransitTests(unittest.TestCase):
    def test_waypoints_interpolate_translation_and_wrapped_orientation(self):
        waypoints = ft._linear_transit_waypoints(
            [0.0, 0.0, 0.0, 179.0, 0.0, 0.0],
            [120.0, 0.0, 0.0, -179.0, 0.0, 0.0],
            50.0,
        )

        self.assertEqual(len(waypoints), 3)
        self.assertEqual([point[0] for point in waypoints], [40.0, 80.0, 120.0])
        self.assertAlmostEqual(waypoints[0][3], 179.0 + 2.0 / 3.0)
        self.assertEqual(waypoints[-1][3], -179.0)

    def test_long_transit_prefers_one_native_continuous_move(self):
        calls = []
        robot = types.SimpleNamespace(
            get_actual_tcp_pose=lambda: [0.0] * 6,
            MoveCart=lambda **kwargs: calls.append(kwargs) or 0,
        )
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.robot = robot

        moved = demo._move_pose_segmented(
            [120.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "test transit",
            max_step_mm=50.0,
        )

        self.assertTrue(moved)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]["desc_pos"][0], 120.0)
        self.assertEqual(calls[0]["blendT"], ft.BLEND_BLOCKING)
        self.assertEqual(calls[0]["timeout_sec"], ft.ROS2_SEGMENT_TIMEOUT_S)

    def test_failed_long_direct_move_uses_blended_segment_fallback(self):
        calls = []

        def move_cart(**kwargs):
            calls.append(kwargs)
            return -2 if len(calls) == 1 else 0

        robot = types.SimpleNamespace(
            get_actual_tcp_pose=lambda: [0.0] * 6,
            MoveCart=move_cart,
        )
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.robot = robot
        demo._recover_robot_ready = mock.Mock()

        moved = demo._move_pose_segmented(
            [120.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "test transit",
            max_step_mm=50.0,
        )

        self.assertTrue(moved)
        self.assertEqual(
            [call["desc_pos"][0] for call in calls],
            [120.0, 40.0, 80.0, 120.0],
        )
        self.assertEqual(
            [call["blendT"] for call in calls],
            [ft.BLEND_BLOCKING, 0.0, 0.0, ft.BLEND_BLOCKING],
        )
        demo._recover_robot_ready.assert_called_once()


class MassageSequenceTests(unittest.TestCase):
    def _demo(self, events, *, fen_ok=True):
        frame = {
            "index": 0,
            "point_mm": [0.0, 0.0, 0.0],
            "tool_z_unit": [0.0, 0.0, 1.0],
            "split_axis_unit": [1.0, 0.0, 0.0],
            "base_pose": [0.0, 0.0, 0.0],
        }
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.massage_frames = [frame]
        demo.massage_points_mm = [frame["point_mm"]]
        demo.massage_target = "back"
        demo.hover_height_mm = 20.0
        demo._build_session_safe_pose = mock.Mock(return_value=([0.0] * 6, False))
        demo._move_to_initial_safe_pose = mock.Mock(return_value=True)
        demo._activate_back_posture_seed = mock.Mock(return_value=True)
        demo._adjust_leg_frames_for_reachability = mock.Mock(return_value=True)
        demo._adjust_back_frames_for_reachability = mock.Mock(return_value=True)
        demo._enter_inner_posture_seed = mock.Mock(return_value=True)
        demo._leave_inner_posture_seed = mock.Mock(return_value=True)
        demo.update_preview_status = mock.Mock()
        demo._pose_from_frame_offset = mock.Mock(return_value=[0.0] * 6)
        demo._move_to_work_pose = mock.Mock(return_value=True)
        demo._move_to_hover_with_fallback = mock.Mock(return_value=True)
        demo.set_force_target_n = mock.Mock()
        demo.close_force_controller = mock.Mock()
        demo.execute_dian_jin = mock.Mock(side_effect=lambda _frame: events.append("dian") or True)
        demo.execute_fen_jin = mock.Mock(
            side_effect=lambda _frame: events.append("fen") or fen_ok
        )
        demo.execute_shun_jin = mock.Mock(
            side_effect=lambda _frames: events.append("shun") or True
        )
        return demo

    def test_default_point_action_is_real_dian_jin(self):
        self.assertEqual(ft.DIAN_JIN_MODE_DEFAULT, "dian")

    def test_sequence_is_dian_then_fen_then_shun(self):
        events = []
        demo = self._demo(events)

        with mock.patch.object(ft, "LASTTIME_ROS2_FORCE", False), mock.patch.object(
            ft,
            "DIAN_JIN_MODE",
            "dian",
        ):
            result = demo.execute_massage_sequence()

        self.assertTrue(result)
        self.assertEqual(events, ["dian", "fen", "shun"])

    def test_shun_is_forbidden_when_no_point_completed_fen_jin(self):
        events = []
        demo = self._demo(events, fen_ok=False)

        with mock.patch.object(ft, "LASTTIME_ROS2_FORCE", False), mock.patch.object(
            ft,
            "DIAN_JIN_MODE",
            "dian",
        ):
            result = demo.execute_massage_sequence()

        self.assertFalse(result)
        self.assertEqual(events, ["dian", "fen"])
        demo.execute_shun_jin.assert_not_called()

    def test_inner_posture_probe_never_runs_contact_actions(self):
        events = []
        demo = self._demo(events)
        demo.massage_target = "leg_inner"

        with mock.patch.object(ft, "LASTTIME_ROS2_FORCE", False), mock.patch.object(
            ft,
            "THIGH_INNER_POSTURE_PROBE_ONLY",
            True,
        ), mock.patch.object(ft, "THIGH_INNER_POSTURE_PROBE_DWELL_S", 0.0):
            result = demo.execute_massage_sequence()

        self.assertTrue(result)
        self.assertEqual(events, [])
        demo._adjust_leg_frames_for_reachability.assert_not_called()
        demo._enter_inner_posture_seed.assert_called_once()
        demo._leave_inner_posture_seed.assert_called_once()
        demo._move_to_work_pose.assert_not_called()
        demo.execute_dian_jin.assert_not_called()
        demo.execute_fen_jin.assert_not_called()
        demo.execute_shun_jin.assert_not_called()

    def test_inner_posture_probe_run_bypasses_vision(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.massage_target = "leg_inner"
        demo.init_robot = mock.Mock()
        demo.execute_massage_sequence = mock.Mock(return_value=True)
        demo.run_leg_interactive = mock.Mock()

        with mock.patch.object(ft, "THIGH_INNER_POSTURE_PROBE_ONLY", True):
            result = demo.run()

        self.assertTrue(result)
        demo.init_robot.assert_called_once_with()
        demo.execute_massage_sequence.assert_called_once_with()
        demo.run_leg_interactive.assert_not_called()


class ContinuousForceApproachTests(unittest.TestCase):
    def test_structured_response_is_parsed(self):
        result = ft._parse_force_approach_response(
            "0,1,12.5,0.75,1,2,-9,0.1,0.2,0.3"
        )

        self.assertEqual(result["ret"], 0)
        self.assertEqual(result["status"], 1)
        self.assertEqual(result["travel_mm"], 12.5)
        self.assertEqual(result["force"], [1.0, 2.0, -9.0, 0.1, 0.2, 0.3])

    def test_continuous_command_has_all_safety_parameters(self):
        calls = []
        robot = types.SimpleNamespace()

        def call(cmd, **kwargs):
            calls.append((cmd, kwargs))
            return 0, "0,1,4.0,0.5,0,0,-10,0,0,0"

        robot._call = call
        controller = ft.Ros2ForceController(robot, target_force_n=10.0)

        result = controller.continuous_approach([1, 2, 3, 4, 5, 6], "test")

        self.assertEqual(result["status"], 1)
        self.assertEqual(len(calls), 1)
        command = calls[0][0]
        self.assertTrue(command.startswith("ServoCartForceApproach("))
        parameters = command.removeprefix("ServoCartForceApproach(").removesuffix(")").split(",")
        self.assertEqual(len(parameters), 24)

    def test_safety_failure_never_falls_back_to_position_motion(self):
        calls = []
        robot = types.SimpleNamespace()

        def call(cmd, **_kwargs):
            calls.append(cmd)
            return -2201, "-2201,-1,2.0,0.2,0,0,-50,0,0,0"

        robot._call = call
        controller = ft.Ros2ForceController(robot, target_force_n=10.0)

        with self.assertRaisesRegex(RuntimeError, "法向力超过软件限位"):
            controller.continuous_approach([1, 2, 3, 4, 5, 6], "test")

        self.assertEqual(len(calls), 1)
        self.assertNotIn("MoveCart", calls[0])

    def test_fixed_posture_offsets_along_the_actual_tool_z_axis(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.motion_orientation = [0.0, 0.0, 0.0]
        frame = {
            "point_mm": [0.0, 0.0, 0.0],
            "tool_z_unit": [1.0, 0.0, 0.0],
            "split_axis_unit": [0.0, 1.0, 0.0],
            "base_pose": [30.0, 40.0, 50.0],
        }

        with mock.patch.object(ft, "ROS2_KEEP_CURRENT_ORIENTATION", True), mock.patch.object(
            ft,
            "TOOL_TIP_LENGTH_MM",
            0.0,
        ):
            pose = demo._pose_from_frame_offset(frame, 10.0)

        self.assertAlmostEqual(pose[0], 0.0)
        self.assertAlmostEqual(pose[1], 0.0)
        self.assertAlmostEqual(pose[2], 10.0)
        self.assertEqual(pose[3:], [0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
