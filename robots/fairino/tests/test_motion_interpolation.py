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

    def test_pose_distance_accepts_equivalent_euler_alias_near_gimbal_lock(self):
        pos_dist, ori_dist = ft._pose_distance(
            [489.579102, -313.276886, 99.411377, 43.4049, -87.486984, 146.253922],
            [489.578298, -313.276696, 99.413581, -136.59375, -92.513338, -33.748],
        )

        self.assertLess(pos_dist, 0.01)
        self.assertLess(ori_dist, 0.01)

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
    def test_inner_image_down_offset_cannot_be_reversed_by_negative_config(self):
        with mock.patch.object(ft, "THIGH_INNER_OFFSET_MM", -25.0), mock.patch.object(
            ft, "THIGH_DIRECTION", "image-down"
        ):
            self.assertEqual(ft._thigh_offset_for_massage_target("leg_inner"), 25.0)

    def test_outer_offset_keeps_legacy_signed_configuration(self):
        with mock.patch.object(ft, "THIGH_OUTER_OFFSET_MM", -25.0):
            self.assertEqual(ft._thigh_offset_for_massage_target("leg"), -25.0)

    def test_inner_and_outer_targets_use_independent_configured_legs(self):
        with mock.patch.object(ft, "THIGH_OUTER_SIDE", "right"), mock.patch.object(
            ft, "THIGH_INNER_SIDE", "left"
        ):
            self.assertEqual(ft._thigh_side_for_massage_target("leg"), "right")
            self.assertEqual(ft._thigh_side_for_massage_target("leg_inner"), "left")

    def test_inner_and_outer_targets_use_independent_pose_rotations(self):
        with mock.patch.object(ft, "THIGH_OUTER_ROTATION", "none"), mock.patch.object(
            ft, "THIGH_INNER_ROTATION", "cw"
        ), mock.patch.object(ft, "THIGH_OUTER_TRY_ROTATIONS", False), mock.patch.object(
            ft, "THIGH_INNER_TRY_ROTATIONS", False
        ):
            self.assertEqual(ft._thigh_rotations_for_massage_target("leg"), ("none",))
            self.assertEqual(ft._thigh_rotations_for_massage_target("leg_inner"), ("cw",))

    def test_inner_rotation_can_try_all_orientations(self):
        with mock.patch.object(ft, "THIGH_INNER_TRY_ROTATIONS", True):
            self.assertEqual(ft._thigh_rotations_for_massage_target("leg_inner"), ft.ROTATIONS)

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
        with mock.patch.object(ft, "BACK_POSTURE_SEED_ENABLE", True), mock.patch.object(
            ft,
            "BACK_FOLLOW_LOCAL_NORMAL",
            False,
        ):
            demo = ft.LastTimeRos2Demo("back")
        frame = {
            "point_mm": [100.0, 200.0, 300.0],
            "tool_z_unit": [0.0, 0.0, 1.0],
            "split_axis_unit": [1.0, 0.0, 0.0],
            "base_pose": [0.0, 0.0, 0.0],
        }

        with mock.patch.object(ft, "BACK_FOLLOW_LOCAL_NORMAL", False):
            pose = demo._pose_from_frame_offset(frame, -60.0)

        self.assertEqual(pose[3:], demo.back_posture_seed["orientation_rpy"])

    def test_back_local_normal_overrides_fixed_orientation_settings(self):
        with mock.patch.object(ft, "BACK_POSTURE_SEED_ENABLE", True), mock.patch.object(
            ft,
            "BACK_FOLLOW_LOCAL_NORMAL",
            True,
        ):
            demo = ft.LastTimeRos2Demo("back")
        demo.motion_orientation = [70.0, 80.0, 90.0]
        frame = {
            "point_mm": [100.0, 200.0, 300.0],
            "tool_z_unit": [1.0, 0.0, 0.0],
            "split_axis_unit": [0.0, 1.0, 0.0],
            "base_pose": [10.0, 20.0, 30.0],
        }

        with mock.patch.object(ft, "BACK_FOLLOW_LOCAL_NORMAL", True), mock.patch.object(
            ft,
            "ROS2_KEEP_CURRENT_ORIENTATION",
            True,
        ), mock.patch.object(ft, "TOOL_TIP_LENGTH_MM", 0.0):
            pose = demo._pose_from_frame_offset(frame, 10.0)

        self.assertEqual(pose, [110.0, 200.0, 300.0, 10.0, 20.0, 30.0])

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

    def _back_entry_demo(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.massage_target = "back"
        demo.back_posture_seed = {"ik_config": 6}
        demo.hover_height_mm = 20.0
        demo.robot = types.SimpleNamespace(
            get_actual_tcp_pose=mock.Mock(
                return_value=[0.0, 0.0, 300.0, 0.0, 0.0, 0.0]
            )
        )
        frames = [
            {"index": 3, "x": 100.0, "y": 0.0},
            {"index": 8, "x": 0.0, "y": 100.0},
        ]

        def pose_from_frame(frame, offset, split_offset_mm=0.0):
            del split_offset_mm
            z = 160.0 if abs(float(offset)) >= 60.0 else 120.0
            return [frame["x"], frame["y"], z, 0.0, 0.0, 0.0]

        demo._pose_from_frame_offset = mock.Mock(side_effect=pose_from_frame)
        # The direct high route toward the requested first point crosses an
        # unreachable x>0 region. Entering above the second point stays on x=0;
        # the return to the first point happens only at safe clearance.
        demo._pose_ik_ok = mock.Mock(
            side_effect=lambda pose: float(pose[2]) < 299.0 or abs(float(pose[0])) < 1e-6
        )
        return demo, frames

    def test_back_start_plan_uses_later_clearance_entry_without_dropping_target(self):
        demo, frames = self._back_entry_demo()

        plan = demo._back_start_entry_plan(frames, target_index=0)

        self.assertEqual(plan["entry_index"], 1)
        self.assertEqual(plan["target_index"], 0)
        self.assertEqual(plan["clearance_mm"], 60.0)
        labels = [label for label, _pose in plan["stages"]]
        self.assertIn("高位进入点9", labels)
        self.assertIn("沿安全悬空轨迹到点4", labels)
        self.assertEqual(labels[-1], "下降到目标点4悬空位")

    def test_back_start_move_preflights_full_path_then_executes_selected_stages(self):
        demo, frames = self._back_entry_demo()
        demo._move_pose_segmented = mock.Mock(return_value=True)

        with mock.patch.object(ft, "BACK_POSTURE_SEED_ENABLE", True):
            result = demo._move_to_start_frame(frames, 0, "移动到起始位置")

        self.assertTrue(result)
        self.assertGreater(demo._pose_ik_ok.call_count, 0)
        self.assertGreater(demo._move_pose_segmented.call_count, 0)
        contexts = [call.args[1] for call in demo._move_pose_segmented.call_args_list]
        self.assertTrue(any("高位进入点9" in context for context in contexts))
        self.assertTrue(any("目标点4悬空位" in context for context in contexts))

    def test_back_start_move_does_not_move_when_no_complete_entry_is_reachable(self):
        demo, frames = self._back_entry_demo()
        demo._pose_ik_ok = mock.Mock(return_value=False)
        demo._move_pose_segmented = mock.Mock(return_value=True)

        with mock.patch.object(ft, "BACK_POSTURE_SEED_ENABLE", True):
            result = demo._move_to_start_frame(frames, 0, "移动到起始位置")

        self.assertFalse(result)
        demo._move_pose_segmented.assert_not_called()

    def test_back_quality_filter_drops_large_depth_and_position_outlier(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.massage_target = "back"
        demo.massage_frames = []
        for index in range(9):
            demo.massage_frames.append(
                {
                    "index": index,
                    "pixel": [400.0 - index * 30.0, 300.0],
                    "point_mm": [600.0, -380.0 + index * 50.0, 104.0],
                    "depth_m": 0.80 + index * 0.01,
                    "depth_patch_stats": {
                        "std_mm": 1.0,
                        "min_m": 0.80 + index * 0.01,
                        "max_m": 0.803 + index * 0.01,
                    },
                }
            )
        demo.massage_frames.append(
            {
                "index": 9,
                "pixel": [95.0, 300.0],
                "point_mm": [345.0, -119.0, 122.0],
                "depth_m": 0.588,
                "depth_patch_stats": {
                    "std_mm": 86.6,
                    "min_m": 0.587,
                    "max_m": 0.872,
                },
            }
        )
        demo.massage_points_mm = [frame["point_mm"] for frame in demo.massage_frames]
        demo.massage_pixels = [frame["pixel"] for frame in demo.massage_frames]

        self.assertTrue(demo._filter_back_trajectory_quality())

        self.assertEqual([frame["index"] for frame in demo.massage_frames], list(range(9)))
        self.assertEqual(len(demo.massage_points_mm), 9)
        self.assertEqual(len(demo.massage_pixels), 9)

    def test_back_quality_filter_rejects_trajectory_when_too_few_points_remain(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.massage_target = "back"
        demo.massage_frames = [
            {
                "index": index,
                "point_mm": [float(index) * 50.0, 0.0, 0.0],
                "depth_m": 0.8,
                "depth_patch_stats": {"std_mm": 100.0, "min_m": 0.5, "max_m": 0.9},
            }
            for index in range(5)
        ]
        original_frames = list(demo.massage_frames)

        self.assertFalse(demo._filter_back_trajectory_quality())
        self.assertEqual(demo.massage_frames, original_frames)

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

    def test_large_low_hover_gap_uses_safe_height_without_direct_attempt(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.robot = types.SimpleNamespace(
            get_actual_tcp_pose=mock.Mock(return_value=[0.0] * 6),
        )
        demo._move_cart_checked = mock.Mock(return_value=True)
        demo._move_to_work_pose = mock.Mock(return_value=True)
        target = [ft.ROS2_HOVER_DIRECT_MAX_DISTANCE_MM + 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        moved = demo._move_to_hover_with_fallback(target, "移动到悬空位")

        self.assertTrue(moved)
        demo._move_cart_checked.assert_not_called()
        demo._move_to_work_pose.assert_called_once_with(
            target,
            "移动到悬空位 安全转场",
            ft.TRANSIT_MOVE_VEL_SLOW,
        )

    def test_normal_hover_gap_still_uses_direct_low_move(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.robot = types.SimpleNamespace(
            get_actual_tcp_pose=mock.Mock(return_value=[0.0] * 6),
        )
        demo._move_cart_checked = mock.Mock(return_value=True)
        demo._move_to_work_pose = mock.Mock(return_value=True)
        target = [ft.ROS2_HOVER_DIRECT_MAX_DISTANCE_MM, 0.0, 0.0, 0.0, 0.0, 0.0]

        moved = demo._move_to_hover_with_fallback(target, "移动到悬空位")

        self.assertTrue(moved)
        demo._move_cart_checked.assert_called_once_with(
            target,
            "移动到悬空位",
            ft.TRANSIT_MOVE_VEL_SLOW,
            required=False,
        )
        demo._move_to_work_pose.assert_not_called()


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

    def test_default_point_action_is_small_up_down_dian_jin(self):
        self.assertEqual(ft.DIAN_JIN_MODE_DEFAULT, "small_fen")

    def test_small_up_down_point_action_keeps_independent_point_rounds(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.hover_height_mm = 20.0
        demo.update_preview_status = mock.Mock()
        demo._pose_from_frame_offset = mock.Mock(
            side_effect=lambda _frame, _offset, split_offset_mm=0.0: [
                float(split_offset_mm),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )
        demo._move_to_hover_for_force = mock.Mock(return_value=True)
        demo._approach_to_target_force = mock.Mock(return_value=(1.0, True))
        demo._continuous_fen_round = mock.Mock(
            side_effect=lambda _frame, offset, _context, _repeat_count, **_kwargs: (
                offset,
                True,
            )
        )
        demo._move_force_pose_checked = mock.Mock(return_value=True)
        demo._hold_target_force = mock.Mock(
            side_effect=lambda _f, _s, offset, _d, _c: (offset, True)
        )
        demo._retract_to_hover = mock.Mock(return_value=True)

        with mock.patch.object(ft, "LASTTIME_ROS2_FORCE", True), mock.patch.object(
            ft,
            "DIAN_JIN_MODE",
            "small_fen",
        ), mock.patch.object(ft, "DIAN_JIN_REPEAT_COUNT", 2), mock.patch.object(
            ft,
            "DIAN_AS_SMALL_FEN_LATERAL_MM",
            3.0,
        ):
            result = demo.execute_dian_jin({"index": 0})

        self.assertTrue(result)
        self.assertEqual(demo._continuous_fen_round.call_count, 2)
        for call in demo._continuous_fen_round.call_args_list:
            self.assertEqual(call.args[3], 1)
            self.assertEqual(call.kwargs["amplitude_mm"], 3.0)
        demo._hold_target_force.assert_not_called()
        demo._move_force_pose_checked.assert_not_called()
        self.assertEqual(demo._approach_to_target_force.call_count, 2)
        self.assertEqual(demo._retract_to_hover.call_count, 2)

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


class ContinuousFenJinTests(unittest.TestCase):
    def test_round_does_not_require_legacy_ft_control_active_state(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.hover_height_mm = 20.0
        demo.force_approach_max_offset_mm = 35.0
        demo._motion_axes_from_frame = mock.Mock(
            return_value=([0.0, 0.0, 1.0], [0.0, 1.0, 0.0])
        )
        demo.force_controller = types.SimpleNamespace(
            active=False,
            continuous_fen_jin=mock.Mock(
                return_value={
                    "normal_correction_mm": 0.0,
                    "elapsed_s": 1.0,
                    "force": [0.0, 0.0, -10.0, 0.0, 0.0, 0.0],
                }
            ),
        )

        with mock.patch.object(ft, "FORCE_CONTINUOUS_FEN_ENABLE", True):
            result = demo._continuous_fen_round(
                {"index": 0},
                -5.0,
                "test",
                1,
                amplitude_mm=3.0,
            )

        self.assertEqual(result, (-5.0, True))
        demo.force_controller.continuous_fen_jin.assert_called_once()
        self.assertEqual(
            demo.force_controller.continuous_fen_jin.call_args.args[2],
            3.0,
        )

    def test_structured_response_is_parsed(self):
        result = ft._parse_force_fen_response(
            "0,1,0.75,1.02,1,2,-9,0.1,0.2,0.3"
        )

        self.assertEqual(result["ret"], 0)
        self.assertEqual(result["status"], 1)
        self.assertEqual(result["normal_correction_mm"], 0.75)
        self.assertEqual(result["force"], [1.0, 2.0, -9.0, 0.1, 0.2, 0.3])

    def test_continuous_command_contains_axes_force_control_and_safety_limits(self):
        calls = []
        robot = types.SimpleNamespace()

        def call(cmd, **kwargs):
            calls.append((cmd, kwargs))
            return 0, "0,1,0.25,1.0,0,0,-10,0,0,0"

        robot._call = call
        controller = ft.Ros2ForceController(robot, target_force_n=10.0)

        result = controller.continuous_fen_jin(
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            8.0,
            3,
            -2.0,
            3.0,
            "test",
        )

        self.assertEqual(result["status"], 1)
        self.assertEqual(len(calls), 1)
        command = calls[0][0]
        self.assertTrue(command.startswith("ServoCartForceFenJin("))
        parameters = command.removeprefix("ServoCartForceFenJin(").removesuffix(")").split(",")
        self.assertEqual(len(parameters), 21)
        self.assertEqual(parameters[7], "3")
        self.assertGreaterEqual(
            calls[0][1]["timeout_sec"],
            ft.FORCE_CONTINUOUS_FEN_CYCLE_S * 3,
        )

    def test_safety_failure_does_not_disable_or_fallback_continuous_motion(self):
        calls = []
        robot = types.SimpleNamespace()

        def call(cmd, **_kwargs):
            calls.append(cmd)
            return -2302, "-2302,-1,0.1,0.2,20,20,-10,0,0,0"

        robot._call = call
        controller = ft.Ros2ForceController(robot, target_force_n=10.0)

        with self.assertRaisesRegex(RuntimeError, "切向力超过软件限位"):
            controller.continuous_fen_jin(
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                8.0,
                1,
                -2.0,
                3.0,
                "test",
            )

        self.assertEqual(len(calls), 1)
        self.assertTrue(controller._continuous_fen_available)

    def test_unsupported_command_disables_continuous_mode_before_fallback(self):
        robot = types.SimpleNamespace(
            _call=mock.Mock(return_value=(-1, "-1")),
        )
        controller = ft.Ros2ForceController(robot, target_force_n=10.0)

        first = controller.continuous_fen_jin(
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            8.0,
            1,
            -2.0,
            3.0,
            "test",
        )
        second = controller.continuous_fen_jin(
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            8.0,
            1,
            -2.0,
            3.0,
            "test",
        )

        self.assertIsNone(first)
        self.assertIsNone(second)
        self.assertFalse(controller._continuous_fen_available)
        robot._call.assert_called_once()

    def test_force_fen_sequence_uses_one_continuous_command_for_all_rounds(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.hover_height_mm = 20.0
        demo.update_preview_status = mock.Mock()
        demo._pose_from_frame_offset = mock.Mock(return_value=[0.0] * 6)
        demo._contact_pose_from_frame = mock.Mock(return_value=[0.0] * 6)
        demo._move_to_hover_for_force = mock.Mock(return_value=True)
        demo._approach_to_target_force = mock.Mock(return_value=(1.0, True))
        demo._hold_target_force = mock.Mock(side_effect=lambda _f, _s, offset, _d, _c: (offset, True))
        demo._continuous_fen_round = mock.Mock(
            side_effect=lambda _f, offset, _c, _repeats: (offset + 0.1, True)
        )
        demo._move_force_pose_checked = mock.Mock(return_value=True)
        demo._retract_to_hover = mock.Mock(return_value=True)

        with mock.patch.object(ft, "LASTTIME_ROS2_FORCE", True), mock.patch.object(
            ft,
            "FEN_JIN_REPEAT_COUNT",
            3,
        ):
            result = demo.execute_fen_jin({"index": 0})

        self.assertTrue(result)
        demo._continuous_fen_round.assert_called_once_with(
            {"index": 0},
            1.0,
            "分筋上下连续拨动",
            3,
        )
        demo._hold_target_force.assert_not_called()
        demo._move_force_pose_checked.assert_not_called()
        demo._retract_to_hover.assert_called_once()

    def test_force_fen_fallback_only_visits_upper_and_lower_endpoints(self):
        demo = object.__new__(ft.LastTimeRos2Demo)
        demo.hover_height_mm = 20.0
        demo.update_preview_status = mock.Mock()
        split_offsets = []
        demo._pose_from_frame_offset = mock.Mock(
            side_effect=lambda _frame, _offset, split_offset=0.0: split_offsets.append(
                split_offset
            )
            or [0.0] * 6
        )
        demo._move_to_hover_for_force = mock.Mock(return_value=True)
        demo._approach_to_target_force = mock.Mock(return_value=(1.0, True))
        demo._continuous_fen_round = mock.Mock(return_value=None)
        demo._move_force_pose_checked = mock.Mock(return_value=True)
        demo._hold_target_force = mock.Mock(
            side_effect=lambda _f, _s, offset, _d, _c: (offset, True)
        )
        demo._retract_to_hover = mock.Mock(return_value=True)

        with mock.patch.object(ft, "LASTTIME_ROS2_FORCE", True), mock.patch.object(
            ft,
            "FEN_JIN_REPEAT_COUNT",
            2,
        ), mock.patch.object(ft, "FORCE_FEN_LATERAL_MM", 8.0):
            result = demo.execute_fen_jin({"index": 0})

        self.assertTrue(result)
        self.assertEqual(split_offsets, [0.0, 8.0, -8.0, 8.0, -8.0])
        held_offsets = [call.args[1] for call in demo._hold_target_force.call_args_list]
        self.assertEqual(held_offsets, [8.0, -8.0, 8.0, -8.0])
        self.assertNotIn(0.0, held_offsets)


if __name__ == "__main__":
    unittest.main()
