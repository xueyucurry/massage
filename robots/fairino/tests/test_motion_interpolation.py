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
