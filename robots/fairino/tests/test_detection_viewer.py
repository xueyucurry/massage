import argparse
import unittest
from unittest import mock

import detection_viewer


class DetectionViewerTargetTests(unittest.TestCase):
    def test_public_targets_and_aliases(self):
        expected = {
            "back": "back",
            "spine": "back",
            "thigh": "leg",
            "outer": "leg",
            "leg": "leg",
            "thigh-inner": "leg_inner",
            "inner": "leg_inner",
            "inner_thigh": "leg_inner",
        }
        for value, target in expected.items():
            with self.subTest(value=value):
                self.assertEqual(detection_viewer.normalize_view_target(value), target)

    def test_unknown_target_is_rejected(self):
        with self.assertRaises(argparse.ArgumentTypeError):
            detection_viewer.normalize_view_target("shoulder")

    def test_vision_demo_does_not_load_motion_posture_seeds(self):
        seen = {}

        def build_demo(massage_target):
            seen["target"] = massage_target
            seen["inner_seed"] = detection_viewer.ft.THIGH_INNER_POSTURE_SEED_ENABLE
            seen["back_seed"] = detection_viewer.ft.BACK_POSTURE_SEED_ENABLE
            return object()

        with mock.patch.object(
            detection_viewer.ft,
            "THIGH_INNER_POSTURE_SEED_ENABLE",
            True,
        ), mock.patch.object(
            detection_viewer.ft,
            "BACK_POSTURE_SEED_ENABLE",
            True,
        ), mock.patch.object(
            detection_viewer.ft,
            "LastTimeRos2Demo",
            side_effect=build_demo,
        ):
            demo = detection_viewer._create_vision_only_demo("leg_inner")

        self.assertIsNotNone(demo)
        self.assertEqual(seen["target"], "leg_inner")
        self.assertFalse(seen["inner_seed"])
        self.assertFalse(seen["back_seed"])


if __name__ == "__main__":
    unittest.main()
