import unittest
from collections import deque

from shoulder_symmetry import stable_result


class StableResultTests(unittest.TestCase):
    def test_thirty_fps_can_lock_after_one_second(self):
        history = deque()
        for index in range(30):
            history.append((index / 30.0, 0.08))
            self.assertIsNone(stable_result(history, 0.05, 20))
        history.append((1.0, 0.08))
        self.assertAlmostEqual(stable_result(history, 0.05, 20), 0.08)

    def test_old_samples_do_not_force_a_new_result(self):
        history = deque((index / 30.0, 0.08) for index in range(40))
        for index in range(40, 80):
            history.append((index / 30.0, -0.08))
        self.assertAlmostEqual(stable_result(history, 0.05, 20), -0.08)


if __name__ == "__main__":
    unittest.main()
