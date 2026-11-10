"""Regression checks for NaN classification and per-world capacity validity."""
import unittest

import numpy as np

from strict_v3_common import box_v4_summary


class BoxValidityTests(unittest.TestCase):
    def test_nonfinite_is_invalid_and_overflow_is_world_specific(self):
        t = np.linspace(0, 1, 6)
        pos = np.zeros((6, 4, 3, 3)); pos[..., 2] = [.02, .06, .10]
        quat = np.zeros((6, 4, 3, 4)); quat[..., 0] = 1
        pos[2, 1, 0, 0] = np.nan
        quat[2, 2, 0, 0] = np.nan
        result = box_v4_summary(t, pos, quat, np.ones((6, 4)), np.ones((6, 4)),
                                half=.02, pushed=2, expected="retain", finite=np.ones(4, bool),
                                truncated=np.array([False, False, False, True]))
        self.assertEqual([r["observed"] for r in result["rollouts"]],
                         ["retain", "invalid", "invalid", "retain"])
        self.assertEqual(result["agree_and_valid_count"], 1)
        self.assertEqual(result["finite_count"], 2)
        self.assertEqual(result["classification_correct_finite_count"], 2)
        self.assertEqual(result["capacity_overflow_worlds"], 1)
        self.assertEqual([r["agrees"] for r in result["rollouts"]], [True, False, False, True])


if __name__ == "__main__":
    unittest.main()
