"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

clip_curvature's limits are overridable per platform. The point of these is that the defaults
are byte-for-byte the old behaviour, so no car that does not ask for more sees any change.
"""
import unittest

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import (MAX_LATERAL_ACCEL_NO_ROLL, MAX_LATERAL_JERK,
                                                            clip_curvature)


class TestClipCurvatureLimits(unittest.TestCase):
  def test_the_defaults_are_the_iso_values(self):
    for v_ego in (5.0, 11.0, 25.0):
      for new in (-0.05, 0.0, 0.002, 0.05):
        self.assertEqual(clip_curvature(v_ego, 0.0, new, 0.0),
                         clip_curvature(v_ego, 0.0, new, 0.0,
                                        MAX_LATERAL_ACCEL_NO_ROLL, MAX_LATERAL_JERK),
                         (v_ego, new))

  def test_the_accel_limit_sets_the_tightest_curve(self):
    v_ego = 11.0                                       # about 25 mph
    # far past the cap and given room to get there, so the rate limit is not what decides
    prev = 0.0
    for _ in range(2000):
      prev, _ = clip_curvature(v_ego, prev, 1.0, 0.0)
    self.assertAlmostEqual(prev, MAX_LATERAL_ACCEL_NO_ROLL / v_ego ** 2, places=6)

    raised = 0.0
    for _ in range(2000):
      raised, _ = clip_curvature(v_ego, raised, 1.0, 0.0, 4.5, MAX_LATERAL_JERK)
    self.assertAlmostEqual(raised, 4.5 / v_ego ** 2, places=6)
    self.assertGreater(raised, prev)

  def test_the_jerk_limit_sets_how_fast_it_winds_in(self):
    v_ego = 11.0
    one, _ = clip_curvature(v_ego, 0.0, 1.0, 0.0)
    self.assertAlmostEqual(one, MAX_LATERAL_JERK / v_ego ** 2 * DT_CTRL, places=9)
    faster, _ = clip_curvature(v_ego, 0.0, 1.0, 0.0, MAX_LATERAL_ACCEL_NO_ROLL, 10.0)
    self.assertAlmostEqual(faster, 10.0 / v_ego ** 2 * DT_CTRL, places=9)
    self.assertGreater(faster, one)

  def test_both_limits_still_fall_with_the_square_of_speed(self):
    slow, _ = clip_curvature(5.0, 0.0, 1.0, 0.0, 4.5, 10.0)
    fast, _ = clip_curvature(25.0, 0.0, 1.0, 0.0, 4.5, 10.0)
    self.assertGreater(slow, fast * 20)

  def test_the_limited_flag_still_reports_the_accel_clamp(self):
    v_ego = 11.0
    prev = 0.0
    limited = False
    for _ in range(2000):
      prev, limited = clip_curvature(v_ego, prev, 1.0, 0.0)
    self.assertTrue(limited)


if __name__ == "__main__":
  unittest.main()
