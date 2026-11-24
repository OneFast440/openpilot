from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_ACCEL_NO_ROLL, clip_curvature

V = 20.0  # m/s


def steady(lat_accel: float) -> float:
  return lat_accel / V ** 2


class TestClipCurvature(OpenpilotTestCase):
  """sunnypilot: clip_curvature's flat-road ceiling is comma's 3.0 m/s^2 unless a car port raises it."""

  def test_the_default_is_commas_ceiling(self):
    curvature, limited = clip_curvature(V, steady(4.0), steady(4.0), 0.0)
    assert limited
    self.assertAlmostEqual(curvature, steady(MAX_LATERAL_ACCEL_NO_ROLL), places=12)

  def test_a_raised_ceiling_lets_its_own_value_through_and_no_more(self):
    curvature, limited = clip_curvature(V, steady(4.0), steady(4.0), 0.0, 4.0)
    assert not limited
    self.assertAlmostEqual(curvature, steady(4.0), places=12)
    curvature, limited = clip_curvature(V, steady(5.0), steady(5.0), 0.0, 4.0)
    assert limited
    self.assertAlmostEqual(curvature, steady(4.0), places=12)

  def test_road_bank_still_shifts_the_window(self):
    roll = 0.05
    curvature, _ = clip_curvature(V, steady(6.0), steady(6.0), roll, 4.0)
    self.assertAlmostEqual(curvature, steady(4.0 + roll * ACCELERATION_DUE_TO_GRAVITY), places=12)

  def test_the_jerk_limit_is_unchanged(self):
    commas, _ = clip_curvature(V, 0.0, steady(4.0), 0.0)
    raised, _ = clip_curvature(V, 0.0, steady(4.0), 0.0, 4.0)
    assert commas == raised < steady(MAX_LATERAL_ACCEL_NO_ROLL)
