import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.modeld import (BIG_LAT_SMOOTH_SECONDS, LAT_SMOOTH_SECONDS, MIN_LAT_CONTROL_SPEED,
                                               get_action_from_model, get_lat_smooth_seconds)

V_EGO = 25.


def curvature(lat_accel):
  return lat_accel / V_EGO ** 2


def run(lat_accels, v_ego=V_EGO, start=0., **kwargs):
  """Desired curvature over consecutive model frames that ask for the given lateral accelerations."""
  action, out = log.ModelDataV2.Action(desiredCurvature=start), []
  for lat_accel in lat_accels:
    action = get_action_from_model({'action': np.array([[lat_accel, 0.]])}, action, 0., 0., v_ego, **kwargs)
    out.append(action.desiredCurvature)
  return np.array(out)


def test_only_the_big_model_is_smoothed():
  # the small model keeps upstream's value: if upstream starts smoothing, revisit the big model's
  assert LAT_SMOOTH_SECONDS == 0., "upstream changed its lateral smoothing, check BIG_LAT_SMOOTH_SECONDS still makes sense"
  assert get_lat_smooth_seconds(False) == LAT_SMOOTH_SECONDS
  assert get_lat_smooth_seconds(True) == BIG_LAT_SMOOTH_SECONDS == .1


def test_small_model_passes_straight_through():
  asked = [1., -.5, 2., 0.]
  np.testing.assert_allclose(run(asked), curvature(np.array(asked)), rtol=1e-6)
  np.testing.assert_allclose(run(asked, lat_smooth_seconds=get_lat_smooth_seconds(False)), curvature(np.array(asked)), rtol=1e-6)


def test_big_model_follows_a_step_with_a_tenth_of_a_second_time_constant():
  out = run([1.] * 40, lat_smooth_seconds=get_lat_smooth_seconds(True))
  elapsed = DT_MDL * np.arange(1, 41)
  np.testing.assert_allclose(out, curvature(1.) * (1 - np.exp(-elapsed / BIG_LAT_SMOOTH_SECONDS)), rtol=1e-5)
  assert out[1] == pytest.approx(.632 * curvature(1.), rel=1e-3)     # 63% of the way after one time constant
  assert out[-1] == pytest.approx(curvature(1.), rel=1e-6)            # and all the way in the end


def test_big_model_frame_to_frame_changes_are_cut_to_a_quarter():
  asked = 1. + .2 * np.tile([1., -1.], 100)        # a steady turn with the worst case on top: a flip every frame
  ripple = np.ptp(run(asked, lat_smooth_seconds=get_lat_smooth_seconds(True))[-20:]) / np.ptp(curvature(asked))
  assert ripple == pytest.approx(.245, abs=.005)
  assert np.ptp(run(asked)[-20:]) == pytest.approx(np.ptp(curvature(asked)), rel=1e-6)


def test_one_hertz_is_mostly_kept():
  t = DT_MDL * np.arange(400)
  out = run(np.sin(2 * np.pi * t), lat_smooth_seconds=get_lat_smooth_seconds(True))
  assert np.abs(out[200:]).max() / curvature(1.) == pytest.approx(.85, abs=.02)


def test_standing_still_holds_the_last_value_either_way():
  for kwargs in ({}, {'lat_smooth_seconds': get_lat_smooth_seconds(True)}):
    np.testing.assert_allclose(run([1., 2.], v_ego=MIN_LAT_CONTROL_SPEED / 2, start=.003, **kwargs), .003, rtol=1e-6)
