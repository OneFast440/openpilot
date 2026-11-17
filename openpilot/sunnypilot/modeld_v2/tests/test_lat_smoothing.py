import numpy as np

import openpilot.sunnypilot.modeld_v2.modeld as modeld_module
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.modeld import BIG_LAT_SMOOTH_SECONDS
from openpilot.sunnypilot.modeld_v2.modeld import ModelState, get_lat_smooth_seconds
from openpilot.sunnypilot.modeld_v2.tests.helpers import DummyBundle, DummyOverride

V_EGO = 30.


def overrides(lat):
  return {'lat': lat, 'long': '.3'}


def test_big_model_gets_the_stock_floor():
  # Mountain Dew v3 as the model list sends it today
  assert get_lat_smooth_seconds(overrides('.0'), chestnut=True, generation=12) == BIG_LAT_SMOOTH_SECONDS == .1
  assert get_lat_smooth_seconds({}, chestnut=True, generation=12) == BIG_LAT_SMOOTH_SECONDS


def test_big_model_keeps_a_longer_bundle_value():
  assert get_lat_smooth_seconds(overrides('.15'), chestnut=True, generation=12) == .15


def test_small_model_is_unchanged():
  assert get_lat_smooth_seconds(overrides('.0'), chestnut=False, generation=12) == 0.
  assert get_lat_smooth_seconds(overrides('.1'), chestnut=False, generation=12) == .1


def test_no_floor_where_curvature_is_not_smoothed():
  # pre-FOF generations and a COMBINED_MODEL_PKL override (no bundle) never smooth, so the floor would only add delay
  assert get_lat_smooth_seconds(overrides('.0'), chestnut=True, generation=9) == 0.
  assert get_lat_smooth_seconds({}, chestnut=True, generation=None) == 0.


def test_model_state_uses_the_floor(monkeypatch):
  bundle = DummyBundle(generation=12)
  bundle.overrides = [DummyOverride('lat', '.0'), DummyOverride('long', '.3')]
  monkeypatch.delenv('COMBINED_MODEL_PKL', raising=False)
  monkeypatch.setattr(modeld_module, 'get_active_bundle', lambda params=None, *, chestnut=None: bundle)
  monkeypatch.setattr(modeld_module, '_find_driving_pkl', lambda b: 'unused.pkl')
  monkeypatch.setattr(ModelState, '_init_combined', lambda self, *args: None)
  assert ModelState(1928, 1208, chestnut=True).LAT_SMOOTH_SECONDS == BIG_LAT_SMOOTH_SECONDS
  assert ModelState(1928, 1208, chestnut=False).LAT_SMOOTH_SECONDS == 0.


def bare_state(lat_smooth_seconds):
  state = object.__new__(ModelState)
  state.generation, state.MIN_LAT_CONTROL_SPEED = 12, 0.3
  state.LAT_SMOOTH_SECONDS, state.LONG_SMOOTH_SECONDS = lat_smooth_seconds, .3
  return state


def run(state, lat_accels):
  action, out = log.ModelDataV2.Action(desiredCurvature=0.), []
  for lat_accel in lat_accels:
    action = state.get_action_from_model({'action': np.array([[lat_accel, 0.]])}, action, 0., 0., V_EGO)
    out.append(action.desiredCurvature)
  return np.array(out)


def test_step_response_follows_the_time_constant():
  out = run(bare_state(BIG_LAT_SMOOTH_SECONDS), [1.] * 10)
  t = DT_MDL * np.arange(1, 11)
  np.testing.assert_allclose(out, (1 - np.exp(-t / BIG_LAT_SMOOTH_SECONDS)) / V_EGO ** 2, rtol=1e-6)


def test_unsmoothed_bundle_passes_straight_through():
  np.testing.assert_allclose(run(bare_state(0.), [1., -1., 1.]), np.array([1., -1., 1.]) / V_EGO ** 2, rtol=1e-6)
