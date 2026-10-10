"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt, FORD_LANE_LOOKAHEAD_S

X = [float(x) for x in np.linspace(0.0, 100.0, 33)]


def _model(plan_y, left_y=-1.8, right_y=1.8):
  """A model whose plan sits at plan_y(x) between straight lane lines (model frame, right positive)."""
  model = messaging.new_message('modelV2').modelV2
  model.position.x = X
  model.position.y = [float(plan_y(x)) for x in X]
  lines = model.init('laneLines', 4)
  for line, y in zip(lines, (-5.4, left_y, right_y, 5.4), strict=True):
    line.x = X
    line.y = [y] * len(X)
  model.laneLineProbs = [0.5, 0.9, 0.9, 0.5]
  return model


def _ahead(model, v_ego):
  dest = custom.CarControlSP.new_message().fordLateral
  ControlsExt.get_ford_lane_ahead(dest, model, v_ego)
  return dest


class TestFordLaneAhead:
  def test_plan_offset_against_lane_centre_at_the_lookahead(self):
    """Plan drifting 0.02 m left per metre, lanes centred 0.1 m right: at 20 m/s the point is 30 m
    ahead, where the plan is 0.6 m left and the centre 0.1 m right."""
    dest = _ahead(_model(lambda x: -0.02 * x, left_y=-1.7, right_y=1.9), 20.0)
    assert abs(dest.laneLookahead - 20.0 * FORD_LANE_LOOKAHEAD_S) < 1e-6
    assert abs(dest.planLaneOffsetAhead - (-0.6 - 0.1)) < 1e-6
    assert abs(dest.laneWidthAhead - 3.6) < 1e-6

  def test_lookahead_is_bounded(self):
    assert _ahead(_model(lambda x: 0.0), 2.0).laneLookahead == 10.0
    assert _ahead(_model(lambda x: 0.0), 60.0).laneLookahead == 60.0

  def test_missing_geometry_leaves_it_off(self):
    model = _model(lambda x: 0.0)
    model.position.x = []
    model.position.y = []
    assert _ahead(model, 20.0).laneLookahead == 0.0
