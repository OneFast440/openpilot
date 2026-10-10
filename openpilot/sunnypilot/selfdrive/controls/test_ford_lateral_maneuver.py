"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt


class FakeSM:
  def __init__(self, maneuver_curvature=None, model_curvature=0.0, v_ego=6.0):
    model = messaging.new_message('modelV2').modelV2
    model.orientationRate.z = [model_curvature * v_ego] * 33
    model.position.y = [0.0] * 33
    car_state = messaging.new_message('carState').carState
    car_state.vEgo = v_ego
    plan = messaging.new_message('lateralManeuverPlan').lateralManeuverPlan
    if maneuver_curvature is not None:
      plan.desiredCurvature = maneuver_curvature
    self.msgs = {
      'modelV2': model, 'carState': car_state, 'lateralManeuverPlan': plan,
      'lateralDelay': messaging.new_message('lateralDelay').lateralDelay,
      'selfdriveState': messaging.new_message('selfdriveState').selfdriveState,
    }
    self.valid = {'lateralManeuverPlan': maneuver_curvature is not None}

  def __getitem__(self, key):
    return self.msgs[key]


def _ford_lateral(sm):
  dest = custom.CarControlSP.new_message().fordLateral
  ControlsExt.get_ford_lateral(dest, sm)
  return list(dest.modelCurvatures)


class TestFordLateralUnderManeuvers:
  def test_model_prediction_when_no_maneuver(self):
    curvatures = _ford_lateral(FakeSM(model_curvature=0.01))
    assert len(curvatures) == 33
    assert all(abs(c - 0.01) < 1e-6 for c in curvatures)

  def test_maneuver_replaces_the_prediction(self):
    """On an empty lot the model predicts straight ahead; the angle controller blends that 50/50
    with the planner, which would halve every maneuver. The prediction must be the maneuver."""
    curvatures = _ford_lateral(FakeSM(maneuver_curvature=0.05, model_curvature=0.0))
    assert len(curvatures) == 33
    assert all(abs(c - 0.05) < 1e-6 for c in curvatures)

  def test_maneuver_blend_is_a_no_op_in_the_controller(self):
    """End to end through the real angle controller: with the maneuver as the prediction, the
    requested curvature is the maneuver, not half of it."""
    from opendbc.car.ford.values import CAR
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
    from opendbc.sunnypilot.car.ford.tests.helpers import make_actuators, make_car_params, make_cc, make_cc_sp, make_cs
    from opendbc.sunnypilot.car.ford.values_ext import PrimaryLateralControl
    CP, CP_SP = make_car_params(CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.angle)
    kappa, v_ego = 0.02, 6.0
    out = {}
    for label, sm in (("fixed", FakeSM(maneuver_curvature=kappa)), ("model only", None)):
      ctrl = LateralAngleExt(CP, CP_SP)
      prediction = _ford_lateral(sm) if sm is not None else [0.0] * 33
      for _ in range(60):
        ctrl.update(make_cc(curvature=kappa), make_cc_sp(model_curvatures=prediction),
                    make_cs(v_ego=v_ego, yaw_rate=-kappa * v_ego), make_actuators(kappa))
      out[label] = ctrl.shadow_curvature
    assert abs(out["fixed"] - kappa) < 1e-6
    assert abs(out["model only"] - kappa / 2) < 1e-6
