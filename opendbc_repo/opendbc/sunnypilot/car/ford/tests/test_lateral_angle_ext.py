"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest
from collections import defaultdict
from types import SimpleNamespace

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR, CarControllerParams, FordFlags
from opendbc.sunnypilot.car.ford.human_turn import (
  HUMAN_TURN_ANGLE_DEG,
  HUMAN_TURN_HOLD_PRETURNED_S,
  HUMAN_TURN_HOLD_S,
)
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
from opendbc.sunnypilot.car.ford.values_ext import (
  FORD_DBC_PATH_ANGLE_MAX,
  FORD_DBC_PATH_ANGLE_MIN,
  T_IDXS,
  PrimaryLateralControl,
)

STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL

MSG_Lane_Assist_Data1 = 0x3CA
MSG_LateralMotionControl = 0x3D3
MSG_LateralMotionControl2 = 0x3D6

# LatCtlCurv_No_Actl and LatCtlPath_An_Actl inactive sentinels, in raw CAN units
INACTIVE_CURVATURE = 1000
INACTIVE_PATH_ANGLE = 1000


def make_cc(lat_active=True):
  return SimpleNamespace(latActive=lat_active)


def make_cc_sp(model_curvature=0.0, lateral_delay=0.12, lane_change_state=0, lane_change_direction=0,
               model_curvatures=None):
  curvatures = model_curvatures if model_curvatures is not None else [model_curvature] * len(T_IDXS)
  return SimpleNamespace(fordLateral=SimpleNamespace(
    modelCurvatures=curvatures,
    lateralDelay=lateral_delay,
    laneChangeState=lane_change_state,
    laneChangeDirection=lane_change_direction,
  ))


def make_cs(v_ego=30.0, yaw_rate=0.0, steering_pressed=False, steering_angle=0.0):
  return SimpleNamespace(out=SimpleNamespace(
    vEgoRaw=v_ego,
    yawRate=yaw_rate,
    steeringPressed=steering_pressed,
    steeringAngleDeg=steering_angle,
  ))


def make_actuators(curvature=0.0):
  return SimpleNamespace(curvature=curvature)


def make_car_params(platform=CAR.FORD_F_150_MK14, angle_mode=True, **tuning):
  CI_cls = interfaces[platform]
  fingerprint = dict.fromkeys(range(7), {})
  CP = CI_cls.get_params(platform, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CI_cls.get_params_sp(CP, platform, fingerprint, [], alpha_long=False, is_release_sp=False, docs=False)
  if angle_mode:
    CP_SP.fordLateralTuning.primaryControl = int(PrimaryLateralControl.angle)
    CP_SP.fordLateralTuning.lowSpeedFactor = tuning.get('low_speed_factor', 1.0)
    CP_SP.fordLateralTuning.highSpeedFactor = tuning.get('high_speed_factor', 1.0)
    CP_SP.fordLateralTuning.highSpeedDampening = tuning.get('high_speed_dampening', 1.0)
    CP_SP.fordLateralTuning.laneChangeFactor = tuning.get('lane_change_factor', 1.0)
  return CP, CP_SP


class TestLateralAngleExt(unittest.TestCase):
  def setUp(self):
    self.CP, self.CP_SP = make_car_params()
    self.lat = LateralAngleExt(self.CP, self.CP_SP)

  def _run(self, frames=1, **kwargs):
    """Step the strategy `frames` times with constant inputs and return the last result."""
    cc = make_cc(kwargs.pop('lat_active', True))
    curvature = kwargs.pop('curvature', 0.0)
    cs_kwargs = {k: kwargs.pop(k) for k in list(kwargs) if k in
                 ('v_ego', 'yaw_rate', 'steering_pressed', 'steering_angle')}
    cc_sp = make_cc_sp(**kwargs)
    result = None
    for _ in range(frames):
      result = self.lat.update(cc, cc_sp, make_cs(**cs_kwargs), make_actuators(curvature))
    return result

  def test_unset_tuning_falls_back_to_defaults(self):
    """CarParamsSP defaults floats to 0.0. A CarParamsSP written before this feature existed must
    not be read as zero steering gain."""
    _, CP_SP = make_car_params(angle_mode=False)
    CP_SP.fordLateralTuning.primaryControl = int(PrimaryLateralControl.angle)
    lat = LateralAngleExt(self.CP, CP_SP)
    self.assertEqual(lat.low_speed_factor, 1.0)
    self.assertEqual(lat.high_speed_factor, 1.0)
    self.assertEqual(lat.high_speed_dampening, 1.0)
    self.assertEqual(lat.lane_change_factor_high, 1.0)

  def test_tuning_is_clamped(self):
    _, CP_SP = make_car_params(low_speed_factor=99.0, high_speed_factor=-5.0,
                               high_speed_dampening=99.0, lane_change_factor=99.0)
    lat = LateralAngleExt(self.CP, CP_SP)
    self.assertEqual(lat.low_speed_factor, 1.5)
    self.assertEqual(lat.high_speed_factor, 0.5)
    self.assertEqual(lat.high_speed_dampening, 1.25)
    self.assertEqual(lat.lane_change_factor_high, 1.5)

  def test_inactive_is_all_zero_with_truthful_shadow(self):
    """While lateral is inactive the command is zero, but the shadow must track the measurement:
    the panda latches it from every LKA frame, so a stale zero would race the first enabled frame
    after re-engage."""
    result = self._run(lat_active=False, v_ego=30.0, yaw_rate=0.15)
    self.assertEqual(result.path_angle, 0.0)
    self.assertEqual(result.apply_curvature, 0.0)
    self.assertEqual(result.path_offset, 0.0)
    self.assertEqual(result.curvature_rate, 0.0)
    self.assertTrue(result.lat_inactive)
    self.assertAlmostEqual(self.lat.shadow_curvature, -0.15 / 30.0, places=6)

  def test_path_angle_tracks_kappa_times_speed(self):
    """The whole point of angle control: path_angle is kappa * v_ego * gain, so it grows with both
    curvature and speed rather than being a curvature the PSCM filters."""
    self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0), make_actuators(0.0))
    small = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0), make_actuators(0.0005)).path_angle
    self.assertGreater(small, 0.0)

    self.lat.path_angle_last = 0.0
    bigger = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0), make_actuators(0.001)).path_angle
    self.assertGreater(bigger, small)

    # sign follows the commanded curvature
    self.lat.path_angle_last = 0.0
    left = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0), make_actuators(-0.001)).path_angle
    self.assertLess(left, 0.0)

  def test_soft_rate_limit(self):
    """A large step in commanded curvature cannot produce a large step in path_angle."""
    for v_ego, max_step in ((10.0, 0.055), (15.0, 0.0425), (25.0, 0.009)):
      lat = LateralAngleExt(self.CP, self.CP_SP)
      last = 0.0
      for _ in range(5):
        result = lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=v_ego, yaw_rate=0.02 * v_ego),
                            make_actuators(0.02))
        self.assertLessEqual(abs(result.path_angle - last), max_step + 1e-9,
                             f"step too large at {v_ego} m/s")
        last = result.path_angle

  def test_never_exceeds_dbc_range(self):
    lat = LateralAngleExt(self.CP, self.CP_SP)
    for _ in range(200):
      result = lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=35.0, yaw_rate=0.02 * 35.0),
                          make_actuators(0.02))
      self.assertLessEqual(result.path_angle, FORD_DBC_PATH_ANGLE_MAX + 1e-9)
      self.assertGreaterEqual(result.path_angle, FORD_DBC_PATH_ANGLE_MIN - 1e-9)

  def test_deviation_clip_binds_and_is_reported(self):
    """The command is clipped to measured curvature +- CURVATURE_ERROR, the same clip the stock
    curvature path applies. Without it the shadow would leave the panda's error band routinely."""
    result = self._run(v_ego=30.0, yaw_rate=0.0, curvature=0.02)
    self.assertTrue(self.lat.curvature_deviation_limited)
    self.assertLessEqual(abs(self.lat.shadow_curvature), CarControllerParams.CURVATURE_ERROR + 1e-9)
    self.assertGreater(result.path_angle, 0.0)

  def test_no_deviation_clip_at_low_speed(self):
    self._run(v_ego=5.0, yaw_rate=0.0, curvature=0.02)
    self.assertFalse(self.lat.curvature_deviation_limited)

  def test_human_turn_forces_lateral_inactive(self):
    # the driver winds the wheel up through the threshold, rather than grabbing an already-turned one
    self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0, steering_pressed=True, steering_angle=5.0),
                    make_actuators(0.01))
    frames = int(HUMAN_TURN_HOLD_S / STEER_DT) + 2
    result = None
    for _ in range(frames):
      result = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0, steering_pressed=True,
                                                                steering_angle=HUMAN_TURN_ANGLE_DEG + 10.0),
                               make_actuators(0.01))
    self.assertTrue(self.lat.human_turn_active)
    self.assertTrue(result.lat_inactive)
    self.assertEqual(result.path_angle, 0.0)
    self.assertEqual(result.ramp_type, 0)

    # on release the command ramps back in from zero rather than snapping to a stale value
    released = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0), make_actuators(0.01))
    self.assertFalse(released.lat_inactive)
    self.assertLessEqual(abs(released.path_angle), 0.0425 + 1e-9)

  def test_grabbing_an_already_turned_wheel_needs_a_longer_hold(self):
    """Lateral control turns the wheel past the angle threshold on its own in a curve, so a brief
    corrective nudge there must not read as a takeover and kill steering mid-curve."""
    frames = int(HUMAN_TURN_HOLD_S / STEER_DT) + 2
    for _ in range(frames):
      self.lat.update(make_cc(), make_cc_sp(),
                      make_cs(v_ego=15.0, steering_pressed=True, steering_angle=HUMAN_TURN_ANGLE_DEG + 10.0),
                      make_actuators(0.01))
    self.assertFalse(self.lat.human_turn_active)

    for _ in range(int((HUMAN_TURN_HOLD_PRETURNED_S - HUMAN_TURN_HOLD_S) / STEER_DT) + 2):
      self.lat.update(make_cc(), make_cc_sp(),
                      make_cs(v_ego=15.0, steering_pressed=True, steering_angle=HUMAN_TURN_ANGLE_DEG + 10.0),
                      make_actuators(0.01))
    self.assertTrue(self.lat.human_turn_active)

  def test_brief_nudge_is_not_a_human_turn(self):
    for _ in range(int(1.0 / STEER_DT)):
      result = self.lat.update(make_cc(), make_cc_sp(),
                               make_cs(v_ego=15.0, steering_pressed=True, steering_angle=10.0),
                               make_actuators(0.01))
    self.assertFalse(self.lat.human_turn_active)
    self.assertFalse(result.lat_inactive)

  def test_hand_off_blip_on_release_of_a_sustained_press(self):
    """A sustained press attenuates the PSCM; a short mode-0 pulse on release resets it while the
    car is still straight, instead of waiting for the reactive detector to watch a curve be missed."""
    press_frames = int(0.6 / STEER_DT)
    for _ in range(press_frames):
      self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0, steering_pressed=True, steering_angle=5.0),
                      make_actuators(0.0))
    self.assertFalse(self.lat.human_turn_active)

    result = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0), make_actuators(0.0))
    self.assertTrue(result.lat_inactive)
    self.assertTrue(self.lat.stall_blip_active)

    # the pulse is short and self-clearing
    for _ in range(8):
      result = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0), make_actuators(0.0))
    self.assertFalse(result.lat_inactive)
    self.assertFalse(self.lat.stall_blip_active)

  def test_stall_blip_fires_when_the_command_cannot_lead_the_car(self):
    """Hands-free, with the deviation clip binding and the car not following, a pulse must fire."""
    fired = False
    for _ in range(int(3.0 / STEER_DT)):
      result = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0, yaw_rate=0.0),
                               make_actuators(0.02))
      if result.lat_inactive:
        fired = True
        break
    self.assertTrue(fired, "stall blip never fired while the deviation clip was pinned")

  def test_lane_change_scaling(self):
    """The lane change factor scales authority only in the direction of the change."""
    base = LateralAngleExt(self.CP, self.CP_SP)
    boosted_cp, boosted_cp_sp = make_car_params(lane_change_factor=1.5)
    boosted = LateralAngleExt(boosted_cp, boosted_cp_sp)

    # laneChangeState=2 (starting), direction=1 (left), curvature negative == left
    args = (make_cc(), make_cc_sp(lane_change_state=2, lane_change_direction=1),
            make_cs(v_ego=12.0, yaw_rate=0.0), make_actuators(-0.0015))
    base_angle = base.update(*args).path_angle
    boosted_angle = boosted.update(*args).path_angle
    self.assertLess(boosted_angle, base_angle)  # both negative; boosted is larger in magnitude

    # precision drops to Comfortable during a lane change in the change direction
    self.assertEqual(base.update(*args).precision_type, 0)

  def test_model_blend_moves_the_command(self):
    """The model's predicted curvature is blended into the planner's, so a model that disagrees
    with the planner changes the command."""
    planner_only = LateralAngleExt(self.CP, self.CP_SP).update(
      make_cc(), make_cc_sp(model_curvature=0.0), make_cs(v_ego=12.0), make_actuators(0.001)).path_angle
    with_model = LateralAngleExt(self.CP, self.CP_SP).update(
      make_cc(), make_cc_sp(model_curvature=0.003), make_cs(v_ego=12.0), make_actuators(0.001)).path_angle
    self.assertGreater(with_model, planner_only)

  def test_missing_model_data_is_survivable(self):
    """carControlSP can arrive before modelV2 has been seen; the planner command must still work."""
    result = self.lat.update(make_cc(), make_cc_sp(model_curvatures=[]), make_cs(v_ego=12.0),
                             make_actuators(0.001))
    self.assertGreater(result.path_angle, 0.0)

  def test_platform_gains_differ(self):
    truck_cp, truck_cp_sp = make_car_params(CAR.FORD_F_150_MK14)
    suv_cp, suv_cp_sp = make_car_params(CAR.FORD_MUSTANG_MACH_E_MK1)
    self.assertNotEqual(LateralAngleExt(truck_cp, truck_cp_sp).path_angle_gain_high_curv,
                        LateralAngleExt(suv_cp, suv_cp_sp).path_angle_gain_high_curv)


class TestFordAngleCarController(unittest.TestCase):
  """End to end through the real CarController: the messages on the wire must match what
  safety/modes/ford.h expects from angle mode."""

  def _build(self, platform, angle_mode=True):
    CP, CP_SP = make_car_params(platform, angle_mode=angle_mode)
    CI = interfaces[platform](CP, CP_SP)
    return CI.CC, CP

  @staticmethod
  def _cs():
    cs = make_cs(v_ego=30.0, yaw_rate=0.0)
    cs.out.cruiseState = SimpleNamespace(available=True, standstill=False)
    cs.out.vEgo = 30.0
    cs.buttons_stock_values = defaultdict(int)
    cs.acc_tja_status_stock_values = defaultdict(int)
    cs.lkas_status_stock_values = defaultdict(int)
    return cs

  @staticmethod
  def _cc(lat_active=True, curvature=0.002):
    return SimpleNamespace(
      latActive=lat_active,
      longActive=False,
      enabled=lat_active,
      actuators=SimpleNamespace(curvature=curvature, accel=0.0, gas=0.0,
                                longControlState=structs.CarControl.Actuators.LongControlState.off,
                                as_builder=lambda: SimpleNamespace(curvature=0.0, accel=0.0, gas=0.0)),
      hudControl=SimpleNamespace(visualAlert=structs.CarControl.HUDControl.VisualAlert.none,
                                 leadDistanceBars=0, leftLaneDepart=False, rightLaneDepart=False,
                                 leftLaneVisible=True, rightLaneVisible=True, lanesVisible=True,
                                 setSpeed=0.0, speedVisible=False, leadVisible=False),
      cruiseControl=SimpleNamespace(cancel=False, resume=False, override=False),
      orientationNED=[],
    )

  def _drive(self, platform, angle_mode=True, frames=40):
    cc_controller, CP = self._build(platform, angle_mode)
    cc_sp = make_cc_sp()
    sends = []
    for _ in range(frames):
      _, can_sends = cc_controller.update(self._cc(), cc_sp, self._cs(), 0)
      sends.extend(can_sends)
    return cc_controller, CP, sends

  def _steer_msgs(self, sends, addr):
    return [m for m in sends if m[0] == addr]

  def test_angle_mode_wire_format(self):
    for platform in (CAR.FORD_F_150_MK14, CAR.FORD_ESCAPE_MK4):
      with self.subTest(platform=platform):
        controller, CP, sends = self._drive(platform)
        addr = MSG_LateralMotionControl2 if CP.flags & FordFlags.CANFD else MSG_LateralMotionControl
        steer = self._steer_msgs(sends, addr)
        self.assertTrue(steer)

        for _, dat, _ in steer:
          if addr == MSG_LateralMotionControl:
            raw_curvature = (dat[0] << 3) | (dat[1] >> 5)
            raw_curvature_rate = ((dat[1] & 0x1F) << 8) | dat[2]
            raw_path_angle = (dat[3] << 3) | (dat[4] >> 5)
            raw_path_offset = (dat[5] << 2) | (dat[6] >> 6)
            inactive_curvature_rate = 4096
          else:
            raw_curvature = (dat[2] << 3) | (dat[3] >> 5)
            raw_curvature_rate = (dat[6] << 3) | (dat[7] >> 5)
            raw_path_angle = ((dat[3] & 0x1F) << 6) | (dat[4] >> 2)
            raw_path_offset = ((dat[4] & 0x3) << 8) | dat[5]
            inactive_curvature_rate = 1024
          # c0/c2/c3 pinned at their sentinels -- ford.h rejects the frame otherwise
          self.assertEqual(raw_curvature, INACTIVE_CURVATURE)
          self.assertEqual(raw_curvature_rate, inactive_curvature_rate)
          self.assertEqual(raw_path_offset, 512)
          self.assertNotEqual(raw_path_angle, INACTIVE_PATH_ANGLE)  # c1 is doing the steering

  def test_lka_carries_angle_mode_state(self):
    controller, _, sends = self._drive(CAR.FORD_F_150_MK14)
    lka = self._steer_msgs(sends, MSG_Lane_Assist_Data1)
    self.assertTrue(lka)
    _, dat, _ = lka[-1]
    self.assertEqual(dat[4] & 0x1, 1)
    shadow = int.from_bytes(dat[5:7], 'big', signed=True) * 1e-6
    # published in the CAN sign convention, i.e. negated from openpilot's
    self.assertAlmostEqual(shadow, -controller.lat_angle.shadow_curvature, places=5)

  def test_curvature_mode_is_byte_for_byte_stock(self):
    """With angle control off nothing about the stock path may change, including the LKA message."""
    _, CP, sends = self._drive(CAR.FORD_F_150_MK14, angle_mode=False)
    for _, dat, _ in self._steer_msgs(sends, MSG_Lane_Assist_Data1):
      self.assertEqual(bytes(dat), b"\x00" * 8)

    addr = MSG_LateralMotionControl2 if CP.flags & FordFlags.CANFD else MSG_LateralMotionControl
    for _, dat, _ in self._steer_msgs(sends, addr):
      raw_path_angle = ((dat[3] & 0x1F) << 6) | (dat[4] >> 2)
      self.assertEqual(raw_path_angle, INACTIVE_PATH_ANGLE)


if __name__ == "__main__":
  unittest.main()
