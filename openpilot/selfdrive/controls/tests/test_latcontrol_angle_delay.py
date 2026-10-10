from collections import deque

from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR as FORD
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces

LAT_DELAY = 0.5   # s, what lagd learns on the F-150


def _controller(car_name):
  CarInterface = interfaces[car_name]
  CP = CarInterface.get_non_essential_params(car_name)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
  CI = CarInterface(CP, CP_SP)
  sunnypilot_interfaces.setup_interfaces(CI)
  return LatControlAngle(CP.as_reader(), convert_to_capnp(CP_SP).as_reader(), CI, DT_CTRL), VehicleModel(CP)


def _curve_entry(car_name, delivery=1.0, wheel_delay=LAT_DELAY):
  """Ramp into a highway curve over 3 s and hold it. The wheel reaches `delivery` of what was asked,
  `wheel_delay` after it was asked. Returns whether the controller ever reported saturation."""
  controller, VM = _controller(car_name)
  CS = car.CarState.new_message()
  CS.vEgo = 30.
  params = log.VehicleParameters.new_message()
  asked = deque([0.0] * int(wheel_delay / DT_CTRL + 1), maxlen=int(wheel_delay / DT_CTRL + 1))
  saturated = False
  for i in range(600):
    desired_curvature = min(i * DT_CTRL / 3.0, 1.0) * 0.004
    CS.steeringAngleDeg = delivery * asked[0]
    _, angle_des, lac_log = controller.update(True, CS, VM, params, False, desired_curvature, None, False, LAT_DELAY)
    asked.append(angle_des)
    saturated |= lac_log.saturated
  return saturated


class TestLatControlAngleDelay:
  def test_on_time_wheel_is_not_saturated(self):
    """The wheel arrives exactly one lateral delay after it is asked: that is the truck being on
    time, not short, and must not raise the take-control alert."""
    assert not _curve_entry(FORD.FORD_F_150_MK14)

  def test_short_wheel_is_still_saturated(self):
    """A wheel that never reaches the command is real saturation whatever the alignment."""
    assert _curve_entry(FORD.FORD_F_150_MK14, delivery=0.8)

  def test_late_beyond_the_learned_delay_is_saturated(self):
    """A wheel later than the delay the planner compensates for is behind the road, and alerts."""
    assert _curve_entry(FORD.FORD_F_150_MK14, wheel_delay=1.0)

  def test_other_brands_unchanged(self):
    """Only Ford is aligned. Nissan's check still compares at the same instant, so the same on-time
    wheel reads as saturated there, as it always has."""
    assert _curve_entry(NISSAN.NISSAN_LEAF)
