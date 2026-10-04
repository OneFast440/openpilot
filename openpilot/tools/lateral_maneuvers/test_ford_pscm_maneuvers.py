from types import SimpleNamespace

import numpy as np

from openpilot.common.constants import CV
from openpilot.tools.lateral_maneuvers.lateral_maneuversd import (
  FORD_PSCM_MANEUVERS, MANEUVERS, TIMER, _F150_STEER_RATIO, _F150_WHEELBASE, _MEASURE_S, _TURN_IN_RATE, maneuvers_for,
)

DT = 0.05

DBC_PATH_ANGLE_MAX = 0.5235   # rad, LatCtlPath_An_Actl
SCHEDULE_GAIN = 1.3           # the angle controller's low-speed gain with default factors


def _peak_accel(m):
  return max(abs(a) for action in m.actions for a in action.accel_bp)


class TestFordPSCMSuite:
  def test_selected_by_brand(self):
    assert maneuvers_for(SimpleNamespace(brand="ford")) is FORD_PSCM_MANEUVERS
    assert maneuvers_for(SimpleNamespace(brand="toyota")) is MANEUVERS

  def test_every_level_fits_the_path_angle_signal(self):
    """A level past the signal range would measure the DBC clip, not the PSCM."""
    for m in FORD_PSCM_MANEUVERS:
      v = m.initial_speed
      path_angle = _peak_accel(m) / v ** 2 * v * SCHEDULE_GAIN
      assert path_angle < 0.9 * DBC_PATH_ANGLE_MAX, m.description

  def test_fits_the_lot_in_the_protocol(self):
    """FORD_PSCM.md asks for a 100 m run and 70 m to the side. Traced with the truck following the
    request exactly, from the 2 s countdown at speed to the end of the straight after it."""
    for m in FORD_PSCM_MANEUVERS:
      v = m.initial_speed
      accel = [0.0] * int(TIMER / DT)
      for action in m.actions:
        accel += list(np.interp(np.arange(0.0, action.time_bp[-1], DT), action.time_bp, action.accel_bp))
      heading = np.cumsum(np.array(accel) / v * DT)
      x = np.cumsum(v * np.cos(heading) * DT)
      y = np.cumsum(v * np.sin(heading) * DT)
      assert np.ptp(x) < 100 and np.ptp(y) < 70, f"{m.description}: {np.ptp(x):.0f} x {np.ptp(y):.0f} m"

  def test_holds_outlast_the_turn_in(self):
    """A hold that ends while the wheel is still coming in measures lag, not delivery."""
    for m in FORD_PSCM_MANEUVERS:
      if not m.description.startswith("hold"):
        continue
      hold = m.actions[0]
      accel = max(abs(a) for a in hold.accel_bp)
      wheel_deg = np.degrees(accel / m.initial_speed ** 2 * _F150_WHEELBASE * _F150_STEER_RATIO)
      assert hold.time_bp[2] - hold.time_bp[1] >= wheel_deg / _TURN_IN_RATE + _MEASURE_S, m.description

  def test_cruise_can_hold_every_speed(self):
    """Ford ACC does not set below 15 mph."""
    assert min(m.initial_speed for m in FORD_PSCM_MANEUVERS) * CV.MS_TO_MPH >= 15.9

  def test_both_directions_and_a_repeat(self):
    descriptions = [m.description for m in FORD_PSCM_MANEUVERS]
    for d in descriptions:
      mirrored = d.replace("+", "#").replace("-", "+").replace("#", "-")
      assert mirrored in descriptions, d
    assert all(m.repeat >= 1 for m in FORD_PSCM_MANEUVERS)
