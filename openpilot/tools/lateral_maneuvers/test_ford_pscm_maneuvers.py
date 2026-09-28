from types import SimpleNamespace

from openpilot.tools.lateral_maneuvers.lateral_maneuversd import FORD_PSCM_MANEUVERS, MANEUVERS, maneuvers_for

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

  def test_no_maneuver_turns_the_truck_around(self):
    """Heading change stays under 90 degrees so each run fits a lot without a U-turn."""
    for m in FORD_PSCM_MANEUVERS:
      v = m.initial_speed
      heading = 0.0
      for action in m.actions:
        t = action.time_bp
        for i in range(1, len(t)):
          mean_accel = abs(action.accel_bp[i] + action.accel_bp[i - 1]) / 2
          heading += mean_accel / v * (t[i] - t[i - 1])   # yaw rate = a / v
      assert heading < 1.57, f"{m.description}: {heading:.2f} rad"

  def test_both_directions_and_a_repeat(self):
    descriptions = [m.description for m in FORD_PSCM_MANEUVERS]
    for d in descriptions:
      mirrored = d.replace("+", "#").replace("-", "+").replace("#", "-")
      assert mirrored in descriptions, d
    assert all(m.repeat >= 1 for m in FORD_PSCM_MANEUVERS)
