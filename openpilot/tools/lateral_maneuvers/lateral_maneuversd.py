#!/usr/bin/env python3
import numpy as np
from dataclasses import dataclass

from openpilot.cereal import messaging
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import MIN_SPEED
from openpilot.tools.longitudinal_maneuvers.maneuversd import Action, Maneuver as _Maneuver

# thresholds for starting maneuvers
MAX_SPEED_DEV = 0.7 # deviation in m/s
MAX_CURV = 0.004 # 250 m radius
MAX_ROLL = 0.12 # 6.8°
TIMER = 2.0 # sec stable conditions before starting maneuver

@dataclass
class Maneuver(_Maneuver):
  _baseline_curvature: float = 0.0

  def get_accel(self, v_ego: float, lat_active: bool, curvature: float, roll: float) -> float:
    self._run_completed = False
    # only start maneuver on straight, flat roads
    ready = abs(v_ego - self.initial_speed) < MAX_SPEED_DEV and lat_active and abs(curvature) < MAX_CURV and abs(roll) < MAX_ROLL
    self._ready_cnt = (self._ready_cnt + 1) if ready else max(self._ready_cnt - 1, 0)

    if self._ready_cnt > (TIMER / DT_MDL):
      if not self._active:
        self._baseline_curvature = curvature
      self._active = True

    if not self._active:
      return 0.0

    return self._step()

  def reset(self):
    super().reset()
    self._ready_cnt = 0


def _sine_action(amplitude, period, duration):
  t = np.linspace(0, duration, int(duration / DT_MDL) + 1)
  a = amplitude * np.sin(2 * np.pi * t / period)
  return Action(a.tolist(), t.tolist())


MANEUVERS = [
  Maneuver(
    "step right 20mph",
    [Action([0.5], [1.0]), Action([-0.5], [1.5])],
    repeat=2,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "step left 20mph",
    [Action([-0.5], [1.0]), Action([0.5], [1.5])],
    repeat=2,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "sine 0.5Hz 20mph",
    [_sine_action(1.0, 2.0, 2.0), Action([0.0], [0.5])],
    repeat=2,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "jitter 20mph",
    [Action([-0.5 if i % 2 == 0 else 0.5], [0.1]) for i in range(10)],
    repeat=2,
    initial_speed=20. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "step right 30mph",
    [Action([0.5], [1.0]), Action([-0.5], [1.5])],
    repeat=2,
    initial_speed=30. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "step left 30mph",
    [Action([-0.5], [1.0]), Action([0.5], [1.5])],
    repeat=2,
    initial_speed=30. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "sine 0.5Hz 30mph",
    [_sine_action(1.0, 2.0, 2.0), Action([0.0], [0.5])],
    repeat=2,
    initial_speed=30. * CV.MPH_TO_MS,
  ),
  Maneuver(
    "jitter 30mph",
    [Action([-0.5 if i % 2 == 0 else 0.5], [0.1]) for i in range(10)],
    repeat=2,
    initial_speed=30. * CV.MPH_TO_MS,
  ),
]


def _hold(accel: float, ramp_s: float = 0.5, hold_s: float = 4.0) -> Action:
  """Ramp to a lateral acceleration, hold it, ramp back out. The hold is long enough for a slow
  actuator to settle, so its steady part measures delivery rather than lag."""
  return Action([0.0, accel, accel, 0.0], [0.0, ramp_s, ramp_s + hold_s, 2 * ramp_s + hold_s])


def _sweep(peak: float, rise_s: float = 6.0) -> Action:
  """A slow ramp to peak and a quick return: maps where an actuator starts reporting its limit
  against a command that is never a step."""
  return Action([0.0, peak, 0.0], [0.0, rise_s, rise_s + 1.0])


# Ford PSCM characterization (see FORD_PSCM.md). Aimed at the low-speed wall: the PSCM follows the
# LateralMotionControl2 path angle slowly, and tight low-speed turns are where that shows. Each
# level stays inside the 0.52 rad path-angle signal at its speed with the default gain schedule:
# kappa = accel / v^2 and path_angle ~ kappa * v * 1.3, so 12 mph tops out at 1.5 m/s^2.
def _ford_holds(mph: int, accels: tuple[float, ...]) -> list[Maneuver]:
  return [Maneuver(f"hold {sign * accel:+.1f}m/s^2 {mph}mph", [_hold(sign * accel), Action([0.0], [1.5])],
                   repeat=1, initial_speed=mph * CV.MPH_TO_MS)
          for accel in accels for sign in (1.0, -1.0)]


# Fastest first: the suite waits at each speed until it is held, so if the truck's cruise cannot
# hold 12 mph, everything else has already run by the time it stalls there.
FORD_PSCM_MANEUVERS = (
  _ford_holds(20, (1.0, 2.0, 2.5)) +
  _ford_holds(16, (1.0, 2.0)) +
  [Maneuver(f"sweep {sign * 2.5:+.1f}m/s^2 16mph", [_sweep(sign * 2.5), Action([0.0], [1.5])],
            repeat=1, initial_speed=16. * CV.MPH_TO_MS) for sign in (1.0, -1.0)] +
  _ford_holds(12, (1.0, 1.5))
)


def maneuvers_for(CP) -> list[Maneuver]:
  return FORD_PSCM_MANEUVERS if CP.brand == "ford" else MANEUVERS


def main():
  params = Params()
  cloudlog.info("lateral_maneuversd is waiting for CarParams")
  CP = messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)

  sm = messaging.SubMaster(['carState', 'carControl', 'controlsState', 'selfdriveState', 'modelV2'], poll='modelV2')
  pm = messaging.PubMaster(['lateralManeuverPlan', 'alertDebug'])

  maneuvers = iter(maneuvers_for(CP))
  maneuver = None
  complete_cnt = 0
  aborted_cnt = 0
  abort_reason = ''
  display_holdoff = 0
  prev_text = ''

  while True:
    sm.update()

    if maneuver is None:
      maneuver = next(maneuvers, None)

    alert_msg = messaging.new_message('alertDebug')
    alert_msg.valid = True

    plan_send = messaging.new_message('lateralManeuverPlan')

    accel = 0
    v_ego = max(sm['carState'].vEgo, 0)
    curvature = sm['controlsState'].desiredCurvature

    if complete_cnt > 0:
      complete_cnt -= 1
      alert_msg.alertDebug.alertText1 = 'Completed'
      alert_msg.alertDebug.alertText2 = maneuver.description
    elif maneuver is not None:
      # any driver input aborts the maneuver
      CS = sm['carState']
      if CS.steeringPressed or CS.gasPressed:
        aborted_cnt = int(1.0 / DT_MDL)
        abort_reason = ('steering pressed' if CS.steeringPressed else 'gas pressed').ljust(20)
      aborted = aborted_cnt > 0
      speed_out_of_range = maneuver.active and abs(v_ego - maneuver.initial_speed) > MAX_SPEED_DEV
      if aborted or speed_out_of_range:
        maneuver.reset()

      roll = sm['carControl'].orientationNED[0] if len(sm['carControl'].orientationNED) == 3 else 0.0
      accel = maneuver.get_accel(v_ego, sm['carControl'].latActive, curvature, roll)

      if maneuver._run_completed:
        complete_cnt = int(1.0 / DT_MDL)
        alert_msg.alertDebug.alertText1 = 'Complete'
        alert_msg.alertDebug.alertText2 = maneuver.description
      elif maneuver.active:
        action_remaining = maneuver.actions[maneuver._action_index].time_bp[-1] - maneuver._action_frames * DT_MDL
        if maneuver.description.startswith('sine'):
          freq = maneuver.description.split()[1]
          alert_msg.alertDebug.alertText1 = f'Active sine {freq} {max(action_remaining, 0):.1f}s'
        else:
          alert_msg.alertDebug.alertText1 = f'Active {accel:+.1f}m/s² {max(action_remaining, 0):.1f}s'
        alert_msg.alertDebug.alertText2 = maneuver.description
      elif aborted_cnt > 0:
        aborted_cnt -= 1
        alert_msg.alertDebug.alertText1 = abort_reason
      elif not (abs(v_ego - maneuver.initial_speed) < MAX_SPEED_DEV and sm['carControl'].latActive):
        alert_msg.alertDebug.alertText1 = f'Set speed to {maneuver.initial_speed * CV.MS_TO_MPH:0.0f} mph'
      elif maneuver._ready_cnt > 0:
        ready_time = max(TIMER - maneuver._ready_cnt * DT_MDL, 0)
        alert_msg.alertDebug.alertText1 = f'Starting: {int(ready_time) + 1}'
        alert_msg.alertDebug.alertText2 = maneuver.description
      else:
        curv_ok = abs(curvature) < MAX_CURV
        reason = 'road not straight' if not curv_ok else 'road not flat'
        alert_msg.alertDebug.alertText1 = f'Waiting: {reason}'
        alert_msg.alertDebug.alertText2 = maneuver.description
    else:
      alert_msg.alertDebug.alertText1 = 'Maneuvers Finished'

    # prevent flickering text
    setup = ('Set speed', 'Starting', 'Waiting')
    text = alert_msg.alertDebug.alertText1
    same = text == prev_text or (text.startswith('Starting') and prev_text.startswith('Starting'))
    if not same and text.startswith(setup) and prev_text.startswith(setup) and display_holdoff > 0:
      alert_msg.alertDebug.alertText1 = prev_text
      display_holdoff -= 1
    else:
      prev_text = text
      display_holdoff = int(0.5 / DT_MDL) if text.startswith(setup) else 0

    pm.send('alertDebug', alert_msg)

    plan_send.valid = maneuver is not None and maneuver.active and complete_cnt == 0
    if plan_send.valid:
      plan_send.lateralManeuverPlan.desiredCurvature = maneuver._baseline_curvature + accel / max(v_ego, MIN_SPEED) ** 2
    pm.send('lateralManeuverPlan', plan_send)

    if maneuver is not None and maneuver.finished and complete_cnt == 0:
      maneuver = None


if __name__ == "__main__":
  main()
