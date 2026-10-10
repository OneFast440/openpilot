import math
from collections import deque

import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

# TODO This is speed dependent
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees
SATURATION_DELAY_BUFFER_SECONDS = 1.0


class LatControlAngle(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.sat_check_min_speed = 5.
    self.use_steer_limited_by_safety = CP.brand in ("tesla", "hyundai")
    # Ford angle mode: compare the wheel against what was asked one lateral delay ago, the same
    # alignment the torque controller uses for its error. The desired angle is a command that leads
    # the truck by the whole steering delay (0.5 s on logged F-150 drives), so comparing it with the
    # wheel at the same instant reads every curve entry as saturation: 29 of 30 steer-saturated
    # alerts on a 151-segment trip, with steady-state delivery at 1.00. Aligned, a wheel that is
    # really short of the command still trips it; one that is only on time does not.
    self.align_saturation_to_delay = CP.brand == "ford"
    self.angle_des_buffer = deque(maxlen=int(SATURATION_DELAY_BUFFER_SECONDS / dt))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    angle_log = log.ControlsState.LateralAngleState.new_message()

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
    else:
      angle_log.active = True
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg

    self.angle_des_buffer.append(angle_steers_des)
    if self.align_saturation_to_delay:
      delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, len(self.angle_des_buffer)))
      angle_control_saturated = abs(self.angle_des_buffer[-delay_frames] - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    elif self.use_steer_limited_by_safety:
      # these cars' carcontrollers calculate max lateral accel and jerk, so we can rely on carOutput for saturation
      angle_control_saturated = steer_limited_by_safety
    else:
      # for cars which use a method of limiting torque such as a torque signal (Nissan and Toyota)
      # or relying on EPS (Ford Q3), carOutput does not capture maxing out torque  # TODO: this can be improved
      angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = angle_steers_des
    return 0, float(angle_steers_des), angle_log
