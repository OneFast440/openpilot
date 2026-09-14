"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford angle-control values, ported from BluePilot bp-7.0.

Angle control replaces curvature (c2) with path_angle (c1) as the steering actuator on Ford.
The PSCM applies a speed-indexed low-pass filter to its internal curvature state, so a c2
command is still being acted on ~0.5 s after openpilot drops it; c1 has no such memory. See
https://bluepilot.dev/announcements/ for the reverse-engineering writeup.
"""

from enum import IntEnum

from opendbc.car.ford.values import CAR


class PrimaryLateralControl(IntEnum):
  """Which signal is the steering actuator. Selected by the FordPrefLateralControl param and
  carried on CP_SP.fordLateralTuning.primaryControl. Read once at car init, because the panda
  safety flag below is derived from the same read and the two layers must never disagree."""
  curvature = 0  # stock: c2 (curvature), c1/c0/c3 held at their inactive sentinels
  angle = 1      # BluePilot: c1 (path_angle), c2/c0/c3 held at their inactive sentinels


class FordSafetyFlagsSP:
  """sunnypilot-level safety flags for Ford.

  Carried in CP_SP.safetyParam and delivered to the safety firmware as current_safety_param_sp
  (the separate SP uint16, USB control 0xdf) -- NOT safetyConfigs[].safetyParam. ford_init reads
  it with GET_FLAG(current_safety_param_sp, ...), same pattern as Subaru STOP_AND_GO. Plain int
  constants, not IntFlag: CP_SP.safetyParam must stay a plain int through capnp serialization.
  """
  ANGLE_CONTROL = 1


# Model time index breakpoints, matching selfdrive/modeld/constants.py ModelConstants.T_IDXS.
# Duplicated here because opendbc must not import openpilot; CarControlSP.FordLateral
# .modelCurvatures is sampled on exactly this grid.
T_IDXS = [
  0.0, 0.009765625, 0.0390625, 0.087890625, 0.15625, 0.244140625, 0.3515625, 0.478515625,
  0.625, 0.791015625, 0.9765625, 1.181640625, 1.40625, 1.650390625, 1.9140625, 2.197265625,
  2.5, 2.822265625, 3.1640625, 3.525390625, 3.90625, 4.306640625, 4.7265625, 5.166015625,
  5.625, 6.103515625, 6.6015625, 7.119140625, 7.65625, 8.212890625, 8.7890625, 9.384765625, 10.0,
]

# DBC LatCtlCurv_No_Actl magnitude limit (1/m). Also the panda's FORD_STEERING_LIMITS.max_curvature.
CURVATURE_MAX = 0.02

# DBC LatCtlPath_An_Actl range (rad). The panda mirror lives in safety/modes/ford.h as
# FORD_DBC_PATH_ANGLE_MIN/MAX; the PSCM enforces the same range in firmware.
FORD_DBC_PATH_ANGLE_MIN = -0.5
FORD_DBC_PATH_ANGLE_MAX = 0.5235

# User-tunable "feel" multipliers. (default, min, max) -- the single source of truth for the
# defaults and clamps used by the settings UI, the sunnylink schema, and the control code.
LOW_SPEED_FACTOR_RANGE = (1.0, 0.5, 1.5)
HIGH_SPEED_FACTOR_RANGE = (1.0, 0.5, 1.5)
HIGH_SPEED_DAMPENING_RANGE = (1.0, 0.25, 1.25)
LANE_CHANGE_FACTOR_RANGE = (1.0, 0.85, 1.5)

# Hard-coded per-platform gain defaults: (low-curvature gain, high-curvature gain), both applied
# only at high speed. The PSCM compensates path_angle against a factory model of the vehicle
# (trim, suspension, weight distribution), so the same commanded angle produces different
# steering across platforms -- and across trims within a platform, which is what the two user
# factors above are for.
_GAIN_CAN = (1.00, 1.15)         # CAN vehicles (Escape Mk4, Bronco Sport, Explorer, Maverick, Edge)
_GAIN_CANFD_BOF = (0.95, 0.95)   # CAN FD body-on-frame trucks (F-150, Lightning, Expedition, Ranger)
_GAIN_CANFD_SUV = (1.00, 1.05)   # CAN FD unibody SUVs (Mustang Mach-E, Escape Mk4.5)

_CANFD_BOF_CARS = frozenset({
  CAR.FORD_F_150_MK14,
  CAR.FORD_F_150_LIGHTNING_MK1,
  CAR.FORD_EXPEDITION_MK4,
  CAR.FORD_RANGER_MK2,
})
_CANFD_SUV_CARS = frozenset({
  CAR.FORD_MUSTANG_MACH_E_MK1,
  CAR.FORD_ESCAPE_MK4_5,
})


def platform_path_angle_gains(car_fingerprint) -> tuple[float, float]:
  """(low-curvature gain, high-curvature gain) defaults for a platform."""
  if car_fingerprint in _CANFD_BOF_CARS:
    return _GAIN_CANFD_BOF
  if car_fingerprint in _CANFD_SUV_CARS:
    return _GAIN_CANFD_SUV
  return _GAIN_CAN
