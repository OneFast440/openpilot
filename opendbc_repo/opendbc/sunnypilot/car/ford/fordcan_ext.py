"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car.ford.fordcan import CanBus

# shadow_curvature wire scale, 1/m per LSB. int16 -> +-0.0327 1/m, comfortably past the +-0.02
# DBC curvature range plus the deviation band.
SHADOW_CURVATURE_SCALE = 1e-6


def create_lka_msg(packer, CAN: CanBus, angle_mode_engaged: bool = False, shadow_curvature: float = 0.0):
  """
  Creates the Ford LKA Command, optionally carrying angle-control state for the panda.

  Upstream sends this message empty. Angle control needs two values to reach safety/modes/ford.h
  that have nowhere else to go:

    * angle_mode_engaged -- corroborates that the wide path_angle range is legitimate, so a frame
      cannot unlock it by merely zeroing curvature.
    * shadow_curvature -- the kappa path_angle was derived from. Angle mode holds the real
      curvature signal at its inactive sentinel, so without this there is no commanded-vs-measured
      deviation check for angle mode at all.

  They ride in bits of Lane_Assist_Data1 that no DBC signal maps to (byte 4 bits 4-0, bytes 5-7;
  see ford_lincoln_base_pt.dbc). ford_tx_hook already reads LkaActvStats_D2_Req straight out of
  these same bytes, in the same call, so the values are read synchronously off the message being
  transmitted -- no separate CAN ID and no RX round trip, which matters because panda does not
  self-receive its own TX.

  Byte layout, and it must match the decode in ford.h exactly:
    byte 4 bit 0:    angle_mode_engaged
    byte 4 bits 1-4: reserved
    bytes 5-6:       shadow_curvature, int16 big-endian, scale SHADOW_CURVATURE_SCALE
    byte 7:          reserved

  shadow_curvature is in the CAN sign convention, i.e. negated from openpilot's, the same way
  path_angle and curvature are negated when they go on the wire. ford.h's measured curvature comes
  from the raw yaw rate with no negation, so an un-negated shadow reads as a permanent divergence.

  Frequency is 33Hz.
  """
  addr, dat, bus = packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})
  dat = bytearray(dat)

  raw = int(round(shadow_curvature / SHADOW_CURVATURE_SCALE))
  raw = max(-32768, min(32767, raw)) & 0xFFFF

  dat[4] |= 1 if angle_mode_engaged else 0
  dat[5] = (raw >> 8) & 0xFF
  dat[6] = raw & 0xFF

  return addr, bytes(dat), bus
