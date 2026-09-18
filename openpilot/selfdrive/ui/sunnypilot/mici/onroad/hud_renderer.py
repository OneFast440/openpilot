"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.ui_state import ui_state

# Ford brake lamp indicator. Red enough to read at a glance without looking like an alert.
BRAKE_LIGHT_COLOR = (255, 60, 60)


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self.brake_lights_on = False

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()
    self.brake_lights_on = self._brake_lights_lit()

  @staticmethod
  def _brake_lights_lit() -> bool:
    """Whether to tint the wheel for the vehicle's brake lamps.

    Gated on carParamsSP rather than on the param directly: the car process reads the setting once
    at init, and that is also what decides whether the CAN messages behind it are even parsed, so
    reading the param here could disagree with what is actually being published.
    """
    if not ui_state.sm.alive['carParamsSP'] or not ui_state.sm.alive['carStateSP']:
      return False
    if not ui_state.sm['carParamsSP'].fordHud.brakeLightStatus:
      return False
    status = ui_state.sm['carStateSP'].fordBrakeLights
    return status.dataAvailable and status.brakeLightsOn

  def _steering_wheel_color(self) -> rl.Color:
    if self.brake_lights_on:
      return rl.Color(*BRAKE_LIGHT_COLOR, int(self._wheel_alpha_filter.x))
    return super()._steering_wheel_color()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
