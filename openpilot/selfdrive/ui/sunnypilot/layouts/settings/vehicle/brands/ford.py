"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp, option_item_sp

from opendbc.sunnypilot.car.ford.values_ext import (
  HIGH_SPEED_DAMPENING_RANGE,
  HIGH_SPEED_FACTOR_RANGE,
  LANE_CHANGE_FACTOR_RANGE,
  LOW_SPEED_FACTOR_RANGE,
  PrimaryLateralControl,
)

# OptionControlSP stores float params scaled by 100, so the widget bounds are the physical
# range x100 and a step of 5 moves in 0.05 increments.
_SCALE = 100
_STEP = 5

OFFROAD_ONLY_DESCRIPTION = tr_noop("This feature is unavailable while the car is onroad.")

DESCRIPTIONS = {
  'primary_control': tr_noop(
    'Which signal steers the car. Curvature is the stock strategy. Angle steers with path_angle instead, which the ' +
    'power steering module acts on immediately rather than filtering for up to a second, and generally holds a lane ' +
    'and exits curves more cleanly. Switch back to Curvature if your vehicle does not respond well.'
  ),
  'low_speed_factor': tr_noop(
    'Scales the steering response below roughly 30 mph (50 km/h). Above 1.0 turns the wheel more, below 1.0 turns it less.'
  ),
  'high_speed_factor': tr_noop(
    'Scales the steering response above roughly 70 mph (115 km/h). Above 1.0 turns the wheel more, below 1.0 turns it less.'
  ),
  'high_speed_dampening': tr_noop(
    'Scales the steering response on high-speed straightaways and gentle curves. Reduce it if the car oversteers on ' +
    'the highway, increase it if it understeers.'
  ),
  'lane_change_factor': tr_noop(
    'Scales steering authority during a lane change. Most vehicles never need this; raise it if lane changes feel too soft.'
  ),
}

ANGLE_TUNING_NOTE = tr_noop(
  'Every vehicle compensates differently, because the power steering module corrects against a model of the trim, ' +
  'suspension and weight distribution it left the factory with. The defaults suit most vehicles; a lift, a level kit, ' +
  'larger tires or a different trim usually needs a small dial-in over a few short drives.'
)


class FordSettings(BrandSettings):
  def __init__(self):
    super().__init__()

    self.primary_control = multiple_button_item_sp(
      lambda: tr("Primary Lateral Control"),
      lambda: tr(DESCRIPTIONS["primary_control"]),
      [tr("Curvature"), tr("Angle")],
      button_width=300,
      callback=self._on_primary_control_selected,
      param="FordPrefLateralControl",
      inline=False,
    )

    self.low_speed_factor = self._factor_item(
      tr_noop("Low Speed Factor"), "FordLowSpeedFactor_ang", DESCRIPTIONS["low_speed_factor"], LOW_SPEED_FACTOR_RANGE)
    self.high_speed_factor = self._factor_item(
      tr_noop("High Speed Factor"), "FordHighSpeedFactor_ang", DESCRIPTIONS["high_speed_factor"], HIGH_SPEED_FACTOR_RANGE)
    self.high_speed_dampening = self._factor_item(
      tr_noop("High Speed Dampening"), "FordHighSpeedDampening_ang", DESCRIPTIONS["high_speed_dampening"],
      HIGH_SPEED_DAMPENING_RANGE)
    self.lane_change_factor = self._factor_item(
      tr_noop("Lane Change Factor"), "FordLaneChangeFactor_ang", DESCRIPTIONS["lane_change_factor"],
      LANE_CHANGE_FACTOR_RANGE)

    self.angle_items = [
      self.low_speed_factor,
      self.high_speed_factor,
      self.high_speed_dampening,
      self.lane_change_factor,
    ]
    self.items = [self.primary_control, *self.angle_items]

  @staticmethod
  def _factor_item(title: str, param: str, description: str, value_range: tuple[float, float, float]):
    _, min_value, max_value = value_range
    return option_item_sp(
      lambda: tr(title),
      param,
      round(min_value * _SCALE),
      round(max_value * _SCALE),
      description=lambda: tr(description),
      value_change_step=_STEP,
      use_float_scaling=True,
      enabled=lambda: ui_state.is_offroad(),
    )

  @staticmethod
  def _on_primary_control_selected(index: int):
    # Read once at car init, together with the panda safety flag, so the change takes a restart.
    ui_state.params.put("FordPrefLateralControl", index)

  def update_settings(self):
    offroad = ui_state.is_offroad()
    primary = int(ui_state.params.get("FordPrefLateralControl") or 0)
    angle_mode = primary == PrimaryLateralControl.angle

    self.primary_control.action_item.set_enabled(offroad)
    self.primary_control.action_item.set_selected_button(primary)

    description = tr(DESCRIPTIONS["primary_control"])
    if not offroad:
      description = "<b>" + tr(OFFROAD_ONLY_DESCRIPTION) + "</b>\n\n" + description
    if self.primary_control.description != description:
      self.primary_control.set_description(description)
      self.primary_control.show_description(True)

    for index, item in enumerate(self.angle_items):
      item.set_visible(angle_mode)
      item.action_item.set_enabled(offroad)
      # the tuning note belongs on the first factor only, where it reads as an intro to the group
      if index == 0 and angle_mode:
        note = tr(ANGLE_TUNING_NOTE) + "\n\n" + tr(DESCRIPTIONS["low_speed_factor"])
        if item.description != note:
          item.set_description(note)
          item.show_description(True)
