"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Vehicle settings for the mici UI.

The large-screen UI reaches these through SettingsLayoutSP's VEHICLE panel. mici's settings
layout has no such panel, so on a comma four none of the per-vehicle tuning was reachable on
the device at all and could only be set through sunnylink. This is that panel.

Same params, same storage, same visibility rules as the large-screen Ford page, rendered with
mici's own widgets.
"""
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_ACCEL_NO_ROLL, MAX_LATERAL_JERK
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle, GreyBigButton
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import (LATERAL_ACCEL_LIMIT_RANGE,
                                                                   LATERAL_JERK_LIMIT_RANGE)
from openpilot.selfdrive.ui.sunnypilot.mici.widgets.stepper import BigParamStepper
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller import NavScroller

from opendbc.car.ford.values import FordFlags
from opendbc.sunnypilot.car.ford.values_ext import (
  BLEND_RATIO_RANGE,
  HIGH_SPEED_DAMPENING_RANGE,
  HIGH_SPEED_FACTOR_RANGE,
  LANE_CHANGE_FACTOR_CURV_RANGE,
  LANE_CHANGE_FACTOR_RANGE,
  LANE_POSITIONING_GAIN_RANGE,
  LOW_SPEED_FACTOR_RANGE,
  PEDAL_OVERRIDE_RANGE,
  PATH_OFFSET_RANGE,
  PrimaryLateralControl,
)


def _stepper(title: str, param: str, value_range: tuple[float, float, float]) -> BigParamStepper:
  # (default, min, max); the default lives on the param itself, same as the large-screen UI
  _, min_value, max_value = value_range
  return BigParamStepper(title, param, min_value, max_value)


class FordPanelMici:
  """Builds the Ford widgets and keeps their visibility in step. Not a scroller itself: the
  layout below owns the one scroller, so there is no nesting to reason about."""

  def __init__(self):
    self.primary_control = BigMultiParamToggle(tr("primary lateral control"), "FordPrefLateralControl",
                                               [tr("stock"), tr("curvature"), tr("angle")])

    # angle mode
    self.low_speed_factor = _stepper(tr("low speed factor"), "FordLowSpeedFactor_ang", LOW_SPEED_FACTOR_RANGE)
    self.high_speed_factor = _stepper(tr("high speed factor"), "FordHighSpeedFactor_ang", HIGH_SPEED_FACTOR_RANGE)
    self.high_speed_dampening = _stepper(tr("high speed dampening"), "FordHighSpeedDampening_ang",
                                         HIGH_SPEED_DAMPENING_RANGE)
    self.lane_change_factor_ang = _stepper(tr("lane change factor"), "FordLaneChangeFactor_ang",
                                           LANE_CHANGE_FACTOR_RANGE)
    self.sat_observer = BigParamControl(tr("detect steering saturation"), "FordSatObserver_ang")
    self.delivery_comp = BigParamControl(tr("adaptive steering gain"), "FordDeliveryCompensation_ang")

    # curvature mode
    self.human_turn = BigParamControl(tr("hand back on manual turns"), "FordHumanTurnDetection_curv")
    self.lane_change_factor_curv = _stepper(tr("lane change factor"), "FordLaneChangeFactor_curv",
                                            LANE_CHANGE_FACTOR_CURV_RANGE)
    self.lane_positioning = BigParamControl(tr("lane centering trim"), "FordLanePositioning_curv")
    self.lane_positioning_gain = _stepper(tr("lane centering strength"), "FordLanePositioningGain_curv",
                                          LANE_POSITIONING_GAIN_RANGE)
    self.lane_full_mode = BigParamControl(tr("use detected lane lines"), "FordLaneFullMode_curv")
    self.path_offset = _stepper(tr("in-lane position"), "FordPathOffset_curv", PATH_OFFSET_RANGE)
    self.custom_profile = BigParamControl(tr("custom tuning profile"), "FordCustomProfile_curv")
    self.blend_ratio_low = _stepper(tr("model blend, straights"), "FordBlendRatioLow_curv", BLEND_RATIO_RANGE)
    self.blend_ratio_high = _stepper(tr("model blend, curves"), "FordBlendRatioHigh_curv", BLEND_RATIO_RANGE)

    # longitudinal and cluster
    self.follow_control = BigParamControl(tr("lead-aware following"), "FordFollowControl")
    self.downhill_compensation = BigParamControl(tr("downhill brake compensation"), "FordDownhillCompensation")
    self.throttle_override_hold = BigParamControl(tr("hold throttle through pedal override"),
                                                  "ThrottleOverrideHold")
    self.pedal_override_threshold = _stepper(tr("pedal override threshold"),
                                             "FordPedalOverrideThreshold", PEDAL_OVERRIDE_RANGE)
    self.lateral_accel_limit = _stepper(tr("cornering limit"), "FordLateralAccelLimit",
                                        (MAX_LATERAL_ACCEL_NO_ROLL, *LATERAL_ACCEL_LIMIT_RANGE))
    self.lateral_jerk_limit = _stepper(tr("turn-in rate limit"), "FordLateralJerkLimit",
                                       (MAX_LATERAL_JERK, *LATERAL_JERK_LIMIT_RANGE))
    self.hands_free_cluster = BigParamControl(tr("hands-free cluster display"), "FordHandsFreeClusterMsg")
    self.driver_monitor_cluster = BigParamControl(tr("driver monitoring in cluster"), "FordDriverMonitorCanMsg")
    self.brake_light_status = BigParamControl(tr("brake light indicator"), "FordBrakeLightStatus")

    self.angle_items = [
      self.low_speed_factor,
      self.high_speed_factor,
      self.high_speed_dampening,
      self.lane_change_factor_ang,
      self.sat_observer,
      self.delivery_comp,
      self.human_turn,
    ]
    self.curvature_items = [
      self.human_turn,
      self.lane_change_factor_curv,
      self.lane_positioning,
      self.lane_positioning_gain,
      self.lane_full_mode,
      self.path_offset,
      self.custom_profile,
      self.blend_ratio_low,
      self.blend_ratio_high,
    ]
    self.always_items = [
      self.follow_control,
      self.downhill_compensation,
      self.throttle_override_hold,
      self.pedal_override_threshold,
      self.lateral_accel_limit,
      self.lateral_jerk_limit,
      self.hands_free_cluster,
      self.driver_monitor_cluster,
      self.brake_light_status,
    ]

    self._offroad_note = GreyBigButton("", tr("settings unlock when the car is off"))

    # angle and curvature share human_turn, so build the list without repeating it
    self.widgets: list[Widget] = [self._offroad_note, self.primary_control]
    for item in self.angle_items + self.curvature_items + self.always_items:
      if item not in self.widgets:
        self.widgets.append(item)

  def update_settings(self):
    offroad = ui_state.is_offroad()
    mode = int(ui_state.params.get("FordPrefLateralControl") or 0)
    is_angle = mode == PrimaryLateralControl.angle
    is_curvature = mode == PrimaryLateralControl.curvature
    is_canfd = bool(ui_state.CP is not None and ui_state.CP.brand == "ford"
                    and ui_state.CP.flags & FordFlags.CANFD)
    custom_profile_on = ui_state.params.get_bool("FordCustomProfile_curv")
    lane_positioning_on = ui_state.params.get_bool("FordLanePositioning_curv")

    self._offroad_note.set_visible(not offroad)
    self.primary_control.set_enabled(offroad)

    for item in self.angle_items:
      if item is self.human_turn:
        continue
      item.set_visible(is_angle)
      item.set_enabled(offroad)

    self.human_turn.set_visible(is_angle or is_curvature)
    self.human_turn.set_enabled(offroad)

    for item in self.curvature_items:
      if item is self.human_turn:
        continue
      visible = is_curvature
      if item in (self.blend_ratio_low, self.blend_ratio_high):
        visible = visible and custom_profile_on
      elif item is self.lane_positioning_gain:
        visible = visible and lane_positioning_on and custom_profile_on
      item.set_visible(visible)
      item.set_enabled(offroad)

    for item in self.always_items:
      item.set_enabled(offroad)
    self.hands_free_cluster.set_visible(is_canfd)

  def refresh(self):
    """Mirror changes made elsewhere, e.g. through sunnylink while this page was closed."""
    for item in self.widgets:
      if isinstance(item, (BigParamControl, BigParamStepper)):
        item.refresh()


class VehicleLayoutMici(NavScroller):
  """Brand router. Only Ford has a panel today; anything else says so rather than showing an
  empty scroller, which is what the missing panel looked like from the outside."""

  def __init__(self):
    super().__init__()
    self._ford_panel: FordPanelMici | None = None
    self._placeholder = GreyBigButton("", tr("no tuning for this vehicle"))
    self._scroller.add_widget(self._placeholder)
    self._brand_shown: str | None = None

  def show_event(self):
    super().show_event()
    self._build_for_brand()
    if self._ford_panel is not None:
      self._ford_panel.refresh()

  def _build_for_brand(self):
    # CarParams only arrives once the car is up, so the brand can change under us
    brand = ui_state.CP.brand if ui_state.CP is not None else None
    if brand == self._brand_shown:
      return
    self._brand_shown = brand

    self._scroller._items.clear()
    if brand == "ford":
      if self._ford_panel is None:
        self._ford_panel = FordPanelMici()
      self._scroller.add_widgets(self._ford_panel.widgets)
      self._ford_panel.update_settings()
    else:
      self._scroller.add_widget(self._placeholder)

  def _update_state(self):
    super()._update_state()
    self._build_for_brand()
    if self._ford_panel is not None:
      self._ford_panel.update_settings()
