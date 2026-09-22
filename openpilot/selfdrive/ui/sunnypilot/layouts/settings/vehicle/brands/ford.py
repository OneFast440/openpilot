"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_ACCEL_NO_ROLL, MAX_LATERAL_JERK
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import (LATERAL_ACCEL_LIMIT_RANGE,
                                                                   LATERAL_JERK_LIMIT_RANGE)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.sunnypilot.widgets.list_view import (
  LineSeparatorSP,
  multiple_button_item_sp,
  option_item_sp,
  toggle_item_sp,
)

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

# OptionControlSP stores float params scaled by 100, so the widget bounds are the physical range
# x100 and a step of 5 moves in 0.05 increments.
_SCALE = 100
_STEP = 5

OFFROAD_ONLY = tr_noop("This feature is unavailable while the car is onroad.")

DESCRIPTIONS = {
  'primary_control': tr_noop(
    'Which signal steers the car. Stock drives curvature alone, as upstream openpilot does. ' +
    'Curvature drives all four of the signals the power steering module accepts, blending the model into the ' +
    'planner and anticipating curves. Angle steers with path_angle instead, which the module acts on immediately ' +
    'rather than filtering for up to a second.'
  ),
  # angle
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
  'lane_change_factor_ang': tr_noop(
    'Scales steering authority during a lane change. Most vehicles never need this; raise it if lane changes feel too soft.'
  ),
  'delivery_compensation': tr_noop(
    'Act on that measurement instead of only noticing it. Measured over four logs this platform ' +
    'turns about 0.87 of the curvature it is asked for, with nothing anywhere correcting for it, ' +
    'so the car runs wide and you add the rest. This scales the steering command by whatever ' +
    'cancels the shortfall it measures. It only ever adds, it is capped, and it fades out ' +
    'whenever you touch the wheel. Needs Detect Steering Saturation on.'
  ),
  'pedal_override_threshold': tr_noop(
    'How far the accelerator has to move before sunnypilot treats it as yours. The vehicle reports ' +
    'any pedal movement at all as a press, so without this the lightest touch hands longitudinal ' +
    'control back and the truck slows under a pedal you have just rested on. A deliberate press ' +
    'measures well above the default. Raising it keeps sunnypilot commanding through a light press, ' +
    'which has not been proven safe on this platform; raise it a little at a time and watch for a ' +
    'cruise fault.'
  ),
  'throttle_override_hold': tr_noop(
    'Keep commanding throttle while you are on the accelerator pedal, instead of handing ' +
    'longitudinal control back for the whole press. The car decides between your pedal and ' +
    'sunnypilot\'s request, so lifting off hands back to whatever sunnypilot was already asking ' +
    'for rather than starting from nothing. The brakes are never applied while you are on the ' +
    'pedal. Has no effect with "Disengage on Accelerator Pedal" enabled.'
  ),
  'lateral_accel_limit': tr_noop(
    'The hardest cornering sunnypilot will ask for, in m/s^2. This is what decides the tightest ' +
    'curve it can command at a given speed, and it falls with the square of speed. The default ' +
    'allows a 6.7 m radius at 10 mph but only 42 m at 25 mph, which is why a tight turn needs ' +
    'the speed brought down to it. The default is the ISO comfort figure. Raising it makes the ' +
    'vehicle corner harder everywhere, not only in tight turns.'
  ),
  'lateral_jerk_limit': tr_noop(
    'How quickly sunnypilot may wind into a curve, in m/s^3. Also falls with the square of ' +
    'speed. The default is the ISO comfort figure. Above roughly 45 mph the angle-mode rate ' +
    'limit binds first, so raising this only has an effect below that.'
  ),
  'sat_observer': tr_noop(
    'Notices when the power steering module has stopped following a larger command, by comparing how much the truck ' +
    'actually turned against how much was asked of it. Without this the car keeps commanding harder into a curve the ' +
    'module is already refusing, which is what snaps the wheel back on the way out. It does not make the steering ' +
    'stronger. Off by default; try it on a road with real corners and watch for any hesitation mid-curve.'
  ),
  # curvature
  'human_turn': tr_noop(
    'Hand steering back to you while you hold a real turn, instead of winding up a command the power steering ' +
    'module has to reconcile when you let go. Control resumes on its own. Turning this off keeps openpilot ' +
    'steering through your input, which follows the plan more closely but leaves the module a larger difference ' +
    'to reconcile when you release the wheel.'
  ),
  'lane_change_factor_curv': tr_noop(
    'Scales steering authority during a lane change. Lower is gentler.'
  ),
  'custom_profile': tr_noop(
    'Tune how much of the model prediction is blended into the planner, and how hard the car works to hold lane ' +
    'position. Leave this off to use the values tested for your vehicle.'
  ),
  'blend_ratio_low': tr_noop(
    'How much of the model prediction is used on straights and gentle curves. 0.0 is planner only, 1.0 is model only.'
  ),
  'blend_ratio_high': tr_noop(
    'How much of the model prediction is used in sharper curves. 0.0 is planner only, 1.0 is model only.'
  ),
  'lane_positioning': tr_noop(
    'Trim the car toward the center of the lane with a separate heading correction, rather than relying on ' +
    'curvature alone. Off by default.'
  ),
  'lane_positioning_gain': tr_noop(
    'How hard the centering correction works. Raise it if the car sits off center, lower it if it wanders.'
  ),
  'lane_full_mode': tr_noop(
    'Blend the detected lane lines into the lane position rather than following the model path alone. Helps where ' +
    'the lane is well marked and the model path is not centered.'
  ),
  'path_offset': tr_noop(
    'Shift the car within its lane. Negative moves left, positive moves right.'
  ),
  # longitudinal and cluster
  'follow_control': tr_noop(
    'Adjust gas and braking based on what the lead vehicle is doing: cut gas when closing on it, cap gas when ' +
    'matched to it, and ease the first brake application in. Highway speeds only, and only behind a lead that is ' +
    'also moving at highway speed.'
  ),
  'downhill_compensation': tr_noop(
    'Let a downhill grade count toward the braking decision. Turn this off if the car pre-charges the brakes every ' +
    'time the road tips forward.'
  ),
  'hands_free_cluster': tr_noop(
    'Use the cluster\'s BlueCruise hands-free presentation while lateral control is active. CAN FD vehicles only.'
  ),
  'brake_light_status': tr_noop(
    'Turn the on-road steering wheel icon red while the vehicle\'s brake lights are lit, whether they were lit by you, ' +
    'by openpilot or by the stock cruise control. Read from the vehicle rather than guessed. On this platform the only ' +
    'working source reports about once a second, so treat it as a rough indication rather than an instant one.'
  ),
  'driver_monitor_cluster': tr_noop(
    'Drive the cluster\'s own hands-on-wheel prompt and warnings from driver monitoring, so the escalation appears ' +
    'in the instrument cluster as well as on the device.'
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
      [tr("Stock"), tr("Curvature"), tr("Angle")],
      button_width=255,
      callback=self._on_primary_control_selected,
      param="FordPrefLateralControl",
      inline=False,
    )

    # angle mode
    self.low_speed_factor = self._slider(
      tr_noop("Low Speed Factor"), "FordLowSpeedFactor_ang", "low_speed_factor", LOW_SPEED_FACTOR_RANGE)
    self.high_speed_factor = self._slider(
      tr_noop("High Speed Factor"), "FordHighSpeedFactor_ang", "high_speed_factor", HIGH_SPEED_FACTOR_RANGE)
    self.high_speed_dampening = self._slider(
      tr_noop("High Speed Dampening"), "FordHighSpeedDampening_ang", "high_speed_dampening", HIGH_SPEED_DAMPENING_RANGE)
    self.lane_change_factor_ang = self._slider(
      tr_noop("Lane Change Factor"), "FordLaneChangeFactor_ang", "lane_change_factor_ang", LANE_CHANGE_FACTOR_RANGE)
    self.sat_observer = self._toggle(
      tr_noop("Detect Steering Saturation"), "FordSatObserver_ang", "sat_observer")
    self.lateral_accel_limit = self._slider(
      tr_noop("Cornering Limit"), "FordLateralAccelLimit", "lateral_accel_limit",
      (MAX_LATERAL_ACCEL_NO_ROLL, *LATERAL_ACCEL_LIMIT_RANGE))
    self.lateral_jerk_limit = self._slider(
      tr_noop("Turn-In Rate Limit"), "FordLateralJerkLimit", "lateral_jerk_limit",
      (MAX_LATERAL_JERK, *LATERAL_JERK_LIMIT_RANGE))
    self.delivery_compensation = self._toggle(
      tr_noop("Correct Steering Shortfall"), "FordDeliveryCompensation_ang", "delivery_compensation")

    # curvature mode
    self.human_turn = self._toggle(tr_noop("Hand Back On Manual Turns"), "FordHumanTurnDetection_curv", "human_turn")
    self.lane_change_factor_curv = self._slider(
      tr_noop("Lane Change Factor"), "FordLaneChangeFactor_curv", "lane_change_factor_curv", LANE_CHANGE_FACTOR_CURV_RANGE)
    self.custom_profile = self._toggle(tr_noop("Custom Tuning Profile"), "FordCustomProfile_curv", "custom_profile")
    self.blend_ratio_low = self._slider(
      tr_noop("Model Blend, Straights"), "FordBlendRatioLow_curv", "blend_ratio_low", BLEND_RATIO_RANGE)
    self.blend_ratio_high = self._slider(
      tr_noop("Model Blend, Curves"), "FordBlendRatioHigh_curv", "blend_ratio_high", BLEND_RATIO_RANGE)
    self.lane_positioning = self._toggle(tr_noop("Lane Centering Trim"), "FordLanePositioning_curv", "lane_positioning")
    self.lane_positioning_gain = self._slider(
      tr_noop("Lane Centering Strength"), "FordLanePositioningGain_curv", "lane_positioning_gain",
      LANE_POSITIONING_GAIN_RANGE)
    self.lane_full_mode = self._toggle(tr_noop("Use Detected Lane Lines"), "FordLaneFullMode_curv", "lane_full_mode")
    self.path_offset = self._slider(
      tr_noop("In-Lane Position"), "FordPathOffset_curv", "path_offset", PATH_OFFSET_RANGE)

    # longitudinal and cluster
    self.follow_control = self._toggle(tr_noop("Lead-Aware Following"), "FordFollowControl", "follow_control")
    self.downhill_compensation = self._toggle(
      tr_noop("Downhill Brake Compensation"), "FordDownhillCompensation", "downhill_compensation")
    self.hands_free_cluster = self._toggle(
      tr_noop("Hands-Free Cluster Display"), "FordHandsFreeClusterMsg", "hands_free_cluster")
    self.driver_monitor_cluster = self._toggle(
      tr_noop("Driver Monitoring In Cluster"), "FordDriverMonitorCanMsg", "driver_monitor_cluster")
    self.brake_light_status = self._toggle(
      tr_noop("Brake Light Indicator"), "FordBrakeLightStatus", "brake_light_status")
    self.throttle_override_hold = self._toggle(
      tr_noop("Hold Throttle Through Pedal Override"), "ThrottleOverrideHold", "throttle_override_hold")
    self.pedal_override_threshold = self._slider(
      tr_noop("Pedal Override Threshold"), "FordPedalOverrideThreshold", "pedal_override_threshold",
      PEDAL_OVERRIDE_RANGE)

    self.angle_items = [
      self.low_speed_factor,
      self.high_speed_factor,
      self.high_speed_dampening,
      self.lane_change_factor_ang,
      self.sat_observer,
      self.delivery_compensation,
      # Shared with curvature mode. Angle mode used to hard-wire this on, so the toggle only ever
      # controlled curvature mode; it is listed in both groups now that it means the same in both.
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

    self.items = [
      self.primary_control,
      *self.angle_items,
      *self.curvature_items,
      LineSeparatorSP(),
      self.lateral_accel_limit,
      self.lateral_jerk_limit,
      self.follow_control,
      self.downhill_compensation,
      self.throttle_override_hold,
      self.pedal_override_threshold,
      self.hands_free_cluster,
      self.driver_monitor_cluster,
      self.brake_light_status,
    ]

  @staticmethod
  def _slider(title: str, param: str, description_key: str, value_range: tuple[float, float, float]):
    _, min_value, max_value = value_range
    return option_item_sp(
      lambda: tr(title),
      param,
      round(min_value * _SCALE),
      round(max_value * _SCALE),
      description=lambda: tr(DESCRIPTIONS[description_key]),
      value_change_step=_STEP,
      use_float_scaling=True,
      enabled=lambda: ui_state.is_offroad(),
    )

  @staticmethod
  def _toggle(title: str, param: str, description_key: str):
    return toggle_item_sp(
      lambda: tr(title),
      description=lambda: tr(DESCRIPTIONS[description_key]),
      initial_state=ui_state.params.get_bool(param),
      callback=lambda state, key=param: ui_state.params.put_bool(key, state),
      enabled=lambda: ui_state.is_offroad(),
    )

  @staticmethod
  def _on_primary_control_selected(index: int):
    # Read once at car init, together with the panda's lateral mode, so the change takes a restart.
    ui_state.params.put("FordPrefLateralControl", index)

  def update_settings(self):
    offroad = ui_state.is_offroad()
    mode = int(ui_state.params.get("FordPrefLateralControl") or 0)
    is_angle = mode == PrimaryLateralControl.angle
    is_curvature = mode == PrimaryLateralControl.curvature
    is_canfd = bool(ui_state.CP is not None and ui_state.CP.brand == "ford"
                    and ui_state.CP.flags & FordFlags.CANFD)

    self.primary_control.action_item.set_enabled(offroad)
    self.primary_control.action_item.set_selected_button(mode)

    description = tr(DESCRIPTIONS["primary_control"])
    if not offroad:
      description = "<b>" + tr(OFFROAD_ONLY) + "</b>\n\n" + description
    if self.primary_control.description != description:
      self.primary_control.set_description(description)
      self.primary_control.show_description(True)

    for index, item in enumerate(self.angle_items):
      item.set_visible(is_angle)
      item.action_item.set_enabled(offroad)
      # the tuning note belongs on the first factor only, where it reads as an intro to the group
      if index == 0 and is_angle:
        note = tr(ANGLE_TUNING_NOTE) + "\n\n" + tr(DESCRIPTIONS["low_speed_factor"])
        if item.description != note:
          item.set_description(note)
          item.show_description(True)

    custom_profile_on = ui_state.params.get_bool("FordCustomProfile_curv")
    lane_positioning_on = ui_state.params.get_bool("FordLanePositioning_curv")
    for item in self.curvature_items:
      visible = is_curvature
      if item in (self.blend_ratio_low, self.blend_ratio_high):
        visible = visible and custom_profile_on
      elif item is self.lane_positioning_gain:
        visible = visible and lane_positioning_on and custom_profile_on
      item.set_visible(visible)
      item.action_item.set_enabled(offroad)

    for item in (self.follow_control, self.downhill_compensation, self.driver_monitor_cluster,
                 self.throttle_override_hold, self.pedal_override_threshold,
                 self.lateral_accel_limit, self.lateral_jerk_limit):
      item.action_item.set_enabled(offroad)
    # the cluster's hands-free presentation only exists on CAN FD vehicles
    self.hands_free_cluster.set_visible(is_canfd)
    self.hands_free_cluster.action_item.set_enabled(offroad)
