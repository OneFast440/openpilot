"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom

from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.modeld_base import ModelStateBase
from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0

EventName = log.OnroadEvent.EventName

# opendbc.sunnypilot.car.ford.values_ext.PrimaryLateralControl: 0 stock, 1 curvature, 2 angle
FORD_PRIMARY_LATERAL_STOCK = 0

# capnp enum name -> the small ints CarControlSP.FordLateral carries. Kept as a lookup rather
# than a .raw read so a reordered log.capnp enum cannot silently change the wire meaning.
LANE_CHANGE_STATE = {'off': 0, 'preLaneChange': 1, 'laneChangeStarting': 2, 'laneChangeFinishing': 3}
LANE_CHANGE_DIRECTION = {'none': 0, 'left': 1, 'right': 2}


class ControlsExt(ModelStateBase):
  def __init__(self, CP: structs.CarParams, params: Params):
    ModelStateBase.__init__(self)
    self.CP = CP
    self.params = params
    self._param_update_time: float = 0.0
    self.blinker_pause_lateral = BlinkerPauseLateral()
    self.throttle_override_hold = params.get_bool("ThrottleOverrideHold")

    cloudlog.info("controlsd_ext is waiting for CarParamsSP")
    self.CP_SP = messaging.log_from_bytes(params.get("CarParamsSP", block=True), custom.CarParamsSP)
    cloudlog.info("controlsd_ext got CarParamsSP")

    self.sm_services_ext = ['radarState', 'selfdriveStateSP']
    self.pm_services_ext = ['carControlSP']

    # Ford's BluePilot lateral strategies consume model curvature, lane lines, lateral delay and
    # driver monitoring state. opendbc must not import openpilot, so the values are published on
    # carControlSP instead of read from a SubMaster inside the car controller. Only populated
    # when one of those strategies is actually selected.
    self.ford_lateral = (self.CP.brand == 'ford' and
                         self.CP_SP.fordLateralTuning.primaryControl != FORD_PRIMARY_LATERAL_STOCK)

  def initialize_lateral_control(self, lac, CI, dt):
    enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    torque_versions = self.params.get("TorqueControlTune")
    if not enforce_torque_control:
      if self.CP.lateralTuning.which() == 'torque':
        return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)  # FIXME-SP: revert when upstream fixes tuning issues with v1
      return lac

    if torque_versions == 0.0:  # v0
      return LatControlTorqueV0(self.CP, self.CP_SP, CI, dt)
    else:
      return lac

  def get_params_sp(self, sm: messaging.SubMaster) -> None:
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.blinker_pause_lateral.get_params()

      if self.CP.lateralTuning.which() == 'torque':
        self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)

      self.throttle_override_hold = self.params.get_bool("ThrottleOverrideHold")

      self._param_update_time = time.monotonic()

  def get_long_active(self, sm: messaging.SubMaster, enabled: bool) -> bool:
    """Longitudinal engagement, with the accelerator override made optional.

    Upstream hands longitudinal control back for as long as the accelerator is down: the
    request drops out, the long control state machine goes to off and its PID resets, so
    lifting off starts again from nothing. With the hold enabled the accelerator override no
    longer disengages, so openpilot keeps computing and keeps sending its own request while
    the driver is on the pedal, and the PCM arbitrates between the two. Lifting off then
    hands back at whatever openpilot was already asking for rather than from a reset.

    Every other longitudinal override still disengages exactly as before.
    """
    overrides = [e for e in sm['onroadEvents'] if e.overrideLongitudinal]
    if self.throttle_override_hold:
      overrides = [e for e in overrides if e.name != EventName.gasPressedOverride]
    return (enabled and not overrides and
            (self.CP.openpilotLongitudinalControl or not self.CP_SP.pcmCruiseSpeed))

  def get_lat_active(self, sm: messaging.SubMaster) -> bool:
    if self.blinker_pause_lateral.update(sm['carState']):
      return False

    ss_sp = sm['selfdriveStateSP']
    if ss_sp.mads.available:
      return bool(ss_sp.mads.active)

    # MADS not available, use stock state to engage
    return bool(sm['selfdriveState'].active)

  @staticmethod
  def get_lead_data(_lead, src: log.RadarState.LeadData) -> None:
    _lead.dRel = src.dRel
    _lead.yRel = src.yRel
    _lead.vRel = src.vRel
    _lead.aRel = src.deprecated.aRel
    _lead.vLead = src.vLead
    _lead.dPath = src.deprecated.dPath
    _lead.vLat = src.deprecated.vLat
    _lead.vLeadK = src.vLeadK
    _lead.aLeadK = src.aLeadK
    _lead.fcw = src.deprecated.fcw
    _lead.status = src.present
    _lead.aLeadTau = src.aLeadTau
    _lead.modelProb = src.modelProb
    _lead.radar = src.radar
    _lead.radarTrackId = src.radarTrackId

  def state_control_ext(self, sm: messaging.SubMaster) -> custom.CarControlSP:
    CC_SP = custom.CarControlSP.new_message()

    self.get_lead_data(CC_SP.leadOne, sm['radarState'].leadOne)
    self.get_lead_data(CC_SP.leadTwo, sm['radarState'].leadTwo)

    # MADS state
    mads_src = sm['selfdriveStateSP'].mads
    CC_SP.mads.state = mads_src.state
    CC_SP.mads.enabled = mads_src.enabled
    CC_SP.mads.active = mads_src.active
    CC_SP.mads.available = mads_src.available

    # ICBM state
    icbm_src = sm['selfdriveStateSP'].intelligentCruiseButtonManagement
    CC_SP.intelligentCruiseButtonManagement.state = icbm_src.state
    CC_SP.intelligentCruiseButtonManagement.sendButton = icbm_src.sendButton
    CC_SP.intelligentCruiseButtonManagement.vTarget = icbm_src.vTarget

    if self.ford_lateral:
      self.get_ford_lateral(CC_SP.fordLateral, sm)

    return CC_SP

  @staticmethod
  def get_ford_lateral(dest, sm: messaging.SubMaster) -> None:
    model = sm['modelV2']
    v_ego = max(sm['carState'].vEgo, 0.01)
    dest.modelCurvatures = [float(z) / v_ego for z in model.orientationRate.z]
    dest.modelPositionY = [float(y) for y in model.position.y]
    dest.lateralDelay = float(sm['lateralDelay'].lateralDelay)
    dest.laneChangeState = LANE_CHANGE_STATE.get(str(model.meta.laneChangeState), 0)
    dest.laneChangeDirection = LANE_CHANGE_DIRECTION.get(str(model.meta.laneChangeDirection), 0)
    dest.alertType = sm['selfdriveState'].alertType

    # the two inner lane lines, which curvature mode blends toward for lane centering
    if len(model.laneLines) >= 3 and len(model.laneLineProbs) >= 3:
      dest.laneLineLeftY = float(model.laneLines[1].y[0]) if len(model.laneLines[1].y) else 0.0
      dest.laneLineRightY = float(model.laneLines[2].y[0]) if len(model.laneLines[2].y) else 0.0
      dest.laneLineLeftProb = float(model.laneLineProbs[1])
      dest.laneLineRightProb = float(model.laneLineProbs[2])

  @staticmethod
  def publish_ext(CC_SP: custom.CarControlSP, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    cc_sp_send = messaging.new_message('carControlSP')
    cc_sp_send.valid = sm['carState'].canValid
    cc_sp_send.carControlSP = CC_SP

    pm.send('carControlSP', cc_sp_send)

  def run_ext(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    CC_SP = self.state_control_ext(sm)
    self.publish_ext(CC_SP, sm, pm)
