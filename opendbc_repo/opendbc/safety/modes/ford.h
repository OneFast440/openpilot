#pragma once

#include "opendbc/safety/declarations.h"

// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
#define FORD_Steering_Data_FD1     0x083U   // TX by OP, various driver switches and LKAS/CC buttons
#define FORD_ACCDATA               0x186U   // TX by OP, ACC controls
#define FORD_ACCDATA_3             0x18AU   // TX by OP, ACC/TJA user interface
#define FORD_Lane_Assist_Data1     0x3CAU   // TX by OP, Lane Keep Assist
#define FORD_LateralMotionControl  0x3D3U   // TX by OP, Lateral Control message
#define FORD_LateralMotionControl2 0x3D6U   // TX by OP, alternate Lateral Control message
#define FORD_IPMA_Data             0x3D8U   // TX by OP, IPMA and LKAS user interface

// CAN bus numbers.
#define FORD_MAIN_BUS 0U
#define FORD_CAM_BUS  2U

static uint8_t ford_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cnt
    cnt = (msg->data[2] >> 2) & 0xFU;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYaw_No_Cnt
    cnt = msg->data[5];
  } else {
  }
  return cnt;
}

static uint32_t ford_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cs
    chksum = msg->data[3];
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYawW_No_Cs
    chksum = msg->data[4];
  } else {
  }
  return chksum;
}

static uint32_t ford_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    chksum += msg->data[0] + msg->data[1];  // Veh_V_ActlBrk
    chksum += msg->data[2] >> 6;                    // VehVActlBrk_D_Qf
    chksum += (msg->data[2] >> 2) & 0xFU;           // VehVActlBrk_No_Cnt
    chksum = 0xFFU - chksum;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    chksum += msg->data[0] + msg->data[1];  // VehRol_W_Actl
    chksum += msg->data[2] + msg->data[3];  // VehYaw_W_Actl
    chksum += msg->data[5];                         // VehRollYaw_No_Cnt
    chksum += msg->data[6] >> 6;                    // VehRolWActl_D_Qf
    chksum += (msg->data[6] >> 4) & 0x3U;           // VehYawWActl_D_Qf
    chksum = 0xFFU - chksum;
  } else {
  }
  return chksum;
}

static bool ford_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == FORD_BrakeSysFeatures) {
    valid = (msg->data[2] >> 6) == 0x3U;           // VehVActlBrk_D_Qf
  } else if (msg->addr == FORD_EngVehicleSpThrottle2) {
    valid = ((msg->data[4] >> 5) & 0x3U) == 0x3U;  // VehVActlEng_D_Qf
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    valid = ((msg->data[6] >> 4) & 0x3U) == 0x3U;  // VehYawWActl_D_Qf
  } else {
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

static const CurvatureSteeringLimits FORD_STEERING_LIMITS = {
  .max_curvature = 1000,              // 0.02 rad/m * curvature_to_can
  .curvature_to_can = 50000,          // CAN units per rad/m
  .frequency = 20,                    // Hz
  .max_curvature_error = 100,         // 0.002 rad/m * curvature_to_can
  .curvature_error_min_speed = 10.0,  // m/s
  .max_steer_power = 0,               // disabled, Ford has no steed power signal
};

// *** sunnypilot: path-angle-primary lateral control ("angle control", from BluePilot) ***
//
// Angle control steers with path_angle (c1) and holds curvature (c2), curvature rate (c3) and
// path offset (c0) at their inactive sentinels, because the PSCM low-pass filters c2 for up to a
// second and keeps acting on a stale command. c1 has no such filter state.
//
// Selected once at car init via the sunnypilot safety param (current_safety_param_sp, delivered
// over USB 0xdf before the safety model is set -- a separate uint16 from ford_init's own param;
// same pattern as Subaru STOP_AND_GO). The checks below are a separate, self-contained branch:
// with angle control off, not one byte of the stock path changes.

// LatCtlPath_An_Actl scaling: 0.0005 rad per LSB. path_angle is the actuator in angle mode, so it
// may use the full signal range; the stock path leaves it at its sentinel.
// There is no value limit to enforce here: the signal is 11 bits, so the extraction above yields
// 0..2047, and subtracting FORD_INACTIVE_PATH_ANGLE gives exactly [-1000, 1047] CAN units, i.e.
// the DBC's own [-0.5, 0.5235] rad range. The wire format is the limit.
#define FORD_PATH_ANGLE_TO_CAN 2000.0f

static bool ford_angle_control = false;       // set in ford_init, never changes during a drive
static bool ford_angle_mode_engaged = false;  // latched out of Lane_Assist_Data1 in ford_tx_hook
static int ford_shadow_curvature_raw = 0;     // wire units, scale 1e-6 1/m
static int ford_desired_path_angle_last = 0;

// shadow_curvature is sent at a wire scale of 1e-6 1/m (see fordcan_ext.py); convert to the CAN
// units the curvature checks use, matching FORD_STEERING_LIMITS.curvature_to_can (50000):
// raw * 1e-6 * 50000 = raw * 0.05.
static int ford_shadow_curvature_to_can(int raw) {
  return ROUND((float)raw * 0.05f);
}

// path_angle rate-of-change limit. Rate is anchored to the command sequence rather than to a
// measurement -- there is no "measured path angle" -- so the previous command is recorded even on
// a violation: holding it stale would desync this window from openpilot's own (tighter) limiter
// and turn one blocked frame into a runaway of blocked frames.
static bool ford_path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled) {
  bool violation = false;

  // Mirrors the soft rate limit in opendbc/sunnypilot/car/ford/lateral_angle_ext.py (_SOFT_ROC_*),
  // scaled x1.02 so this backstop is always slightly looser than openpilot's own limit and never
  // blocks a well-behaved command. Values are per call, and LateralMotionControl(2) is sent once
  // per CarControllerParams.STEER_STEP (5), i.e. at 20 Hz, not per 100 Hz tick. lookup_t holds
  // three points; openpilot's 9 and 10 m/s nodes are both 0.055 (flat top), so {10, 15, 25}
  // reproduces the curve exactly and lower speeds clamp to the first point.
  static const struct lookup_t FORD_PATH_ANGLE_ROC = {
    {10., 15., 25.},
    {0.0561, 0.04335, 0.00918}
  };

  if (steer_control_enabled) {
    // fudge the speed by 1 m/s so the limit is always slightly above openpilot's, in case a
    // newer speed is read between two commands
    const float fudged_speed = (vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;
    const int delta = (safety_interpolate(FORD_PATH_ANGLE_ROC, fudged_speed) * FORD_PATH_ANGLE_TO_CAN) + 1.;
    violation |= safety_max_limit_check(desired_path_angle, ford_desired_path_angle_last + delta,
                                        ford_desired_path_angle_last - delta);
  } else {
    // path_angle must be at its inactive sentinel while not steering
    violation |= desired_path_angle != 0;
  }

  ford_desired_path_angle_last = desired_path_angle;
  if (!(controls_allowed || controls_allowed_lateral)) {
    ford_desired_path_angle_last = 0;
  }

  return violation;
}

// Angle mode pins the curvature signal at its inactive sentinel, so steer_curvature_cmd_checks
// has nothing to compare against measured curvature and the deviation protection would be lost
// entirely. openpilot therefore publishes the curvature its path_angle was derived from
// alongside the LKA message (see fordcan_ext.create_lka_msg), and it is checked here.
//
// Narrower than steer_curvature_cmd_checks in one respect: no lateral-jerk rate-of-change term.
// shadow_curvature is not an actuator -- path_angle is, and it carries its own tuned rate limit
// above. Imposing a second, curvature-tuned rate limit on a pure cross-check value blocks at low
// speed for reasons unrelated to how the car is actually steering. The absolute cap, the ISO
// lateral acceleration cap and the deviation-from-measured band all still apply, so angle mode
// stays inside the same cornering envelope as every other platform.
static bool ford_shadow_curvature_checks(int shadow_curvature, bool steer_control_enabled,
                                         const CurvatureSteeringLimits limits) {
  static const float MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL + (EARTH_G * AVERAGE_ROAD_ROLL);  // ~3.6 m/s^2
  bool violation = false;

  if (steer_control_enabled) {
    violation |= safety_max_limit_check(shadow_curvature, limits.max_curvature, -limits.max_curvature);

    // *** ISO lateral accel limit ***
    const float fudged_speed = SAFETY_MAX((vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.0, 1.0);
    const int max_curvature_can = (MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed) * limits.curvature_to_can) + 1.;
    violation |= safety_max_limit_check(shadow_curvature, max_curvature_can, -max_curvature_can);

    if ((limits.max_curvature_error != 0) &&
        ((vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits.curvature_error_min_speed)) {
      const int lowest_allowed = curvature_state.meas.min - limits.max_curvature_error - 1;
      const int highest_allowed = curvature_state.meas.max + limits.max_curvature_error + 1;
      violation |= safety_max_limit_check(shadow_curvature, highest_allowed, lowest_allowed);
    }
  }

  return violation;
}

static bool ford_angle_mode_tx_checks(bool steer_control_enabled, unsigned int raw_curvature,
                                      unsigned int raw_curvature_rate, unsigned int raw_path_angle,
                                      unsigned int raw_path_offset, unsigned int inactive_curvature_rate) {
  bool violation = false;

  // Ford adjusts its limits by speed, so it checks two speed sources against each other and drops
  // controls when they disagree. steer_curvature_cmd_checks does this for the stock path; angle
  // mode does not call it, so do it here, before the controls_allowed gate below sees the result.
  speed_mismatch_check((float)vehicle_speed_2.values[0] / VEHICLE_SPEED_FACTOR);

  // path_angle is the only actuator in angle mode; everything else stays at its sentinel
  violation |= raw_curvature != FORD_INACTIVE_CURVATURE;
  violation |= raw_curvature_rate != inactive_curvature_rate;
  violation |= raw_path_offset != FORD_INACTIVE_PATH_OFFSET;

  const int desired_path_angle = (int)raw_path_angle - (int)FORD_INACTIVE_PATH_ANGLE;
  violation |= ford_path_angle_cmd_checks(desired_path_angle, steer_control_enabled);

  violation |= ford_shadow_curvature_checks(ford_shadow_curvature_to_can(ford_shadow_curvature_raw),
                                            steer_control_enabled, FORD_STEERING_LIMITS);

  // path_angle's rate limit is per message, so without this openpilot could slew five times
  // faster than intended simply by sending LateralMotionControl at 100 Hz instead of 20 Hz.
  // steer_curvature_cmd_checks does this for the stock path; angle mode does not call it, and
  // curvature_state's real-time window is otherwise unused here.
  if (steer_control_enabled) {
    violation |= rt_curvature_rate_limit_check(FORD_STEERING_LIMITS);
  }

  // Corroboration: the LKA message must independently say angle mode is engaged before
  // path_angle is allowed to leave its sentinel, so one crafted LateralMotionControl frame
  // cannot steer on its own.
  violation |= steer_control_enabled && !ford_angle_mode_engaged;

  // No lateral control at all when controls are not allowed
  violation |= steer_control_enabled && !(controls_allowed || controls_allowed_lateral);

  // keep the stock path's state coherent: nothing is ever commanded on the curvature signal here
  curvature_state.desired_last = 0;

  return violation;
}

static void ford_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == FORD_MAIN_BUS) {
    // Update in motion state from standstill signal
    if (msg->addr == FORD_DesiredTorqBrk) {
      // Signal: VehStop_D_Stat
      vehicle_moving = ((msg->data[3] >> 3) & 0x3U) != 1U;
    }

    // Update vehicle speed
    if (msg->addr == FORD_BrakeSysFeatures) {
      // Signal: Veh_V_ActlBrk
      UPDATE_VEHICLE_SPEED(((msg->data[0] << 8) | msg->data[1]) * 0.01 * KPH_TO_MS);
    }

    // Check vehicle speed against a second source
    if (msg->addr == FORD_EngVehicleSpThrottle2) {
      // Disable controls if speeds from ABS and PCM ECUs are too far apart.
      // Signal: Veh_V_ActlEng
      float filtered_pcm_speed = ((msg->data[6] << 8) | msg->data[7]) * 0.01 * KPH_TO_MS;
      UPDATE_VEHICLE_SPEED_2(filtered_pcm_speed);
    }

    // Update vehicle yaw rate
    if (msg->addr == FORD_Yaw_Data_FD1) {
      // FIXME: safety can receive yaw before new vehicle speed, it should recompute meas on either received
      // Signal: VehYaw_W_Actl
      // TODO: we should use the speed which results in the closest angle measurement to the desired angle
      float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
      float current_curvature = ford_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
      // convert current curvature into units on CAN for comparison with desired curvature
      update_sample(&curvature_state.meas, ROUND(current_curvature * FORD_STEERING_LIMITS.curvature_to_can));
    }

    // Update gas pedal
    if (msg->addr == FORD_EngVehicleSpThrottle) {
      // Pedal position: (0.1 * val) in percent
      // Signal: ApedPos_Pc_ActlArb
      gas_pressed = (((msg->data[0] & 0x03U) << 8) | msg->data[1]) > 0U;
    }

    // Update brake pedal and cruise state
    if (msg->addr == FORD_EngBrakeData) {
      // Signal: BpedDrvAppl_D_Actl
      brake_pressed = ((msg->data[0] >> 4) & 0x3U) == 2U;

      // Signal: CcStat_D_Actl
      unsigned int cruise_state = msg->data[1] & 0x07U;
      bool cruise_engaged = (cruise_state == 4U) || (cruise_state == 5U);
      pcm_cruise_check(cruise_engaged);
      acc_main_on = (cruise_state == 3U) || cruise_engaged;
    }

    if (msg->addr == FORD_Steering_Data_FD1) {
      mads_button_press = GET_BIT(msg, 40U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
  }
}

static bool ford_tx_hook(const CANPacket_t *msg) {
  const LongitudinalLimits FORD_LONG_LIMITS = {
    // acceleration cmd limits (used for brakes)
    // Signal: AccBrkTot_A_Rq
    .max_accel = 5641,       //  1.9999 m/s^s
    .min_accel = 4231,       // -3.4991 m/s^2
    .inactive_accel = 5128,  // -0.0008 m/s^2

    // gas cmd limits
    // Signal: AccPrpl_A_Rq & AccPrpl_A_Pred
    .max_gas = 700,          //  2.0 m/s^2
    .min_gas = 450,          // -0.5 m/s^2
    .inactive_gas = 0,       // -5.0 m/s^2
  };

  bool tx = true;

  // Safety check for ACCDATA accel and brake requests
  if (msg->addr == FORD_ACCDATA) {
    // Signal: AccPrpl_A_Rq
    int gas = ((msg->data[6] & 0x3U) << 8) | msg->data[7];
    // Signal: AccPrpl_A_Pred
    int gas_pred = ((msg->data[2] & 0x3U) << 8) | msg->data[3];
    // Signal: AccBrkTot_A_Rq
    int accel = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    // Signal: CmbbDeny_B_Actl
    bool cmbb_deny = (msg->data[4] >> 5) & 1U;

    // Signal: AccBrkPrchg_B_Rq & AccBrkDecel_B_Rq
    bool brake_actuation = ((msg->data[6] >> 6) & 1U) || ((msg->data[6] >> 7) & 1U);

    bool violation = false;
    violation |= longitudinal_accel_checks(accel, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas_pred, FORD_LONG_LIMITS);

    // Safety check for stock AEB
    violation |= cmbb_deny; // do not prevent stock AEB actuation

    violation |= !get_longitudinal_allowed() && brake_actuation;

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Steering_Data_FD1 button signals
  // Note: Many other signals in this message are not relevant to safety (e.g. blinkers, wiper switches, high beam)
  // which we passthru in OP.
  if (msg->addr == FORD_Steering_Data_FD1) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    bool violation = false;
    violation |= ((msg->data[1] >> 0) & 1U) && !cruise_engaged_prev;   // Signal: CcAslButtnCnclPress (cancel)
    violation |= ((msg->data[3] >> 1) & 1U) && !controls_allowed;     // Signal: CcAsllButtnResPress (resume)

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Lane_Assist_Data1 action
  if (msg->addr == FORD_Lane_Assist_Data1) {
    // Do not allow steering using Lane_Assist_Data1 (Lane-Departure Aid).
    // This message must be sent for Lane Centering to work, and can include
    // values such as the steering angle or lane curvature for debugging,
    // but the action (LkaActvStats_D2_Req) must be set to zero.
    unsigned int action = msg->data[0] >> 5;
    if (action != 0U) {
      tx = false;
    }

    // sunnypilot: angle control carries angle_mode_engaged and shadow_curvature in bits of this
    // message that no DBC signal maps to (byte 4 bit 0, bytes 5-6; see fordcan_ext.create_lka_msg
    // for the layout). Read straight out of the frame being transmitted, in the same tx_hook call
    // that already reads LkaActvStats_D2_Req above -- panda does not self-receive its own TX, so
    // a dedicated CAN id would never arrive.
    if (ford_angle_control) {
      ford_angle_mode_engaged = (msg->data[4] & 0x1U) != 0U;
      const uint32_t shadow_unsigned = ((uint32_t)msg->data[5] << 8) | (uint32_t)msg->data[6];
      ford_shadow_curvature_raw = (shadow_unsigned > 32767U) ? ((int)shadow_unsigned - 65536) : (int)shadow_unsigned;
    }
  }

  // Safety check for LateralMotionControl action
  if (msg->addr == FORD_LateralMotionControl) {
    // Signal: LatCtl_D_Rq
    bool steer_control_enabled = ((msg->data[4] >> 2) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[0] << 3) | (msg->data[1] >> 5);
    unsigned int raw_curvature_rate = ((msg->data[1] & 0x1FU) << 8) | msg->data[2];
    unsigned int raw_path_angle = (msg->data[3] << 3) | (msg->data[4] >> 5);
    unsigned int raw_path_offset = (msg->data[5] << 2) | (msg->data[6] >> 6);

    bool violation;
    if (ford_angle_control) {
      violation = ford_angle_mode_tx_checks(steer_control_enabled, raw_curvature, raw_curvature_rate,
                                            raw_path_angle, raw_path_offset, FORD_INACTIVE_CURVATURE_RATE);
    } else {
      // These signals are not yet tested with the current safety limits
      violation = (raw_curvature_rate != FORD_INACTIVE_CURVATURE_RATE) || (raw_path_angle != FORD_INACTIVE_PATH_ANGLE) || (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);

      // Check angle error and steer_control_enabled
      int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.curvature_to_can to get real curvature
      violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, FORD_STEERING_LIMITS);
    }

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl2 action
  if (msg->addr == FORD_LateralMotionControl2) {
    // Signal: LatCtl_D2_Rq
    bool steer_control_enabled = ((msg->data[0] >> 4) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[2] << 3) | (msg->data[3] >> 5);
    unsigned int raw_curvature_rate = (msg->data[6] << 3) | (msg->data[7] >> 5);
    unsigned int raw_path_angle = ((msg->data[3] & 0x1FU) << 6) | (msg->data[4] >> 2);
    unsigned int raw_path_offset = ((msg->data[4] & 0x3U) << 8) | msg->data[5];

    bool violation;
    if (ford_angle_control) {
      violation = ford_angle_mode_tx_checks(steer_control_enabled, raw_curvature, raw_curvature_rate,
                                            raw_path_angle, raw_path_offset, FORD_CANFD_INACTIVE_CURVATURE_RATE);
    } else {
      // These signals are not yet tested with the current safety limits
      violation = (raw_curvature_rate != FORD_CANFD_INACTIVE_CURVATURE_RATE) || (raw_path_angle != FORD_INACTIVE_PATH_ANGLE) || (raw_path_offset != FORD_INACTIVE_PATH_OFFSET);

      // Check angle error and steer_control_enabled
      int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;  // /FORD_STEERING_LIMITS.curvature_to_can to get real curvature
      violation |= steer_curvature_cmd_checks(desired_curvature, 0, steer_control_enabled, FORD_STEERING_LIMITS);
    }

    if (violation) {
      tx = false;
    }
  }

  return tx;
}

static safety_config ford_init(uint16_t param) {
  // warning: quality flags are not yet checked in openpilot's CAN parser,
  // this may be the cause of blocked messages
  static RxCheck ford_rx_checks[] = {
    {.msg = {{FORD_BrakeSysFeatures, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
    // FORD_EngVehicleSpThrottle2 has a counter that either randomly skips or by 2, likely ECU bug
    // Some hybrid models also experience a bug where this checksum mismatches for one or two frames under heavy acceleration with ACC
    // It has been confirmed that the Bronco Sport's camera only disallows ACC for bad quality flags, not counters or checksums, so we match that
    {.msg = {{FORD_EngVehicleSpThrottle2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Yaw_Data_FD1, 0, 8, 100U, .max_counter = 255U}, { 0 }, { 0 }}},
    // These messages have no counter or checksum
    {.msg = {{FORD_EngBrakeData, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_EngVehicleSpThrottle, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_DesiredTorqBrk, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Steering_Data_FD1, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  #define FORD_COMMON_TX_MSGS \
    {FORD_Steering_Data_FD1, 0, 8, .check_relay = false}, \
    {FORD_Steering_Data_FD1, 2, 8, .check_relay = false}, \
    {FORD_ACCDATA_3, 0, 8, .check_relay = true},          \
    {FORD_Lane_Assist_Data1, 0, 8, .check_relay = true},  \
    {FORD_IPMA_Data, 0, 8, .check_relay = true},          \

#ifdef ALLOW_DEBUG
  static const CanMsg FORD_CANFD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };
#endif

  static const CanMsg FORD_CANFD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  const uint16_t FORD_PARAM_CANFD = 2;
  const bool ford_canfd = GET_FLAG(param, FORD_PARAM_CANFD);

  // sunnypilot: angle control, from the SP safety param. openpilot sets the matching
  // CP_SP.fordLateralTuning at the same moment it sets this flag, so the two layers can never
  // disagree about which signal is steering.
  const uint16_t FORD_PARAM_SP_ANGLE_CONTROL = 1;
  ford_angle_control = GET_FLAG(current_safety_param_sp, FORD_PARAM_SP_ANGLE_CONTROL);
  ford_angle_mode_engaged = false;
  ford_shadow_curvature_raw = 0;
  ford_desired_path_angle_last = 0;

  safety_config ret;
  if (ford_canfd) {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
#ifdef ALLOW_DEBUG
    const uint16_t FORD_PARAM_LONGITUDINAL = 1;
    if (GET_FLAG(param, FORD_PARAM_LONGITUDINAL)) {
      ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS);
    }
#endif
  } else {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS);
  }
  return ret;
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .get_counter = ford_get_counter,
  .get_checksum = ford_get_checksum,
  .compute_checksum = ford_compute_checksum,
  .get_quality_flag_valid = ford_get_quality_flag_valid,
};
