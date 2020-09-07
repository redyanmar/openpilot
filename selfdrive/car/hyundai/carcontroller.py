from cereal import car, messaging
from common.realtime import DT_CTRL
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert


def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):
  sys_warning = (visual_alert == VisualAlert.steerRequired)

  # initialize to no line visible
  sys_state = 1
  if left_lane and right_lane or sys_warning:  # HUD alert only display when LKAS status is active
    if enabled or sys_warning:
      sys_state = 3
    else:
      sys_state = 4
  elif left_lane:
    sys_state = 5
  elif right_lane:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in [CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.vdiff = 0

    self.smartspeed = 20.
    self.recordsetspeed = 20.
    self.setspeed = 0.
    self.button_cnt = 0
    self.stopcontrolupdate = False
    self.last_button_frame = 0
    self.button_set_stop = self.button_res_stop = 0
    self.sm = messaging.SubMaster(['radarState'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart):

    self.lfa_available = True if self.car_fingerprint in FEATURES["send_lfa_mfa"] else False

    self.high_steer_allowed = True if self.car_fingerprint in FEATURES["allow_high_steer"] else False

    # Steering Torque
    new_steer = actuators.steer * SteerLimitParams.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and ((abs(CS.out.steeringAngle) < 90.) or self.high_steer_allowed)

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 16.7 and self.car_fingerprint == CAR.HYUNDAI_GENESIS:
      lkas_active = False

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning =\
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    can_sends = []
    self.clu11_cnt = frame % 0x10

    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.lfa_available))

    if pcm_cancel_cmd:
      self.vdiff = 0.
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL, self.clu11_cnt))
    elif CS.out.cruiseState.standstill and not self.longcontrol and CS.vrelative > 0:
      self.vdiff += (CS.vrelative - self.vdiff)
      if self.vdiff > 1. or CS.lead_distance > 8.:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL, self.clu11_cnt))
    else:
      self.vdiff = 0.

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.lfa_available:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    #detect stopped vehicle logic -> object with relative velocity -ve current vehicle speed + some hysteresis
    #Once detected allow maturation and calculate appropriate distance for decelration target

    self.sm.update(0)

    dat = self.sm['radarState'].leadOne
    if CS.is_set_speed_in_mph:
      self.setspeed = CS.out.cruiseState.speed * CV.MS_TO_MPH
    else:
      self.setspeed = CS.out.cruiseState.speed * CV.MS_TO_KPH

    if not CS.radar_obj_valid and dat.status and dat.vLead < 3. \
            and  CS.out.cruiseState.enabled and not CS.out.gasPressed:
      aRel = (dat.vLead**2 - CS.out.vEgo**2)/(2 * dat.dRel)
      print("aRel", aRel)
      if aRel < -.5 or self.stopcontrolupdate:
        self.stopcontrolupdate = True
        print("STOPPED VEHICLE")
        self.smartspeed = 20 if CS.is_set_speed_in_mph else 30
        if not self.stopcontrolupdate:
          self.button_cnt = 0
          self.recordsetspeed = self.setspeed
    else:
      if self.setspeed != self.recordsetspeed and self.stopcontrolupdate and  CS.out.cruiseState.enabled \
              and CS.radar_obj_valid and not CS.out.gasPressed:
        self.smartspeed = self.recordsetspeed
      else:
        self.stopcontrolupdate = False
        self.recordsetspeed = 0

    framestoskip = 10

    if (frame - self.last_button_frame) > framestoskip and self.stopcontrolupdate:
      if self.setspeed > (self.smartspeed * 1.005):
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.SET_DECEL, self.button_cnt))
        print("AUTO SLOW DOWN")
        if CS.cruise_buttons == 1:
          self.button_res_stop += 2
        else:
          self.button_res_stop -= 1
      elif self.setspeed < (self.smartspeed / 1.005):
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL, self.button_cnt))
        print("AUTO SPEED UP")
        if CS.cruise_buttons == 2:
          self.button_set_stop += 2
        else:
          self.button_set_stop -= 1
      else:
        self.button_res_stop = self.button_set_stop = 0

      if (abs(self.smartspeed - self.setspeed) < 0.5) or (self.button_res_stop >= 50) or (self.button_set_stop >= 50):
        self.stopcontrolupdate = False

      self.button_cnt += 1
      if self.button_cnt > 5:
        self.last_button_frame = frame
        self.button_cnt = 0
    else:
      self.button_set_stop = self.button_res_stop = 0

    return can_sends
