import math
import numpy as np
from iqpilot.cereal import car
from iqpilot.common.realtime import DT_CTRL
from iqpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from iqpilot.common.filter_simple import FirstOrderFilter
from iqpilot.common.pid import PIDController
from iqpilot.selfdrive.iqmodeld.config import ModelConstants
from iqpilot.selfdrive.controls.lib.smooth_stops import SMOOTH_STOP_MAX_SPEED, SmoothStopController

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# measured accel response: EV6 0.15 s dead time + 0.7 s lag, 2021 Elantra 0.4 s + 1.1 s, where a speed-error loop hunts
SPEED_ERROR_PID_CARS = {"KIA_EV6"}
FILTERED_A_EGO_CARS = {"HYUNDAI_ELANTRA_2021"}
A_EGO_FILTER_TS = 0.3
# closed-loop replay of 2021 Elantra drives: at gain 1 the accel-error loop cycles throttle and brake around a smooth plan
# (17.7 flips/min in the city), at 0.4 it halves that and actual accel tracking is unchanged
ACCEL_ERROR_KP = {"HYUNDAI_ELANTRA_2021": 0.4}

# Resume brake ramp (StarPilot longcontrol.py; there the ResumeBrakeRamp toggle, default off; baked in on here at the
# owner's request, 2026-10-05). Route 000002cc 3:32 and 5:20: the planner keeps a decel target while the driver holds
# the gas above a lower set/limit speed; on release the command stepped 0 -> -1.3 in one frame and the Civic's
# late-then-overshooting brakes turned that into a lurch. For RESUME_BRAKE_RAMP_TIME after off -> pid the brake command
# may deepen by at most RESUME_BRAKE_RAMP_RATE per second. Never applied while stopping, for targets at or below
# RESUME_BRAKE_RAMP_BYPASS_DECEL, or with a lead closer than RESUME_BRAKE_RAMP_BYPASS_DREL: a real brake demand is not
# delayed. Releasing brake is never limited. Open-loop replay evidence only; not driven on IQ.Pilot.
RESUME_BRAKE_RAMP = True
RESUME_BRAKE_RAMP_CARS = {"HONDA_CIVIC_BOSCH"}  # the evidence is Civic Bosch only; other cars keep IQ.Pilot's behavior
RESUME_BRAKE_RAMP_TIME = 1.5  # s
RESUME_BRAKE_RAMP_RATE = 1.5  # m/s^3; 2b7 command jerk p95 while braking was 1.27
RESUME_BRAKE_RAMP_BYPASS_DECEL = -2.0  # m/s^2
RESUME_BRAKE_RAMP_BYPASS_DREL = 30.0  # m
_DREL_ROLLING_FRAMES = 10
_DREL_FILTER_ALPHA = 0.3


class LongControl:
  resume_brake_ramp = False
  resume_brake_ramp_frames = 0
  _drel_window: tuple[float, ...] = ()
  _drel_filtered = float("inf")

  def __init__(self, CP, CP_IQ):
    self.CP = CP
    self.CP_IQ = CP_IQ
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.stopping_decel_rate = CP_IQ.stoppingDecelRateOverride or 1.0
    self.smooth = SmoothStopController()
    self.speed_error_pid = CP.carFingerprint in SPEED_ERROR_PID_CARS
    self.filter_a_ego = CP.carFingerprint in FILTERED_A_EGO_CARS
    self.a_ego_filter = FirstOrderFilter(0.0, A_EGO_FILTER_TS, DT_CTRL, initialized=False)
    if CP.carFingerprint in ACCEL_ERROR_KP:
      self.pid._k_p = ([0.0], [ACCEL_ERROR_KP[CP.carFingerprint]])
    self.resume_brake_ramp = RESUME_BRAKE_RAMP and str(CP.carFingerprint) in RESUME_BRAKE_RAMP_CARS

  def reset(self):
    self.pid.reset()

  def _update_state(self, active, car_state, stop_requested):
    if not active:
      self.long_control_state = LongCtrlState.off
      return

    holding = self.long_control_state == LongCtrlState.stopping
    hold_requested = stop_requested
    if self.smooth.enabled and not holding:
      hold_requested = self.smooth.want_hold(stop_requested, car_state.vEgo, car_state.standstill)

    if hold_requested:
      self.long_control_state = LongCtrlState.stopping
      return

    if self.long_control_state == LongCtrlState.pid:
      return

    release_blocked = car_state.brakePressed or (
      car_state.cruiseState.standstill and not self.CP_IQ.enableGasInterceptor
    )
    self.long_control_state = LongCtrlState.stopping if release_blocked else LongCtrlState.pid

  def update(self, active, CS, a_target, should_stop, accel_limits, lead_distance=0.0, has_lead=False, gas_override=False, *, v_target_now):
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    self.smooth.update()
    a_ego = self.a_ego_filter.update(CS.aEgo) if self.filter_a_ego else CS.aEgo
    # leadDistance is 0 with no lead; StarPilot's filter wants "no lead" as inf
    drel_filtered = self._update_drel_filter(lead_distance if has_lead else None)

    previous_long_control_state = self.long_control_state
    self._update_state(active, CS, should_stop)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      self.smooth.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.stopping_decel_rate * DT_CTRL
      self.reset()
      self.smooth.reset()

    else:
      if self.smooth.enabled and active and should_stop and CS.vEgo <= SMOOTH_STOP_MAX_SPEED:
        output_accel = self.smooth.settle(a_target, CS.vEgo, lead_distance, has_lead, self.last_output_accel)
        self.reset()
      else:
        error = v_target_now - CS.vEgo if self.speed_error_pid else a_target - a_ego
        freeze_integrator = gas_override
        # the resume brake ramp holds output above the target on purpose; don't let the integrator wind up behind it
        if self.resume_brake_ramp and a_target < 0.0 and \
           (self.resume_brake_ramp_frames > 0 or previous_long_control_state == LongCtrlState.off):
          freeze_integrator = True
        output_accel = self.pid.update(error, speed=CS.vEgo,
                                       feedforward=a_target,
                                       freeze_integrator=freeze_integrator)
        self.smooth.reset()

    if self.resume_brake_ramp:
      output_accel = self._ramp_resume_brake(output_accel, previous_long_control_state, a_target, should_stop, drel_filtered)
    else:
      self.resume_brake_ramp_frames = 0

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel

  def _update_drel_filter(self, drel):
    """StarPilot's lead-distance filter: rolling min over 10 frames, then an EMA. inf when there is no lead."""
    if drel is None or not math.isfinite(drel):
      self._drel_window = ()
      self._drel_filtered = float("inf")
      return float("inf")
    self._drel_window = (*self._drel_window, float(drel))[-_DREL_ROLLING_FRAMES:]
    rolling_min = min(self._drel_window)
    if not math.isfinite(self._drel_filtered):
      self._drel_filtered = rolling_min
    else:
      self._drel_filtered = _DREL_FILTER_ALPHA * rolling_min + (1.0 - _DREL_FILTER_ALPHA) * self._drel_filtered
    return self._drel_filtered

  def _ramp_resume_brake(self, output_accel, previous_long_control_state, a_target, should_stop, drel_filtered):
    if self.long_control_state != LongCtrlState.pid:
      self.resume_brake_ramp_frames = 0
      return output_accel
    if previous_long_control_state == LongCtrlState.off:
      self.resume_brake_ramp_frames = int(round(RESUME_BRAKE_RAMP_TIME / DT_CTRL))
    if self.resume_brake_ramp_frames <= 0:
      return output_accel
    self.resume_brake_ramp_frames -= 1
    if should_stop or a_target <= RESUME_BRAKE_RAMP_BYPASS_DECEL or drel_filtered < RESUME_BRAKE_RAMP_BYPASS_DREL:
      self.resume_brake_ramp_frames = 0
      return output_accel
    floor = min(self.last_output_accel, 0.0) - RESUME_BRAKE_RAMP_RATE * DT_CTRL
    return max(output_accel, floor)
