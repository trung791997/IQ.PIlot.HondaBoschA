#!/usr/bin/env python3
import math
import numpy as np

import iqpilot.cereal.messaging as messaging
from iqdbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from iqpilot.common.constants import CV
from iqpilot.common.filter_simple import FirstOrderFilter
from iqpilot.common.params import Params, UnknownKeyName
from iqpilot.common.realtime import DT_MDL
from iqpilot.selfdrive.iqmodeld.config import ModelConstants
from iqpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from iqpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from iqpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from iqpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, DEFAULT_STOPPING_SPEED, get_accel_from_plan
from iqpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from iqpilot.common.swaglog import cloudlog
from iqpilot.common.issue_debug import log_issue_limited

from iqpilot.selfdrive.controls.lib.iq_longitudinal_planner import LongitudinalPlannerIQ
from iqpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from iqpilot.selfdrive.controls.lib.e2e_distance_controller import E2EDistanceController

A_CRUISE_MAX_VALS = [2.0, 1.6, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -1.2
J_CRUISE = 1.0
CRUISE_STOP_SPEED = 1.0
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

LAUNCH_DISARM_SPEED = 2.0
LAUNCH_COMMIT_T = 3.5
LAUNCH_MOVING_SPEED = 1.2
LAUNCH_MAX_ACCEL = 1.5

E2E_CRUISE_CONVERGENCE_TAU = 15.0
E2E_CRUISE_ACCEL_MAX = 0.5
E2E_MODEL_SPEED_HORIZON = 5.0
E2E_ACCEL_INTENT_BP = [-0.05, 0.05]
E2E_MODEL_SPEED_INTENT_BP = [-0.5, 0.0]

_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

MODE_BLEND_MAX_TIME = 3.0
MODE_BLEND_JERK_UP = 1.0
MODE_BLEND_JERK_DOWN = 2.5
MODE_BLEND_BRAKE_PASS = -1.0

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

HUMAN_ACCEL_CITY_SPEED_LIMIT = 25.  # m/s, FrogPilot CITY_SPEED_LIMIT


def get_max_accel_low_speeds(max_accel, v_cruise):
  """Scale max accel by the set speed: 1/4 at 0, 1/2 at 12.5 m/s, full from 25 m/s."""
  return float(np.interp(v_cruise, [0., HUMAN_ACCEL_CITY_SPEED_LIMIT / 2, HUMAN_ACCEL_CITY_SPEED_LIMIT],
                         [max_accel / 4, max_accel / 2, max_accel]))


def get_max_accel_ramp_off(max_accel, v_cruise, v_ego):
  """Ease off as v_ego nears the set speed: 0 at it, 0.5 at 1 m/s below, full at 5 m/s below."""
  return float(np.interp(v_cruise - v_ego, [0., 1., 5.], [0., 0.5, max_accel]))


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3

def get_lead_distance(radarState):
  if radarState.leadOne.status and (not radarState.leadTwo.status or radarState.leadOne.dRel < radarState.leadTwo.dRel):
    return radarState.leadOne.dRel
  if radarState.leadTwo.status:
    return radarState.leadTwo.dRel
  return 0

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  if e2e:
    max_accel = ACCEL_MAX
  else:
    max_accel = get_max_accel(v_ego)
    # HumanAcceleration (StarPilot starpilot_acceleration.py, always on there): scale by the set speed, then ease off
    # as v_ego nears it. Throttle side only.
    max_accel = get_max_accel_low_speeds(max_accel, v_cruise)
    max_accel = min(get_max_accel_ramp_off(max_accel, v_cruise, v_ego), max_accel)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
  if not allow_throttle:
    clipped_accel_coast = max(accel_coast, ACCEL_MIN)
    coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
    max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  target_accel = float(np.clip(target_accel, a_cruise_prev - J_CRUISE * dt, a_cruise_prev + J_CRUISE * dt))

  cruise_should_stop = v_cruise == 0.0 and v_ego < CRUISE_STOP_SPEED
  return target_accel, cruise_should_stop


def get_e2e_accel(v_ego, v_cruise, model_v, a_target, should_stop):
  if should_stop or v_cruise <= v_ego or len(model_v) != len(T_IDXS_MPC):
    return a_target

  accel = min((v_cruise - v_ego) / E2E_CRUISE_CONVERGENCE_TAU, E2E_CRUISE_ACCEL_MAX)
  if accel <= a_target:
    return a_target
  accel_intent = np.interp(a_target, E2E_ACCEL_INTENT_BP, [0.0, 1.0])
  model_speed = np.interp(E2E_MODEL_SPEED_HORIZON, T_IDXS_MPC, model_v)
  speed_intent = np.interp(model_speed - v_ego, E2E_MODEL_SPEED_INTENT_BP, [0.0, 1.0])
  return float(np.interp(min(accel_intent, speed_intent), [0.0, 1.0], [a_target, accel]))


def limit_mode_transition(a_target, a_prev, dt):
  upper = a_prev + MODE_BLEND_JERK_UP * dt
  if a_target <= MODE_BLEND_BRAKE_PASS:
    return min(a_target, upper)
  lower = a_prev - MODE_BLEND_JERK_DOWN * dt
  return float(np.clip(a_target, lower, upper))


def get_accel_candidates(e2e, has_lead, mpc_candidate, cruise_candidate, e2e_candidate):
  candidates = []
  if not e2e or has_lead:
    candidates.append(mpc_candidate)
  candidates.append(cruise_candidate)
  if e2e:
    candidates.append(e2e_candidate)
  return candidates



STOCK_BRAKE_FEEL = True
STOCK_FEEL_DEPTH_BP = [2.0, 2.25, 2.75, 3.5, 4.5, 5.5, 6.5, 7.5, 9.0, 11.0, 13.5, 17.5, 25.0]  # s
STOCK_FEEL_DEPTH_V = [-3.5, -3.2, -2.4, -2.3, -2.3, -2.1, -1.7, -1.6, -1.28, -1.09, -0.88, -0.74, -0.35]  # m/s^2
STOCK_FEEL_JERK_BP = [2.0, 2.5, 6.0, 10.0]  # s
STOCK_FEEL_JERK_V = [3.0, 2.0, 1.0, 0.6]  # m/s^3
STOCK_FEEL_TTC_FLOOR_S = 2.0
STOCK_FEEL_MIN_CLOSING = 0.5  # m/s
# Outside the fitted law (TTC at or under the floor, or no lead closing) the planner's depth is kept but it still deepens
# no faster than this. Replay (STATUS 220): with no limit there, the target stepped -0.6..-1.65 in one 50 ms frame when
# a simulated gap crossed TTC 2 s or a lead began opening (e5 -11.9, dfa -33, dfb -15 m/s^3). Stock's own p98 rate is
# 2-3.4 m/s^3 at every TTC; 5 reaches -3.5 from -1.85 in 0.33 s.
STOCK_FEEL_JERK_OUTSIDE = 5.0  # m/s^3
# Newborn lead aLeadK bound (D-080, proposed; replay only, not driven), now part of the StockBrakeFeel toggle (D-086). STATUS 212:
# on 2e2 (bookmark ~283 s) a radar lead first seen while its velocity estimate was still converging published aLeadK
# -3 / -2.5 when ~0.75 m/s^2 was needed, and on 2e1 a jumping track reached -13.7; the planner braked early and twice,
# and the (since removed) onset limit stood aside because aLeadK < -1.0. For the first NEWBORN_LEAD_FRAMES a radar
# track has been a lead (or after it drops out for NEWBORN_LEAD_GAP_FRAMES or its range jumps NEWBORN_LEAD_JUMP_M),
# its aLeadK may go no lower than -NEWBORN_LEAD_MIN_BRAKE, or lower only as far as a quadratic fit to its own range
# history (once NEWBORN_LEAD_FIT_FRAMES long) says it is slowing. Planner input only: dRel, vRel, TTC and radarState
# are untouched and the point is never dropped (D-041/D-042); only the assumed lead decel is bounded.


# Brake release slew and dwell (StarPilot, built in on there; closed-loop replay only, not driven). While braking the
# target may rise at most BRAKE_RELEASE_JERK, and a rise is held for BRAKE_RELEASE_DWELL_TICKS cycles first.
# Cost: every brake release starts 0.1 s later, and -2.5 -> 0 takes 1 s.
BRAKE_RELEASE_JERK = 2.5  # m/s^3
BRAKE_RELEASE_DWELL_TICKS = 2

# D-072 (StarPilot, built in on; it shipped as the PlannerShortActionTime toggle, default on): read the MPC output
# 0.30 s along the plan instead of at the actuator delay. Only the read-off point moves. Replay evidence only.
PLANNER_ACTION_T_OVERRIDE = True
PLANNER_ACTION_T_S = 0.30


def get_planner_action_t(actuator_delay: float) -> float:
  return PLANNER_ACTION_T_S if PLANNER_ACTION_T_OVERRIDE else actuator_delay + DT_MDL


def brake_onset_ttc(leads, min_closing: float = 1e-3) -> float:
  """Worst time to contact (s) over the active closing leads; inf when none is closing."""
  ttc_min = float('inf')
  for lead in leads:
    if lead is None or not bool(getattr(lead, 'status', False)):
      continue
    closing = max(0.0, -float(lead.vRel))
    if closing > min_closing:
      ttc_min = min(ttc_min, float(lead.dRel) / closing)
  return ttc_min

def stock_feel_target(leads, prev: float, target: float, dt: float) -> float:
  """D-086 stock Honda ACC brake law: while a lead is closing and the worst TTC is over STOCK_FEEL_TTC_FLOOR_S, the
  target goes no deeper than stock's depth at that TTC and deepens no faster than stock's rate; otherwise the planner's
  depth is kept and deepens at most STOCK_FEEL_JERK_OUTSIDE."""
  ttc = brake_onset_ttc(leads, STOCK_FEEL_MIN_CLOSING)
  if not ttc > STOCK_FEEL_TTC_FLOOR_S or ttc == float('inf'):
    return brake_onset_limited_target(prev, target, dt, STOCK_FEEL_JERK_OUTSIDE)
  target = max(target, float(np.interp(ttc, STOCK_FEEL_DEPTH_BP, STOCK_FEEL_DEPTH_V)))
  return brake_onset_limited_target(prev, target, dt, float(np.interp(ttc, STOCK_FEEL_JERK_BP, STOCK_FEEL_JERK_V)))

def brake_onset_limited_target(prev: float, target: float, dt: float, jerk: float | None) -> float:
  """A brake below min(prev, 0) may deepen at most jerk * dt per step; throttle cuts, rises and jerk None pass through."""
  start = min(prev, 0.0)
  if jerk is None or target >= start:
    return float(target)
  return float(max(target, start - jerk * dt))

def brake_release_limited_target(prev: float, target: float, dt: float) -> float:
  """While braking, the target may rise at most BRAKE_RELEASE_JERK * dt per step; it may always fall."""
  if prev >= 0.0:
    return float(target)
  return float(min(target, prev + BRAKE_RELEASE_JERK * dt))

def brake_release_dwell_target(prev: float, target: float, rise_ticks: int) -> tuple[float, int]:
  """While braking, a rise is held for the first BRAKE_RELEASE_DWELL_TICKS cycles that ask for it; returns (target, rise_ticks)."""
  if target <= prev + 1e-3:
    return float(target), 0
  rise_ticks += 1
  if prev < 0.0 and rise_ticks <= BRAKE_RELEASE_DWELL_TICKS:
    return float(prev), rise_ticks
  return float(target), rise_ticks


class LongitudinalPlanner(LongitudinalPlannerIQ):
  def __init__(self, CP, CP_IQ, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.stopping_speed = CP_IQ.longitudinalStoppingSpeedOverride or DEFAULT_STOPPING_SPEED
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerIQ.__init__(self, self.CP, CP_IQ, self.mpc)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.output_a_target = 0.0
    self.output_should_stop = False
    self.launch_armed = False
    self.prev_e2e = False
    self.mode_blend_timer = 0.0
    try:
      self.exp_speed_conv = Params().get_bool("expSpeedConv")
    except UnknownKeyName:
      self.exp_speed_conv = False
    try:
      distance_control_enabled = Params().get_bool("IQE2EDistanceControl")
    except UnknownKeyName:
      distance_control_enabled = False
    self.distance_control = E2EDistanceController(distance_control_enabled, dt)
    # D-086 StockBrakeFeel: StarPilot's toggle (default off there), baked in on here (owner, 2026-10-05)
    self.stock_brake_feel = STOCK_BRAKE_FEEL
    self.brake_release_rise_ticks = BRAKE_RELEASE_DWELL_TICKS + 1
    try:
      accel_boost_enabled = Params().get_bool("IQGasOverrideBoost")
    except UnknownKeyName:
      accel_boost_enabled = False
    self.accel_boost = AccelBoost(accel_boost_enabled, dt)

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    LongitudinalPlannerIQ.update(self, sm)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.a_desired

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, model_v, model_a, _, throttle_prob = self.parse_model(sm['modelV2'])
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    personality = sm['selfdriveState'].personality
    self.mpc.set_weights(personality=personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['modelV2'], sm['radarState'], personality=personality)
    v_cruise = LongitudinalPlannerIQ.update_targets(self, sm, self.v_desired_filter.x, v_cruise)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")
    a_prev = self.a_desired

    action_t = get_planner_action_t(self.CP.longitudinalActuatorDelay)

    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, stopping_speed=self.stopping_speed)

    accel_boost = self.accel_boost.update(sm['selfdriveState'].enabled, v_ego, sm['carState'].gasPressed)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration + accel_boost

    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    output_a_target_e2e, output_should_stop_e2e = self.apply_e2e_stop_distance(sm, v_ego, output_a_target_e2e, output_should_stop_e2e)
    if self.is_e2e(sm) and self.exp_speed_conv and not self.mpc.status:
      output_a_target_e2e = get_e2e_accel(v_ego, v_cruise, model_v, output_a_target_e2e, output_should_stop_e2e)

    if sm['carState'].standstill:
      self.launch_armed = True
    elif v_ego > LAUNCH_DISARM_SPEED:
      self.launch_armed = False
    if (self.launch_armed and self.is_e2e(sm) and not output_should_stop_e2e and
        np.interp(LAUNCH_COMMIT_T, T_IDXS_MPC, model_v) > LAUNCH_DISARM_SPEED):
      t_cut = min(float(T_IDXS_MPC[np.argmax(model_v > LAUNCH_MOVING_SPEED)]), LAUNCH_COMMIT_T)
      t_shifted = T_IDXS_MPC + t_cut
      v_shifted = np.interp(t_shifted, T_IDXS_MPC, model_v)
      a_shifted = np.interp(t_shifted, T_IDXS_MPC, model_a)
      # the model-launch read keeps the actuator delay (D-072 moves only the MPC read-off point)
      a_launch = get_accel_from_plan(v_shifted, a_shifted, T_IDXS_MPC, action_t=self.CP.longitudinalActuatorDelay + DT_MDL)[0]
      a_launch_max = np.interp(v_ego, [LAUNCH_MOVING_SPEED, LAUNCH_DISARM_SPEED], [LAUNCH_MAX_ACCEL, 0.])
      output_a_target_e2e = max(output_a_target_e2e, min(a_launch, a_launch_max))

    e2e = self.is_e2e(sm)
    self.a_cruise, cruise_should_stop = get_cruise_accel(e2e, v_cruise, v_ego, self.a_cruise,
                                                          steer_angle_without_offset, self.CP, self.dt,
                                                          accel_coast, self.allow_throttle)

    if self.distance_control.enabled:
      lateral_accel = v_ego ** 2 * steer_angle_without_offset * CV.DEG_TO_RAD / (self.CP.steerRatio * self.CP.wheelbase)
    else:
      lateral_accel = 0.0
    output_a_target_e2e = self.distance_control.update(
      sm, self.mpc, a_model=output_a_target_e2e, a_mpc=output_a_target_mpc, a_cruise=self.a_cruise,
      e2e=e2e, engaged=self.CP.openpilotLongitudinalControl and not reset_state and sm['selfdriveState'].enabled and sm['carControl'].longActive,
      should_stop=output_should_stop_e2e or output_should_stop_mpc or cruise_should_stop or self.forcing_stop,
      fcw=self.fcw, allow_throttle=self.allow_throttle, lateral_accel=lateral_accel,
    )

    candidates = get_accel_candidates(
      e2e,
      self.mpc.status,
      (output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
      (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop),
      (output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e),
    )

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)

    self.output_should_stop = self.output_should_stop or self.forcing_stop

    if e2e != self.prev_e2e:
      self.mode_blend_timer = MODE_BLEND_MAX_TIME
    self.prev_e2e = e2e
    if reset_state or self.fcw or self.output_should_stop:
      self.mode_blend_timer = 0.0
    if self.mode_blend_timer > 0.0:
      limited_a_target = limit_mode_transition(output_a_target, a_prev, self.dt)
      self.mode_blend_timer = 0.0 if limited_a_target == output_a_target else max(self.mode_blend_timer - self.dt, 0.0)
      output_a_target = limited_a_target

    # StarPilot brake shaping, same gates as there: never while stopping, at standstill or on a planner reset
    standstill = bool(sm['carState'].standstill)
    if self.stock_brake_feel and not reset_state and not standstill and not self.output_should_stop:
      leads = (sm['radarState'].leadOne, sm['radarState'].leadTwo)
      output_a_target = stock_feel_target(leads, a_prev, output_a_target, self.dt)
    if not reset_state and not standstill:
      output_a_target = brake_release_limited_target(a_prev, output_a_target, self.dt)
      output_a_target, self.brake_release_rise_ticks = brake_release_dwell_target(a_prev, output_a_target,
                                                                                  self.brake_release_rise_ticks)
    else:
      self.brake_release_rise_ticks = BRAKE_RELEASE_DWELL_TICKS + 1
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    self.a_desired = float(self.output_a_target)
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    gate_services = ['carState', 'controlsState', 'selfdriveState', 'radarState']
    plan_send.valid = sm.all_checks(service_list=gate_services)
    if not plan_send.valid:
      log_issue_limited(
        "longitudinal_plan_invalid",
        "planner",
        f"longitudinalPlan invalid alive={ {s: sm.alive[s] for s in gate_services} } "
        f"freq_ok={ {s: sm.freq_ok[s] for s in gate_services} } valid={ {s: sm.valid[s] for s in gate_services} } "
        f"subchecks=({sm.all_alive(gate_services)},{sm.all_freq_ok(gate_services)},{sm.all_valid(gate_services)}) "
        f"recheck={sm.all_checks(service_list=gate_services)}",
        interval_sec=5.0,
      )

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.leadDistance = get_lead_distance(sm['radarState'])
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.leadTrajectoryX0 = self.mpc.lead_xv_0[:, 0].tolist()
    longitudinalPlan.leadTrajectoryV0 = self.mpc.lead_xv_0[:, 1].tolist()
    longitudinalPlan.leadTrajectoryX1 = self.mpc.lead_xv_1[:, 0].tolist()
    longitudinalPlan.leadTrajectoryV1 = self.mpc.lead_xv_1[:, 1].tolist()

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_iq(sm, pm)
