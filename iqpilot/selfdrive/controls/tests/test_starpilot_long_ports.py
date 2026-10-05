"""StarPilot longitudinal ports: HumanFollowing, Resume Brake Ramp, Civic Bosch mid-band over-brake compensation."""
from types import SimpleNamespace

import numpy as np
import pytest

from iqpilot.common.realtime import DT_CTRL
from iqpilot.selfdrive.controls.lib import longcontrol as lc
from iqpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from iqpilot.selfdrive.controls.lib.longitudinal_mpc_lib import long_mpc
from iqpilot.selfdrive.iqmodeld.config import ModelConstants
from iqdbc.car.honda.carcontroller import bosch_overbrake_compensation


# ---------- Civic Bosch mid-band over-brake compensation ----------

@pytest.mark.parametrize("accel,expected", [
  (-0.5, 0.0), (-1.0, 0.0),          # above the band
  (-1.5, 0.05), (-2.0, 0.10),        # mid band
  (-2.5, 0.15), (-2.75, 0.075),
  (-3.0, 0.0), (-3.5, 0.0),          # saturated / emergency: never weakened
])
def test_overbrake_comp_table(accel, expected):
  assert bosch_overbrake_compensation(accel, stopping=False) == pytest.approx(expected)


def test_overbrake_comp_not_while_stopping():
  assert bosch_overbrake_compensation(-2.5, stopping=True) == 0.0


# ---------- HumanFollowing ----------

def _model_lead(x0, v0, a=0.0, prob=0.9):
  t = np.asarray(ModelConstants.LEAD_T_IDXS, dtype=np.float64)
  v = np.maximum(v0 + a * t, 0.0)
  x = x0 + v0 * t + 0.5 * a * t ** 2
  return SimpleNamespace(prob=prob, x=x.tolist(), v=v.tolist())


def _radar_lead(d_rel, v_lead, status=True):
  return SimpleNamespace(status=status, dRel=d_rel, vLead=v_lead, aLeadK=0.0, aLeadTau=1.5, vRel=0.0, modelProb=0.9)


def test_human_following_anchors_on_radar():
  xv = long_mpc.build_model_lead_trajectory(_model_lead(45.0, 20.0, a=-1.0), _radar_lead(40.0, 22.0), v_ego=22.0)
  assert xv is not None
  assert xv.shape == (long_mpc.N + 1, 2)
  assert xv[0, 0] == pytest.approx(40.0)    # radar distance, not model's 45
  assert xv[0, 1] == pytest.approx(22.0)    # radar speed, not model's 20
  assert xv[-1, 1] < 22.0                   # model's braking delta carried
  assert np.all(np.diff(xv[:, 0]) >= -1e-9)


def test_human_following_threshold():
  assert long_mpc.build_model_lead_trajectory(_model_lead(40, 20, prob=0.30), _radar_lead(40, 20), 20.0) is None
  assert long_mpc.build_model_lead_trajectory(_model_lead(40, 20, prob=0.40), _radar_lead(40, 20), 20.0) is not None


def test_human_following_needs_radar_lead():
  assert long_mpc.build_model_lead_trajectory(_model_lead(40, 20), _radar_lead(40, 20, status=False), 20.0) is None
  assert long_mpc.build_model_lead_trajectory(None, _radar_lead(40, 20), 20.0) is None


def test_human_following_urgent_closing_falls_back():
  # 20 m gap closing at 12 m/s -> 1.67 s TTC < 2.0 s
  assert long_mpc.build_model_lead_trajectory(_model_lead(20, 10), _radar_lead(20, 10), v_ego=22.0) is None
  # 30 m closing at 12 m/s -> 2.5 s: model path kept
  assert long_mpc.build_model_lead_trajectory(_model_lead(30, 10), _radar_lead(30, 10), v_ego=22.0) is not None


def test_human_following_distance_bounded_by_speed():
  # model claims the lead jumps ahead much faster than its speed allows
  m = _model_lead(40.0, 15.0)
  m.x = (np.asarray(m.x) * 3.0).tolist()
  xv = long_mpc.build_model_lead_trajectory(m, _radar_lead(40.0, 15.0), v_ego=15.0)
  covered = xv[0, 0] + np.concatenate(([0.0], np.cumsum(long_mpc.T_DIFFS[1:] * (xv[:-1, 1] + xv[1:, 1]) / 2)))
  assert np.all(xv[:, 0] <= covered + 1e-6)


def test_human_following_is_on():
  assert long_mpc.HUMAN_FOLLOWING is True


# ---------- Resume Brake Ramp ----------

def _loc(fingerprint="HONDA_CIVIC_BOSCH"):
  CP = SimpleNamespace(longitudinalTuning=SimpleNamespace(kpBP=[0.], kpV=[0.], kiBP=[0.], kiV=[0.]),
                       carFingerprint=fingerprint, stopAccel=-0.5)
  CP_IQ = SimpleNamespace(stoppingDecelRateOverride=0.0, enableGasInterceptor=False)
  return LongControl(CP, CP_IQ)


@pytest.mark.parametrize("fingerprint", ["HONDA_CIVIC", "HONDA_ACCORD", "KIA_EV6"])
def test_resume_ramp_only_on_civic_bosch(fingerprint):
  loc = _loc(fingerprint)
  _run(loc, -1.3, 3, active=False)
  assert _run(loc, -1.3, 1)[0] == pytest.approx(-1.3)


def _cs(v=20.0):
  return SimpleNamespace(vEgo=v, aEgo=0.0, standstill=False, brakePressed=False,
                         cruiseState=SimpleNamespace(standstill=False))


def _run(loc, a_target, frames, lead_distance=0.0, has_lead=False, active=True):
  out = []
  for _ in range(frames):
    out.append(float(loc.update(active, _cs(), a_target, False, (-3.5, 2.0), lead_distance, has_lead,
                                v_target_now=20.0)))
  return out


def test_resume_ramp_limits_brake_buildup():
  loc = _loc()
  _run(loc, -1.3, 5, active=False)            # gas override: long inactive
  out = _run(loc, -1.3, 200)                  # released: off -> pid with a -1.3 target
  steps = np.diff([0.0] + out)
  assert min(steps) >= -lc.RESUME_BRAKE_RAMP_RATE * DT_CTRL - 1e-9   # deepens at most 1.5 m/s^3
  assert out[0] > -0.05                                              # no one-frame step to -1.3
  assert out[-1] == pytest.approx(-1.3, abs=1e-6)                     # reaches the target afterwards


def test_resume_ramp_bypassed_for_hard_brake():
  loc = _loc()
  _run(loc, -2.5, 3, active=False)
  assert _run(loc, -2.5, 1)[0] == pytest.approx(-2.5)


def test_resume_ramp_bypassed_for_close_lead():
  loc = _loc()
  _run(loc, -1.3, 3, lead_distance=20.0, has_lead=True, active=False)
  assert _run(loc, -1.3, 1, lead_distance=20.0, has_lead=True)[0] == pytest.approx(-1.3)


def test_resume_ramp_far_lead_still_ramped():
  loc = _loc()
  _run(loc, -1.3, 3, lead_distance=60.0, has_lead=True, active=False)
  assert _run(loc, -1.3, 1, lead_distance=60.0, has_lead=True)[0] > -0.05


def test_resume_ramp_never_limits_release():
  loc = _loc()
  _run(loc, -1.3, 3, active=False)
  _run(loc, -1.3, 200)
  assert _run(loc, 0.5, 1)[0] == pytest.approx(0.5)


def test_resume_ramp_window_expires():
  loc = _loc()
  _run(loc, 0.0, 3, active=False)
  _run(loc, 0.0, int(lc.RESUME_BRAKE_RAMP_TIME / DT_CTRL) + 5)   # window passes with no brake demand
  assert _run(loc, -1.3, 1)[0] == pytest.approx(-1.3)               # later brake demand is not delayed
