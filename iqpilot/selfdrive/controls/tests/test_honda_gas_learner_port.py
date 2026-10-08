"""StarPilot's Bosch gas path on IQ.Pilot: LongGasLearner, its persistence, the Civic hill gain, the brake-mode
hysteresis and the Civic Bosch [0, 750] gas lookup they are fitted against."""
import json
import math

import pytest

from iqdbc.car import ACCELERATION_DUE_TO_GRAVITY
from iqdbc.car.car_helpers import interfaces
from iqdbc.car.honda import gas_learner as gl
from iqdbc.car.honda.values import CAR, CarControllerParams


def _tick(lr, **kw):
  args = dict(accel_cmd=0.5, a_ego=0.3, gas_pedal_force=0.6, wind_brake_ms2=0.1, long_active=True, long_pid=True,
              gas_pressed=False, brake_pressed=False, v_ego=20.0, at_standstill=False, pitch=gl.CIVIC_BOSCH_PITCH_BIAS,
              brake_addon=0.0, at_accel_max=False)
  args.update(kw)
  return lr.update(**args)


def test_learner_raises_gas_when_under_delivering():
  lr = gl.LongGasLearner(1.0, 1.0, "HONDA_CIVIC_BOSCH")
  for _ in range(200):
    _tick(lr)
  assert lr.learning
  assert lr.raw_gasfactor > 1.0 and lr.raw_windfactor > 1.0
  assert 1.0 < lr.gasfactor < lr.raw_gasfactor  # applied value trails the integrator through the slow filter


@pytest.mark.parametrize("kw", [
  dict(pitch=0.05),                # hill
  dict(brake_addon=-1.5),          # extra brake working
  dict(gas_pressed=True),
  dict(brake_pressed=True),
  dict(long_pid=False),
])
def test_learner_frozen(kw):
  lr = gl.LongGasLearner(1.0, 1.0, "HONDA_CIVIC_BOSCH")
  for _ in range(200):
    _tick(lr, **kw)
  assert not lr.learning
  assert lr.raw_gasfactor == pytest.approx(1.0)


def test_learner_lag_aligned_reference():
  lr = gl.LongGasLearner(1.0, 1.0, "HONDA_CIVIC_BOSCH")
  _tick(lr, accel_cmd=0.5)  # engagement edge pre-fills the deque
  _tick(lr, accel_cmd=0.5, a_ego=0.5)
  assert lr.last_gas_error == pytest.approx(0.0)


def test_learner_hard_clamps_and_nan():
  lr = gl.LongGasLearner(float("nan"), 9.0, "HONDA_CIVIC_BOSCH")
  assert lr.gasfactor == 1.0 and lr.windfactor == pytest.approx(1.6)


def test_hill_gain_flat_road_unchanged():
  hill_level = math.sin(gl.CIVIC_BOSCH_PITCH_BIAS) * ACCELERATION_DUE_TO_GRAVITY
  # at the mount's pitch bias the hill gain does nothing
  flat = gl.bosch_gas_lookup_accel(0.8, hill_level, 1.2, 0.0, gl.CIVIC_BOSCH_HILL_GAS_GAIN, hill_level)
  assert flat == pytest.approx(gl.bosch_gas_lookup_accel(0.8, hill_level, 1.2, 0.0))
  # gasfactor scales the flat-road part only, not the hill term
  assert gl.bosch_gas_lookup_accel(1.0, 0.5, 1.2, 0.0) == pytest.approx(0.5 * 1.2 + 0.5)


def test_braking_hysteresis():
  b = gl.update_honda_bosch_braking
  assert not b(False, -0.10, False, True)   # above the on threshold
  assert b(False, -0.13, False, True)
  assert b(True, -0.05, False, True)        # held until release
  assert not b(True, -0.01, False, True)
  assert b(False, 0.5, True, True)          # stopping
  assert not b(True, -1.0, True, False)     # not long active


def test_persistence_roundtrip_and_fingerprint_check(tmp_path):
  path = str(tmp_path / "learner.json")
  lr = gl.LongGasLearner(1.3, 0.8, "HONDA_CIVIC_BOSCH")
  gl.save_gas_learner(lr, path=path, background=False)
  none = str(tmp_path / "missing")
  assert gl.load_gas_learner("HONDA_CIVIC_BOSCH", path, none, none) == pytest.approx((1.3, 0.8))
  assert gl.load_gas_learner("HONDA_ACCORD", path, none, none) == (1.0, 1.0)


def test_seeds_from_starpilot(tmp_path):
  meta = tmp_path / "meta.json"
  meta.write_text(json.dumps({"car_fingerprint": "HONDA_CIVIC_BOSCH", "learn_version": gl.LEARN_VERSION}))
  (tmp_path / "HondaGasFactorParams").write_text("1.3609723878477027")
  (tmp_path / "HondaWindFactorParams").write_text("0.7262530056834318")
  missing = str(tmp_path / "none.json")
  assert gl.load_gas_learner("HONDA_CIVIC_BOSCH", missing, str(meta), str(tmp_path)) == pytest.approx((1.3610, 0.7263), abs=1e-4)
  meta.write_text(json.dumps({"car_fingerprint": "HONDA_CIVIC_BOSCH", "learn_version": 1}))
  assert gl.load_gas_learner("HONDA_CIVIC_BOSCH", missing, str(meta), str(tmp_path)) == (1.0, 1.0)


def _cp(cand):
  CI = interfaces[cand]
  fp = {0: {}, 1: {}, 2: {}}
  return CI.get_params(cand, fp, [], alpha_long=True, is_release=False, docs=False)


def test_civic_bosch_gas_lookup():
  assert CarControllerParams(_cp(str(CAR.HONDA_CIVIC_BOSCH))).BOSCH_GAS_LOOKUP_V == [0, 750]
  assert CarControllerParams(_cp(str(CAR.HONDA_ACCORD))).BOSCH_GAS_LOOKUP_V == [0, 1600]
