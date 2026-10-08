"""StarPilot's Bosch gas path: LongGasLearner, the Civic hill gain and the brake-mode hysteresis.

Ported from StarPilot's opendbc honda carcontroller.py (owner's branch, 2026-10-08). The learner, its rails and the
hill/brake helpers are StarPilot's, unchanged. They are coupled to the Civic Bosch BOSCH_GAS_LOOKUP_V = [0, 750] set in
CarControllerParams, so they ship together: the gasfactor the learner converges to is only meaningful against that
lookup. IQ.Pilot's prebuilt params library has no HondaGasFactorParams / HondaWindFactorParams keys, so the factors
persist to a JSON file instead; on first boot it seeds from StarPilot's own persisted values when StarPilot's sidecar
says they were learned on this fingerprint and learner version.
"""
import json
import math
import os
import threading
from collections import deque

import numpy as np

from iqdbc.car import DT_CTRL

# Civic Bosch hill gain (StarPilot 2026-09-29, offline fit over 26 Bosch-A op-long routes, 101k quasi-steady samples,
# leave-one-route-out; shipped at 1.1 rather than the fitted 1.2 because 1.2 doubled crest overshoot in the hill sim).
# The gain acts about the device mount's +0.013 rad CC pitch bias, so flat-road gas is unchanged. Log/sim evidence.
CIVIC_BOSCH_HILL_GAS_GAIN = 1.1
CIVIC_BOSCH_PITCH_BIAS = 0.013  # rad

# Bosch brake-mode hysteresis on the road-load-adjusted force (m/s^2)
BOSCH_BRAKE_FORCE_ON = -0.12
BOSCH_BRAKE_FORCE_RELEASE = -0.02

LEARNER_PATH = "/data/iq_honda_gas_learner.json"
STARPILOT_META_PATH = "/data/honda_learner_meta.json"
STARPILOT_PARAMS_DIR = "/data/params/d"

# Bump when learner semantics change so persisted values are discarded (StarPilot's LEARN_VERSION)
LEARN_VERSION = 2

# Learner tick cadence: update() runs every 2 controller frames at 100 Hz -> 0.02 s per tick
_LEARNER_DT = 2 * DT_CTRL
# Lag alignment: typical longitudinalActuatorDelay ~0.5 s -> 25 learner ticks
_LAG_TICKS = 25
# Quasi-steady gate: |d accel_cmd| / dt must be < 0.3 m/s^3 across the deque window
_ACCEL_RATE_THRESH = 0.3

# soft relative clamps (relative to nominal 1.0)
_HARD_LO = 0.6
_HARD_HI = 1.6
_SOFT_LO = 0.8
_SOFT_HI = 1.25
_DECAY_RATE_PER_MIN = 0.01  # fraction/min decayed toward 1.0 while outside soft band
_DECAY_PER_TICK = _DECAY_RATE_PER_MIN / 60.0 * _LEARNER_DT

# Applied-factor first-order filter (rc ~7.5 s)
_FACTOR_FILTER_RC = 7.5
_FACTOR_FILTER_ALPHA = _LEARNER_DT / (_FACTOR_FILTER_RC + _LEARNER_DT)

# Hill/saturation deadband
_PITCH_DEADBAND = 0.02  # rad
_BRAKE_ADDON_DEADBAND = 1.0  # m/s^2


def bosch_gas_lookup_accel(gas_pedal_force: float, hill_brake: float, gasfactor: float, min_gas: float,
                           hill_gain: float = 1.0, hill_level: float = 0.0) -> float:
  # The learned gasfactor scales the flat-road part of the request only; the hill feed-forward is added on top
  # unscaled. The learner freezes above |pitch| 0.02 rad, so gasfactor is fitted on flat road and was never checked
  # against the hill term (StarPilot route 280: scaling the hill term by gasfactor ran the car +0.10..+0.27 m/s^2 over
  # the command on a climb). Anchored at min_gas so gasfactor scales the offset from the pedal-on threshold.
  return (gas_pedal_force - hill_brake - min_gas) * gasfactor + min_gas + hill_brake + (hill_gain - 1.0) * (hill_brake - hill_level)


def update_honda_bosch_braking(braking: bool, gas_pedal_force: float, stopping: bool, long_active: bool) -> bool:
  """Select Bosch brake mode from the same road-load-adjusted force used for gas."""
  if not long_active:
    return False
  if stopping:
    return True
  if braking:
    return gas_pedal_force <= BOSCH_BRAKE_FORCE_RELEASE
  return gas_pedal_force < BOSCH_BRAKE_FORCE_ON


class LongGasLearner:
  """Lag-aligned gas/wind factor learner with safety rails (StarPilot's, unchanged).

  raw_gasfactor / raw_windfactor are the learned integrators (persisted); gasfactor / windfactor are the slow-filtered
  applied values, initialised from the persisted ones so there is no startup transient. Pure logic: persistence is
  load_gas_learner / save_gas_learner. Called every 2 controller frames (0.02 s).
  """

  def __init__(self, init_gasfactor: float, init_windfactor: float, car_fingerprint: str):
    init_gasfactor = self._safe_clamp(init_gasfactor)
    init_windfactor = self._safe_clamp(init_windfactor)

    self.raw_gasfactor = init_gasfactor
    self.raw_windfactor = init_windfactor
    self.gasfactor = init_gasfactor
    self.windfactor = init_windfactor

    self.car_fingerprint = str(car_fingerprint)
    self.last_gas_error = 0.0
    self.learning = False

    self._accel_deque: deque = deque(maxlen=_LAG_TICKS + 1)

    # anti-windup shadows
    self.gasfactor_before_maxgas = init_gasfactor
    self.windfactor_before_maxgas = init_windfactor
    self.windfactor_before_brake = init_windfactor

    self._was_engaged = False

  @staticmethod
  def _safe_clamp(v: float, lo: float = _HARD_LO, hi: float = _HARD_HI) -> float:
    if not math.isfinite(v):
      return 1.0
    return float(np.clip(v, lo, hi))

  @staticmethod
  def _decay_toward_nominal(v: float) -> float:
    if v < _SOFT_LO or v > _SOFT_HI:
      v = min(1.0, v + _DECAY_PER_TICK) if v < 1.0 else max(1.0, v - _DECAY_PER_TICK)
    return v

  def reset_deque(self, accel_cmd: float):
    self._accel_deque.clear()
    for _ in range(_LAG_TICKS + 1):
      self._accel_deque.append(accel_cmd)

  def update(self, accel_cmd: float, a_ego: float, gas_pedal_force: float, wind_brake_ms2: float, long_active: bool,
             long_pid: bool, gas_pressed: bool, brake_pressed: bool, v_ego: float, at_standstill: bool, pitch: float,
             brake_addon: float, at_accel_max: bool):
    engaged = long_active and long_pid
    self.learning = False

    if (not self._was_engaged and engaged) or gas_pressed:
      self.reset_deque(accel_cmd)
    self._was_engaged = engaged

    self._accel_deque.append(accel_cmd)

    should_learn = engaged and not gas_pressed and not brake_pressed and not at_standstill

    if should_learn and len(self._accel_deque) == _LAG_TICKS + 1:
      # lag-aligned reference: the command that was current ~0.5 s ago
      lagged_accel = self._accel_deque[0]
      accel_rate = abs(self._accel_deque[-1] - self._accel_deque[0]) / (_LAG_TICKS * _LEARNER_DT)
      quasi_steady = accel_rate < _ACCEL_RATE_THRESH

      pitch_ok = abs(pitch - (CIVIC_BOSCH_PITCH_BIAS if self.car_fingerprint == "HONDA_CIVIC_BOSCH" else 0.0)) < _PITCH_DEADBAND
      brake_addon_ok = abs(brake_addon) < _BRAKE_ADDON_DEADBAND

      if quasi_steady and pitch_ok and brake_addon_ok:
        self.learning = True
        gas_error = lagged_accel - a_ego
        self.last_gas_error = float(gas_error)

        if gas_error != 0.0 and gas_pedal_force > 0.0:
          if self.car_fingerprint in ("HONDA_INSIGHT", "HONDA_CIVIC_BOSCH"):  # gas pedal reacts too slowly
            learn_speed = 150.0
          elif self.car_fingerprint in ("ACURA_RDX_3G", "ACURA_RDX_3G_MMR"):  # prevent overreacting to turbo lag
            learn_speed = 300.0
          else:
            learn_speed = 50.0
          self.raw_gasfactor = np.clip(self.raw_gasfactor + gas_error / learn_speed * gas_pedal_force, _HARD_LO, _HARD_HI)

        if gas_error != 0.0 and v_ego > 0.0:
          wind_learn_speed = 100.0 if self.car_fingerprint in ("ACURA_RDX_3G", "ACURA_RDX_3G_MMR") else 1000.0
          wind_adjust = 1.0 + wind_brake_ms2 / wind_learn_speed
          self.raw_windfactor = np.clip(self.raw_windfactor * (wind_adjust if gas_error > 0.0 else 1.0 / wind_adjust),
                                        _HARD_LO, _HARD_HI)

    if gas_pedal_force <= 0.0:
      # braking: don't reduce windfactor, allow increases
      self.raw_windfactor = max(self.raw_windfactor, self.windfactor_before_brake)
    else:
      self.windfactor_before_brake = self.raw_windfactor

    if at_accel_max:
      # saturation: don't increase either factor, and decay gasfactor slightly while clipped
      self.raw_gasfactor = min(self.raw_gasfactor, self.gasfactor_before_maxgas)
      self.raw_windfactor = min(self.raw_windfactor, self.windfactor_before_maxgas)
      self.raw_gasfactor = max(_HARD_LO, self.raw_gasfactor - _DECAY_PER_TICK)
    else:
      self.gasfactor_before_maxgas = self.raw_gasfactor
      self.windfactor_before_maxgas = self.raw_windfactor

    if not math.isfinite(self.raw_gasfactor):
      self.raw_gasfactor = 1.0
      self.gasfactor_before_maxgas = 1.0
    if not math.isfinite(self.raw_windfactor):
      self.raw_windfactor = 1.0
      self.windfactor_before_maxgas = 1.0
      self.windfactor_before_brake = 1.0

    self.raw_gasfactor = self._decay_toward_nominal(self.raw_gasfactor)
    self.raw_windfactor = self._decay_toward_nominal(self.raw_windfactor)

    self.raw_gasfactor = float(np.clip(self.raw_gasfactor, _HARD_LO, _HARD_HI))
    self.raw_windfactor = float(np.clip(self.raw_windfactor, _HARD_LO, _HARD_HI))

    self.gasfactor = _FACTOR_FILTER_ALPHA * self.raw_gasfactor + (1.0 - _FACTOR_FILTER_ALPHA) * self.gasfactor
    self.windfactor = _FACTOR_FILTER_ALPHA * self.raw_windfactor + (1.0 - _FACTOR_FILTER_ALPHA) * self.windfactor

    if not math.isfinite(self.gasfactor):
      self.gasfactor = 1.0
    if not math.isfinite(self.windfactor):
      self.windfactor = 1.0

    return self.gasfactor, self.windfactor


def _clamped_pair(gas, wind) -> tuple[float, float] | None:
  gas, wind = float(gas), float(wind)
  if not (math.isfinite(gas) and math.isfinite(wind)):
    return None
  return float(np.clip(gas, _HARD_LO, _HARD_HI)), float(np.clip(wind, _HARD_LO, _HARD_HI))


def load_gas_learner(car_fingerprint: str, path: str = LEARNER_PATH, starpilot_meta_path: str = STARPILOT_META_PATH,
                     starpilot_params_dir: str = STARPILOT_PARAMS_DIR) -> tuple[float, float]:
  """Persisted (gasfactor, windfactor) for this fingerprint and learner version, else (1.0, 1.0)."""
  try:
    with open(path, encoding="utf-8") as f:
      d = json.load(f)
    if d.get("car_fingerprint") == str(car_fingerprint) and d.get("learn_version") == LEARN_VERSION:
      pair = _clamped_pair(d["gas"], d["wind"])
      if pair is not None:
        return pair
  except (OSError, ValueError, KeyError, TypeError):
    pass

  # first boot on IQ.Pilot: resume from StarPilot's learned values on the same device, under the same checks
  try:
    with open(starpilot_meta_path, encoding="utf-8") as f:
      meta = json.load(f)
    if meta.get("car_fingerprint") == str(car_fingerprint) and meta.get("learn_version") == LEARN_VERSION:
      with open(os.path.join(starpilot_params_dir, "HondaGasFactorParams"), encoding="utf-8") as f:
        gas = f.read().strip()
      with open(os.path.join(starpilot_params_dir, "HondaWindFactorParams"), encoding="utf-8") as f:
        wind = f.read().strip()
      pair = _clamped_pair(gas, wind)
      if pair is not None:
        return pair
  except (OSError, ValueError, KeyError, TypeError):
    pass

  return 1.0, 1.0


def _write_atomic(path: str, payload: dict):
  try:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump(payload, f, sort_keys=True)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  except OSError:
    pass


def save_gas_learner(learner: LongGasLearner, path: str = LEARNER_PATH, background: bool = True):
  """Persist the raw integrators (not the filtered values) off the control thread."""
  payload = {
    "car_fingerprint": learner.car_fingerprint,
    "learn_version": LEARN_VERSION,
    "gas": float(learner.raw_gasfactor),
    "wind": float(learner.raw_windfactor),
  }
  if background:
    threading.Thread(target=_write_atomic, args=(path, payload), name="honda-gas-learner-save", daemon=True).start()
  else:
    _write_atomic(path, payload)
